#!/usr/bin/env python3
"""The sim in a browser, served with viser (https://viser.studio).

module::ViserViewer launches this under --viser. It rebuilds the scene from the compiled model the sim
saved (--model), says hello to the sim's UDP port on 127.0.0.1 (--sim-port), and then moves the scene
with the joint state the sim streams back at 30 Hz. The page's buttons send commands the other way.

Each distinct shape (a mesh, or a primitive's type and size) and colour is one instanced viser mesh,
with an instance per geom that has it, so --robots 20 sends each robot mesh once. Textured meshes (the
ball) are ordinary meshes, since instances can't carry a texture.

Exits when the sim does.
"""
import argparse
import hashlib
import math
import os
import signal
import socket
import struct
import sys
import time

import numpy as np

# Ctrl+C in the sim's terminal reaches this process too. The sim stops it with SIGTERM as it shuts
# down, so ignore SIGINT rather than dying first with a traceback.
signal.signal(signal.SIGINT, signal.SIG_IGN)

try:
    import mujoco
    import trimesh
    import viser
    from PIL import Image
except ImportError as e:
    sys.exit(f"[viser] {e}: this k1sim image predates the browser viewer, rebuild it with `./b image`")

# One state datagram: this header, then qpos[nq], mocap_pos[3 * nmocap], mocap_quat[4 * nmocap] as
# doubles. Must match FrameHeader in ViserViewer.cpp.
HEADER = struct.Struct("<4sIddii16s")
MAGIC = b"NSV1"

# The geom groups the window shows by default: 0 (collision shapes), 1 (robot visuals), 2 (field markings)
VISIBLE_GROUPS = 3
# Half extent standing in for an infinite (size 0) plane [m]
INFINITE_PLANE = 50.0
# A geom's rgba when it doesn't set one; its material's colour applies instead
DEFAULT_RGBA = (0.5, 0.5, 0.5, 1.0)
# viser's image-based lighting at full strength washes the field's greens out
ENVIRONMENT_INTENSITY = 0.4
# The starting camera's distance from the model's centre, in model extents. The window uses 1.5,
# which at the browser's wider field of view leaves the robot a speck.
CAMERA_DISTANCE = 0.6

Geom = mujoco.mjtGeom


def quat_mul(a, b):
    """Hamilton product of (N, 4) wxyz quaternions"""
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=1,
    )


def texture_image(model, tex):
    h, w, c = model.tex_height[tex], model.tex_width[tex], model.tex_nchannel[tex]
    adr = model.tex_adr[tex]
    pixels = model.tex_data[adr : adr + h * w * c].reshape(h, w, c)
    # MuJoCo's texture coordinates count rows from the first in memory, trimesh's from the image's bottom
    return Image.fromarray(np.ascontiguousarray(pixels[::-1]).squeeze())


def appearance(model, g):
    """The geom's rgba and, for a mesh with texture coordinates, its texture image, resolving the
    material the way MuJoCo's renderer does"""
    rgba = model.geom_rgba[g].copy()
    mat = model.geom_matid[g]
    if mat < 0:
        return rgba, None
    if np.allclose(rgba, DEFAULT_RGBA):
        rgba = model.mat_rgba[mat].copy()

    tex = model.mat_texid[mat, mujoco.mjtTextureRole.mjTEXROLE_RGB]
    if tex < 0 or model.tex_type[tex] != mujoco.mjtTexture.mjTEXTURE_2D:
        return rgba, None
    image = texture_image(model, tex)
    if model.geom_type[g] == Geom.mjGEOM_MESH and model.mesh_texcoordadr[model.geom_dataid[g]] >= 0:
        return rgba, image
    # Nothing to map it with (the field's grass on a plane): tint by its average colour instead
    pixels = np.asarray(image.convert("RGB"), dtype=np.float64)
    rgba[:3] *= pixels.reshape(-1, 3).mean(axis=0) / 255.0
    return rgba, None


def mesh_digest(model, m):
    """A mesh's identity by content: each --robots copy of the robot has its own copy of its meshes"""
    v, nv = model.mesh_vertadr[m], model.mesh_vertnum[m]
    f, nf = model.mesh_faceadr[m], model.mesh_facenum[m]
    digest = hashlib.sha1(model.mesh_vert[v : v + nv].tobytes())
    digest.update(model.mesh_face[f : f + nf].tobytes())
    return digest.hexdigest()


def shape_key(model, g, digests):
    """What makes two geoms the same shape"""
    if model.geom_type[g] == Geom.mjGEOM_MESH:
        m = int(model.geom_dataid[g])
        if m not in digests:
            digests[m] = mesh_digest(model, m)
        return ("mesh", digests[m])
    return (int(model.geom_type[g]), tuple(np.round(model.geom_size[g], 6)))


def geom_mesh(model, g):
    """The geom's untextured mesh in its own frame, or None for types this viewer doesn't draw"""
    t = model.geom_type[g]
    size = model.geom_size[g]
    if t == Geom.mjGEOM_PLANE:
        x = size[0] if size[0] > 0 else INFINITE_PLANE
        y = size[1] if size[1] > 0 else INFINITE_PLANE
        return trimesh.Trimesh([[-x, -y, 0], [x, -y, 0], [x, y, 0], [-x, y, 0]], [[0, 1, 2], [0, 2, 3]])
    if t == Geom.mjGEOM_SPHERE:
        mesh = trimesh.creation.icosphere(subdivisions=3, radius=size[0])
    elif t == Geom.mjGEOM_ELLIPSOID:
        mesh = trimesh.creation.icosphere(subdivisions=3)
        mesh.apply_scale(size)
    elif t == Geom.mjGEOM_CAPSULE:
        mesh = trimesh.creation.capsule(height=2 * size[1], radius=size[0], count=[32, 16])
    elif t == Geom.mjGEOM_CYLINDER:
        mesh = trimesh.creation.cylinder(radius=size[0], height=2 * size[1], sections=32)
    elif t == Geom.mjGEOM_BOX:
        mesh = trimesh.creation.box(extents=2 * size)
    elif t == Geom.mjGEOM_MESH:
        m = model.geom_dataid[g]
        v, nv = model.mesh_vertadr[m], model.mesh_vertnum[m]
        f, nf = model.mesh_faceadr[m], model.mesh_facenum[m]
        mesh = trimesh.Trimesh(model.mesh_vert[v : v + nv], model.mesh_face[f : f + nf], process=False)
    else:
        return None  # height fields, SDFs
    # The browser smooths normals across shared vertices; split them at sharp edges so boxes and
    # the robot's machined edges stay sharp
    return mesh.smooth_shaded


def textured_mesh(model, g, image):
    """A mesh geom with its texture, in its own frame"""
    m = model.geom_dataid[g]
    v = model.mesh_vertadr[m]
    f, nf = model.mesh_faceadr[m], model.mesh_facenum[m]
    t = model.mesh_texcoordadr[m]
    n = model.mesh_normaladr[m]
    # Texture coordinates and normals are per face corner, so give every corner its own vertex
    corners = model.mesh_face[f : f + nf].reshape(-1)
    return trimesh.Trimesh(
        vertices=model.mesh_vert[v + corners],
        faces=np.arange(3 * nf).reshape(-1, 3),
        vertex_normals=model.mesh_normal[n + model.mesh_facenormal[f : f + nf].reshape(-1)],
        visual=trimesh.visual.TextureVisuals(
            uv=model.mesh_texcoord[t + model.mesh_facetexcoord[f : f + nf].reshape(-1)], image=image
        ),
        process=False,
    )


class Scene:
    """The model's visible geoms as viser meshes, moved by each state frame"""

    def __init__(self, server, model):
        self.model = model
        self.data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, self.data)  # the model's rest pose until the first frame
        moving = (model.body_weldid != 0) | (model.body_mocapid >= 0)
        positions, wxyzs = self.geom_poses()

        batches = {}
        textured = []
        digests = {}
        for g in range(model.ngeom):
            if model.geom_group[g] >= VISIBLE_GROUPS:
                continue
            rgba, image = appearance(model, g)
            if rgba[3] <= 0:
                continue
            if image is not None:
                textured.append((g, image))
            else:
                batches.setdefault(shape_key(model, g, digests) + (tuple(np.round(rgba, 4)),), []).append(g)

        self.geoms = sum(len(geoms) for geoms in batches.values()) + len(textured)
        self.meshes = len(batches) + len(textured)

        # Instanced meshes, and the geoms each instance follows
        self.batches = []
        for i, (key, geoms) in enumerate(batches.items()):
            mesh = geom_mesh(model, geoms[0])
            if mesh is None:
                continue
            rgba = np.array(key[-1])
            geoms = np.array(geoms)
            handle = server.scene.add_batched_meshes_simple(
                f"/geoms/{i}",
                vertices=np.asarray(mesh.vertices, dtype=np.float32),
                faces=np.asarray(mesh.faces, dtype=np.uint32),
                batched_wxyzs=wxyzs[geoms],
                batched_positions=positions[geoms],
                batched_colors=tuple(int(c) for c in np.round(255 * rgba[:3])),
                opacity=float(rgba[3]) if rgba[3] < 1 else None,
            )
            if moving[model.geom_bodyid[geoms]].any():
                self.batches.append((handle, geoms))

        # Textured meshes, and the geom each follows
        self.frames = []
        for g, image in textured:
            frame = server.scene.add_frame(f"/textured/{g}", show_axes=False, position=positions[g], wxyz=wxyzs[g])
            server.scene.add_mesh_trimesh(f"/textured/{g}/mesh", textured_mesh(model, g, image))
            if moving[model.geom_bodyid[g]]:
                self.frames.append((frame, g))

    def geom_poses(self):
        """Every geom's world position and orientation (wxyz)"""
        return self.data.geom_xpos.copy(), quat_mul(self.data.xquat[self.model.geom_bodyid], self.model.geom_quat)

    def update(self, qpos, mocap_pos, mocap_quat):
        self.data.qpos[:] = qpos
        self.data.mocap_pos[:] = mocap_pos
        self.data.mocap_quat[:] = mocap_quat
        mujoco.mj_kinematics(self.model, self.data)
        positions, wxyzs = self.geom_poses()
        for handle, geoms in self.batches:
            handle.batched_positions = positions[geoms]
            handle.batched_wxyzs = wxyzs[geoms]
        for frame, g in self.frames:
            frame.position = positions[g]
            frame.wxyz = wxyzs[g]


def default_camera(model):
    """The starting camera: looking at the model's centre from the window's free camera angle"""
    az = math.radians(model.vis.global_.azimuth)
    el = math.radians(model.vis.global_.elevation)
    forward = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    look_at = np.array(model.stat.center)
    return look_at - CAMERA_DISTANCE * model.stat.extent * forward, look_at


def latest_datagram(sock):
    """The newest datagram waiting, blocking briefly for one; None if none came"""
    try:
        data = sock.recv(65536)
    except socket.timeout:
        return None
    # Frames can queue up while the scene updates; skip to the newest
    sock.setblocking(False)
    try:
        while True:
            data = sock.recv(65536)
    except BlockingIOError:
        pass
    finally:
        sock.settimeout(0.5)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--model", required=True, help="the compiled model the sim saved (.mjb)")
    parser.add_argument("--sim-port", type=int, required=True, help="the sim's UDP port on 127.0.0.1")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port to serve the page on")
    args = parser.parse_args()

    parent = os.getppid()
    model = mujoco.MjModel.from_binary_path(args.model)
    try:
        os.remove(args.model)
    except OSError:
        pass

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(0.5)
    sim = ("127.0.0.1", args.sim_port)

    def send(command):
        sock.sendto(command, sim)

    server = viser.ViserServer(host="0.0.0.0", port=args.port, label="NUSim")
    server.scene.set_up_direction("+z")
    position, look_at = default_camera(model)
    server.initial_camera.position = tuple(position)
    server.initial_camera.look_at = tuple(look_at)
    server.scene.configure_environment_map(environment_intensity=ENVIRONMENT_INTENSITY)

    start = time.monotonic()
    scene = Scene(server, model)
    print(
        f"[viser] {scene.geoms} geoms as {scene.meshes} meshes, built in {time.monotonic() - start:.1f} s", flush=True
    )

    status = server.gui.add_markdown("Waiting for the sim...")
    server.gui.add_button("Reset", hint="Reset the sim to its startup state (Backspace in the window)").on_click(
        lambda _: send(b"reset")
    )
    server.gui.add_button("Shove robot", hint="Push the robot over (F in the window)").on_click(
        lambda _: send(b"shove")
    )
    follow = server.gui.add_checkbox("Follow robot", initial_value=False, hint="Move the camera with the robot")

    # The robot the sim controls: the first free joint's body
    robot = model.jnt_bodyid[0] if model.njnt > 0 and model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE else None
    robot_at = None

    last_frame = last_hello = last_status = 0.0
    while os.getppid() == parent:
        now = time.monotonic()
        if now - last_frame > 1.0 and now - last_hello > 1.0:
            send(b"hello")
            last_hello = now

        frame = latest_datagram(sock)
        if frame is None or len(frame) < HEADER.size:
            continue
        magic, _, sim_time, rtf, nq, nmocap, mode = HEADER.unpack_from(frame)
        if (
            magic != MAGIC
            or nq != model.nq
            or nmocap != model.nmocap
            or len(frame) != HEADER.size + 8 * (nq + 7 * nmocap)
        ):
            continue
        last_frame = now
        values = np.frombuffer(frame, dtype="<f8", offset=HEADER.size)

        with server.atomic():
            scene.update(
                values[:nq], values[nq : nq + 3 * nmocap].reshape(-1, 3), values[nq + 3 * nmocap :].reshape(-1, 4)
            )

            if robot is not None:
                at = scene.data.xpos[robot].copy()
                if follow.value and robot_at is not None:
                    shift = at - robot_at
                    shift[2] = 0.0
                    for client in server.get_clients().values():
                        try:
                            client.camera.position = client.camera.position + shift
                        except AssertionError:
                            pass  # the client hasn't reported its camera yet
                robot_at = at

        if now - last_status > 0.25:
            last_status = now
            mode = mode.split(b"\0")[0].decode()
            status.content = f"**Sim time** {sim_time:.1f} s  \n**RTF** {rtf:.2f}  \n**Mode** {mode}"

    server.stop()


if __name__ == "__main__":
    main()
