"""The browser viewer's scene (module/ViserViewer/python/viser_server.py), built offline.

Asserts:
  - the state datagram header is the 48 bytes ViserViewer.cpp packs.
  - every visible, non-transparent geom of the soccer scene is drawn, the ball with its texture.
  - a second robot attached the way --robots does it (its own copy of every mesh) adds instances,
    not meshes.
  - after a state update each instance sits at its geom's pose: the position MuJoCo computes, and
    the orientation of its geom_xmat.
"""
import os
import sys

import mujoco
import numpy as np
import viser

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "module", "ViserViewer", "python")
)
import viser_server  # noqa: E402

SCENE = "models/k1/k1_scene_robocup.xml"
ROBOT = "models/k1/K1_22dof.xml"

failures = 0


def check(cond, what):
    global failures
    print(f"[{'PASS' if cond else 'FAIL'}] {what}")
    failures += 0 if cond else 1


def two_robot_model():
    """The soccer scene plus a second K1, attached with a prefix as SimCore does for --robots"""
    scene = mujoco.MjSpec.from_file(SCENE)
    robot = mujoco.MjSpec.from_file(ROBOT)
    for key in list(robot.keys):
        robot.delete(key)
    frame = scene.worldbody.add_frame(pos=[0, 2, 0.6])
    frame.attach_body(robot.body("Trunk"), "sub01_", "")
    return scene.compile()


def visible(model):
    return sum(
        1
        for g in range(model.ngeom)
        if model.geom_group[g] < viser_server.VISIBLE_GROUPS and viser_server.appearance(model, g)[0][3] > 0
    )


def main():
    check(viser_server.HEADER.size == 48, "state header is 48 bytes")

    server = viser.ViserServer(port=0, verbose=False)
    try:
        model = mujoco.MjModel.from_xml_path(SCENE)
        scene = viser_server.Scene(server, model)
        check(scene.geoms == visible(model), f"all {visible(model)} visible geoms drawn (got {scene.geoms})")
        check(len(scene.frames) == 1, "the ball is drawn textured")

        two = two_robot_model()
        scene2 = viser_server.Scene(server, two)
        check(scene2.geoms > scene.geoms, f"a second robot adds geoms ({scene.geoms} -> {scene2.geoms})")
        check(scene2.meshes == scene.meshes, f"but no meshes ({scene.meshes} -> {scene2.meshes})")

        # Move to a keyframe and compare every instance with MuJoCo's own geom poses
        data = mujoco.MjData(two)
        mujoco.mj_resetDataKeyframe(two, data, 0)
        data.qpos[3:7] = [0.9238795, 0.0, 0.0, 0.3826834]  # yaw the main robot 45 degrees
        mujoco.mj_kinematics(two, data)
        scene2.update(data.qpos, data.mocap_pos, data.mocap_quat)

        worst_pos = worst_rot = 0.0
        for handle, geoms in scene2.batches:
            for i, g in enumerate(geoms):
                worst_pos = max(worst_pos, np.abs(handle.batched_positions[i] - data.geom_xpos[g]).max())
                want = np.zeros(4)
                mujoco.mju_mat2Quat(want, data.geom_xmat[g])
                worst_rot = max(worst_rot, 1.0 - abs(np.dot(handle.batched_wxyzs[i], want)))
        check(worst_pos < 1e-5, f"instances at their geoms' positions (worst {worst_pos:.2e} m)")
        check(worst_rot < 1e-6, f"instances at their geoms' orientations (worst 1 - |q.q'| {worst_rot:.2e})")
    finally:
        server.stop()

    print(f"test_viser_scene: {'PASS' if failures == 0 else 'FAIL'}")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
