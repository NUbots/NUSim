#!/usr/bin/env python3
"""Generate the pitch (floor, painted line markings and goals) for a RoboCup field scene.

Prints the <worldbody> fragment for one field, to paste into a k1_scene_*.xml between the
lights and the ball. `kid` reproduces k1_scene_robocup.xml's markings exactly; `middle` is the
2026 Humanoid Soccer League M-Field (HSL rules, Law 1 "Dimensions", table "Exemplary field
dimensions by field type"), used by the Middle Division.

    python3 models/k1/generate_field.py middle

Lines are thin white box geoms centred on the nominal dimensions, the same convention as
k1_scene_robocup.xml (the rules measure to the outside of the lines, 2.5 cm further out).
"""

import math
import sys
from dataclasses import dataclass

# Every marking is a flat white box, visual only (no contact)
LINE = 'rgba="1 1 1 1" contype="0" conaffinity="0" group="2"'
# Circle and arc segments overlap their neighbours so the painted line has no gaps
SEGMENT_OVERLAP = 1.15
SEGMENT_STEP = math.radians(9)


@dataclass
class Field:
    name: str  # used in the goal comment
    length: float
    width: float
    line_width: float
    goal_area_length: float
    goal_area_width: float
    penalty_area_length: float
    penalty_area_width: float
    penalty_mark_distance: float
    centre_circle_diameter: float
    corner_arc_radius: float  # 0 for no corner arcs
    mark_half_length: float  # half the arm length of the centre and penalty mark crosses
    goalpost_centre_y: float  # goalpost centres are at y = +-this
    goal_height: float  # height of the crossbar centreline
    goalpost_radius: float
    border: float


FIELDS = {
    # RoboCup KidSize, as ported from the Webots proto into k1_scene_robocup.xml
    "kid": Field(
        name="kid-size",
        length=9.0,
        width=6.0,
        line_width=0.05,
        goal_area_length=1.0,
        goal_area_width=3.0,
        penalty_area_length=2.0,
        penalty_area_width=5.0,
        penalty_mark_distance=1.5,
        centre_circle_diameter=1.5,
        corner_arc_radius=0.0,
        mark_half_length=0.075,
        goalpost_centre_y=1.3,
        goal_height=1.2,
        goalpost_radius=0.05,
        border=1.0,
    ),
    # 2026 HSL M-Field (former AdultSize). The rules give a range for the goal (width 2.4-2.6,
    # height 1.5-1.9); this uses 2.5 between the inner post edges and a 1.8 m crossbar.
    "middle": Field(
        name="HSL M-Field",
        length=14.0,
        width=9.0,
        line_width=0.05,
        goal_area_length=1.0,
        goal_area_width=4.0,
        penalty_area_length=3.0,
        penalty_area_width=6.0,
        penalty_mark_distance=2.0,
        centre_circle_diameter=3.0,
        corner_arc_radius=0.5,
        mark_half_length=0.05,
        goalpost_centre_y=1.3,
        goal_height=1.8,
        goalpost_radius=0.05,
        border=1.0,
    ),
}


def box(x, y, half_x, half_y, quat=None):
    rotation = f' quat="{quat[0]:.6f} 0 0 {quat[1]:.6f}"' if quat else ""
    return f'<geom type="box" pos="{x:.4f} {y:.4f} 0.001"{rotation} size="{half_x:.4f} {half_y:.4f} 0.001" {LINE}/>'


def arc(cx, cy, radius, start, count):
    """`count` segments of a circle about (cx, cy), the first centred at angle `start`."""
    half = radius * SEGMENT_STEP / 2 * SEGMENT_OVERLAP
    lines = []
    for i in range(count):
        theta = start + i * SEGMENT_STEP
        # Rotate the box's long (x) axis onto the tangent
        heading = (theta + math.pi / 2) / 2
        lines.append(
            box(
                cx + radius * math.cos(theta),
                cy + radius * math.sin(theta),
                half,
                f.line_width / 2,
                (math.cos(heading), math.sin(heading)),
            )
        )
    return lines


def cross(x, label):
    """Centre/penalty mark: two crossed boxes."""
    lw = f.line_width / 2
    return [
        f'<geom type="box" pos="{label} 0 0.001" size="{f.mark_half_length:.4f} {lw:.4f} 0.001" {LINE}/>',
        f'<geom type="box" pos="{label} 0 0.001" size="{lw:.4f} {f.mark_half_length:.4f} 0.001" {LINE}/>',
    ]


def markings():
    lw = f.line_width / 2
    hl, hw = f.length / 2, f.width / 2
    lines = [
        box(0, hw, hl, lw),  # touchlines
        box(0, -hw, hl, lw),
        box(hl, 0, lw, hw),  # goal lines
        box(-hl, 0, lw, hw),
        box(0, 0, lw, hw),  # halfway line
    ]
    # Goal areas, then penalty areas: front line plus the two sides, +x end then -x end
    for length, width in ((f.goal_area_length, f.goal_area_width), (f.penalty_area_length, f.penalty_area_width)):
        for s in (1, -1):
            lines += [
                box(s * (hl - length), 0, lw, width / 2),
                box(s * (hl - length / 2), width / 2, length / 2, lw),
                box(s * (hl - length / 2), -width / 2, length / 2, lw),
            ]
    lines += arc(0, 0, f.centre_circle_diameter / 2, 0, round(2 * math.pi / SEGMENT_STEP))
    # Centre mark, then the penalty marks
    mark_x = hl - f.penalty_mark_distance
    lines += cross(0, "0") + cross(mark_x, f"{mark_x:.4f}") + cross(-mark_x, f"{-mark_x:.4f}")
    # Corner arcs: quarter circles inside the field, centred on each corner
    if f.corner_arc_radius > 0:
        quarter = round(math.pi / 2 / SEGMENT_STEP)
        for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
            # Angle pointing from the corner into the field
            start = math.atan2(-sy, -sx) - math.pi / 4 + SEGMENT_STEP / 2
            lines += arc(sx * hl, sy * hw, f.corner_arc_radius, start, quarter)
    return lines


def goal(sign):
    name = "goal_pos_x" if sign > 0 else "goal_neg_x"
    y, h, r = f"{f.goalpost_centre_y:g}", f"{f.goal_height:g}", f"{f.goalpost_radius:g}"
    return [
        f'<body name="{name}" pos="{sign * f.length / 2:g} 0 0">',
        f'  <geom name="{name}_post1" type="cylinder" fromto="0 {y} 0  0 {y} {h}" size="{r}" material="post_mat" contype="1" conaffinity="1"/>',
        f'  <geom name="{name}_post2" type="cylinder" fromto="0 -{y} 0  0 -{y} {h}" size="{r}" material="post_mat" contype="1" conaffinity="1"/>',
        f'  <geom name="{name}_bar" type="cylinder" fromto="0 -{y} {h}  0 {y} {h}" size="{r}" material="post_mat" contype="1" conaffinity="1"/>',
        "</body>",
    ]


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in FIELDS:
        sys.exit(f"usage: {sys.argv[0]} {{{','.join(FIELDS)}}}")
    f = FIELDS[sys.argv[1]]

    half_x, half_y = f.length / 2 + f.border, f.width / 2 + f.border
    print(
        f"    <!-- Pitch: half-size {half_x:g} x {half_y:g} m ({2 * half_x:g} x {2 * half_y:g} m total = "
        f"{f.length:g}x{f.width:g} field + {f.border:g}m border). -->"
    )
    print(
        f'    <geom name="floor" type="plane" pos="0 0 0" size="{half_x:.1f} {half_y:.1f} 0.1" material="grass_mat" '
        'condim="3" friction="0.8 0.02 0.001"/>'
    )
    print(
        "    <!-- Field line markings (touchlines, goal lines, center line/circle/mark, goal areas, penalty areas, "
        f"penalty marks{', corner arcs' if f.corner_arc_radius > 0 else ''}). -->"
    )
    for line in markings():
        print("      " + line)
    print()
    print(
        f"    <!-- Goals: {f.name}, {2 * f.goalpost_centre_y:g} m wide (post-center to post-center), "
        f"{f.goal_height:g} m high, post radius {f.goalpost_radius:g} m, at x=+-{f.length / 2:g}. -->"
    )
    for sign in (1, -1):
        for line in goal(sign):
            print("    " + line)
