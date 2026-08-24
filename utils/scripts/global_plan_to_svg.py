#!/usr/bin/env python

import sys
import math
import rosbag


TOPIC = "/move_base_flex/FTCPlanner/global_plan"

COLORS = [
    "#e41a1c",  # red
    "#377eb8",  # blue
    "#4daf4a",  # green
    "#984ea3",  # purple
    "#ff7f00",  # orange
    "#a65628",  # brown
    "#f781bf",  # pink
    "#999999",  # gray
    "#00a6d6",  # cyan
    "#ffd92f",  # yellow
]


def svg_escape(text):
    """Escape text for use inside SVG/XML."""
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def make_svg(bag_path, output_path):
    paths = []

    print("Reading:", bag_path)
    print("Topic:", TOPIC)

    with rosbag.Bag(bag_path, "r") as bag:
        for msg_index, (_, msg, timestamp) in enumerate(
            bag.read_messages(topics=[TOPIC])
        ):
            points = []

            for pose_index, pose_stamped in enumerate(msg.poses):
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y

                points.append((x, y))

            if len(points) < 2:
                print(
                    "Warning: message {} contains only {} poses, skipping".format(
                        msg_index,
                        len(points),
                    )
                )
                continue

            paths.append(
                {
                    "index": msg_index,
                    "timestamp": timestamp.to_sec(),
                    "points": points,
                }
            )

            print(
                "  message {:3d}: {:5d} poses".format(
                    msg_index,
                    len(points),
                )
            )

    if not paths:
        raise RuntimeError(
            "No usable messages found on {}".format(TOPIC)
        )

    # ------------------------------------------------------------------
    # Determine world-coordinate bounds.
    # ------------------------------------------------------------------

    all_points = [
        point
        for path in paths
        for point in path["points"]
    ]

    min_x = min(p[0] for p in all_points)
    max_x = max(p[0] for p in all_points)
    min_y = min(p[1] for p in all_points)
    max_y = max(p[1] for p in all_points)

    # Avoid zero-size dimensions.
    if max_x == min_x:
        max_x += 1.0
        min_x -= 1.0

    if max_y == min_y:
        max_y += 1.0
        min_y -= 1.0

    # ------------------------------------------------------------------
    # SVG dimensions.
    # ------------------------------------------------------------------

    width = 1400
    height = 1000
    margin = 80

    world_width = max_x - min_x
    world_height = max_y - min_y

    # Equal scale in X and Y.
    scale = min(
        float(width - 2 * margin) / world_width,
        float(height - 2 * margin) / world_height,
    )

    actual_width = world_width * scale
    actual_height = world_height * scale

    offset_x = (width - actual_width) / 2.0
    offset_y = (height - actual_height) / 2.0

    def transform(x, y):
        """
        Convert world coordinates to SVG coordinates.

        SVG Y increases downward, so Y is inverted.
        """
        sx = offset_x + (x - min_x) * scale
        sy = height - offset_y - (y - min_y) * scale

        return sx, sy

    # ------------------------------------------------------------------
    # Start SVG.
    # ------------------------------------------------------------------

    svg = []

    svg.append('<?xml version="1.0" encoding="UTF-8"?>')

    svg.append(
        '<svg xmlns="http://www.w3.org/2000/svg" '
        'width="{}" height="{}" '
        'viewBox="0 0 {} {}">'.format(
            width,
            height,
            width,
            height,
        )
    )

    # ------------------------------------------------------------------
    # CSS.
    # ------------------------------------------------------------------

    svg.append("""
<style>
    .path {
        fill: none;
        stroke-width: 2.5;
        stroke-linejoin: round;
        stroke-linecap: round;
    }

    .start {
        stroke: #000000;
        stroke-width: 1.5;
    }

    .arrow {
        stroke-width: 1.5;
        fill: none;
    }

    .label {
        font-family: sans-serif;
        font-size: 14px;
    }

    .title {
        font-family: sans-serif;
        font-size: 18px;
        font-weight: bold;
    }

    .axis {
        stroke: #cccccc;
        stroke-width: 1;
    }
</style>
""")

    # ------------------------------------------------------------------
    # Background.
    # ------------------------------------------------------------------

    svg.append(
        '<rect x="0" y="0" width="{}" height="{}" '
        'fill="white"/>'.format(
            width,
            height,
        )
    )

    # ------------------------------------------------------------------
    # Coordinate axes.
    # ------------------------------------------------------------------

    # X = 0
    if min_x <= 0 <= max_x:
        x0, _ = transform(0, 0)

        svg.append(
            '<line class="axis" '
            'x1="{:.2f}" y1="{}" '
            'x2="{:.2f}" y2="{}"/>'.format(
                x0,
                margin,
                x0,
                height - margin,
            )
        )

    # Y = 0
    if min_y <= 0 <= max_y:
        _, y0 = transform(0, 0)

        svg.append(
            '<line class="axis" '
            'x1="{}" y1="{:.2f}" '
            'x2="{}" y2="{:.2f}"/>'.format(
                margin,
                y0,
                width - margin,
                y0,
            )
        )

    # ------------------------------------------------------------------
    # Title.
    # ------------------------------------------------------------------

    svg.append(
        '<text class="title" x="20" y="25">'
        'Global plan: {}'
        '</text>'.format(
            svg_escape(TOPIC)
        )
    )

    # ------------------------------------------------------------------
    # Draw each nav_msgs/Path.
    # ------------------------------------------------------------------

    for path_number, path in enumerate(paths):

        color = COLORS[path_number % len(COLORS)]
        points = path["points"]

        svg_points = [
            transform(x, y)
            for x, y in points
        ]

        # --------------------------------------------------------------
        # Construct an SVG PATH.
        #
        # This is the important difference from the previous version.
        #
        # Every ROS pose becomes an SVG L command, which means Inkscape
        # exposes every pose as a node with the Node Tool.
        # --------------------------------------------------------------

        path_commands = []

        for pose_index, (x, y) in enumerate(svg_points):

            if pose_index == 0:
                path_commands.append(
                    "M {:.3f},{:.3f}".format(x, y)
                )
            else:
                path_commands.append(
                    "L {:.3f},{:.3f}".format(x, y)
                )

        d = " ".join(path_commands)

        # --------------------------------------------------------------
        # Actual trajectory.
        # --------------------------------------------------------------

        svg.append(
            '<path '
            'id="path_message_{}" '
            'class="path" '
            'stroke="{}" '
            'd="{}">'.format(
                path["index"],
                color,
                d,
            )
        )

        # Tooltip when hovering over the path.
        svg.append(
            '<title>'
            'ROS message {}, timestamp {:.6f}, {} poses'
            '</title>'.format(
                path["index"],
                path["timestamp"],
                len(points),
            )
        )

        svg.append("</path>")

        # --------------------------------------------------------------
        # Start marker.
        # --------------------------------------------------------------

        sx, sy = svg_points[0]

        svg.append(
            '<circle '
            'class="start" '
            'cx="{:.2f}" '
            'cy="{:.2f}" '
            'r="5" '
            'fill="{}">'.format(
                sx,
                sy,
                color,
            )
        )

        svg.append(
            '<title>'
            'Message {}, pose 0 (start)'
            '</title>'.format(
                path["index"]
            )
        )

        svg.append("</circle>")

        # --------------------------------------------------------------
        # Direction arrows.
        #
        # Arrows are placed approximately every 10% of the path.
        # They indicate the direction from pose[i] -> pose[i+1].
        # --------------------------------------------------------------

        arrow_spacing = max(
            1,
            len(points) // 10,
        )

        for i in range(
            arrow_spacing,
            len(points) - 1,
            arrow_spacing,
        ):

            x1, y1 = svg_points[i]
            x2, y2 = svg_points[i + 1]

            dx = x2 - x1
            dy = y2 - y1

            length = math.hypot(dx, dy)

            if length < 1e-6:
                continue

            ux = dx / length
            uy = dy / length

            # Arrow shaft length.
            arrow_length = 10.0

            ax = x1 + ux * arrow_length
            ay = y1 + uy * arrow_length

            # Arrow head.
            head_length = 6.0
            head_angle = math.radians(30)

            left_x = ax - head_length * (
                ux * math.cos(head_angle)
                - uy * math.sin(head_angle)
            )

            left_y = ay - head_length * (
                uy * math.cos(head_angle)
                + ux * math.sin(head_angle)
            )

            right_x = ax - head_length * (
                ux * math.cos(head_angle)
                + uy * math.sin(head_angle)
            )

            right_y = ay - head_length * (
                uy * math.cos(head_angle)
                - ux * math.sin(head_angle)
            )

            # Arrow shaft.
            svg.append(
                '<line '
                'class="arrow" '
                'x1="{:.2f}" y1="{:.2f}" '
                'x2="{:.2f}" y2="{:.2f}" '
                'stroke="{}"/>'.format(
                    x1,
                    y1,
                    ax,
                    ay,
                    color,
                )
            )

            # Arrow head.
            svg.append(
                '<polyline '
                'class="arrow" '
                'points="{:.2f},{:.2f} '
                '{:.2f},{:.2f} '
                '{:.2f},{:.2f}" '
                'stroke="{}"/>'.format(
                    left_x,
                    left_y,
                    ax,
                    ay,
                    right_x,
                    right_y,
                    color,
                )
            )

        # --------------------------------------------------------------
        # Optional pose-number labels.
        #
        # Only label a subset so the SVG doesn't become unreadable.
        # The actual nodes are still all present.
        # --------------------------------------------------------------

        label_spacing = max(
            1,
            len(points) // 20,
        )

        for pose_index in range(
            0,
            len(points),
            label_spacing,
        ):

            x, y = svg_points[pose_index]

            svg.append(
                '<text '
                'class="label" '
                'x="{:.2f}" '
                'y="{:.2f}" '
                'fill="{}">'.format(
                    x + 5,
                    y - 5,
                    color,
                )
            )

            svg.append(
                '<title>'
                'Message {}, pose {}'
                '</title>'.format(
                    path["index"],
                    pose_index,
                )
            )

            svg.append(
                str(pose_index)
            )

            svg.append("</text>")

    # ------------------------------------------------------------------
    # Legend.
    # ------------------------------------------------------------------

    legend_x = 20
    legend_y = 50

    for path_number, path in enumerate(paths):

        color = COLORS[path_number % len(COLORS)]
        y = legend_y + path_number * 22

        svg.append(
            '<line '
            'x1="{}" y1="{}" '
            'x2="{}" y2="{}" '
            'stroke="{}" '
            'stroke-width="4"/>'.format(
                legend_x,
                y - 5,
                legend_x + 25,
                y - 5,
                color,
            )
        )

        svg.append(
            '<text '
            'class="label" '
            'x="{}" '
            'y="{}">'.format(
                legend_x + 32,
                y,
            )
        )

        svg.append(
            'message {} — {} poses — t={:.3f}s'.format(
                path["index"],
                len(path["points"]),
                path["timestamp"],
            )
        )

        svg.append("</text>")

    # ------------------------------------------------------------------
    # Finish SVG.
    # ------------------------------------------------------------------

    svg.append("</svg>")

    with open(output_path, "w") as f:
        f.write("\n".join(svg))

    print()
    print("Wrote:", output_path)
    print("Messages:", len(paths))
    print("World bounds:")
    print("  X: {:.3f} .. {:.3f}".format(min_x, max_x))
    print("  Y: {:.3f} .. {:.3f}".format(min_y, max_y))


def main():

    if len(sys.argv) != 2:
        print(
            "Usage: {} <bagfile.bag>".format(
                sys.argv[0]
            )
        )
        sys.exit(1)

    bag_path = sys.argv[1]

    if "." in bag_path:
        output_path = bag_path.rsplit(".", 1)[0] + "_global_plan.svg"
    else:
        output_path = bag_path + "_global_plan.svg"

    try:
        make_svg(
            bag_path,
            output_path,
        )

    except Exception as e:
        print(
            "ERROR: {}".format(e),
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
