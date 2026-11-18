#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import yaml


FIELDS = (
    "result",
    "success",
    "ee_offset_m",
    "wall_target_z_m",
    "axis_x",
    "axis_y",
    "axis_z",
    "ee_torque_Nm",
    "max_constraint_violation",
    "max_abs_joint_torque_Nm",
    "max_joint",
)


def summarize(path, result_root):
    with path.open() as stream:
        data = yaml.safe_load(stream)

    axis = data["ee_target_axis_world"]
    joint_names = data.get("joint_names", [])
    joint_torques = data.get("joint_torques", [])
    if joint_torques:
        index = max(range(len(joint_torques)), key=lambda i: abs(joint_torques[i]))
        max_joint = joint_names[index]
        max_joint_torque = abs(joint_torques[index])
    else:
        max_joint = ""
        max_joint_torque = float("nan")

    return {
        "result": str(path.relative_to(result_root)),
        "success": bool(data["success"]),
        "ee_offset_m": data["ee_offset_along_wall_normal"],
        "wall_target_z_m": data["ee_wall_target_point_world"][2],
        "axis_x": axis[0],
        "axis_y": axis[1],
        "axis_z": axis[2],
        "ee_torque_Nm": data["ee_wrench_arm_ee"][5],
        "max_constraint_violation": data["maximum_constraint_violation"],
        "max_abs_joint_torque_Nm": max_joint_torque,
        "max_joint": max_joint,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Summarize existing static-wrench result YAMLs without rerunning optimization."
    )
    parser.add_argument("result_dir", type=Path)
    args = parser.parse_args()

    paths = sorted(args.result_dir.glob("offset_*/axis_*.yaml"))
    rows = [summarize(path, args.result_dir) for path in paths]
    if not rows:
        parser.error("no offset_*/axis_*.yaml files found in {}".format(args.result_dir))

    csv_path = args.result_dir / "summary.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    markdown_path = args.result_dir / "summary.md"
    with markdown_path.open("w") as stream:
        stream.write("# Static wrench result summary\n\n")
        heights = sorted({row["wall_target_z_m"] for row in rows})
        height_text = ", ".join("{:.3f}".format(value) for value in heights)
        stream.write("Wall target height [m]: {}\n\n".format(height_text))
        stream.write("| Result | Status | Offset [m] | Unit axis | EE torque [Nm] | Max violation | Max abs. joint torque [Nm] | Joint |\n")
        stream.write("|---|---|---:|---:|---:|---:|---:|---|\n")
        for row in rows:
            axis = "[{:.6g}, {:.6g}, {:.6g}]".format(row["axis_x"], row["axis_y"], row["axis_z"])
            stream.write(
                "| `{}` | {} | {:.3f} | `{}` | {:.6f} | {:.3e} | {:.6f} | `{}` |\n".format(
                    row["result"],
                    "success" if row["success"] else "failed",
                    row["ee_offset_m"],
                    axis,
                    row["ee_torque_Nm"],
                    row["max_constraint_violation"],
                    row["max_abs_joint_torque_Nm"],
                    row["max_joint"],
                )
            )

    print("wrote {} and {}".format(csv_path, markdown_path))


if __name__ == "__main__":
    main()
