from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq


@dataclass
class MotorFixedSpec:
    # 固定寸法（M206系の取付条件）
    flange_size: float = 60.0
    body_diameter: float = 60.0
    mount_hole_pitch: float = 49.5
    mount_hole_diameter: float = 4.5
    mount_hole_clearance: float = 0.5
    pilot_diameter: float = 54.0
    pilot_clearance: float = 2.0


@dataclass
class BracketParams:
    # Lブラケットの設計パラメータ（単位: mm）
    thickness: float = 5.0
    hole_edge_margin: float = 12.0  # モーター取付穴から板端までの最小余裕
    body_clearance: float = 8.0
    top_margin: float = 10.0
    base_length_extra: float = 20.0
    base_hole_diameter: float = 6.6
    base_hole_edge_offset: float = 18.0


def build_l_bracket(motor: MotorFixedSpec, params: BracketParams) -> cq.Workplane:
    # 固定寸法を使ってLブラケットのソリッドを生成
    pitch_w = motor.mount_hole_pitch
    pitch_h = motor.mount_hole_pitch

    # ベース板とモーター外径が干渉しない高さに取付中心を設定
    mount_center_z = params.thickness + motor.body_diameter / 2.0 + params.body_clearance
    wall_height = mount_center_z + max(
        motor.flange_size / 2.0 + params.top_margin,
        pitch_h / 2.0 + params.hole_edge_margin,
    )
    wall_width = max(
        motor.flange_size + 2.0 * params.body_clearance,
        pitch_w + 2.0 * params.hole_edge_margin,
    )
    base_length = max(
        wall_width + params.base_length_extra,
        2.0 * params.base_hole_edge_offset + 24.0,
    )

    t = params.thickness

    base = cq.Workplane("XY").box(
        base_length,
        wall_width,
        t,
        centered=(False, True, False),
    )
    wall = cq.Workplane("XY").box(
        t,
        wall_width,
        wall_height,
        centered=(False, True, False),
    )

    # 局所(u,v)=(Y,Z)として扱える取付面ワークプレーンを定義
    mount_plane = cq.Plane(origin=(t, 0.0, mount_center_z), xDir=(0, 1, 0), normal=(1, 0, 0))

    mount_hole_d = motor.mount_hole_diameter + motor.mount_hole_clearance
    center_hole_d = motor.pilot_diameter + motor.pilot_clearance
    half_pitch = motor.mount_hole_pitch / 2.0
    hole_points = [
        (-half_pitch, -half_pitch),
        (-half_pitch, +half_pitch),
        (+half_pitch, -half_pitch),
        (+half_pitch, +half_pitch),
    ]
    hole_cutter = (
        cq.Workplane(mount_plane)
        .pushPoints(hole_points)
        .circle(mount_hole_d / 2.0)
        .extrude(t + 2.0, both=True)
    )
    shaft_cutter = (
        cq.Workplane(mount_plane)
        .circle(center_hole_d / 2.0)
        .extrude(t + 2.0, both=True)
    )
    wall = wall.cut(hole_cutter).cut(shaft_cutter)

    x1 = params.base_hole_edge_offset
    x2 = base_length - params.base_hole_edge_offset
    y_off = max(wall_width * 0.28, params.base_hole_diameter + 4.0)
    y_off = min(y_off, wall_width / 2.0 - params.hole_edge_margin + 2.0)

    base_hole_points = [
        (x1 - base_length / 2.0, -y_off),
        (x1 - base_length / 2.0, +y_off),
        (x2 - base_length / 2.0, -y_off),
        (x2 - base_length / 2.0, +y_off),
    ]
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints(base_hole_points)
        .hole(params.base_hole_diameter)
    )

    return base.union(wall)


def main() -> None:
    # CLI引数を受け取り、固定寸法ブラケットをSTEP出力する
    script_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(
        description="Generate an L-shaped motor bracket (fixed dimensions, no STL required)."
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=script_dir / "M206-03-LBracket.step",
        help="Output STEP path",
    )
    parser.add_argument("--thickness", type=float, default=5.0, help="Bracket thickness [mm]")
    args = parser.parse_args()

    motor_spec = MotorFixedSpec()
    params = BracketParams(thickness=args.thickness)
    bracket = build_l_bracket(motor_spec, params)

    out_path = args.out if args.out.is_absolute() else (script_dir / args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(bracket, str(out_path))

    print(f"[OK] STEP exported: {out_path}")
    print(f"  Mount hole pitch: {motor_spec.mount_hole_pitch:.3f} mm")
    print(f"  Mount hole dia (with clearance): {motor_spec.mount_hole_diameter + motor_spec.mount_hole_clearance:.3f} mm")
    print(f"  Center clearance dia: {motor_spec.pilot_diameter + motor_spec.pilot_clearance:.3f} mm")


if __name__ == "__main__":
    main()
