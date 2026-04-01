from __future__ import annotations

from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq


def replay_to_tables(replay) -> dict[str, pa.Table]:
    frame_rows: list[dict[str, object]] = []
    body_rows: list[dict[str, object]] = []
    contact_rows: list[dict[str, object]] = []

    for frame in replay.frames():
        frame_rows.append(
            {
                "frame_index": int(frame.frame_index),
                "sim_time": float(frame.sim_time),
                "body_count": int(frame.trace.body_count),
                "articulation_count": int(frame.trace.articulation_count),
                "broadphase_pair_count": int(frame.trace.broadphase_pair_count),
                "manifold_count": int(frame.trace.manifold_count),
                "contact_count": int(frame.trace.solver.contact_count),
                "constraint_count": int(frame.trace.solver.constraint_count),
                "max_penetration": float(frame.trace.solver.max_penetration),
            }
        )

        for body in frame.bodies:
            shape_name = getattr(body.shape, "name", str(body.shape)).lower()
            body_rows.append(
                {
                    "frame_index": int(frame.frame_index),
                    "body_index": int(body.id.index),
                    "body_generation": int(body.id.generation),
                    "shape": "box" if shape_name.endswith("box") else "sphere",
                    "translation_x": float(body.translation.x),
                    "translation_y": float(body.translation.y),
                    "translation_z": float(body.translation.z),
                    "dimension_x": float(body.dimensions.x),
                    "dimension_y": float(body.dimensions.y),
                    "dimension_z": float(body.dimensions.z),
                    "rotation_w": float(body.rotation.w),
                    "rotation_x": float(body.rotation.x),
                    "rotation_y": float(body.rotation.y),
                    "rotation_z": float(body.rotation.z),
                }
            )

        for contact in frame.contacts:
            contact_rows.append(
                {
                    "frame_index": int(frame.frame_index),
                    "body_a_index": int(contact.body_a.index),
                    "body_a_generation": int(contact.body_a.generation),
                    "body_b_index": int(contact.body_b.index),
                    "body_b_generation": int(contact.body_b.generation),
                    "position_x": float(contact.position.x),
                    "position_y": float(contact.position.y),
                    "position_z": float(contact.position.z),
                    "normal_x": float(contact.normal.x),
                    "normal_y": float(contact.normal.y),
                    "normal_z": float(contact.normal.z),
                    "penetration": float(contact.penetration),
                }
            )

    return {
        "frames": pa.Table.from_pylist(frame_rows),
        "bodies": pa.Table.from_pylist(body_rows),
        "contacts": pa.Table.from_pylist(contact_rows),
    }


def write_replay_dataset(replay, output_dir: str | Path, *, compression: str = "zstd") -> dict[str, Path]:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    written_paths: dict[str, Path] = {}
    for name, table in replay_to_tables(replay).items():
        parquet_path = output_path / f"{name}.parquet"
        pq.write_table(table, parquet_path, compression=compression)
        written_paths[name] = parquet_path

    return written_paths
