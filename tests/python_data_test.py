import importlib.util
import pathlib
import sys
import tempfile

import pyarrow.parquet as pq


def load_module(name: str, module_path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {name} from {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    if len(sys.argv) != 3:
        raise RuntimeError("usage: python_data_test.py <rex_py-module-path> <rex_data-module-path>")

    rex_py = load_module("rex_py", pathlib.Path(sys.argv[1]))
    rex_data = load_module("rex_data", pathlib.Path(sys.argv[2]))

    replay = rex_py.build_demo_replay(6)

    with tempfile.TemporaryDirectory(prefix="rex-data-test-") as temp_dir:
        written = rex_data.write_replay_dataset(replay, temp_dir)
        frames_table = pq.read_table(written["frames"])
        bodies_table = pq.read_table(written["bodies"])
        contacts_table = pq.read_table(written["contacts"])

        expected_body_rows = sum(len(frame.bodies) for frame in replay.frames())
        expected_contact_rows = sum(len(frame.contacts) for frame in replay.frames())

        assert frames_table.num_rows == replay.size()
        assert bodies_table.num_rows == expected_body_rows
        assert contacts_table.num_rows == expected_contact_rows
        assert "rotation_w" in bodies_table.schema.names
        assert "rotation_x" in bodies_table.schema.names
        assert "position_x" in contacts_table.schema.names
        assert "contact_count" in frames_table.schema.names

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
