import importlib.util
import struct
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "map_results.py"
SPEC = importlib.util.spec_from_file_location("map_results", MODULE_PATH)
map_results = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(map_results)


def write_pcd(path, points, fields=("x", "y", "z", "intensity")):
    header = (
        f"VERSION .7\nFIELDS {' '.join(fields)}\n"
        f"SIZE {' '.join(['4'] * len(fields))}\n"
        f"TYPE {' '.join(['F'] * len(fields))}\n"
        f"COUNT {' '.join(['1'] * len(fields))}\n"
        f"WIDTH {len(points)}\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    ).encode("ascii")
    payload = b"".join(struct.pack("<" + "f" * len(point), *point) for point in points)
    path.write_bytes(header + payload)


class MapResultsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        map_results.MAPS = Path(self.temporary.name) / "maps"
        self.session = map_results.MAPS / "office_test"
        self.session.mkdir(parents=True)

    def tearDown(self):
        self.temporary.cleanup()

    def test_merge_orders_chunks_then_final_tail(self):
        write_pcd(self.session / "scans_2.pcd", [(2, 0, 0, 2)])
        write_pcd(self.session / "scans_1.pcd", [(1, 0, 0, 1)])
        write_pcd(self.session / "scans.pcd", [(3, 0, 0, 3)])

        merged = map_results.merge_session("office_test")
        info = map_results.read_pcd_header(merged)
        with merged.open("rb") as stream:
            stream.seek(info["payload_offset"])
            payload = stream.read()

        self.assertEqual(info["points"], 3)
        self.assertEqual(
            payload,
            struct.pack("<12f", 1, 0, 0, 1, 2, 0, 0, 2, 3, 0, 0, 3),
        )

    def test_merge_rejects_different_fields(self):
        write_pcd(self.session / "scans_1.pcd", [(1, 0, 0, 1)])
        write_pcd(self.session / "scans_2.pcd", [(2, 0, 0, 2)], fields=("x", "y", "z", "ring"))

        with self.assertRaisesRegex(ValueError, "字段不匹配"):
            map_results.merge_session("office_test")


if __name__ == "__main__":
    unittest.main()
