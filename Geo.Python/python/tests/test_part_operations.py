import unittest

from camber import Part, PartOperation, set_progress_log
from camber.view import _load_operations, _operation_row_text


class PartOperationTests(unittest.TestCase):
    def test_feature_ledger_records_successful_model_operations(self):
        set_progress_log(False)
        part = Part((-10, -10, -10), (10, 10, 10), tolerance=.01)
        base = part.cuboid((0, 0, 0), (4, 4, 4), name="base")
        tool = part.cylinder(origin=(2, 2, -1), radius=.5, height=6, name="tool")
        drilled = part.cut(base, tool, name="drilled")
        edge = next(name for name in drilled.curve_names if "ExtrudeTop" in name)
        finished = part.fillet(drilled, edge, .2, name="finished")

        operations = part.operations
        self.assertEqual(["Cuboid", "Cylinder", "Cut", "Fillet"], [item.kind for item in operations])
        self.assertTrue(all(isinstance(item, PartOperation) for item in operations))
        self.assertEqual("finished", operations[-1].result)
        self.assertEqual(("drilled",), operations[-1].inputs)
        self.assertEqual((edge.split(":", 1)[1],), operations[-1].entities)
        self.assertIn("radius=0.2", operations[-1].details)
        self.assertEqual(operations, tuple(_load_operations(part)))
        self.assertIn("Fillet: finished", _operation_row_text(operations[-1]))
        self.assertEqual(finished.name, operations[-1].result)

    def test_failed_operation_is_not_recorded(self):
        set_progress_log(False)
        part = Part((-2, -2, -2), (2, 2, 2), tolerance=.01)
        body = part.cuboid((0, 0, 0), (1, 1, 1), name="body")
        before = part.operations
        with self.assertRaises(Exception):
            part.fillet(body, "not-a-curve", .2)
        self.assertEqual(before, part.operations)


if __name__ == "__main__":
    unittest.main()
