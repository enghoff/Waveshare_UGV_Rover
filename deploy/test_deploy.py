import importlib.util
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("ugv_deploy", HERE / "deploy.py")
deploy = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(deploy)


class DeployHelpersTest(unittest.TestCase):
    def test_flattened_sources_keep_git_mode(self):
        component = {
            "name": "rover",
            "sources": [
                {"patterns": ["one/*.py", "one/*.sh"], "strip": "one"},
                {"patterns": ["two/*.py"], "strip": "two"},
            ],
        }
        tracked = {"one/a.py": 0o644, "one/run.sh": 0o755, "two/b.py": 0o644}
        self.assertEqual(
            deploy.component_files(component, tracked),
            [
                ("one/a.py", "a.py", 0o644),
                ("two/b.py", "b.py", 0o644),
                ("one/run.sh", "run.sh", 0o755),
            ],
        )

    def test_flatten_collision_is_refused(self):
        component = {
            "name": "rover",
            "sources": [
                {"patterns": ["one/*.py"], "strip": "one"},
                {"patterns": ["two/*.py"], "strip": "two"},
            ],
        }
        with self.assertRaises(deploy.DeployError):
            deploy.component_files(component, {"one/same.py": 0o644, "two/same.py": 0o644})

    def test_deleted_path_maps_to_remote_relative_path(self):
        component = {
            "sources": [{"patterns": ["oak_depth/**"], "strip": "oak_depth"}],
        }
        self.assertEqual(
            deploy.archive_path_for(component, "oak_depth/old/helper.py"),
            "old/helper.py",
        )

    def test_tilde_state_path_expands_on_remote_shell(self):
        self.assertEqual(
            deploy.state_cat_command("~/.ugv/deploy-state.json"),
            'cat "$HOME/.ugv/deploy-state.json" 2>/dev/null || true',
        )


if __name__ == "__main__":
    unittest.main()
