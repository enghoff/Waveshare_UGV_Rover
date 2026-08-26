import importlib.util
import json
import re
import subprocess
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("ugv_deploy", HERE / "deploy.py")
deploy = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(deploy)


class ExecutableBitsTest(unittest.TestCase):
    """Scripts that are run as programs have to be executable *in git*.

    `component_files` carries git's mode to the rover, faithfully -- so a
    `restart.sh` committed 644 lands 644, and the deploy that copies it kills the
    service and then cannot start it again. That is not hypothetical: a full
    deploy of `rover_daemon` and `drive_web` did exactly that, took the daemon
    and the console down together, and left them down, because their supervisors
    are `@reboot` crontab lines with nothing to respawn them.

    Two rules, matching the two ways a script gets run as a program here: the
    ones a manifest command invokes, and the ones a `restart.sh` starts when it
    finds no supervisor. Installers are exempt -- they are run as `sh install.sh`
    and never on their own.
    """

    @staticmethod
    def tracked_modes():
        out = subprocess.run(["git", "ls-files", "-s"], cwd=HERE.parent,
                             capture_output=True, text=True, check=True)
        modes = {}
        for line in out.stdout.splitlines():
            bits, rest = line.split(" ", 1)
            modes[rest.split("	", 1)[1]] = int(bits, 8) & 0o777
        return modes

    def test_manifest_commands_name_executable_scripts(self):
        modes = self.tracked_modes()
        manifest = json.loads((HERE / "manifest.json").read_text(encoding="utf-8"))
        for component in manifest["components"]:
            wanted = []
            for command in component.get("commands", []):
                wanted.append(command)
            for special in component.get("special_commands", []):
                wanted.extend(special.get("commands", []))
            for command in wanted:
                for word in re.findall(r"[~$][^\s;|&]*\.sh", command):
                    name = word.rsplit("/", 1)[-1]
                    matches = [path for path in modes
                               if path.endswith("/" + name)
                               and path.split("/")[0] in
                               (component["name"], *[
                                   pattern.split("/")[0]
                                   for source in component.get("sources", [])
                                   for pattern in source["patterns"]])]
                    for path in matches:
                        self.assertEqual(
                            modes[path], 0o755,
                            f"{path} is run by {component['name']} but is not "
                            f"executable in git")

    def test_restart_scripts_can_start_what_they_supervise(self):
        modes = self.tracked_modes()
        for path, mode in sorted(modes.items()):
            if not path.endswith("restart.sh"):
                continue
            self.assertEqual(mode, 0o755,
                             f"{path} is invoked directly and is not executable")
            body = (HERE.parent / path).read_text(encoding="utf-8")
            folder = path.rsplit("/", 1)[0] if "/" in path else ""
            started_names = []
            for line in body.splitlines():
                stripped = line.strip()
                # `. "$DIR/dds.sh"` and `source ...` read a file into the current
                # shell; only the shell needs to be able to read it. It is being
                # *run* that needs the bit.
                if stripped.startswith(". ") or stripped.startswith("source "):
                    continue
                started_names.extend(re.findall(r'"\$DIR/([^"]+\.sh)"', line))
            for name in started_names:
                started = f"{folder}/{name}" if folder else name
                if started not in modes:
                    # Generated on the host rather than carried there --
                    # `ros_nav/env.sh` is written by `install.sh` and holds the
                    # conda prefix that only exists on the rover. Nothing to
                    # check: git never sets its mode.
                    continue
                self.assertEqual(
                    modes[started], 0o755,
                    f"{started} is started by {path} and is not executable in git")


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
