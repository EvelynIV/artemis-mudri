import json
import os
import subprocess
import sys
import sysconfig
import tempfile
import unittest
from pathlib import Path


class CommandLineInterfaceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.repo_root = Path(__file__).resolve().parents[2]
        self.pythonpath = os.pathsep.join(
            filter(
                None,
                [
                    str(self.repo_root / "src"),
                    os.environ.get("PYTHONPATH"),
                ],
            )
        )

    def _run(self, *args: str) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["PYTHONPATH"] = self.pythonpath
        return subprocess.run(
            args,
            cwd=self.repo_root,
            env=env,
            capture_output=True,
            text=True,
            check=True,
        )

    def test_python_m_help_displays_typer_output(self) -> None:
        completed = self._run(sys.executable, "-m", "artemis_mudri", "--help")
        self.assertIn("Usage:", completed.stdout)
        self.assertIn("--task", completed.stdout)
        self.assertIn("--summary-json", completed.stdout)
        self.assertIn("--chassis", completed.stdout)
        self.assertIn("--drift-mode", completed.stdout)

    def test_python_m_and_console_script_match_task1_output(self) -> None:
        module_run = self._run(sys.executable, "-m", "artemis_mudri", "--task", "1", "--no-render")
        scripts_dir = Path(sysconfig.get_path("scripts"))
        console_script = scripts_dir / "artemis-demo"
        if not console_script.exists():
            console_script = Path(sys.executable).resolve().with_name("artemis-demo")
        script_run = self._run(str(console_script), "--task", "1", "--no-render")
        self.assertEqual(module_run.stdout, script_run.stdout)

    def test_python_m_writes_summary_json(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "summary.json"
            completed = self._run(
                sys.executable,
                "-m",
                "artemis_mudri",
                "--task",
                "3",
                "--no-render",
                "--summary-json",
                str(output_path),
            )
            self.assertTrue(output_path.exists())
            payload = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["task_id"], "3")
            self.assertTrue(payload["reached_goal"])
            self.assertIn("Summary written to:", completed.stdout)

    def test_python_m_accepts_lqr_controller(self) -> None:
        completed = self._run(
            sys.executable,
            "-m",
            "artemis_mudri",
            "--task",
            "1",
            "--no-render",
            "--line-follow-controller",
            "lqr",
        )
        self.assertIn("Reached goal:", completed.stdout)

    def test_python_m_accepts_ackermann_corner_drift(self) -> None:
        completed = self._run(
            sys.executable,
            "-m",
            "artemis_mudri",
            "--task",
            "3",
            "--no-render",
            "--chassis",
            "ackermann",
            "--drift-mode",
            "corner",
        )
        self.assertIn("Reached goal: True", completed.stdout)

    def test_python_m_reads_chassis_and_drift_env_vars(self) -> None:
        env = os.environ.copy()
        env["PYTHONPATH"] = self.pythonpath
        env["ARTEMIS_CHASSIS"] = "ackermann"
        env["ARTEMIS_DRIFT_MODE"] = "corner"
        completed = subprocess.run(
            [sys.executable, "-m", "artemis_mudri", "--task", "3", "--no-render"],
            cwd=self.repo_root,
            env=env,
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertIn("Reached goal: True", completed.stdout)


if __name__ == "__main__":
    unittest.main()
