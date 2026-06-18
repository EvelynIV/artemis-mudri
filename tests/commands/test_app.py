import os
import subprocess
import sys
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
                    str(self.repo_root),
                    os.environ.get("PYTHONPATH"),
                ],
            )
        )

    def _run(self, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["PYTHONPATH"] = self.pythonpath
        return subprocess.run(
            args,
            cwd=self.repo_root,
            env=env,
            capture_output=True,
            text=True,
            check=check,
        )

    def test_python_m_help_displays_service_command(self) -> None:
        completed = self._run(sys.executable, "-m", "artemis_mudri", "--help")
        self.assertIn("Usage:", completed.stdout)
        self.assertIn("serve", completed.stdout)

    def test_serve_help_displays_service_options(self) -> None:
        completed = self._run(sys.executable, "-m", "artemis_mudri.commands.app", "serve", "--help")
        self.assertIn("--bind", completed.stdout)
        self.assertIn("--render", completed.stdout)
        self.assertIn("--noise-config", completed.stdout)
        self.assertIn("--viewer-state-bind", completed.stdout)

    def test_missing_noise_config_returns_error(self) -> None:
        completed = self._run(
            sys.executable,
            "-m",
            "artemis_mudri.commands.app",
            "serve",
            "--noise-config",
            "examples/configs/noise/missing.yaml",
            check=False,
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Noise config file not found", completed.stderr)


if __name__ == "__main__":
    unittest.main()
