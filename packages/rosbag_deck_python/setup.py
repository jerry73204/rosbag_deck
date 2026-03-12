import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py

# Search order for cargo build profiles
PROFILES = ["dev-release", "release", "debug"]


class BuildPyWithCdylib(build_py):
    """Build the Rust cdylib via cargo and copy it into the Python package.

    The Rust crate rosbag_deck_python depends on rosbag_deck_core and compiles
    to a single cdylib (librosbag_deck.so) for Python. This command invokes
    cargo build if needed, then copies the cdylib into the setuptools build
    directory so that `setup.py install` places it alongside __init__.py.
    """

    def run(self):
        super().run()
        self._build_and_copy_cdylib()

    def _build_and_copy_cdylib(self):
        workspace_root = self._find_workspace_root()
        so_path = self._find_cdylib(workspace_root)

        if so_path is None:
            so_path = self._cargo_build(workspace_root)

        ext_suffix = sysconfig.get_config_var("EXT_SUFFIX") or ".so"
        dest = Path(self.build_lib) / "rosbag_deck" / f"rosbag_deck{ext_suffix}"
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(so_path, dest)

    def _cargo_build(self, workspace_root):
        """Invoke cargo build for the rosbag_deck_python crate."""
        profile = PROFILES[0]  # default to dev-release
        cmd = [
            "cargo", "build",
            "--profile", profile,
            "-p", "rosbag_deck_python",
        ]
        print(f"Building cdylib: {' '.join(cmd)}", file=sys.stderr)
        subprocess.check_call(cmd, cwd=workspace_root)

        so_path = self._find_cdylib(workspace_root)
        if so_path is None:
            raise RuntimeError(
                f"cargo build succeeded but librosbag_deck.so not found "
                f"in {workspace_root / 'target'}"
            )
        return so_path

    def _find_workspace_root(self):
        """Walk up from this file to find the Cargo workspace root."""
        pkg_dir = Path(__file__).resolve().parent
        d = pkg_dir
        while d != d.parent:
            cargo_toml = d / "Cargo.toml"
            if cargo_toml.exists() and "[workspace]" in cargo_toml.read_text():
                return d
            d = d.parent
        raise RuntimeError("Could not find Cargo workspace root")

    def _find_cdylib(self, workspace_root):
        """Search for librosbag_deck.so in the cargo target directory."""
        target_dir = workspace_root / "target"
        so_name = "librosbag_deck.so"

        for profile in PROFILES:
            candidate = target_dir / profile / so_name
            if candidate.exists():
                return candidate

        return None


package_name = "rosbag_deck_python"

setup(
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    cmdclass={"build_py": BuildPyWithCdylib},
)
