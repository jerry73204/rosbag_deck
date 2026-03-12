import shutil
import sysconfig
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py


class BuildPyWithCdylib(build_py):
    """Extend build_py to copy the pre-built Rust cdylib into the package.

    The Rust cdylib (librosbag_deck.so) is compiled by cargo as part of
    the workspace build. This command finds it and copies it into the
    setuptools build directory as a Python extension module, so that
    `setup.py install` places it in dist-packages alongside __init__.py.
    """

    def run(self):
        super().run()
        self._copy_cdylib()

    def _copy_cdylib(self):
        so_path = self._find_cdylib()
        if so_path is None:
            raise RuntimeError(
                "Pre-built rosbag_deck cdylib not found. "
                "Run `cargo build --profile dev-release -p rosbag-deck-python` first."
            )

        ext_suffix = sysconfig.get_config_var("EXT_SUFFIX") or ".so"
        dest = Path(self.build_lib) / "rosbag_deck" / f"rosbag_deck{ext_suffix}"
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(so_path, dest)

    def _find_cdylib(self):
        """Search for librosbag_deck.so in the cargo target directory."""
        pkg_dir = Path(__file__).resolve().parent

        # Walk up to find workspace root (directory with Cargo.toml containing [workspace])
        workspace_root = pkg_dir
        while workspace_root != workspace_root.parent:
            cargo_toml = workspace_root / "Cargo.toml"
            if cargo_toml.exists() and "[workspace]" in cargo_toml.read_text():
                break
            workspace_root = workspace_root.parent

        target_dir = workspace_root / "target"
        so_name = "librosbag_deck.so"

        # Search common profile directories
        for profile in ["dev-release", "release", "debug"]:
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
