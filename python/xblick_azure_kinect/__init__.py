"""xblick-azure-kinect as a Python wheel (``uv tool install xblick-azure-kinect``): the programs xblick-azure-kinect
and xblick-azure-kinect-parameters with the libraries they need (libk4a and libk4arecord built from the Sensor SDK's
source, libzenohc.so, the EGL shim and the CUDA module); nvJPEG, NPP and the CUDA runtime come from NVIDIA's packages,
dependencies of the wheel. The depth engine is not in the wheel: ``xblick-azure-kinect-setup`` (``sdk.py``) fetches
it from Microsoft into the node's cache directory, where the program loads it at the start.

The commands ``xblick-azure-kinect`` and ``xblick-azure-kinect-parameters`` (``main``, ``parameters_main``) replace
themselves with the programs, with the same arguments; ``binary(name)`` is where a program is."""

import os
import sys
from importlib.metadata import version
from pathlib import Path

__version__ = version("xblick-azure-kinect")

PROGRAMS = ("xblick-azure-kinect", "xblick-azure-kinect-parameters")


def binary(name: str = "xblick-azure-kinect") -> Path:
    """A program of this wheel (xblick_azure_kinect/bin/<name>)."""
    if name not in PROGRAMS:
        raise ValueError(f"{name}: the wheel's programs are {', '.join(PROGRAMS)}")
    path = Path(__file__).parent / "bin" / name
    if not path.is_file():
        raise FileNotFoundError(f"{path} is missing: xblick-azure-kinect is installed without its programs (an "
                                f"editable or broken install); reinstall it from a wheel")
    return path


def _exec(name: str) -> None:
    program = str(binary(name))
    os.execv(program, [program, *sys.argv[1:]])


def main() -> None:
    """The command xblick-azure-kinect: the camera node."""
    _exec("xblick-azure-kinect")


def parameters_main() -> None:
    """The command xblick-azure-kinect-parameters: a node's parameters over its ROS 2 parameter services."""
    _exec("xblick-azure-kinect-parameters")
