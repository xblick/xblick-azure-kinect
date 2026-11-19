"""The wheel's programs at their start, without a camera, a Zenoh router or a GPU: --help, and the stop with one line
where a part is missing (the NVIDIA driver or a visible GPU, NVIDIA's packages, the depth engine). Also the setup
command's unpacking of Microsoft's package, with a package made here (no download). Run in the environment the wheel
is installed in:

    python -m pytest tests/test_wheel.py
"""

import ctypes
import hashlib
import io
import os
import shutil
import subprocess
import tarfile
from pathlib import Path

import pytest

xblick_azure_kinect = pytest.importorskip("xblick_azure_kinect")
from xblick_azure_kinect import sdk  # noqa: E402

CAMERA = str(xblick_azure_kinect.binary("xblick-azure-kinect"))
PARAMETERS = str(xblick_azure_kinect.binary("xblick-azure-kinect-parameters"))


def has_driver() -> bool:
    try:
        ctypes.CDLL("libcuda.so.1")
    except OSError:
        return False
    return True


def run(*args, program=CAMERA, env=None, timeout=30):
    return subprocess.run([program, *args], capture_output=True, text=True, timeout=timeout,
                          env=dict(os.environ, **(env or {})))


def error_line(result) -> str:
    """The one line on stderr of a program that stopped at its start with exit 1."""
    lines = [line for line in result.stderr.splitlines() if line.strip()]
    assert result.returncode == 1 and len(lines) == 1, (result.returncode, result.stdout, result.stderr)
    return lines[0]


def test_the_programs_start():
    """The camera's --help; the parameter client (no --help, as in the source build) names --node."""
    result = run("--help")
    assert result.returncode == 0 and "--cuda-device" in result.stdout, result.stderr
    assert error_line(run(program=PARAMETERS)) == "xblick-azure-kinect-parameters: --node <prefix>, e.g. cam/azurekinect"
    result = subprocess.run(["xblick-azure-kinect-setup", "--help"], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0 and "libk4a1.4" in result.stdout, result.stderr


def test_default_gpu_without_a_gpu_stops():
    """--cuda-device 0 is the default; without a visible GPU the node stops and names --cuda-device none."""
    line = error_line(run(env={"CUDA_VISIBLE_DEVICES": ""}))
    assert line.startswith("xblick-azure-kinect: --cuda-device 0: "), line
    assert line.endswith("; pass --cuda-device none to decode the colour on the CPU"), line
    assert "no CUDA device is visible (CUDA_VISIBLE_DEVICES='')" in line or "no NVIDIA driver" in line, line


@pytest.mark.skipif(has_driver(), reason="this PC has an NVIDIA driver")
def test_without_a_driver_stops():
    line = error_line(run())
    assert line == ("xblick-azure-kinect: --cuda-device 0: no NVIDIA driver (libcuda.so.1 cannot be loaded); "
                    "pass --cuda-device none to decode the colour on the CPU"), line


def test_without_nvidia_packages_stops(tmp_path):
    """A copy of the package (with the libraries auditwheel put beside it) outside the site-packages that holds
    NVIDIA's packages: the CUDA module does not load, the node stops and names the packages."""
    package = Path(CAMERA).resolve().parent.parent
    shutil.copytree(package, tmp_path / package.name, symlinks=True)
    libs = package.parent / f"{package.name}.libs"
    if libs.is_dir():
        shutil.copytree(libs, tmp_path / libs.name, symlinks=True)
    line = error_line(run(program=str(tmp_path / package.name / "bin" / "xblick-azure-kinect")))
    assert line.startswith("xblick-azure-kinect: --cuda-device 0: cannot load nvJPEG and the CUDA runtime ("), line
    assert "nvidia-nvjpeg-cu12 and nvidia-cuda-runtime-cu12 must be installed" in line, line


def test_missing_depth_engine_names_the_setup_command(tmp_path):
    """A camera and --sdk-depth-to-color need the depth engine; the node stops before its session."""
    engine = tmp_path / "cache" / "azure-kinect" / f"depth-engine-{sdk.K4A_PACKAGE_VERSION}" / sdk.ENGINE
    expected = (f"xblick-azure-kinect: the depth engine of Microsoft's Azure Kinect Sensor SDK is missing ({engine}): "
                "run xblick-azure-kinect-setup once, which downloads it from Microsoft")
    for args in (["--cuda-device", "none"],
                 ["--cuda-device", "none", "--playback", str(tmp_path / "missing.mkv"), "--sdk-depth-to-color"]):
        line = error_line(run(*args, env={"XBLICK_HOME": str(tmp_path)}))
        assert line == expected, (args, line)


def test_playback_needs_no_depth_engine(tmp_path):
    """A recording plays without the engine (as in the source build): with a missing file the node gets as far as
    opening it, and libk4a's three lines about the engine at the end stay out."""
    result = run("--cuda-device", "none", "--listen", "tcp/127.0.0.1:0", "--playback", str(tmp_path / "missing.mkv"),
                 env={"XBLICK_HOME": str(tmp_path)})
    line = error_line(result)
    assert "depth engine" not in line and "missing.mkv" in line, line
    assert "depthengine" not in result.stdout, result.stdout


def test_setup_without_acceptance_stops(tmp_path):
    """Without a terminal, the option or the variable the setup command downloads nothing and names both ways."""
    env = {key: value for key, value in os.environ.items() if key != sdk.ACCEPT_VARIABLE}
    result = subprocess.run(["xblick-azure-kinect-setup"], capture_output=True, text=True, timeout=30,
                            stdin=subprocess.DEVNULL, env=dict(env, XBLICK_HOME=str(tmp_path)))
    assert result.returncode == 1 and result.stdout == "", (result.stdout, result.stderr)
    line = result.stderr.strip()
    assert "\n" not in line and sdk.ACCEPT_OPTION in line and f"{sdk.ACCEPT_VARIABLE}=yes" in line, line
    assert sdk.TERMS_URL in line, line
    assert not (tmp_path / "cache" / "azure-kinect" / "downloads").exists()
    result = subprocess.run(["xblick-azure-kinect-setup"], capture_output=True, text=True, timeout=30,
                            stdin=subprocess.DEVNULL, env=dict(env, XBLICK_HOME=str(tmp_path),
                                                               **{sdk.ACCEPT_VARIABLE: "no"}))
    assert result.returncode == 1 and "only with the value yes" in result.stderr, result.stderr


def test_acceptance_is_recorded(tmp_path, monkeypatch):
    """The variable accepts; the record beside the engine names how, the terms and the link; then no question."""
    target = tmp_path / "depth-engine"
    target.mkdir()
    monkeypatch.setenv(sdk.ACCEPT_VARIABLE, "yes")
    how = sdk.accept(target, option=False, out=io.StringIO())
    sdk.record_acceptance(target, how)
    record = (target / sdk.ACCEPTANCE_FILE).read_text()
    assert f"how: {sdk.ACCEPT_VARIABLE}=yes" in record and sdk.TERMS_URL in record and sdk.PACKAGE in record
    assert sdk.accepted(target)


def fake_package(path: Path, engine: bytes) -> None:
    """A Debian package (ar archive) whose data.tar.xz has the two files the setup command unpacks."""
    data = io.BytesIO()
    with tarfile.open(fileobj=data, mode="w:xz") as tar:
        for name, content in ((sdk.ENGINE_MEMBER, engine), (sdk.LICENCE_MEMBER, b"terms")):
            info = tarfile.TarInfo(name)
            info.size = len(content)
            tar.addfile(info, io.BytesIO(content))
    members = [("debian-binary", b"2.0\n"), ("data.tar.xz", data.getvalue())]
    with open(path, "wb") as f:
        f.write(b"!<arch>\n")
        for name, content in members:
            f.write(f"{name:<16}{0:<12}{0:<6}{0:<6}{100644:<8}{len(content):<10}`\n".encode())
            f.write(content + (b"\n" if len(content) % 2 else b""))


def test_setup_unpacks_only_the_pinned_engine(tmp_path, monkeypatch):
    engine = b"\x7fELF not the engine"
    package = tmp_path / "package.deb"
    fake_package(package, engine)
    target = tmp_path / "depth-engine"
    with pytest.raises(RuntimeError, match="expected " + sdk.ENGINE_SHA256):
        sdk.unpack_engine(package, target)
    assert not target.exists() and [p.name for p in tmp_path.iterdir()] == ["package.deb"]
    monkeypatch.setattr(sdk, "ENGINE_SHA256", hashlib.sha256(engine).hexdigest())
    sdk.unpack_engine(package, target)
    assert sorted(p.name for p in target.iterdir()) == ["LICENSE.txt", sdk.ENGINE]
    assert (target / sdk.ENGINE).read_bytes() == engine
