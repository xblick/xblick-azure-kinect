"""The command xblick-azure-kinect-setup: what the wheel cannot contain, on this PC.

    xblick-azure-kinect-setup

1. The depth engine (libdepthengine.so.2.0, closed source, Microsoft's terms): Microsoft's package libk4a1.4 at the
   pinned version and SHA-256 that tools/setup.sh uses, downloaded into the node's cache directory
   (``xblick.paths.cache_dir("azure-kinect")``: ``~/.cache/xblick/azure-kinect``, ``$XBLICK_HOME/cache/azure-kinect``)
   below ``downloads/``, the same file tools/setup.sh downloads; a file there with the right checksum is not
   downloaded again. The engine and Microsoft's licence text are unpacked into ``depth-engine-<version>/``, where the
   program xblick-azure-kinect loads the engine at its start (src/wheel.cpp). Nothing else of the package is used.
2. The system libraries the engine and libk4a load (libGL, libX11, libEGL, libudev): the ``sudo apt install``
   command for those missing, exit 1.
3. The udev rule that lets a user other than root open the camera: the ``sudo`` commands when it is missing.

It never asks and never uses sudo. Run again, it downloads nothing when the engine is in place.
"""

from __future__ import annotations

import argparse
import ctypes
import glob
import hashlib
import io
import os
import re
import shlex
import shutil
import sys
import tarfile
import tempfile
from pathlib import Path

from xblick import paths, weights

PROG = "xblick-azure-kinect-setup"
# Microsoft's package (packages.microsoft.com, Ubuntu 18.04 pool), as tools/setup.sh pins it. CMakeLists.txt reads
# this line: the program looks for the engine in depth-engine-<K4A_PACKAGE_VERSION>/.
K4A_PACKAGE_VERSION = "1.4.1"
PACKAGE = f"libk4a1.4_{K4A_PACKAGE_VERSION}_amd64.deb"
PACKAGE_URL = f"https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/{PACKAGE}"
PACKAGE_SIZE = 7383330
PACKAGE_SHA256 = "c1c63f81641eed1326136a44e5a5fd229e1e6315b2b20b2dddb5523a556c9329"
ENGINE = "libdepthengine.so.2.0"
ENGINE_MEMBER = f"./usr/lib/x86_64-linux-gnu/libk4a1.4/{ENGINE}"
ENGINE_SHA256 = "1adbaf9c9311e685e55a31400b345ee13241c5a2002a80601bbcea5cb8ca89e1"
LICENCE_MEMBER = "./usr/share/doc/libk4a1.4/LICENSE.txt"
TERMS_URL = f"https://www.nuget.org/packages/Microsoft.Azure.Kinect.Sensor/{K4A_PACKAGE_VERSION}/License"
TERMS = (f"{PROG}: Microsoft's package libk4a1.4 {K4A_PACKAGE_VERSION} contains the depth engine ({ENGINE}), "
         "closed source and under the Microsoft Software License Terms of the Azure Kinect Sensor SDK: "
         f"{TERMS_URL}. Only the depth engine and the terms are unpacked from it.")
# the libraries the depth engine and libk4a need from the system, with their Ubuntu and Debian packages
SYSTEM_LIBRARIES = {"libGL.so.1": "libgl1", "libX11.so.6": "libx11-6", "libEGL.so.1": "libegl1",
                    "libudev.so.1": "libudev1"}
UDEV_RULES = Path(__file__).with_name("99-k4a.rules")  # scripts/99-k4a.rules of the Sensor SDK v1.4.2 (MIT)


def engine_dir() -> Path:
    """Where the engine goes: <cache_dir("azure-kinect")>/depth-engine-<version>/ (src/wheel.cpp looks there)."""
    return paths.cache_dir("azure-kinect") / f"depth-engine-{K4A_PACKAGE_VERSION}"


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def data_tar(deb: Path) -> bytes:
    """The member data.tar.* of a Debian package (an ar archive)."""
    with open(deb, "rb") as f:
        if f.read(8) != b"!<arch>\n":
            raise RuntimeError(f"{deb} is not a Debian package")
        while header := f.read(60):
            if len(header) < 60 or header[58:60] != b"`\n":
                raise RuntimeError(f"{deb}: a damaged ar header")
            name = header[:16].decode("ascii").strip().rstrip("/")
            size = int(header[48:58].decode("ascii").strip())
            data = f.read(size)
            if size % 2:
                f.read(1)
            if name.startswith("data.tar"):
                return data
    raise RuntimeError(f"{deb} has no data.tar member")


def unpack_engine(deb: Path, target: Path) -> None:
    """The engine and the licence text from the package into ``target`` (complete or not at all)."""
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix=f".{target.name}.", dir=target.parent))
    try:
        with tarfile.open(fileobj=io.BytesIO(data_tar(deb)), mode="r:*") as tar:
            for member, name in ((ENGINE_MEMBER, ENGINE), (LICENCE_MEMBER, "LICENSE.txt")):
                source = tar.extractfile(member)
                if source is None:
                    raise RuntimeError(f"{deb} has no file {member}")
                with source, open(tmp / name, "wb") as out:
                    shutil.copyfileobj(source, out)
        digest = sha256_of(tmp / ENGINE)
        if digest != ENGINE_SHA256:
            raise RuntimeError(f"{ENGINE} in {deb} has the SHA-256 {digest}, expected {ENGINE_SHA256}")
        (tmp / ENGINE).chmod(0o755)
        if target.exists():
            shutil.rmtree(target)
        os.replace(tmp, target)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def missing_libraries() -> list[str]:
    missing = []
    for library in SYSTEM_LIBRARIES:
        try:
            ctypes.CDLL(library)
        except OSError:
            missing.append(library)
    return missing


def has_udev_rule() -> bool:
    """Whether a udev rule names the depth camera (045e:097c), as tools/setup.sh checks."""
    for rules in glob.glob("/etc/udev/rules.d/*.rules") + glob.glob("/lib/udev/rules.d/*.rules") + \
            glob.glob("/usr/lib/udev/rules.d/*.rules"):
        try:
            if re.search(r"045e.*097c", Path(rules).read_text(errors="replace")):
                return True
        except OSError:
            continue
    return False


def has_nvidia_egl() -> bool:
    """Whether GLVND lists NVIDIA's EGL driver (the file the NVIDIA driver installs)."""
    return bool(glob.glob("/usr/share/glvnd/egl_vendor.d/*nvidia*.json") +
                glob.glob("/etc/glvnd/egl_vendor.d/*nvidia*.json"))


def setup() -> int:
    target = engine_dir()
    engine = target / ENGINE
    if engine.is_file() and sha256_of(engine) == ENGINE_SHA256:
        print(f"{PROG}: the depth engine is in place: {engine}", flush=True)
    else:
        store = weights.Store(PROG, node="azure-kinect", terms=TERMS)
        package = weights.Weight(Path("downloads") / PACKAGE, weights.Url(PACKAGE_URL), PACKAGE_SIZE, PACKAGE_SHA256)
        try:
            downloaded = store.ensure([package], verify=True)
        except weights.WeightsError as error:
            return store.fail(error)
        if not downloaded:  # the terms are printed before a download, and before unpacking a package found there
            print(TERMS, flush=True)
        unpack_engine(store.file(package), target)
        print(f"{PROG}: the depth engine is in place: {engine} (Microsoft's terms: {target / 'LICENSE.txt'})",
              flush=True)

    code = 0
    missing = missing_libraries()
    if missing:
        print(f"{PROG}: the depth engine and libk4a need {', '.join(missing)} from the system. Install them with:\n"
              f"  sudo apt install {' '.join(SYSTEM_LIBRARIES[library] for library in missing)}", flush=True)
        code = 1
    else:
        try:
            ctypes.CDLL(str(engine))
        except OSError as error:
            print(f"{PROG}: the depth engine does not load: {error}", flush=True)
            code = 1
    if not has_nvidia_egl():
        print(f"{PROG}: notice: GLVND lists no NVIDIA EGL driver. The depth engine needs an NVIDIA GPU with its "
              "driver (EGL); recordings play without it unless --sdk-depth-to-color is given.", flush=True)
    if not has_udev_rule():
        rules = shlex.quote(str(UDEV_RULES))
        print(f"{PROG}: notice: no udev rule for the Azure Kinect, so only root can open a camera. Install it with:\n"
              f"  sudo cp {rules} /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger",
              flush=True)
    if code == 0:
        print(f"{PROG}: done; xblick-azure-kinect loads the depth engine from {target}", flush=True)
    return code


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        prog=PROG,
        description="Downloads Microsoft's package libk4a1.4 (pinned version and SHA-256) into the node's cache "
                    "directory and unpacks its depth engine there, where xblick-azure-kinect loads it; names the "
                    "sudo commands for the system libraries and the udev rule that are missing.",
        epilog=f"The package goes to {paths.cache_dir('azure-kinect') / 'downloads'}: a file there with the right "
               "checksum is not downloaded again. The engine is under Microsoft's terms: " + TERMS_URL)
    parser.parse_args(argv)
    sys.exit(setup())
