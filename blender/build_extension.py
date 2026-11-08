"""Package the Blender add-on and a packer executable into an installable extension zip."""

from __future__ import annotations

import argparse
import platform
import re
import sys
import zipfile
from pathlib import Path


ADDON_DIR = Path(__file__).resolve().parent / "mesh_packer"
MANIFEST = "blender_manifest.toml"
PLATFORMS = ("windows-x64", "windows-arm64", "linux-x64", "macos-arm64", "macos-x64")
SEMVER = re.compile(r"\d+\.\d+\.\d+(-[0-9A-Za-z.-]+)?(\+[0-9A-Za-z.-]+)?")


def host_platform() -> str:
    system = {"win32": "windows", "darwin": "macos"}.get(sys.platform, "linux")
    arch = "arm64" if platform.machine().lower() in {"arm64", "aarch64"} else "x64"
    return f"{system}-{arch}"


def replace_line(text: str, pattern: str, replacement: str) -> str:
    result, count = re.subn(pattern, replacement, text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise SystemExit(f"{MANIFEST}: no line matches {pattern!r}")
    return result


def build_manifest(version: str | None, target: str) -> tuple[str, str]:
    text = (ADDON_DIR / MANIFEST).read_text(encoding="utf-8")
    if version is not None:
        text = replace_line(text, r'^version = ".*"$', f'version = "{version}"')
    # Each zip bundles one native executable, so Blender should only install it on that platform
    text = replace_line(text, r'^(blender_version_min = ".*")$', f'\\1\nplatforms = ["{target}"]')
    return text, re.search(r'^version = "(.*)"$', text, flags=re.MULTILINE).group(1)


def addon_files() -> list[Path]:
    files = []
    for path in sorted(ADDON_DIR.rglob("*")):
        parts = path.relative_to(ADDON_DIR).parts
        if path.is_file() and path.name != MANIFEST and parts[0] != "bin" and "__pycache__" not in parts:
            files.append(path)
    return files


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True, help="packer executable to bundle")
    parser.add_argument("--platform", choices=PLATFORMS, default=host_platform(), help="platform the binary runs on")
    parser.add_argument("--version", help="extension version (default: the one in the manifest)")
    parser.add_argument("--output-dir", type=Path, default=Path("dist"), help="where to write the zip")
    args = parser.parse_args()

    if args.version is not None and not SEMVER.fullmatch(args.version):
        raise SystemExit(f"--version must be a semantic version like 1.2.3, got {args.version!r}")
    if not args.binary.is_file():
        raise SystemExit(f"binary not found: {args.binary}")

    manifest, version = build_manifest(args.version, args.platform)
    executable = "bin/packer.exe" if args.platform.startswith("windows") else "bin/packer"

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / f"{ADDON_DIR.name}-{version}-{args.platform}.zip"
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(MANIFEST, manifest)
        for path in addon_files():
            archive.write(path, path.relative_to(ADDON_DIR).as_posix())

        info = zipfile.ZipInfo(executable, date_time=(2000, 1, 1, 0, 0, 0))
        info.compress_type = zipfile.ZIP_DEFLATED
        info.external_attr = 0o755 << 16
        archive.writestr(info, args.binary.read_bytes())

    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
