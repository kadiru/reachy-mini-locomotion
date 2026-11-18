"""Hydrate only official MJCF assets, verifying each Git LFS SHA256 and size.

No git-lfs executable is required. Run after cloning the official source.
"""
import concurrent.futures
import hashlib
from pathlib import Path
import subprocess
import urllib.request

root = Path(__file__).resolve().parent / "upstream-reachy-mini"
commit = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
assets = root / "src/reachy_mini/descriptions/reachy_mini/mjcf/assets"
tasks = []
for path in assets.rglob("*"):
    if path.is_file() and path.stat().st_size < 200:
        text = path.read_text(errors="ignore")
        if text.startswith("version https://git-lfs.github.com/spec/v1"):
            lines = text.splitlines()
            oid = next(x.split("sha256:")[1] for x in lines if x.startswith("oid "))
            size = int(next(x.split()[1] for x in lines if x.startswith("size ")))
            tasks.append((path, oid, size))


def fetch(task):
    path, oid, size = task
    relative = path.relative_to(root).as_posix()
    url = "https://media.githubusercontent.com/media/pollen-robotics/reachy_mini/" + commit + "/" + relative
    with urllib.request.urlopen(url, timeout=30) as response:
        data = response.read()
    if len(data) != size or hashlib.sha256(data).hexdigest() != oid:
        raise ValueError("LFS verification failed: " + relative)
    path.write_bytes(data)
    return len(data)


if __name__ == "__main__":
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        sizes = list(pool.map(fetch, tasks))
    print("Verified", len(sizes), "assets;", sum(sizes), "bytes at", commit)
