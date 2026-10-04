"""Archive committed bench dependencies so concurrent work cannot change a replay."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import zipfile


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--revision", default="30c3bce")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    revision = subprocess.check_output(["git", "rev-parse", args.revision], text=True).strip()
    archive = subprocess.check_output(["git", "archive", "--format=zip", revision,
                                       "world_state", "face_tracking", "ros_nav"])
    args.output.mkdir(parents=True, exist_ok=False)
    with zipfile.ZipFile(io.BytesIO(archive)) as z:
        z.extractall(args.output)
    (args.output / "source-manifest.json").write_text(json.dumps({
        "revision": revision, "archive_sha256": hashlib.sha256(archive).hexdigest()}, indent=2)+"\n")
    print(revision, args.output)
