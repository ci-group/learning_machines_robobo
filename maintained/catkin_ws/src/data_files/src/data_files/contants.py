from pathlib import Path

# This only works because I'm assuming docker is started with run.ps1/run.sh
RESULT_DIR = Path("/root/results")

if not RESULT_DIR.is_dir():
    raise ImportError("Could not resolve location for results dir, or it is a file")

FIGRURES_DIR = RESULT_DIR / "figures"
FIGRURES_DIR.mkdir(exist_ok=True)

if not FIGRURES_DIR.is_dir():
    raise ImportError("Could not resolve location for figures dir, or it is a file")

__all__ = ("RESULT_DIR", "FIGRURES_DIR")
