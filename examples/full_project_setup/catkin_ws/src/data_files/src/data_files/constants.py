from pathlib import Path

# This only works because I'm assuming docker is started with run.ps1/run.sh
RESULTS_DIR = Path("/root/results")

if not RESULTS_DIR.is_dir():
    raise ImportError("Could not resolve location for results dir, or it is a file")

FIGURES_DIR = RESULTS_DIR / "figures"
FIGURES_DIR.mkdir(exist_ok=True)

if not FIGURES_DIR.is_dir():
    raise ImportError("Could not resolve location for figures dir, or it is a file")

__all__ = ("RESULTS_DIR", "FIGURES_DIR")
