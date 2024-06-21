from pathlib import Path

_ERROR_STR = """Could not resolve location of results/figures directory, or it is a file.
    Likely, this is because the `/results` directory is not correctly mounted inside the container.
    Make sure the container is started with the `-v [wd]/results:/root/results` flag like in the runscripts.
"""

# This only works because I'm assuming docker is started with run.ps1/run.sh
RESULTS_DIR = Path("/root/results")

if not RESULTS_DIR.is_dir():
    raise ImportError(_ERROR_STR)

FIGURES_DIR = RESULTS_DIR / "figures"
FIGURES_DIR.mkdir(exist_ok=True)

if not FIGURES_DIR.is_dir():
    raise ImportError(_ERROR_STR)

__all__ = ("RESULTS_DIR", "FIGURES_DIR")
