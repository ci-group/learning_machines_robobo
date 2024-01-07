#!/usr/bin/env python3
import sys
import argparse
from pathlib import Path

from typing import List


def parse_pathstr(ctx: argparse.ArgumentParser, pathstr: str) -> List[Path]:
    pathspec = Path(pathstr)
    paths = [pathspec] if pathspec.exists() else list(Path().glob(pathstr))
    if not paths:
        ctx.error(f"Could not resolve to filepaths: {pathspec}")
    if not all(path.is_file() and path.exists() for path in paths):
        ctx.error(f"File does not exist: {pathspec}")
    return [path.resolve() for path in paths]


def parse_args(args: List[str]) -> List[Path]:
    parser = argparse.ArgumentParser(
        description=r"Remove all instances of \r from a file"
    )
    parser.add_argument(
        "paths",
        help="The paths to convert line endings for",
        nargs="+",
    )

    arguments = parser.parse_args(args)
    return [file for path in arguments.paths for file in parse_pathstr(parser, path)]


def run(paths: List[Path]) -> None:
    for path in paths:
        with path.open("r") as f:
            intext = f.read()
        with path.open("w", encoding="utf-8", newline="\n") as f:
            f.write(intext)


if __name__ == "__main__":
    run(parse_args(sys.argv[1:]))
