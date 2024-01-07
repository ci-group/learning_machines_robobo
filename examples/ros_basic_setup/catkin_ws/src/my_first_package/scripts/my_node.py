#!/usr/bin/env python3
import sys
import logging
import argparse

# Import our own code from src/
from my_first_package import ExampleProcess
from data_files import RESULT_DIR

from typing import List


def setup_logging():
    # We are simply using the root logger for everything.
    # You could also do it more properly, but I honestly don't see why you would.
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[
            logging.FileHandler(str(RESULT_DIR / "example.log"), encoding="utf-8"),
            logging.StreamHandler(sys.stdout),
        ],
    )


def parse_args(args: List[str]) -> str:
    parser = argparse.ArgumentParser(
        description="A small sample project that prints and logs",
    )
    parser.add_argument("what", help="What to print and save to a file", type=str)
    return parser.parse_args(args).what


def report(what: str):
    print(what)

    setup_logging()
    logging.info("Started")
    process = ExampleProcess(logging.info)
    process.example_method(what)
    logging.info("Finished")
    with RESULT_DIR.joinpath("hello.txt").open("w") as f:
        f.write("Hello!")


if __name__ == "__main__":
    report(parse_args(sys.argv[1:]))
