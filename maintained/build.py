#!/usr/bin/env python3
# The scripts that moves the stuff that is written to the right examples.
# This should really be a Cmake file or shell script, but, to make sure all TAs can use it,
# It's python.

# To spam yes at all the questions, use:
# yes | ./build.py

# To run scripts after building, run:
# chmod -R u+x **/*.sh

import sys
import shutil
from pathlib import Path

from typing import List, Optional, Tuple, Union

# Using Cmake style stuff because what else would I use.
EXAMPLES = Path(__file__).parent.parent.joinpath("examples").resolve(strict=True)

COPPELIA_SIM_TUTORIAL = EXAMPLES / "coppelia_sim_tutorial"
DOCKER_TUTORIAL = EXAMPLES / "docker_tutorial"
HARDWARE_SETUP = EXAMPLES / "hardware_setup"
ROS_TUTORIAL_HELP = EXAMPLES / "ros_tutorial_help"
ROS_BASIC_SETUP = EXAMPLES / "ros_basic_setup"
FULL_PROJECT_SETUP = EXAMPLES / "full_project_setup"

MANAGED_DIRS = [
    COPPELIA_SIM_TUTORIAL,
    DOCKER_TUTORIAL,
    HARDWARE_SETUP,
    ROS_TUTORIAL_HELP,
    ROS_BASIC_SETUP,
    FULL_PROJECT_SETUP,
]

for directory in MANAGED_DIRS:
    directory.mkdir(exist_ok=True)
    if not directory.is_dir():
        raise FileNotFoundError(f"Cannot find example {directory}")

BASE = Path(__file__).parent.resolve(strict=True)


def remove_existing_in(directory: Path) -> None:
    to_del: List[Path] = []
    for file in directory.iterdir():
        if file.name in ["README.md", "CoppeliaSim"]:
            continue

        if input(f"\n{file} exists. Should it be removed? [y/N]") in ["y", "yes", "Y"]:
            to_del.append(file)
        else:
            sys.exit("Not removing any files. Exiting")

    for file in to_del:
        if file.is_dir():
            shutil.rmtree(str(file))
        else:
            file.unlink()


def make_tutorial(
    where: Path,
    *,
    scripts: List[Union[str, Tuple[str, str]]] = [],
    caktin_packages: List[str] = [],
    models: List[Union[str, Tuple[str, str]]] = [],
    scenes: List[Union[str, Tuple[str, str]]] = [],
    dockerfile: Optional[str] = None,
    lua: bool = False,
    requirements: bool = False,
):
    def get_base_target(name: str) -> Tuple[Path, Path]:
        base = BASE / name
        target = where / name
        target.mkdir(exist_ok=True)
        return base, target

    def expand_if_str(name: Union[str, Tuple[str, str]]) -> Tuple[str, str]:
        return name if isinstance(name, Tuple) else (name, name)

    for script in scripts:
        base, target = get_base_target("scripts")
        base_script, target_script = expand_if_str(script)
        shutil.copyfile((base / base_script), (target / target_script))

    for scene in scenes:
        base, target = get_base_target("scenes")
        base_scene, target_scene = expand_if_str(scene)
        shutil.copyfile((base / base_scene), (target / target_scene))

    for model in models:
        base, target = get_base_target("models")
        base_model, target_model = expand_if_str(model)
        shutil.copyfile((base / base_model), (target / target_model))

    for package in caktin_packages:
        base, target = get_base_target("catkin_ws")
        shutil.copyfile((base / ".catkin_workspace"), (target / ".catkin_workspace"))
        shutil.copytree((base / "src" / package), (target / "src" / package))

    if dockerfile is not None:
        shutil.copyfile((BASE / "dockerfiles" / dockerfile), (where / "Dockerfile"))

    if lua:
        shutil.copytree((BASE / "lua"), (where / "lua_scripts"))

    if requirements:
        shutil.copyfile((BASE / "requirements.txt"), (where / "requirements.txt"))


def main() -> None:
    for directory in MANAGED_DIRS:
        remove_existing_in(directory)

    make_tutorial(DOCKER_TUTORIAL, scripts=["convert_line_endings.py"])

    make_tutorial(
        HARDWARE_SETUP,
        scripts=["convert_line_endings.py", "setup_ros_uri.bash"],
        dockerfile="hardware.dockerfile",
    )

    make_tutorial(
        ROS_TUTORIAL_HELP,
        scripts=["convert_line_endings.py"],
        dockerfile="ros_tutorial.dockerfile",
    )

    make_tutorial(COPPELIA_SIM_TUTORIAL, lua=True)

    make_tutorial(
        ROS_BASIC_SETUP,
        scripts=[
            "convert_line_endings.py",
            "run.ps1",
            "run.sh",
            ("entrypoint_example.bash", "entrypoint.bash"),
            "setup.bash",
        ],
        caktin_packages=["my_first_package"],
        dockerfile="full.dockerfile",
        requirements=True,
    )

    make_tutorial(
        FULL_PROJECT_SETUP,
        scripts=[
            "convert_line_endings.py",
            "entrypoint.bash",
            "setup.bash",
            "run.sh",
            "run.ps1",
            "start_coppelia_sim.sh",
            "start_coppelia_sim.ps1",
        ],
        caktin_packages=[
            "coppelia_sim",
            "learning_machines",
            "learning_machines_prey",
            "robobo_interface",
            "robobo_msgs",
        ],
        models=["robobo.ttm", "robobo-pusher.ttm"],
        scenes=[
            "arena_approach.ttt",
            "arena_obstacles.ttt",
            "arena_push_easy.ttt",
            "arena_push_hard.ttt",
            "Robobo_Scene.ttt",
        ],
        dockerfile="full.dockerfile",
        requirements=True,
    )


if __name__ == "__main__":
    main()
