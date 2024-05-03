#!/usr/bin/env python3
# The scripts that moves the stuff that is written to the right examples.
# This should really be a Cmake file or shell script, but, to make sure all TAs can use it,
# It's python.

# To run scripts after building, run:
# chmod -R u+x **/*.sh

import sys
import shutil
import argparse
from pathlib import Path
from dataclasses import dataclass

from typing import List, Optional, Sequence, Tuple, Union

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


def is_ignored_path(path: Path) -> bool:
    """When true, the item of this name will not be deleted when building"""
    return (
        any(
            reserved in path.name.lower()
            for reserved in ["coppeliasim", "assets", ".venv"]
        )
        or path.suffix == ".md"
    )


def remove_existing_in(directory: Path, yes: bool) -> None:
    to_del: List[Path] = []
    for file in directory.iterdir():
        if is_ignored_path(file):
            continue

        if yes or input(f"\n{file} exists. Delete? [y/N]") in ["y", "yes", "Y"]:
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
    scripts: Sequence[Union[str, Tuple[str, str]]] = [],
    caktin_packages: Sequence[str] = [],
    models: Sequence[Union[str, Tuple[str, str]]] = [],
    scenes: Sequence[Union[str, Tuple[str, str]]] = [],
    dockerfile: Optional[str] = None,
    lua: bool = False,
    requirements: Optional[str] = None,
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

    if requirements is not None:
        shutil.copyfile(
            (BASE / "requirements" / requirements), (where / "requirements.txt")
        )


@dataclass
class Args:
    yes: bool
    adv_coppelia_sim: bool
    cache_cpp: bool


def parse_args(args: List[str]) -> Args:
    parser = argparse.ArgumentParser(
        prog="learning_machines_robobo_build",
        description="""
            The build script of learning_machines_robobo.

            This script deletes everything that is in `examples/` and re-creates it from maintained/.
            The exceptions, the files that are not getting deleted, are:
                **/assets/
                **/.venv/
                **/coppeliasim/
                **/*.md
            """,
    )
    parser.add_argument(
        "-y",
        action="store_true",
        help="Answer yes to all prompts. Specifically prompts on if you want to delete files.",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--advanced_coppelia_sim",
        action="store_true",
        help="If passed, a more advanced Dockerfile to run CoppeliaSim is created.",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--cached_cpp_builds",
        action="store_true",
        help="If passed, a more complex Dockerfile for the full project setup is creaetd, massivly speeding up compile times.",
        default=False,
        required=False,
    )
    arguments = parser.parse_args(args)
    return Args(
        yes=arguments.y,
        adv_coppelia_sim=arguments.advanced_coppelia_sim,
        cache_cpp=arguments.cached_cpp_builds,
    )


def main(args: List[str]) -> None:
    arguments = parse_args(args)

    for directory in MANAGED_DIRS:
        remove_existing_in(directory, arguments.yes)

    make_tutorial(DOCKER_TUTORIAL)

    make_tutorial(
        HARDWARE_SETUP,
        caktin_packages=["robobo_msgs"],
        scripts=[("setup_ros_uri.bash", "setup.bash")],
        dockerfile="hardware.dockerfile",
    )

    make_tutorial(
        ROS_TUTORIAL_HELP,
        caktin_packages=["my_robot_controller"],
        dockerfile="ros_tutorial.dockerfile",
    )

    coppelia_dockerfile = "coppelia.dockerfile" if arguments.adv_coppelia_sim else None
    scripts = (
        [
            "start_coppelia_docker.sh",
            "start_coppelia_docker.ps1",
            "start_coppelia_docker_apple_sillicon.zsh",
        ]
        if arguments.adv_coppelia_sim
        else [
            "start_coppelia_sim.sh",
            "start_coppelia_sim.ps1",
            "start_coppelia_sim.zsh",
        ]
    )
    make_tutorial(
        COPPELIA_SIM_TUTORIAL,
        scripts=scripts,
        scenes=["Robobo_Scene.ttt"],
        lua=True,
        dockerfile=coppelia_dockerfile,
        requirements="coppeliasim_tutorial_requirements.txt",
    )

    make_tutorial(
        ROS_BASIC_SETUP,
        scripts=[
            "run.ps1",
            "run.sh",
            "run_apple_sillicon.zsh",
            ("entrypoint_example.bash", "entrypoint.bash"),
            "setup.bash",
        ],
        caktin_packages=["my_first_package", "data_files"],
        dockerfile="full.dockerfile",
        requirements="full_requirements.txt",
    )

    full_docker = "full_cached.dockerfile" if arguments.cache_cpp else "full.dockerfile"
    make_tutorial(
        FULL_PROJECT_SETUP,
        scripts=[
            "entrypoint.bash",
            "setup.bash",
            "run.sh",
            "run.ps1",
            "run_apple_sillicon.zsh",
            "start_coppelia_sim.sh",
            "start_coppelia_sim.ps1",
            "start_coppelia_sim.zsh",
        ],
        caktin_packages=[
            "learning_machines",
            "data_files",
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
        dockerfile=full_docker,
        requirements="full_requirements.txt",
    )


if __name__ == "__main__":
    main(sys.argv[1:])
