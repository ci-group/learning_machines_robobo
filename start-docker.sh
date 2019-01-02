#!/bin/bash

set -e

PROJECT_FOLDER="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )/"
exec docker run --rm -it -v "${PROJECT_FOLDER}:/root/projects/" cigroup/learning-machines bash