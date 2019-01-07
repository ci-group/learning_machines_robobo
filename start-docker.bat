SET project_folder=%~dp0
docker run --rm -it -v %project_folder%:/root/projects cigroup/learning-machines bash
