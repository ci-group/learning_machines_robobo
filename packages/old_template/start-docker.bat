SET project_folder=%~dp0
docker run --rm -it -v %project_folder%:/root/projects --net=host cigroup/learning-machines:python3 bash
