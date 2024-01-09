# Basic ROS Docker setup

For this course, it is not expected you learn what [ROS](https://www.ros.org/) (Robot Operating System) is or how to use it. However, we do use it in the backend, and you might end up hitting your head against words like `catkin_workspace`and `ros node`. If you are the kind of person who prefers knowing what they're doing over guessing and checking, you might prefer following an introductory tutorial on ROS. [This one, by Robotics Backend](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q) is pretty good, and will be a good place to get yourself more familiar with the subject.

This tutorial works by installing ROS noetic on a virtual machine of Ubuntu 20. However, for this course, we use Docker, meaning you might prefer following the tutorial in Docker as well. Little GUI things like `rqt_graph` and the turtle example won't work, but those are not necessary. It is kind of sad you cannot run the turtle sim for episodes 7, 8, and 9, but you can still follow those episodes, just without any visible result.

If you want the short version instead, ROS is kind of like the Apache Kafka or RabbitMQ of the robotics world, if you've worked with enough server backends to have touched those. Separate independent nodes publish to a queue and read from one.

### Running the container

In this directory, you will find a single `Dockerfile`. This will perform the same ROS installation as the first episode of this tutorial series in a docker container.

As always when running docker, make sure the docker daemon is running before running any commands.

To build it, cd into this directory, and then run in your OS's main terminal:

```sh
docker build --tag ros_tutorial .
```

This will build / construct a container with the name (tag) `ros_tutorial`, from the current directory `.` This command will take a while (~5 min) to run the first time, but Docker cashes everything, so rebuilds won't take as long.

To then run it, and change into its shell, run in your OS's main terminal:

```sh
docker run --rm -it ros_tutorial bash
```

This will run the container called `ros_tutorial` (which we built in the previous step), it will remove / reset the container when it exits (`--rm`), it will run with an interactive `-i` mode (meaning that any input you put into your terminal gets redirected to the container), and it will display any output from the container in your shell as well, with `-t`. On startup, it will run the command `bash` to launch you into a shell.

If you want to be able to do the GUI things anyway, you can run the following, presuming you are on Linux using X11 and have installed the Docker engine directly, not Docker Desktop:

```sh
xhost + && docker run --rm -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --env="QT_X11_NO_MITSHM=1" ros_tutorial bash
```

Since this is a docker container, you need to re-build if you changed the Dockerfile or want to get an updated version of any local files in `./catkin_ws`. Simply shutting down a docker container will delete all its contents, and re-running it will give you a clean new image like it was just built. To exit a container, type `exit`.

On Linux, because we are using `--net=host` you can start up separate docker containers for each of the ros nodes you want to run. You can `docker run` the container, and start-up roscore in it, and then, in a different terminal, run `docker run` again and start a node there. This doesn't work if you installed docker desktop, however. For this, please view the help below.

---

To start `roscore` (or any ROS node) without having your terminal blocked you can run this in the shell of your docker container:

```sh
roscore &> ~/projects/roscore_output.txt &
```

This will run the node (in this case `roscore`), keep the terminal open (by ending in `&`), and redirect all output of that program (`&>` redirects `stdout` and `sterr`) to a file (in this case `~/projects/roscore_output.txt`).

To stop anything you started like this, you can run this from the terminal of the container:

```sh
pkill roscore
```

and you can view the output of them with any terminal-based text viewer. I'd use [less](<https://en.wikipedia.org/wiki/Less_(Unix)>), but you can install any terminal-based text editor (such as nano, vim, or emacs), by changing line 15 of the Dockerfile. Please note that piping is buffered (and this is non-trivial to circumvent: neither `script -f` nor `stdbuf -o0` work.), meaning you might not see the output of a node until it is closed.

```sh
less --follow-name ~/projects/roscore_output.txt
```

In this command, the --follow-name command tells less to update its viewing contents when another program writes to the file.

### Catkin Workspace (ROS Tutorial 3 and up)

In this directory, you will also find a `catkin_ws` workspace. From tutorial 3 and on, you have to change files in this directory. In a VM, this means changing the files inside the VM, but, for a docker container, this means changing the files in `catkin_ws` _locally_, and then re-running `docker build --tag ros_tutorial .`, and re-starting all containers every time you changed something. The docker build will run `catkin_make` on it during the build, and all `devel/setup.bash` will be sourced in the shell you spawn in when running the docker container. Lastly, all files will be chmod'ed, so you don't need to do that either.

Because you don't have catkin installed locally, you won't be able to run `catkin_create_pkg` and have the output saved locally. To counteract this, the files currently in `catkin_ws` are the output of running `carkin_create_pkg` with the arguments from the tutorial.
