## Basic template containing an ROS package

This is a small package that shows you how an ROS system is set up.

The reason this is a seperate repository is such that you can look around and see the simplest version of each file. The end result of this package is the same as the chatter from Tutorial 2 from the ROS tutorial, only using the exact same structure as the larger project of this course. This example project has one publisher node, that publishes the string "Hello ROS" to the `/test/my_topic` topic, and one subscriber node that listens to that topic and logs anything posted on it. This string "Hello ROS" is exported from the main package, and is in message.py, standing in for a larger codebase.

If you haven't followed the `basic_ros_setup`, this code itself is not going to be easy to follow, but you should still read trough the documentation around it. What matters is not the code, it is the file structure and all that.

### Running the code inside

The funcionality of this package is a publisher and a subscriber node in ROS. You don't have to understand what that is. In fact, you don't have to run this code, and can instead jump to the structure explanation imediately. However, it might be smart to run it anyway, not to understand what all this does, but to get a feel for it. To run it, first build the docker image.

As always when running docker, make sure the docker daemon is running before running any commands.

```sh
docker build --tag my_first_ros_package .
```

After that, you can the containers. However, the containers won't start executing on their own. Instead, like the ROS tutorial, we are going to be entering the bash shell of the container to run some code in it. Because we need a publisher, subscriber, and a master node, we need to run three seperate docker containers, and run code in each of them.

First, to run a container, run:

```sh
docker run --rm -it --net=host my_first_ros_package bash
```

After you have done this three times in three different shells, you'll have three seperate bash shells in seperate containers.

In the first contianer, run the ROS master:
```sh
roscore
```

In the second, run the publisher:

```sh
rosrun my_first_package my_publisher.py
```

In the third, run the subscriber:

```sh
rosrun my_first_package my_subscriber.py
```

Now, observe that `my_subscriber` is printing stuff to the terminal that is being sent from `my_publisher`. 


You only have to understand the basics of this example:
This is the strucute (`tree -a --dirsfirst`)
```
├── catkin_ws
│   ├── src
│   │   └── my_first_package
│   │       ├── scripts
│   │       │   ├── my_publisher.py
│   │       │   └── my_subscriber.py
│   │       ├── src
│   │       │   └── my_first_package
│   │       │       ├── __init__.py
│   │       │       └── message.py
│   │       ├── CMakeLists.txt
│   │       ├── package.xml
│   │       └── setup.py
│   └── .catkin_workspace
├── Dockerfile
└── requirements.txt
```

### Dockerfile / requirements.txt

The first thing you should take a look at is the `Dockerfile`. Unlike the basic ros tutorial help, this one uses the `ros:noetic` base image. This is because here, we don't need the full ROS-desktop install, with the turtlesim examples and all that installed, we just need to run our code, for which we only need the bare bones.

After the base image is loaded, we install some dependencies. Specifically, I install python in a way to make sure `requirements.txt` works as you'd expect. Any python packages that are not ROS specific you might want to install as a dependency go here. I used numpy as an example, but you can also install sklearn and all that this way. Just make sure you are using the verson for python 3.8.

If a package is not specified in requirements.txt, it won't exist in the container, and your code won't be able to find it. If you never wrote one of these files before, check out the [docs](https://pip.pypa.io/en/stable/reference/requirements-file-format/).

After the python dependencies are installed, we run copy over the catkin workspace and run `catkin_make` (This has to be done in a slightly conveluted manner with an inline script... don't worry about it.) If you haven't followed the ros tutorial, you won't know what that is, but it doesn't matter either. It's just something we need to run to make ROS recognise this as a project. If we had C code in here, it would be compiled in this step.

Next up, you'll see that `chmod` is called. This is a linux command that tells the OS that the specified file is allowed to be executed. ROS likes this to be true for some files in your workspace, and, because we are lazy (and are running this in an isolated container), we set it true for all files.

Lastly, a few things are sourced into bashrc. This is to make sure that the shell we are opening when running the code works, and executables like `roscore` and `rosrun` exist. This is the only thing that is different in the actual example, as, there, we call these executables directly from the Dockerfile.

### catkin_ws
This directory contains the catkin workspace. The full documentation on how this is supposed to be structured can be found [here](http://wiki.ros.org/catkin/workspaces), but you don't have to bother. It contains the directory `src`, in which all code will live, and an empty `.catkin_workspace` that is there because catkin works that way.

In `src/` you'll find one package, named "my_first_package". In the actual code, there are multiple, but all of those follow the same structure. All code needs to be in a package. This is an *ROS package*, not jet a python package. Techincally speaking, this directory could have C++ code in it. 

In the root of the package, you'll find three files: `CMakeLists.txt`, `package.xml` and `setup.py`. These, you can completely forget. They are required to make this be a package, and that's all. The ROS tutorial explains a bit on how to use them. For example, you can add ROS dependencies and modules here, but all that is done for you already in the full example.

What matters more are the two directories: `scripts` and `src`. Everything in scripts is an executable: they should all be single files that call some code. The full example only needs one, likely you'll too. Just know this is where the place is of the code that is being called.

`src/` is finally our python package. I made it have the same name as the ROS package, because that is imo the only clean way to do it. Here, you can write any code you want like you would in any other python package, just remember to structure it with `__init__.py` files and all that. `setup.py` is installing this code as a proper python package, so hacks importing via relative filepaths and all that might not work. This package here can be imported in `scripts/` to actually run the code. 

### References

The subscriber and publisher node examples are taken from [The rospy docs](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber). See that page for further documentation on it.

The general package structure is taken from [The Roboticks Back-End tutorial](https://roboticsbackend.com/ros-import-python-module-from-another-package/). See that page for further dockumentation on it.