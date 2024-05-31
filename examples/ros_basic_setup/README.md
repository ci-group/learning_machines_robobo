## Basic template containing a ROS package

This is a small package that shows you how a ROS system is set up.

The reason this is a separate repository is that you can look around and see the simplest version of each file. The result of this package is just a Hello World. It prints the one argument you pass through the command line, it then logs that, saves the log in a file, and saves "Hello!" to another file.

If you haven't followed the [ros tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_tutorial_help), this thing is going to be a bit harder to follow, as you're new to the idea of a `catikin_ws` and all that. However, fear not, what you need to know will be explained.

### Running the code inside

To run, for the first time, you're not going to call docker yourself. Instead, there is a script under the `scripts/` directory, `run.sh` and `run.ps1`, that contains the commands you need to run. These are mostly there for convenience, as they are quite a mouthful. Of course, to debug, you might still need to manually build and run, so you should still try to understand these commands, and know how to change them. They pass quite a few more flags than the commands from the docker tutorial, with all flags being `-t --rm -p 45100:45100 -p 45101:45101 -v ./results:/root/results`. You don't need to understand these, just know that, for interactive mode as per the docker tutorial, you can replace `-t` for `-it`, keeping all other arguments the same.

This running script takes one argument: what to print / log, so calling it should just be as simple as `bash ./scripts/run.sh "hello"` on Linux / macOS or `.\scripts\run.ps1 "Hello"` on Windows. This should print, log to file and to stdout, and save "Hello!" to `results/hello.txt`

Here is the sample output of what the output should look like:

```
bash ./scripts/run.sh hello
+ docker build --tag learning_machines .
[+] Building 0.7s (18/18) FINISHED                        docker:default
 => [internal] load build definition from Dockerfile                0.0s
...
 => => naming to docker.io/library/learning_machines                0.0s
+ docker run -t --rm -p 45100:45100 -p 45101:45101 -v ./results:/root/results learning_machines hello
hello
2024-01-06 19:01:55,090 [INFO] Started
2024-01-06 19:01:55,090 [INFO] hello
2024-01-06 19:01:55,090 [INFO] Finished
```

(Note: if you're wondering: you indeed don't need the `-p` for this tutorial, but it's needed for the full setup, so I included it here, too.)

## Project structure.

Everything here is structured as follows (`tree -a --dirsfirst`):

```
├── catkin_ws
│   ├── src
│   │   ├── my_first_package
│   │   │   ├── scripts
│   │   │   │   └── my_node.py
│   │   │   ├── src
│   │   │   │   └── my_first_package
│   │   │   │       ├── example_process.py
│   │   │   │       └── __init__.py
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   └── setup.py
│   │   └── data_files
│   │       └── ...
│   └── .catkin_workspace
├── results
│   └── hello.txt
├── scripts
│   ├── entrypoint.bash
│   ├── run.ps1
│   ├── run.sh
│   └── setup.bash
├── Dockerfile
└── requirements.txt
```

### Dockerfile / requirements.txt

The first thing you should take a look at is the `Dockerfile`. Unlike the basic ros tutorial help, this one uses the `ros:noetic` base image. This is because here, we don't need the full ROS-desktop install, with the turtlesim examples and all that installed, we just need to run our code, for which we only need the bare bones.

After the base image is loaded, we install some dependencies. Specifically, I install Python in a way to make sure `requirements.txt` works as you'd expect. Any Python packages that are not ROS specific you might want to install as a dependency go here. I used numpy as an example, but you can also install sklearn and all that this way. Just make sure you are using the version for Python 3.8.

If a package is not specified in requirements.txt, it won't exist in the container, and your code won't be able to find it. If you never written one of these files before, check out the [docs](https://pip.pypa.io/en/stable/reference/requirements-file-format/).

After the Python dependencies are installed, we run copy over the catkin workspace and call `dos2unix`, this is to convert any files with windows-isms like `\r\n` newlines or `utf-8-sig` encodings to Unix standards. Note that this is still needed on Unix systems if you have teammates on Windows that might commit that stuff in. (Git has a setting that allows to to specify if you want to commit in Unix or Dos mode, but most people don't set it.)

Once everything is ready, we run `catkin_make`. If you haven't followed the ROS tutorial, you won't know what that is, but it doesn't matter either. It's just something we need to run to make ROS recognize this as a project. If we had C code in here, it would be compiled in this step.

Next up, you'll see that `chmod` is called. This is a Linux command that tells the OS that the specified file is allowed to be executed. ROS likes this to be true for some files in your workspace, and, because we are lazy (and are running this in an isolated container), we set it true for all files.

Then, commited-out lines source a few things into bashrc. You can uncomment these when debugging to make sure that the shell you are opening when running the code works, and that executables like `roscore` and `rosrun` exist.

Lastly, `entrypoint.bash` is set as the entrypoint. For the full project setup, this is needed to deal with some weird setup, but, for now, it's only for parity.

### catkin_ws

This directory contains the catkin workspace. The full documentation on how this is supposed to be structured can be found [here](http://wiki.ros.org/catkin/workspaces), but you don't have to bother. It contains the directory `src`, in which all code will live, and an empty `.catkin_workspace` that is there because catkin works that way.

In `src/` you'll find a package named "my_first_package". This is the package you'll be writing your code in. There is another package, called `data_files`, which is given to you. You can inspect it if you want, but you can use it as if it were a black box.

When writing code for ROS, all code needs to be in a package. This is an _ROS package_, not jet a Python package. Technically speaking, this directory could have C++ code in it.

In the root of `my_first_package`, you'll find three files: `CMakeLists.txt`, `package.xml`, and `setup.py`. These, you can completely forget. They are required to make this be a package, and that's all. The ROS tutorial explains a bit about how to use them. For example, you can add ROS dependencies and modules here, but all that is done for you already in the full example.

What matters more are the two directories: `scripts` and `src`. Everything in scripts is an executable: they should all be single files that call some code. We only need one, likely you'll too. Just know this is where the place is of the code that is being called by ROS.

`src/` is finally our Python package. I made it have the same name as the ROS package because that is imo the only clean way to do it. Here, you can write any code you want as you would in any other Python package, just remember to structure it with `__init__.py` files and all that. `setup.py` is installing this code as a proper Python package, so hacks importing via relative file paths and all that might not work. This package here can be imported in `scripts/` to run the code, but this is the place you should write it.

### scripts

The scripts directory might be a bit cryptic at first, but you should by now be familiar with all of one of these. The ones you should know are.

- `run.sh` and `run.ps1` are the scripts we used earlier to start docker.
- `setup.bash` you should already be familiar with from the `hardware_setup`. It's unused for this specific project (as we're only printing Hello World), but it is the file that contains the IPs the project needs to be able to connect to the different devices, specifically the phone of the robobo and the IP of your computer to find CoppeliaSim.

Lastly, there is `entrypoint.bash`. This is the script that the docker container will call to run your code. We need this separate bash script to make sure we have the entire environment ROS set up available, including all packages we have installed. As you can see, it calls the `my_node.py` script from the `my_first_package` package with `rosrun`, passing through any and all command line arguments passed into itself (That's what `"$@"` does).

### results

Once you have run the code, you'll find that the `./results` (relative to where you ran the project) is created. This is the directory you can save stuff to by importing `RESULT_DIR` and `FIGURES_DIR` from the `data_files` package. This is where your persistent files will live.

### References

This general package template is taken from [The Robotics Back-End tutorial](https://roboticsbackend.com/ros-import-python-module-from-another-package/). See that page for further documentation on it.
