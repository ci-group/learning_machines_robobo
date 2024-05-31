## An Intro to Docker

If you have never touched it before, Docker is going to be a bit overwhelming to use, especially if you're on Windows and never touched a Unix shell before. But, don't worry, though there is a lot to learn about docker, you're not going to need a lot of it.

## What is Docker, and why do we use it?

Docker is the professional standard for running code. It creates an isolated environment that allows you to have reproducible systems to run in.

In short, it is a system to create small mini-computers on your computer. You can think of it as setting up a small computer, with an operating system and some installed software, on top of your current computer and operating system. Every docker container is a small mini-computer that behaves like it's a separate machine running on separate hardware.

If you've used virtual machines before, it's a bit like that. The difference between Docker and a VM is that a VM virtualizes the _Hardware_, whereas Docker virtualizes the _Kernel_, this allows Docker to be way faster and more lightweight than running a VM.

To run code in a container, then, is effectively to start up a separate computer, and run your code in there. This is how you should think of it. First you `docker build`, which is to say you create this small computer, and then you `docker run`, which is to say you run the computer.

The reason this is useful is because of docker images. This is what you create when you `docker build`. They are, effectively, small mini-computers, that are completely set up and configured. you can create a file, called a `Dockerfile`, that specifies exactly how this system should be set up. It installs any needed software, applies all needed configurations, sets all environment variables, etc. Without docker, we'd have stuff like "Oh, this code works on my machine, but not on your machine, because I have ROS installed differently than you" or "I have this setting applied whereas you don't" or a myriad of other things. This way, the configuration of the operating system under which the code is run is completely reproducible. If the code runs in the container, it runs everywhere.

The main reason we use it is ROS noetic, which is a mess to install, and works barely on Windows and not at all on MacOS. Instead of that, we just use the `ros:noetic` docker base image, which will have everything installed properly.

## Installing Docker

The first thing that's important is installing it. What we are going to need is the [Docker engine](https://docs.docker.com/engine/install/), which can only be directly installed on Linux. This is not an accident, Docker is Linux-based and depends on it. Luckily enough, there exists the [Docker Desktop](https://docs.docker.com/desktop/), which is a software suite that will install the Docker engine for you on any operating system (by installing a Linux VM or WSL and running docker in that). It also comes with a ton more stuff, like a GUI and Docker-compose, but we won't touch those here. We only care about the engine and the CLI.

During the installation process, you can use all default settings, and you can click "continue without logging in," as you don't need an account, either.

#### Installing Docker Desktop on Windows.

To install Docker Desktop on Windows, first things first, you're going to need to install [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) (Windows Subsystem for Linux), for which you need to [enable hardware virtualization](https://docs.docker.com/desktop/troubleshoot/topics/#virtualization) in your BIOS and Windows Control Panel. The supported versions are Windows 10 Home and Pro (22H2 or higher) and Windows 11 Home and Pro (21H2 or higher) If WSL is not installed, docker will install, but it won't work. We're not explicitly _using_ WSL, and you never have to open a WSL terminal. It's just that Docker needs it.

The full official installation guide can be found [at the Docker docs](https://docs.docker.com/desktop/install/windows-install/).

There is also this thing called a "Hyper-V backend" with Windows containers. We don't want that: we want the WSL2 backend, as we're running Linux containers.

#### Installing Docker Desktop on macOS

To install Docker Desktop on MacOS, you need to have MacOS 12 (Monterkey) or higher (that is, Ventura or Sonoma). If you do, you can just follow the installation instructions on [the Docker docs](https://docs.docker.com/desktop/install/mac-install/).

#### Installing Docker on Linux.

If you want to have an identical experience to the plebs using MacOS or Windows and are on Fedora, Debian, or Ubuntu, you can install Docker Desktop as per the [Docker docs](https://docs.docker.com/desktop/install/linux-install/).

However, on Linux, on any distribution, you can also install the Docker Engine directly with the instructions [here](https://docs.docker.com/engine/install/). This won't install any graphical interface, but will also remove a layer of virtualization, allowing you more control and insight over the docker images than when using Docker Desktop, meaning you can play with visual pass-trough and other things like that.

#### My OS is not supported?

If you are on an OS that is too old, or too niche, you have to install something that is supported. You can run a VM or dual-boot. Probably the easiest thing to quickly set up that is also easy to use for this course is to dual-boot [LMDE](https://www.linuxmint.com/download_lmde.php). This is based on Debian Linux, so you can google "How to X on Debian" and follow any command-line results to the letter, but also comes with a nice installer that's easy to set up for a dual-boot.

## Using Docker

To use docker, type the command `docker` in the command line. This is the only command we will use for this course. This should display a bunch of command line options. If it doesn't, go back the the installation instructions.

From this entire docker desktop thing you installed, you don't need any of its GUI elements. We only care about the command line executables.

If that command works, we need to make sure that the Docker Deamon is running before we can do anything else. This is something you need to do whenever you want to run any docker command.

If you installed Docker Desktop, you should just open the GUI of Docker Desktop. We are not _using_ this gui, but opening it (and then closing it,) makes sure it is running.

On linux, if you did not install the Docker Desktop, you can start the docker daemon by running:

```sh
systemctl start docker
```

If you want to, you can, at this point, go through the [Getting Started Guide](https://docs.docker.com/get-started/). You don't have to, but this is the easiest place to go through if you, at any point, feel like you're stuck on Docker and want to learn how it works.

### Basic docker commands: run and build

There are three things you need to know `docker build`, `Dockerfile`, and `docker run`.

The Dockerfile is the thing that specifies what's inside the little computer. It specifies what to install, what configuration to set up, and all that.

The first thing we need to say about this is that Docker does not like Windows `\r\n` (CRLF) line endings, and wants files to have `\n` (LF) line endings instead. This shouldn't be something you have to worry about, as files are converted (using a tool called `dos2unix` and a powershell command) for you in all scripts, but if anything inexplicable doesn't work and you are on Windows, checking the line endings (including those of your Dockerfile) is one of the first things you should try.

Here is a small example, that installs Ubuntu as a base, and then installs git on ubuntu:

```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Run apt-get to install git
# This has three stages:
#  * First, we update apt, to make sure we are installing the latest version of everything
#  * Second, we are installing git
#  * Third, we are removing some junk files that were created in the process.
#    (You don't have to bother about this third step)
RUN apt-get -y update && apt-get install -y git && rm -rf /var/lib/apt/lists/*
```

Notice how we don't need to specify `sudo`, we're in a separate computer that is being run specifically for this task: we can run everything as root.

Create a file called `Dockerfile` (no extension), and paste this code in. Then, run on your own OS's terminal:

```sh
docker build --tag my_first_container .
```

This will create this small computer under the name (tag) `my_first_container`, from the current directory, which is what the period stands for.

If you installed Docker Desktop, it will once again ask you to log in after building a container. Again, you can ignore this.

We can then run the container with (again on your main terminal):

```sh
docker run my_first_container
```

This won't do anything. That makes sense, it'll just start up and, since we didn't specify it to do anything, shut down immediately after.

Let's instead tell it to echo hello world as an entrypoint, which is to say, we tell it that is the command it should run when started up.

```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Run apt-get to install git
RUN apt-get -y update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

# Specify what to run, in this case, the `echo` command with as argument `Hello world`
CMD ["echo", "Hello World"]
```

When we now build this new container with `docker build --tag my_first_container .` and then run with `docker run my_first_container`, we'll see it print "Hello World"

---

Let's now say we want to view (`head`) this README in there instead of installing git and printing Hello World. let's do:

```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Specify what to run, in this case, the `head` command (print the first few lines of a file to the terminal) with as argument `./README.md`
CMD ["head", "./REAMDE.md"]
```

Again, build with `docker build --tag my_first_container .` and then run with `docker run my_first_container`.

This won't work. REAME.md might exist on your computer, but it doesn't exist on the container. We have to copy that file in from your computer to the container, and then view it:

```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Let's CD to a more sensible working directory
WORKDIR /root/workdir

# Copy the file from your own machine to the container
COPY ./README.md ./README.md

# Specify what to run, in this case, the `head` command (print the first few lines of a file to the terminal) with as argument `./README.md`
CMD ["head", "./README.md"]
```

Again, you build this new container with with `docker build --tag my_first_container .` and then run with `docker run my_first_container`.

You now know almost everything you need to know. There are two more things that might end up fooling you. The first: debugging.

### Debugging code inside containers

Let's say you made a typo in the Dockerfile, and wrote `COPY ./README.md ./REAMDE.md` instead, and let's say you don't spot this. How do you debug this?

First, let's remove the entrypoint from the docker container:

```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Let's CD to a more sensible working directory
WORKDIR /root/workdir

# Copy over the file from your own machine to the container, misspelled
COPY ./README.md ./REAMDE.md
```

Now, you can again `docker build --tag my_first_container .`, but we'll run it with:

```sh
docker run -it my_first_container bash
```

This will run the container in interactive mode with `-it` (Technically, it stands for something else, but don't bother about that), and it will launch a single command on startup: `bash`.

Now, you are spawned inside a bash shell in your container. You can `apt-get install`, you can `cat`, `ls`, and `cd` around the place, and do anything else you might want to do for troubleshooting. This is the previously mentioned point where being good at a Linux command line really pays off. The more debugging you can do while in here, the less cumbersome working with containers is. To exit a container you started like this and return to the terminal of your own OS, type `exit` and hit enter.

This is a general pattern for debugging docker containers. You remove everything that breaks, you build and then run in interactive mode to troubleshoot what is going on.

### Managing running containers

Similarly, for debugging, you might find yourself in a situation where the requested ports are already occupied. This means that the container is already running.

To see all running docker containers type on the terminal of your own OS:

```sh
docker ps
```

After that, you can shut down a container by typing:

```sh
docker container stop "container id"
```

Or, if you want to just kill all containers on Unix:

```sh
docker ps -q | xargs docker container stop
```

### Passing commandline arguments to the command we run inside the container

The last thing that still needs to be explained is how to pass command line arguments from your shell to the docker container. This is quite complicated, but, luckily enough, you only need to understand the oversimplified version: instead of `CMD`, we are going to be using `ENTRYPOINT`.

```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Specify what to run, in this case, the `echo` command. What we are echoing is going to depend on what we pass through the command line.
ENTRYPOINT ["echo"]
```

You can build this again as usual with `docker build --tag my_first_container .` After that, you can run it with:

```sh
docker run my_first_container "Hello World"
```

As you can see, all arguments after the container name are passed to the entrypoint before running.

### Deleting built images

Every now and then, you might want to delete all the images you have build in the past. They don't take up a lof of space, and you will usually overwrite them anyway, but it's good to clean stuff up every now and then.

To do this, run:

```sh
docker container prune
```

### Docker permission issues.

If you're on macOS or Linux, you might find yourself with docker permission issues. Errors like: `ERROR: failed to solve: ros:noetic: error getting credentials - err: exit status 1, out: `, or having to always call docker with `sudo`. There are several things you might need to do to fix this. The first command you should try running on your terminal is:

```sh
ls -la $HOME | grep .docker
```

If that says the file is not owned by your user (e.g. you don't see something resembling your username in the output), you might be able to fix your issues by running:

```sh
sudo chown -R $(id -u):$(id -g) $HOME/.docker
```

If that doesn't work. You should really just google and, if you cannot find anything, ask. Docker can be hard to install, and this not working might happen depending on how you specifically set up your own computer.

### Running with Apple Silicon

Running docker with a machine on Apple silicon, so far, has just worked. But, later on in the course, it won't. Because of that, in `full_project_setup`, we provide you with another script: `run_apple_sillicon.zsh`, in which we add one flag: `--platform linux/amd64`, which specifies to `buildx` to run or build the container under the `x86` (amd64) CPU architecture (and virtualise that when needed).

To get all this running, first of all, enable experimental features in docker desktop. It's in the settings somewhere, but they move it around so much it's hard to tell you exactly where to look.

After you have enabled experimental features (and maybe restart your machine after that,) you can run in your terminal:

```zsh
docker buildx create --use
```

Another issue you might run into with docker on Apple Scillicon is:

```
Operation not permitted (src/thread.cpp:281)
qemu: uncaught target signal 6 (Aborted) - core dumped
```

This is usually caused by Docker desktop trying to run your code multi-threaded somehow, but the Docker virtualisation layer not allowing for this. For this, go into the docker desktop settings, Resources, and then make sure the amount of CPUs Docker Desktop is allowed to use is set to 1.

After that, the `--platform linux/amd64` flag is going to work on your system, and you can use the `run_apple_sillicon.zsh` script to run instead of the `run.sh` script. Macs with intel processors are not affected by this.

### Looking ahead

In `run.sh` / `run.ps1`, which are the scripts you'll use to start docker for the full project setup, we use a bunch more flags and things. These, you don't have to worry about. However, here is a quick summary of what they are, and how we use them:

- `-v [host_path]:[container_path]` mount a volume. Basically, this allows you to have some directory on your own system to which the container can read and write. This is used such that you can save your results on your own system but should be used with care, as having two operating systems mount the same file system can cause unexpected issues, especially on Windows.
- `-p [host_port]:[container_port]` Expose or link a TCP port from the container to your host. Used for talking with the robot.
- `--rm` Remove any container of the same name that is already running.
- `-t` Allocate a pseudo-TTY. Without it, some print functions from the container wouldn't show up on your own terminal when running.
