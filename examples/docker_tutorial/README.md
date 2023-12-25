## An intro to Docker

If you have never touched it before, Docker is going to be a bit overwhelming to use, especially if you're on windows and never touched a Unix shell before. But, don't worry, tough there is a lot to learn about docker, you're not going to be needing a lot of it.


## What is Docker?

Docker is the profesional standard for running code. It creates an isolated envoirement that allows you to have reproducable systems to run in.

In short, it is a system to create small mini-computers on your computer. You can think of it as setting up a small computer, with an operating system and some installed software, on top of your current computer and operating system. Every docker container is a small mini-computer that behaves like it's a seperate machine running on seperate hardware.

If you've used virtual machines before, it's a bit like that. The difference between Docker and a VM is that a VM virtualises the *Hardware*, wheras docker virtualises the *Kernel*, this allows docker to be way faster and more lightweight than running a VM.

To run code in a container, then, is effectively to start up a seperate computer, and run your code in there. This is how you should think of it. First you `docker build`, which is to say you create this small computer, and then you `docker run`, which is to say you run the computer.

The reason this is useful is because of docker images. This is what you create when you `docker build`. They are, effectively, small mini-computers, that are completely set up and configured. you can create a file, called a `Dockerfile`, that specifies exactly how this sytem should be set up. It installs any needed software, applies all needed configuration, sets all envoirement variables, etc. Without docker, we'd have stuff like "Oh, this code works on my machine, but not on your machine, because I have ROS installed differently than you" or "I have this setting applied wheras you don't" or a meriad of other things. This way, the configuration of the operating system under which the code is run is completely reproducable. If the code runs in the container, it runs everywhere.

The main reason we use it is ROS noetic, which is a mess to install, and doesn't even work on MacOS. Instead of that, we just use the `ros:noetic` docker base image, which will have everything installed properly. 

## Installing Docker

The first thing that's important is installing it. What we are going to need is the [Docker engine](https://docs.docker.com/engine/install/), which can only be direcly installed on Linux. This is not an accident, Docker is Linux-based, and depends on it. Luckily enough, there exists the [Docker Desktop](https://docs.docker.com/desktop/), which is a software suite that will install the Docker engine for you on any operating system. It also comes with a ton more stuff, like a gui and Docker-compose, but we won't touch those here. We only care about the engine and the CLI.

#### Installing Docker Desktop on Windows.

To install Docker Deskop on Windows, first things first, you're going to need to install [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) (Windows Subsystem for Linux), for which you need to [enable hardware virtualisation](https://docs.docker.com/desktop/troubleshoot/topics/#virtualization) in your BIOS and Windows Control Panel. The supported versions are Windows 10 Home and Pro (22H2 or higher) and Windows 11 Home and Pro (21H2 or higher)

The full official installation guide can be found [at the Docker docs](https://docs.docker.com/desktop/install/windows-install/).

#### Installing Docker Desktop on MacOS

To instal Docker Deskopt on MacOS, you need to have MacOS 12 (Monterkey) or higher (that is, Ventura or Sonoma). If you do, you can just follow the installation instructions on [the Docker docs](https://docs.docker.com/desktop/install/mac-install/).

#### Installing Docker on Linux.
If you want to have an identical experience to the plebs using MacOS or Windows, and are on Fedora, Debian or Ubuntu, you can install Docker Desktop as per the [Docker docs](https://docs.docker.com/desktop/install/linux-install/).

However, on linux, on any distribution, you can also install the Docker Engine directly with the instructions [here](https://docs.docker.com/engine/install/). This won't install any graphical interface, but will also remove a layer of virtualisation, allowing you more control and insight over the docker images than when using Docker Desktop, meaning you can play with visual pass-trough and other things like that.

#### My OS is not supported?
If you are on an OS that is too old, or too niche, you have to install something that is suppored. You can run a VM, or dual-boot. Probably the easiest thing to quickly set up that is also easy to use for this course is to dual-boot [LMDE](https://www.linuxmint.com/download_lmde.php). This is based on Debian Linux, so you can google "How to X on Debian" and follow any command-line results to the letter, but also comes with a nice installer that's easy to set up for a dual-boot. 

## Using Docker

To use docker, type the command `docker` in the commandline. This is the only command we will use for this course. This should display a bunch of commandline options. If it doesn't, go back the the installation instructions.

If that command works, we need to make sure that the Docker Deamon (`docker.service` & `containerd.service`) is running before we can do anything else. This is something you need to do whenever you want to run any docker command.

On linux, you can start the docker daemon by running:
```sh
systemctl start docker
```

If you instead installed Docker Desktop, you should just open the gui of Docker desktop. We are not *using* this gui, but having it open makes sure everything is running.

If you want to, you can, at this point, go trough the [Getting Started Guide](https://docs.docker.com/get-started/). You don't have to, but this is the easiest place to go trough if you, at any point, feel like you're stuck on Docker and want to learn how it works.


### What you need to know

There are three things you need to know `docker build`, `Dockerfile` and `docker run`.

The Dockerfile is the thing that specifies what's inside the little computer. It specifies what to install, what configuration to set up, and all that.

Here is a small example, that installs ubuntu as a base, and then installs git on unbuntu:
```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Run apt-get to install git
# This has three stages:
#  * First, we update apt, to make sure we are installing the latest version of everything
#  * Second, we are installing git
#  * Third, we are removing some junk files that were created in the process. 
#    (You don'& have to bother about this third step)
RUN apt-get -y update && apt-get install -y git && rm -rf /var/lib/apt/lists/*
```

Notice how we don't need to specify `sudo`, we're in a seperate computer that is being run specifically for this task: we can run everything as root.

Create a file called `Dockerfile` (no extension), and paste this code in. Then, run:

```sh
docker build --tag my_first_container .
```

This will create this small computer under the name (tag) `my_first_container`, from the current directory, which is what the period stands for. 

We can then run the container with:
```sh
docker run my_first_container
```
This won't do anything. That makes sense, it'll just start up and, since we didn't specify it to do anything, shut down imediately after.

Let's instead tell it to echo hello world as an entrypoint, which is to say, we tell it that is the command it should run when started up.

```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Run apt-get to install git
RUN apt-get -y update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

# Specify what to run, in this case the `echo` command with as argument `Hello world`
CMD ["echo", "Hello World"]
```

When we now build this new container with `docker build --tag my_first_container .` and then run with `docker run my_first_container`, we'll see it print "Hello World"

---

Let's now say we want to view (`head`) this README in there instead of installing git. let's do:

```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Specify what to run, in this case the `echo` command with as argument `Hello world`
CMD ["head", "./REAMDE.md"]
```

Again, build with `docker build --tag my_first_container .` and then run with `docker run my_first_container`. 

This won't work. REAME.md might exist on your computer, but it doesn't exist on the container. We have to copy that file in from your computer to the container, and then view it:
```Dockerfile
# We base ourselves on Unbuntu. This is the base OS we are installing.
FROM ubuntu:20.04

# Let's CD to a more sensible working directory
WORKDIR /root/workdir

# Copy over the file from your own machine to the container
COPY ./README.md ./README.md

# Specify what to run, in this case the `echo` command with as argument `Hello world`
CMD ["head", "./README.md"]
```
Again, you build this new container with with `docker build --tag my_first_container .` and then run with `docker run my_first_container`.

You now know almost everything you need to know. There is one more thing that might end up fooling you: debugging.

---

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

Now, you are spawned inside a bash shell in your container. You can `apt-get install`, you can `cat`, `ls` and `cd` around the place, and do anything else you might want to do for troubleshooting.

This is a general pattern for debugging docker containers. You remove everything that breaks, you build, and then run in interactive mode to troubleshoot what is going on.