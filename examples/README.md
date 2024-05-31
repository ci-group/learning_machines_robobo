## Intro

I am going to need to introduce the code for this course a little. You see, this course kind of has a fundamental flaw: It's a robotics course given to AI students, where the robotics is kind of meant to stay in the background. The problem is that robotics is _hard_, and requires a whole new set of tooling you will likely have never touched before.

If you are the type to first implement/install stuff and understand later, you can follow any of the quick setup files in the full project folder, which you can find for [Windows](https://github.com/ci-group/learning_machines_robobo/blob/master/examples/full_project_setup/quick_setup_windows.md), [MacOS](https://github.com/ci-group/learning_machines_robobo/blob/master/examples/full_project_setup/quick_setup_macos.md), and [Linux](https://github.com/ci-group/learning_machines_robobo/blob/master/examples/full_project_setup/quick_setup_linux.md). These will guide you through the entire installation of the framework you will use throughout the course, but bare in mind you might not have the prerequisites to directly follow it.

Robotics generally uses something called ROS: Robotics Operating System. There are a few closed-source proprietary alternatives to ROS, but this is the stuff you'd be taught in the first year of a robotics Bachelor. ROS is going to be very different from the software you've touched before. It is a framework for writing C++ robotics packages that run on top of the Linux kernel. There is _some_ second-party support for Python and "experimental" releases for Windows, but you're supposed to develop C++ on Linux. (Yes, ROS2 has support for Windows and MacOS now, but we're stuck on ROS1)

Most of you, however, are AI students, some might have come here from a Bachelor's that didn't involve any programming and would be completely overwhelmed if I started talking about how variadic templates influence rvalue casts because of return value optimization in the latest version of Clang. So, we're not doing any of that. We're doing no C++ and we have support for Windows, MacOS, and, of course, Linux.

This doesn't come for free, of course. I haven't taken it upon myself to write ROS1 support for Windows and MacOS, and I didn't manage to update to ROS2 either. No, we're using virtualization. Specifically, we're going to be using Docker. This is going to be making parts of this course a bit inconvenient, as you have to build and start docker containers all the time. But, keep in mind: the alternative is forcing all of you to Linux.

The second hard thing we're using for this course is CoppelaSim. This is actually pretty alright to use, all things considered. It should run well on all operating systems (though we had issues with macs in previous years), and has native Python bindings (though we're stuck with the Lua bindings for internal scripts). In either case, it _is_ a rather large software package you're likely unfamiliar with, meaning you will need to spend some time to get familiar with it.

With that out of the way, and you hopefully understanding why parts of this course are a bit inconvenient (I don't want to hear no "but why do we have to use Docker?"), I can start introducing the stuff you need for this course to you. First, there are some small blurps on technologies you might have not touched before but should learn to use for this course. Just a little help if this is the first time having to do this kind of stuff. After that, this README will redirect you to the example projects in this tutorial. These will guide you through setting up and learning everything you need to know to start working on the assignments of this course.

Before we start some notes:

- All code in the examples is fine to copy-paste into your assignment project. In fact, you are _encouraged_ to do so. There is a lot of Python code, Dockerfiles, and CMakeLists you're not expected to understand. Copy-paste it in, and don't worry about it.
- Setting everything up and going through this thing is going to take you some time. Don't expect to be done in an hour, and don't be afraid to ask questions. Though we tried our best to keep it simple, the nature of robotics software means that setting everything up might be one of the hardest parts of this entire course, especially if you don't have a background in lower-level development.
- Don't be afraid of not knowing. One of the most important skills for software development is having a high tolerance for confusion. If you want to, you can spend some extra time to learn everything properly, but a "this seems to be working" is perfectly fine.

### The command-line

Wait, seriously? There is a blurp on the command line? Well, yes. Obviously, you have opened a command line before, but, for some of you, this might be the first time using applications that don't have GUI support _at all_. You're going to need to connect to remote (Linux) systems that don't have a screen attached, meaning you're going to need to `cd` and `ls` around the place; use `less` and `cat` to inspect a file; and `nano` (or `vim` if you're as big of a nerd as me) to edit something. This also means that your IDE's debugger and "run" button are not going to work. Use your terminal. If that's not something you're comfortable with, don't worry: it's not hard, and you'll learn as you're doing it. But, I want you to keep in mind that you cannot avoid it, and shouldn't try to. If you're on Windows make sure you're on PowerShell, not CMD. Also, install a nice terminal theme. It's a lot comfier here with pretty colors and a proper font.

### Python

We are using Python version 3.8, this is because of the limitations of ROS. Make sure that your everything is tested for Python 3.8, and you're not confusing yourself by using newer versions when looking at documentation or testing small scripts. We're also using Python like an embedded programming language, meaning that stuff like Jupiter notebooks are not going to work and that you're going to have to structure your Python code in modules, as you're not calling the code yourself. The code will be installed, after which another program will call it for you.

### Docker

For this course, you are going to use Docker. This is the professional standard for running software. If you've ever come close to professional software development, you will have used it before. But, in short, it is a way to quickly set up and install things that are hard to install and a way to isolate what you are running from the rest of your computer, making it completely reproducible.

If you've never used it before, it's going to be a similar experience to the first time using git. There are a _ton_ of tutorials online ranging from five minutes to ten hours; there are quite some GUIs available, but, professionally, everyone uses the command line; and it naively runs on Linux, but getting it to work on Windows or Mac is a bit of a hastle and requires jumping trough some hoops.

Of course, this course won't require you to learn all of docker. We tried our absolute best to make sure you have to touch as little of it as possible, but every time you want to run your code you will do so by typing a docker command in the command-line.

If you have never used Docker before, or have very little experience with it, you can check out the small tutorial on basic Docker usage that is in this examples directory. [Docker Tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/docker_tutorial). This will also tell you what you need to have installed, and how to install that.

### ROS

For this course, we are using ROS. Again, you don't have to learn anything about it, but you might still find it pleasant to follow a small tutorial on it. There is a basic ROS setup in the examples directory that is meant to get you started. [ROS tutorial help](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_tutorial_help)

This is especially useful if you're not that good at programming jet, and aren't able to intuit what code does by relying on experience.

### Using the Robobo

Getting the hardware running is not as easy as it might seem. There is a quick tutorial in the examples directory to get you started [Getting the hardware running](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/hardware_setup)

### Using CopellaSim.

Getting CopellaSim running isn't trivial either. You know the deal, tutorial in examples. [Setting up CopellaSim](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/coppelia_sim_tutorial)

### Full project example

Finally, we have reached the point where you can start working on the project. In the examples directory, there is a fully working project that has everything you need. [Full project example](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/full_project_setup), this is the place you'll likely end up copy-pasting the most code from. In fact, I encourage you to copy-paste everything and use it as a template.

Just running this one will get you a basic project that makes the robot move around a bit, both in the simulation and with the actual hardware.

You might (are going to) find the code in this example a bit overwhelming. At [ros_basic_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup), there is a simpler version that doesn't do anything but does have the minimum required structure to run some code. Take a look through that one every time you are confused about what something in here does, or why it's there, especially when it comes to the shell scripts and other odd files.
