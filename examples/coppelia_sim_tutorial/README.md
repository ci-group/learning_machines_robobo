# CoppeliaSim tutorial

For this course, we use CoppeliaSim to simulate the robot. This is not the easiest program to use or install, so this should serve as a short tutorial on how to install and run it.

First, download the edu version of CoppeliaSim from their [website](https://www.coppeliarobotics.com/downloads). Click "edu" and then select your operating system. For linux, it says "Ubuntu," but the package ships with most of its dependencies, so it will run on any distro. (Tested Debian, Arch and Fedora) On Arch, to get a fully smooth experience, you the only dependency you might want to install is [icu60](https://aur.archlinux.org/packages/icu60). On windows, use the zip package without installer. On mac, everything should be fine by default.

After it is all downloaded, you will find yourself with a zipfile. When extracted, this will expose quite a large amount of scripts and executables you might want to run, so you should extract this to a location where you think you'll have access to it from the commandline. Personally, I extracted it to `./CoppeliaSim` in the projects' working directory, so next to `catkin_ws`. All example commands presume you did this, too.

By default, running CoppeliaSim is as easy as just running (You should always run with SHELL=true, which is to say, from the commandline):

```sh
# This one should usually work.
./CoppeliaSim/coppeliaSim

# You might need to instead run the shell script that launches this executable.
# This fixes some unexplained issues sometimes:
./CoppeliaSim/coppeliaSim.sh
```

For the full startup options, please refer to the [docs](https://www.coppeliarobotics.com/helpFiles/en/commandLine.htm).

This will complain there is no ZMQ or Zero-MQ library available. This is expected, you did not install that, and likely won't.

If you want to make sure you installed things correctly, the first thing to run is its error checker. This will report if all dynamically linked libraries are available.

```
./CoppeliaSim/libLoadErrorCheck.sh
```

For this course, we are accessing this simulator from python code, meaning we need to open a TCP port to connect to. For this, we use port 19999. This is specified on a script on the robot itself, which starts the 

If you want to not have to press the play button and such yourself, you can do this by installing the ZMQ plugin to CoppeliaSim from the [github](https://github.com/CoppeliaRobotics/zmqRemoteApi), to then interact with the simulator trough code like [described in the docs](https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm), however, this requires compling C++ code with cmake (I cannot find any binaries), which might be a bridge too far if you're on Windows.

### Lua scripts

Wait? We are learning an entirely new programming language? Well, yes. But, don't worry. Lua is a language that is designed specifically to be easy to pick up. You'll come across it more often if you end up doing profesional software developlement. It's a programming language designed to write config files and small add-ons in. I usually call it "sentient json" for that reason. It's famously easy to interface with from C, and quite fast. For example, it's the configuration / modding language of choise for games like World of Warcraft, Roblox and Factorio, and development tools ranging from Neovim to Redis to MediaWiki (the backend of Wikipedia and WikiData). Most people who write it are just making config files, and don't really know the langauge either. Just... bluff your way trough this one. Just [read the wikipedia page](https://en.wikipedia.org/wiki/Lua_(programming_language)#Features), and copy that when you need a loop, if-statement of similar. The standard library is effectively nonexistant, and the syntax is extremly minimal, without support for classes or overloading or any other non-essential feature, so everything the language has to offer is in those few examples.
 