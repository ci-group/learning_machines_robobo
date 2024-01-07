# CoppeliaSim tutorial

For this course, we use CoppeliaSim to simulate the robot. This is not the easiest program to use or install, so this should serve as a short tutorial on how to install and run it.

First, download the edu version of CoppeliaSim from their [website](https://www.coppeliarobotics.com/downloads). Click "edu" and then select your operating system. For Linux, it says "Ubuntu," but the package ships with most of its dependencies, so it will run on any distro. (Tested Debian, Arch and Fedora) On Arch, to get a fully smooth experience, the only dependency you might want to install is [icu60](https://aur.archlinux.org/packages/icu60). On Windows, use the zip package without an installer. On Mac, everything should be fine by default.

One thing to note when running on Linux is that there have previously been unexpected issues with the program when running on Wayland. Exactly why this is the case, we don't know. But, on Debian KDE with Wayland, it doesn't start at all, and on Fedora Gnome with Wayland, it randomly crashes every now and then. If you experience weird issues, consider switching to X11.

After it is all downloaded, you will find yourself with a zip file. When extracted, this will expose quite a large amount of scripts and executables you might want to run, so you should extract this to a location where you think you'll have access to it from the command line. You should extract this to `./CoppeliaSim` such that the commands below work. This exact path is required because some startup scripts, later on, rely on it being in this location.

By default, running CoppeliaSim is as easy as just running (You should always run with SHELL=true, which is to say, from the command line):

```sh
# This one should usually work.
./CoppeliaSim/coppeliaSim

# Or, on Windows:
./CoppeliaSim/coppeliaSim.exe

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

For this course, we are accessing this simulator from Python code, meaning we need to open a TCP port to connect to. For this, we start CoppeliaSim with the `-gREMOTEAPISERVERSERVICE_19999_FALSE_TRUE` flag. You don't have to remember this, you'll find `./scripts/start_coppelia_sim.sh` (and `".ps1`), which starts the program with this flag, too.

This script also takes one argument, the scene to load. So, to load CoppeliaSim with a scene, for example, the `./scenes/Robobo_Scene.ttt`, you can simply run `./scripts/start_coppelia_sim.sh ./scenes/Robobo_Scene.ttt`. This will open everything for you, ready to use.

If all this worked, you have installed CoppeliaSim correctly. Just copy-paste this `CoppeliaSim` directory around, from this example directory to the `full_project_set` example to your own project directory to make sure it's available everywhere.

You can now open it, and click and move around a bit. It's admittedly a rather awkward UI, but you'll need to be somewhat familiar with it.

One thing you'll notice is little text/script icons next to some nodes in the Scene, mostly on the Robobo. These are Lua scripts that are running on the CoppeliaSim side. Double-click the script symbol to open them.

## Lua

Wait? We are learning an entirely new programming language? Well, yes. But, don't worry. Lua is a language that is designed specifically to be easy to pick up. You'll come across it more often if you end up doing professional software development. It's a programming language designed to write config files and small add-ons in. I usually call it "sentient json" for that reason. It's famously easy to interface with from C, and quite fast. For example, it's the configuration / modding language of choice for games like World of Warcraft, Roblox and Factorio, and development tools ranging from Neovim to Redis to MediaWiki (the backend of Wikipedia and WikiData). Most people who write it are just making config files, and don't really know the language either. Just... bluff your way through this one. Just [read the Wikipedia page](<https://en.wikipedia.org/wiki/Lua_(programming_language)#Features>), and copy that when you need a loop, if-statement of similar. The standard library is effectively nonexistent, and the syntax is extremely minimal, without support for classes or overloading or any other non-essential feature, so everything the language has to offer is in those few examples.

Technically, CoppeliaSim also supports Python scripts. However, this has only been since recently, and we didn't get that working in time for the course. You can try to play around with that if you want. However, I encourage you to use Lua, as that is what the Robobo itself uses, meaning you can peak that code whenever you need to see how to get something working.

### The used Lua scripts

In this repository, you'll find the Lua scripts used in this course. The Robobo scripts are (for this course modified) versions of what is officially supplied for the Robobo. They have comments and variable names in Portuguese and generally overuse global variables. However, they are the best we have.

Next to the Robobo scripts, you'll find a `food.lua` script under the arena. This is the script used for food for task 3. You're encouraged to modify this if and when you want to change the behavior of the food.

### Writing your own Lua script for Python.

Writing / calling your own Lua scripts is quite easy, but it requires a bit of a weird setup.
In `full_project_setup`, in the `robobo_interface` package in the `simulation.py` file, you can find some examples of lua code being called from Python. To create a lua function to call from Python, you should use this signature:

```lua
theFunctionName = function(inIntegers, inFloats, inStrings, inBuffer)
    -- Do something here

    -- If you don't want to return a byte buffer, you can replace it with an empty string,
    -- In Lua, the empty string and empty bye array are the same object.
    return { output, integers }, { output, floats}, { output, strings}, outByteBuffer
end
```

The type signature from the Python side will look like this:

```python
from coppelia_sim import sim

out_ints, out_floats, out_strings, out_byte_buffer = sim.simxCallScriptFunction(
    connection_client_id,
    "Name_Of_Node",
    sim.sim_scripttype_childscript,
    "theFunctionName",
    [any, input, integers],
    [any, input, floats],
    [any, input, strings],
    bytearray(), # An input byte-array
    sim.simx_opmode_blocking,
)
```
