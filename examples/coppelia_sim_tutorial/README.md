# CoppeliaSim tutorial

For this course, we use CoppeliaSim to simulate the robot. This is not the easiest program to use or install, so this should serve as a short tutorial on how to install and run it.

First, download the edu version of CoppeliaSim from their [website](https://www.coppeliarobotics.com/downloads). Click "edu" and then select your operating system. On Windows, use the zip package without an installer. On Mac, everything should be fine by default, just select the right version for your hardware. For Linux, it says "Ubuntu," but the package ships with most of its dependencies, so it will run on any distro with some minor troubleshooting. (Tested Debian, Arch and Fedora) On Arch, to get a smooth experience, the only dependency you might want to install is [`icu60`](https://aur.archlinux.org/packages/icu60), on Fedora, you want to install [`libbsd`](https://packages.fedoraproject.org/pkgs/libbsd/libbsd/).

One thing to note when running on Linux is that there have previously been unexpected issues with the program when running on Wayland. Exactly why this is the case, we don't know. But, on Debian KDE with Wayland, it doesn't start at all, and on Fedora Gnome with Wayland, it randomly crashes every now and then. If you experience weird issues, consider switching to X11.

#### Windows / Linux post-download

On Linux and Windows after it is all downloaded, you will find yourself with a zip file. When extracted, this will expose quite a large amount of scripts and executables you might want to run, so we are going to extract it to a location we have access to from the terminal, specifically `./CoppeliaSim` such that you can run the exact same commands as the ones in this README. This exact path is also required because some startup scripts, later on, rely on it being in this location.

If you're wondering what extracting to `./CoppeliaSim` means here, it is to say that you should extract to a new directory called "CoppeliaSim" that is located in the directory this README is in.

#### Mac post-download

On MacOS, after it is downloaded you will have a `coppeliaSim.app` file. Move this file to the directory this README is in, (to `./coppeliaSim.app`) so the commands below work. Note that MacOS might tell you it cannot verify the integrity of the app and refuse to run it. To give it permissions anyway, open finder in the current directory (with `open .`), and control-click the application to show the menu that lets you overwrite these settings.

### Running CoppeliaSim

#### The python venv

The first thing to do when running CoppeliaSim is to create a Python virtual environment and install some requirements on it. Technically speaking, we don't need it just jet, but CoppeliaSim gives you nice red error messages if you don't, and those can be confusing.

I am going to presume you know what a venv is, but, if you don't, you can just use the [default `venv` module](https://docs.python.org/3.8/library/venv.html), as we don't need any of the fancy features of poetry or anaconda. (In fact, Anaconda specifically is known to cause issues, and I recommend using venv or Virtualenv.) So, just `python3.8 -m venv .venv` and then `source ./venv/bin/activate` or `.\.venv\Scripts\Activate.ps1`. After this, the Python modules you need are already in `requirements.txt`, so you can just run `python -m pip install -r requirements.txt` to install those.

#### Running the executable

By default, running CoppeliaSim is as easy as just running the following from a terminal: (You should always run it from the command line, never by clicking an executable in a graphical interface.)

```sh
# On Windows:
./CoppeliaSim/coppeliaSim.exe

# Or, on MacOS:
./coppeliaSim.app/Contents/MacOS/coppeliaSim

# This one should usually work on Linux.
./CoppeliaSim/coppeliaSim

# You might need to instead run the shell script that launches this executable.
# This fixes some unexplained issues sometimes:
./CoppeliaSim/coppeliaSim.sh
```

If these commands say that the file does not exist, either you're not in the correct working directory with your terminal, or you did not extract CoppeliaSim to the correct location.

For the full startup options, please refer to the [docs](https://www.coppeliarobotics.com/helpFiles/en/commandLine.htm).

The executable takes several command line arguments, one of them being the scene to load. So, to load CoppeliaSim with a scene, for example, the `./scenes/Robobo_Scene.ttt`, you can simply run `./CoppeliaSim/coppeliaSim ./scenes/Robobo_Scene.ttt` or similar for your OS.

If all this worked, you have installed CoppeliaSim correctly. Just copy-paste this `CoppeliaSim` directory (or, on macOS, `coppeliaSim.app` file) around, from this example directory to the `full_project_setup` example to your own project directory to make sure it's available everywhere.

You can now open it, and click and move around a bit. It's admittedly a rather awkward UI, but you'll need to be somewhat familiar with it.

One thing you'll notice is little text/script icons next to some nodes in the Scene, mostly on the Robobo. These are Lua scripts that are running on the CoppeliaSim side. Double-click the script symbol to open them.

### Running with the shell scripts

To make running CoppeliaSim a bit more ergonomic, we provide some really simple shell scripts to call CoppeliaSim. In some sense, they are nothing more than some documentation on what arguments you might want to pass. You can call these like `bash ./scripts/start_coppelia_sim.sh` on Linux, `zsh ./scripts/start_coppelia_sim.zsh` on MacOS, and `.\start_coppelia_sim.ps1` on Windows. The arguments you can pass are:

- (required positional first argument) the scene you want to load, which is to say, some `.ttt` file.
- (second positional argument) The TCP port to host the ZMQ Remote API on. Which is to say, the port your Python code will use to connect to it. Passing this can be useful for training your controllers, as CoppeliaSim usually uses around 2 cores, so any system with more than that can get an advantage by running multiple in parallel, which means connecting over different ports.
- (only flag argument, but needs to be after the positional arguments) -h. If passed, CoppeliaSim will run in headless mode (which is to say, without a graphical interface,) this is also useful when training your controllers, as it makes the sim take fewer resources. CoppeliaSim itself indeed also accepts a `-H` for "true headless" mode, but beware: on some systems, this might cause issues with the camera not giving out any image.

## Lua

Wait? We are learning an entirely new programming language? Well, yes. But, don't worry. Lua is a language that is designed specifically to be easy to pick up. You'll come across it more often if you end up doing professional software development. It's a programming language designed to write config files and small add-ons in. I usually call it "sentient json" for that reason. It's famously easy to interface with from C and quite fast. For example, it's the configuration / modding language of choice for games like World of Warcraft, Roblox and Factorio, and development tools ranging from Neovim to Redis to MediaWiki (the backend of Wikipedia and WikiData). Most people who write it are just making config files, and don't really know the language either. Just... bluff your way through this one. Just [read the Wikipedia page](<https://en.wikipedia.org/wiki/Lua_(programming_language)#Features>), and copy that when you need a loop, if-statement of similar. The standard library is effectively nonexistent, and the syntax is extremely minimal, without support for classes or overloading or any other non-essential feature, so everything the language has to offer is in those few examples.

Technically, CoppeliaSim also supports Python scripts. However, this has only been since recently, and we didn't get that working in time for the course. You can try to play around with that if you want. However, I encourage you to use Lua, as that is what the Robobo itself uses, meaning you can peek at that code whenever you need to see how to get something working.

### The used Lua scripts

In this repository, you'll find the Lua scripts used in this course. The Robobo scripts are (for this course modified) versions of what is officially supplied for the Robobo. They have comments and variable names in Portuguese and generally overuse global variables. However, they are the best we have.

Next to the Robobo scripts, you'll find a `food.lua` script under the arena. This is the script used for food for task 3. You're encouraged to modify this if and when you want to change the behaviour of the food.

## Talking to CoppeliaSim from Python

For this course, most of the Python code that talks to CoppeliaSim directly is abstracted away, so you're not really required to understand how to do it. However, it might still be good to get a Hello World of sorts running, just so you know what it is doing, and to make sure that it all works. If you want a more detailed explanation of what is going on, the official source code, documentation, and examples can be found on the [GitHub repo](https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python).

Let's create a Python file `send_commands.py` like this:

```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


client = RemoteAPIClient(
    host="localhost",  # This is just to say, "this same computer"
    port=23000,  # The default port CoppeliaSim launches the ZMQ API at.
)
# This is Lua nonsense. We are gathering a global Lua object called "sim",
# and calling python functions on *that*.
# You can ignore it, and pretend we created a "sim" object.
sim = client.require("sim")

# Start the simulation
sim.startSimulation()

# Wait for 5 second
sim.wait(5)

# Stop the simulation
sim.stopSimulation()
```

After this, you can run CoppeliaSim (e.g. `./CoppeliaSim/coppeliaSim.sh ./scenes/Robobo_Scene.ttt`) in your venv, and, in a new terminal in the same venv run `python send_commands.py`. You should see the simulation starting (but still nothing moving, as we're not telling any robot to do anything,) and then stopping again. Nothing special.

## Troubleshooting

If something isn't working correctly (So, kind of working, but with errors), the first thing to run is its dependency error checker. This will report if all dynamically linked libraries are available (`.dll` on Windows, `.so` on Unix). Its name is `libLoadErrorCheck`, and where it lives depends on your OS and installation method. Just searching for a file of this name in the directory you extracted CoppeliaSim to should let you find it.

If this script fails on MacOS and Windows, you have found the problem, but all you can do is try to re-download and re-extract everything, and if that doesn't work, your OS is having problems and should be troubleshoot on a case-by-case basis. You should try googling some error messages.

If you really cannot get it working (and this also applies to Linux,) you can always resort to the [advanced usage](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/coppelia_sim_tutorial#advanced-usage---docker) Docker running method of CoppeliaSim. This is not ideal, as it does not give you the option of seeing the GUI at all, but it at least gives you the option to run the code to make sure it does _something_. This is especially useful if there are some minor details you cannot get running (e.g. The camera won't manage to send you images on your machine, sending only black instead.)

If the script fails on Linux, you can try to install the `.so` file yourself. It might be that your distribution simply doesn't ship it by default, and it can just be installed through your package manager. If that doesn't work, you can find a working version of the file elsewhere, and put it in `./CoppeliaSim/` (Maybe modifying `$LD_LIBRARY_PATH` to get it recognised). Wrince and repeat with recursive dependencies (Find those with `ldd libSomething.so.0`)

For example, if you get this common error:

```
[Connectivity >> ZMQ remote API server@addOnScript:error]   plugin simZMQ: Cannot load library [PATH]/CoppeliaSim/libsimZMQ.so: (libbsd.so.0: cannot open shared object file: No such file or directory)
```

You should be able to read that `libbsd` does not exist on this system. This error happens on Fedora because it doesn't ship with `libbsd`. Installing it (`sudo dnf install libbsd`) solves the issue.

This is a recursive dependency, so `libLoadErrorCheck` finds nothing, but `ldd ./CoppeliaSim/libsimZMQ.so` reports the `libbsd.so.0 => not found`

Note that `ldd ./CoppeliaSim/*.so | grep "not found"` is likely to find missing objects regardless. As long as there are no errors, it's not trying to load or use those missing libraries, so it's not a problem.

## Advanced usage - Docker

At some point, you might want to start CoppeliaSim inside a container. This can be because you want to train your simulation on a server somewhere, or because you simply cannot get it running on your system. For this, you can use the [official CoppeliaSim docker image](https://github.com/CoppeliaRobotics/docker-image-coppeliasim) as a guide, or you can use the one provided in this repository, which is effectively identical, but comes with runscripts to help you play around with it, if you had issues with that.

To get access to these, you have to re-build the examples (as will be the case for all "advanced usage" sections,) This deletes any changed files inside the `examples/` directory, so make sure you don't have any files saved here before running this. To re-build cd into `maintained/` of this project, and run `python3 build.py --advanced_coppelia_sim` from the `maintained/` directory.

After you have run this, you will find that the runscripts have changed to `_docker` variants, where it should be noted that (like `run.*` from the full project setup,) MacOS users with intel chips should use the `.sh` version of the runscript, the same as Linux, and MacOS users with Apple Sillicon should use the `apple_sillicon.zsh` version.

Once this example has been rebuilt, you should download the `Ubuntu 22.04` version of CoppeliaSim, and rename it to `./CoppeliaSim.tar.xz`, to make the naming consistent. After this, you should be able to call the runscripts, which will launch a headless instance of CoppeliaSim inside a freshly built container. You can test if this worked by creating and running the `send_commands.py` from earlier again.
