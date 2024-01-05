## Getting the hardware running

First install the app. Make sure it is [this one](https://play.google.com/store/apps/details?id=com.mytechia.robobo.app.ros.robobodeveloper). While testing, we has some issues running it on android versions later than android 10, but this appears to be fixed upstream now, and the OS of the phone you've gotten should be supported.

After that, use Bluetooth to connect the robot to the phone, and connect the app. Start it up, and let the robot sit somewhere running.

Then, to be able to inspect if everything is working, the dockerfile in this directory builds to something gives you the command line you need. This dockerfile is quite complex, and uses stuff from `catkin_ws`. You don't have to touch any of this, or understand what it does.

As always when running docker, make sure the docker daemon is running before running any commands.

build it with:

```sh
docker build --tag hardware_setup .
```

run it with:

```sh
docker run --rm -it hardware_setup bash
```

Make sure the system you are running the container on is on the exact same network as the phone (note, public networks like Eduroam won't work.), and observe the IP adress shown on the top left of your phone screen. The robobo UI has a problem where it sometimes cuts of too long IP adresses. You can scan for all active hosts on adresses using nmap (which you need to install as per your operating system). Simply enter all groups you can see, and then scan like so: `nmap -sn "192.168.0.*"` (Note that nmap is a pentesting tool and should be used responsibly.)

```sh
curl http://[Adress shown on top left]:11311

# for example:
curl http://192.168.0.1:11311
```

This should show:

```
Empty reply from server
```

If it says something like:

```
Failed to connect to 192.168.0.1 port 11311: No route to host
```

The system is not currently running as it should, and you are probably not on the same network, or you entered the wrong IP.

This address (everything you put after the `curl` command) will from now on out be known as the `ROS_MASTER_URI`, and there are a few places where you will have to specify it.
If you are wondering, this is the local IP address of the phone, followed by the port number roscore (the master node of ros) is listening on.

---

Once this is working, you can update the setup.bash file in the scripts directory. Currently, it contains this line:

```sh
export ROS_MASTER_URI=http://localhost:11311
```

You should update it with the local IP address you just used. For example:

```sh
export ROS_MASTER_URI=http://192.168.0.1:11311
```

After this, you should re-build and re-run the docker container. Once that is done, you can run this to check if it was set correctly. (it should show the value you just set it to)

```sh
echo $ROS_MASTER_URI
```

If it is, you can run:

```sh
rostopic list
```

This will show you a long list of topics that your robobo robot is currently publishing data on and listening to data on.

or:

```sh
rostopic echo /robot/accel
```

To give you real-time updates in your shell on the accelerometer of your phone. (make sure the robot isn't sleeping.)
You don't have to understand these `rostopic` commands, we won't use them for anything, but, in case you are curious or want to use them for debugging, you can find how to use them at the ROS tutorial example.

After this, you should be able to run (again, commands you don't have to understand):

```sh
rosservice call /robot/talk "text: {data: "Hello"}"
```

Which should make the text-to-speech of the robot say "hello".

Lastly, you should try running:

```sh
rosservice call /robot/moveWheels "lspeed: {data: 100}
rspeed: {data: 100}
time: {data: 1000}
unlockid: {data: 1}"
```

This command should make the robot move foreward for a second. If everything prior to this (including the text-to-speech) worked, but this one failed, you should go to the lab and get a new robobo. Some of them are broken and cannot move.

If all this is working, you have now successfully setup everything, and can properly start working on this project.
