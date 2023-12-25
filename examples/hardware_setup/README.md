## Getting the hardware running

First install the app. Make sure it is [this one](https://play.google.com/store/apps/details?id=com.mytechia.robobo.app.ros.robobodeveloper), which is currently only available for android 5-10. The phone you got should be on android 10, but, if it is updated, you should roll it back to that version.

After that, use Bluetooth to connect the robot to the phone, and connect the app. Start it up, and let the robot sit somewhere running.

Then, to be able to inspect if everything is working, the dockerfile in this directory builds to something basic that just gives you the command line you need.

As always when running docker, make sure the docker daemon is running before running any commands.

build it with:
```sh
docker build --tag hardware_setup .
```

run it with:
```sh
docker run --rm -it --net=host hardware_setup bash
```

Make sure the system you are running the container on is on the exact same network as the phone, and then run:

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
The system is not currently running as it should, and you are probably not on the same network.

This address (everything you put after the `curl` command) will from now on out be known as the `ROS_MASTER_URI`, and there are a few places where you will have to specify it.
If you are wondering, this is the local IP address of the phone, followed by the port number roscore (the master node of ros) is listening on.

---

Once this is working, you can update the setup.bash file in this directory. Currently, it contains this line:
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

If all this is working, you have now successfully setup everything, and can properly start working on this project.
