# jetbot_ros
ROS2 nodes and Gazebo model for NVIDIA JetBot with Jetson Nano

> note:  if you want to use ROS Melodic, see the [`melodic`](https://github.com/dusty-nv/jetbot_ros/tree/melodic) branch

### Start the JetBot ROS2 Foxy container

``` bash
git clone https://github.com/dusty-nv/jetbot_ros
cd jetbot_ros
docker/run.sh
```
 
### Run JetBot

If you have a real JetBot, you can start the camera / motors like so:

``` bash
ros2 launch jetbot_ros jetbot_nvidia.launch.py
```

or (for a Sparkfun Jetbot)
``` bash
ros2 launch jetbot_ros jetbot_sparkfun.launch.py
```

Otherwise, see the [`Launch Gazebo`](#launch-gazebo) section below to run the simulator.

### Launch Gazebo

``` bash
ros2 launch jetbot_ros gazebo_world.launch.py
```

### Headless Steps:

1. Xvfb :1 -screen 0 1024x768x24
2. export DISPLAY=:1
3. gazebo_ros_world (from ~/.bashrc)
4. run_robot (from ~/.bashrc)
5. ros2 topic list
6. export DISPLAY=192.168.0.6:0
7. ros2 jetbot launch teleop_keyboard

You will see:
```bash
(base) chris@Chriss-MBP-2 jetbot_ros % docker run -it -v="/tmp/.gazebo/:/root/.gazebo/"  --env DISPLAY=host.docker.internal:0 --env QT_X11_NO_MITSHM=1  --volume /tmp/.X11-unix:/tmp/.X11-unix --name=humbl2 jetbot_foxy_ros_cpu:v3.5.0
root@c56a374b1a00:/home/ros2_ws# sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 67170598AF249743
Warning: apt-key is deprecated. Manage keyring files in trusted.gpg.d instead (see apt-key(8)).
Executing: /tmp/apt-key-gpghome.ey4pEl0gAv/gpg.1.sh --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 67170598AF249743
gpg: key 67170598AF249743: "OSRF Repository (OSRF Repository GPG key) <osrfbuild@osrfoundation.org>" not changed
gpg: Total number processed: 1
gpg:              unchanged: 1
root@c56a374b1a00:/home/ros2_ws# export DISPLAY=:1
root@c56a374b1a00:/home/ros2_ws# Xvfb :1 -screen 0 1024x768x24
^Croot@c56a374b1a00:/home/ros2_ws# echo $DISPLAY
:1
root@c56a374b1a00:/home/ros2_ws# gazebo_ros_world 
Gazebo multi-robot simulator, version 11.10.2
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 172.17.0.3
[Err] [RenderEngine.cc:749] Can't open display: :1
[Wrn] [RenderEngine.cc:89] Unable to create X window. Rendering will be disabled
[Wrn] [RenderEngine.cc:292] Cannot initialize render engine since render path type is NONE. Ignore this warning ifrendering has been turned off on purpose.
[Wrn] [ModelDatabase.cc:340] Getting models from[http://models.gazebosim.org/]. This may take a few seconds.
[Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel [model://ground_plane_grass]
[Wrn] [SystemPaths.cc:459] File or path does not exist [""] [model://ground_plane_grass]
[Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel [model://dirt_path_curves]
[Wrn] [SystemPaths.cc:459] File or path does not exist [""] [model://dirt_path_curves]
Error Code 12 Msg: Unable to find uri[model://ground_plane_grass]
Error Code 12 Msg: Unable to find uri[model://dirt_path_curves]
[Msg] Loading world file [/home/ros2_ws/install/jetbot_ros/share/jetbot_ros/worlds/dirt_path_curves.world]
ALSA lib confmisc.c:855:(parse_card) cannot find card '0'
ALSA lib conf.c:5178:(_snd_config_evaluate) function snd_func_card_inum returned error: No such file or directory
ALSA lib confmisc.c:422:(snd_func_concat) error evaluating strings
ALSA lib conf.c:5178:(_snd_config_evaluate) function snd_func_concat returned error: No such file or directory
ALSA lib confmisc.c:1334:(snd_func_refer) error evaluating name
ALSA lib conf.c:5178:(_snd_config_evaluate) function snd_func_refer returned error: No such file or directory
ALSA lib conf.c:5701:(snd_config_expand) Evaluate error: No such file or directory
ALSA lib pcm.c:2664:(snd_pcm_open_noupdate) Unknown PCM default
AL lib: (EE) ALCplaybackAlsa_open: Could not open playback device 'default': No such file or directory
[Err] [OpenAL.cc:84] Unable to open audio device[default]
 Audio will be disabled.
[Wrn] [Publisher.cc:135] Queue limit reached for topic /gazebo/default/physics/contacts, deleting message. This warning is printed only once.
[Err] [CameraSensor.cc:125] Unable to create CameraSensor. Rendering is disabled.
[INFO] [1720568915.521379119] [jetbot.diff_drive]: Wheel pair 1 separation set to [0.200000m]
[INFO] [1720568915.522328780] [jetbot.diff_drive]: Wheel pair 1 diameter set to [0.200000m]
[INFO] [1720568915.525701945] [jetbot.diff_drive]: Subscribed to [/jetbot/cmd_vel]
[INFO] [1720568915.535179800] [jetbot.diff_drive]: Advertise odometry on [/jetbot/odom]
[INFO] [1720568915.545728287] [jetbot.diff_drive]: Publishing odom transforms between [odom] and [chassis]
[INFO] [1720568915.545936077] [jetbot.diff_drive]: Publishing wheel transforms between [chassis], [left_wheel_hinge] and [right_wheel_hinge]
```


Then to run the following commands, launch a new terminal session into the container:

``` bash
sudo docker exec -it jetbot_ros /bin/bash
```

### Test Teleop

``` bash
ros2 launch jetbot_ros teleop_keyboard.launch.py
```

The keyboard controls are as follows:

```
w/x:  increase/decrease linear velocity
a/d:  increase/decrease angular velocity

space key, s:  force stop
```

Press Ctrl+C to quit.

### Data Collection

``` bash
ros2 launch jetbot_ros data_collection.launch.py
```

It's recommended to view the camera feed in Gazebo by going to `Window -> Topic Visualization -> gazebo.msgs.ImageStamped` and selecting the `/gazebo/default/jetbot/camera_link/camera/image` topic.

Then drive the robot and press the `C` key to capture an image.  Then annotate that image in the pop-up window by clicking the center point of the path.  Repeat this all the way around the track.  It's important to also collect data of when the robot gets off-course (i.e. near the edges of the track, or completely off the track).  This way, the JetBot will know how to get back on track.

Press Ctrl+C when you're done collecting data to quit.

### Train Navigation Model

Run this from inside the container, substituting the path of the dataset that you collected (by default, it will be in a timestamped folder under `/workspace/src/jetbot_ros/data/datasets/`)

``` bash
cd /workspace/src/jetbot_ros/jetbot_ros/dnn
python3 train.py --data /workspace/src/jetbot_ros/data/datasets/20211018-160950/
```

### Run Navigation Model

After the model has finished training, run the command below to have the JetBot navigate autonomously around the track.  Substitute the path to your model below:

``` bash
ros2 launch jetbot_ros nav_model.launch.py model:=/workspace/src/jetbot_ros/data/models/202106282129/model_best.pth
```

> note:  to reset the position of the robot in the Gazebo environment, press `Ctrl+R`

<a href="https://youtu.be/gok9pvUzZeY" target="_blank"><img src=https://github.com/dusty-nv/jetbot_ros/raw/dev/docs/images/jetbot_gazebo_sim_video.jpg width="750"></a>

