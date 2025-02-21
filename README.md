# mirobot_ros2
Control a WLKata Mirobot using ROS2, Moveit and WLKataPythonSDK
This repository is based on:
- the WLKata python sdk (https://github.com/wlkata/WLKATA-Python-SDK-wlkatapython/tree/main?tab=readme-ov-fileLinks to an external site.)
- The WLKata ROS repository (https://github.com/wlkata/RosForMirobot-masterLinks to an external site.), which contains the stl files and some related support files
- This repo (https://github.com/kimsooyoung/mirobot_ros2/tree/mainLinks to an external site.), which ports some of the features we need but uses isaac sim (which we won't be using)
  
![ros2_mirobot_control](https://github.com/user-attachments/assets/1d9b6737-1709-4644-aaad-0a0998e721f2)

## Instructions
- Clone this repository into your project src folder
- Build with colcon and source:
  cd /home/ros2_ws # replace with your own workspace
  colcon build --symlink-install
  source install/setup.bash
- Launch with RViz and joint_state_publisher_gui
ros2 launch mirobot_description mirobot_rviz_control.launch.py

You can now control the mirobot with the joint_state_publisher_gui

## Dependencies
As can be seen in the dockerfile for this project, we need to install a number of new libraries. These include:

pyserial - a standard python library for serial/uart communications
wlkatapython - wlktata's python sdk. We'll use this for communicating with the mirobot
ros-humble-moveit - a ros package for manipulating robots, and includes a lot of tools for path planning, kinematics, perception etc. Full documentation is available here: https://moveit.picknik.ai/humble/index.htmlLinks to an external site. 

# Running with Docker
This repo was implemented using a docker container. The Dockerfile is included. If running your container on Windows, you need to set up USB access for your container. Please follow the instructions below.

## USB access with Docker
Fun fact: docker containers don't have access to USB ports by default. Obviously a bit of an issue. The good news is, we can fix this with a tool called usbipd. The bad news is that it's a bit finicky.

Put simply, usbipd is a tool that maps the windows USB port addresses to WSL. From there, you can map the USB address to the docker container.

### Instructions
#### Install usbipd

- Open a Windows Powershell as an Administrator
- Install usbipd with winget:
winget install usbipd
- List the USB devices on your system by typing
usbipd list
- You should see an output like this:

image.png

Look for "USB-SERIAL CH340", and note the BUSID. In this example, the BUSID is 2-6

The next step is to bind the BUSID to WSL. I don't know what this means either. 
- Open a Powershell as an Administrator and run:
usbipd bind --busid 2-6  # replace the busid number with your busid.
- Once this step is done, we will attach the busid to wsl. Again, not what the difference between attach and bind is, but apparently this is what's needed...
usbipd attach --wsl --busid 2-6  # replace the busid number with your busid
- Verify this operation was successful by opening a wsl terminal, and check the following:
- In a cmd or powershell window, type "wsl". Then, run the following:

ls /dev/ttyUSB0
This should return "/dev/ttyUSB0", as shown below. If nothing is returned, then something went wrong. Repeat the previous steps.

image.png

#### Launching our container with USB access
Now that we have attached the USB to wsl, we can now launch our docker container with USB access.

Build the supplied docker as usual
When we run our container, we need to add an extra term "--device=/dev/ttyUSB0:/dev/ttyUSB0"
docker run -it --rm --privileged --name mirobot_container -e DISPLAY=host.docker.internal:0.0 --env="QT_X11_NO_MITSHM=1" --device=/dev/ttyUSB0:/dev/ttyUSB0 -v C:\Users\0109491s\PycharmProjects\EE5109\mirobot\mirobot_ros2\mirobot_ros2\src:/home/ros2_ws/src mirobot_container
This maps our USB device from WSL to our docker container.

Once our container is up and running, we can check the mapping was successful:
ls /dev/ttyUSB0
image.png

 

Again, if this command returns nothing, something has gone wrong, repeat the previous steps.

Finally, we need to give read/write access to the USB drive. We do this by the following command:
chmod +x /dev/ttyUSB0
