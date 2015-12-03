# netcam_stream

Use the package to stream network cameras to the ROS network as well as their rectified streams. 
Additionally give ability to find homography to the plane defined by a calibration pattern.

# Usage

To start a stream on an individual camera call the
```
roslaunch netcams_stream netcam.launch id:=X
```
where 'X' is the ID of the camera you want to launch. Additionally you can specify launch_user and launch_pass if they are not set in the paramter server.

To launch all cameras, run
```
roslaunch netcams_stream all_netcams.launch
```

# Details

Each camera is associated with a calibration file ('calibration/cam{id}.yaml'), which needs to be located at the base of the file tree for the package.

In order to calibrate the camera with id X, after starting the stream, run
```
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.108 image:=/netcam_stream_X/image_raw camera:/
```
Remember to move the resulting calibration file to the right place.

To convert it, rename the ost.txt found after uncompressing to ost.ini and parse it

```
tar -zxvf /tmp/calibrationdata.tar.gz
mv ost.txt ost.ini
rosrun camera_calibration_parsers convert ost.ini camX.yaml
mv camX.yaml ~/catkin_ws/src/netcam_stream/calibration
```

# Homography

Homography can be estimated by launching the associated launch file:

```
roslaunch netcams_stream homography.launch id:=X
```

Afterwards, place the calibration board on the plane that the homography transformation needs to be estimated to.
Press 'h' to calculate. Repeat until results are satisfied.

This also generates a 'homographyX.yaml' file in the calibration directory.

# Install

To install the package, run 
```
catkin_make install
```

Then navigate to the catkin_ws/install directory and copy the content of lib/network_stream and share/network_stream
```
roscd
sudo cp -r install/lib/netcam_stream/ /opt/ros/indigo/lib
sudo cp -r install/share/netcam_stream/ /opt/ros/indigo/share/
```
