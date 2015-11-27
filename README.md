# netcam_stream

Use the package to stream network cameras to the ROS network as well as their rectified streams. 
Additionally give ability to find homography to the plane defined by a calibration pattern.

# Usage

To start a stream on an individual camera call the
```
roslaunch netcams_stream netcam.launch id:=X
```
where 'X' is the ID of the camera you want to launch.

To launch all cameras, run
```
roslaunch netcams_stream all_netcams.launch
```

# Details

Each camera is associated with a calibration file ('cam{id}.yaml'), which needs to be located at the base of the file tree for the package.

In order to calibrate the camera with id X, after starting the stream, run
```
rosrun camera_calibration cameracalibrator.py --siz7x6 --square 0.108 image:=/netcam_stream_X/image_raw camera:/
```
Remember to move the resulting calibration file to the right place.

To convert it, rename the /tmp/ost.txt found after uncompressing (tar -zxvf calibrationdata.tar.gz) to ost.ini and run

```
tar -zxvf /tmp/calibrationdata.tar.gz
rosrun camera_calibration_parsers convert /tmp/ost.ini cam0.yaml
mv cam0.yaml ~/catkin_ws/src/netcam_stream/
```