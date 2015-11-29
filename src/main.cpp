#include <string>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>

using namespace cv;
using namespace ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "netcam_stream");

    if (!ros::master::check())
    {
        ROS_ERROR("[NETCAM_STREAM] Cannot detect ROS master!");
        return 1;
    }
    ROS_INFO("[NETCAM_STREAM] Advertising.");

    ros::NodeHandle node("~");  

    int id; std::string CAMERA_ID;
    if (!node.getParam("camera_id", id))
        ROS_ERROR("[NETCAM_STREAM] Cannot read camera_id from param server.");
    std::stringstream ss;
    ss << id;
    CAMERA_ID = ss.str();

    std::string user, pass;
    if (!node.getParam("/cam_user", user)){
        if (!node.getParam("/launch_user", user) || (user == "user")){
            ROS_ERROR("[NETCAM_STREAM] Cannot read username from param server (cam_user) or launch file is not changed (launch_user).");
            return 1;
        }
        ROS_INFO("The user is: %s", user.c_str());
    }
    if (!node.getParam("/cam_pass", pass)){
        if (!node.getParam("/launch_pass", pass) || (pass == "pass")){
            ROS_ERROR("[NETCAM_STREAM] Cannot read password from param server (cam_pass) or launch file is not changed (launch_pass).");
            return 1;
        }
    }

    image_transport::ImageTransport it(node);
    image_transport::Publisher pub = it.advertise("image_raw", 1);
    std::string path = ros::package::getPath("netcam_stream");
    ROS_INFO("[NETCAM_STREAM] Path to calibration files: %s", path.c_str());
    camera_info_manager::CameraInfoManager cim(node, "netcam_stream_" + CAMERA_ID, "file://" + path + "/calibration/cam"+ CAMERA_ID + ".yaml");
    
    ros::Publisher pub_info = node.advertise<sensor_msgs::CameraInfo>("camera_info", 1);


    if (cim.isCalibrated())
        ROS_INFO("[NETCAM_STREAM] Camera is calibrated!");

    cv::VideoCapture net_cam;
    bool isOpened = net_cam.open("http://" + user + ":" + pass + "@192.168.107." + CAMERA_ID + "/mjpg/video.mjpg"); 
    if (!isOpened){
        ROS_ERROR("[NETCAM_STREAM] Network camera stream is not opened! Have you checked the username and password. Trying to access '%s'", ("http://" + user + ":{the_password}@192.168.107." + CAMERA_ID + "/mjpg/video.mjpg").c_str());
        return 1;
    }

    ros::Rate loop_rate(30); // 30Hz
    unsigned int frame_id = 0;
    while (node.ok()) {
        ros::spinOnce();

        cv::Mat frame;
        net_cam >> frame;   
        if (frame.empty()){
            ROS_WARN("[NETCAM_STREAM] Camera returned empty frame! Skipping...");
            continue;
        }
        ++frame_id;
        sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        
        image->header.frame_id = frame_id;
        // Get current CameraInfo data
        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cim.getCameraInfo()));
        ci->header.frame_id = frame_id;
        ci->header.stamp = image->header.stamp;

        pub.publish(image);
        pub_info.publish(ci);

        loop_rate.sleep();
    }

    
    ros::shutdown();
    return 0;
}
