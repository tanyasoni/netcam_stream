#include <string>
#include <sstream>
#include <fstream>

#include <signal.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>

#include <ros/xmlrpc_manager.h>

#include "yaml-cpp/yaml.h"
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include <netcam_stream/SetPose.h>

using namespace cv;
using namespace ros;
using namespace std;

bool to_publish = false;
bool pose_calibrated = false; // True if pose has been calibrated.
geometry_msgs::TransformStamped camera_pose;
string CAMERA_ID;

void clean_up_gracefully(std::string id = "") {
    ros::param::del("~");
    ros::shutdown();
    if (id.size() != 0) {
        ROS_INFO(
            "[NETCAM_STREAM_%s] Attempting to kill image_proc in 2 sec.",
            id.c_str());
        ros::Duration(5).sleep(); // Wait before killing the node
        system(("rosnode kill netcam_stream_" + id + "/image_proc_" + id).c_str());
    }
}

// Replacement SIGINT handler
void mySigIntHandler(int sig) {
    ROS_WARN("[NETCAM_STREAM] Shutdown request received. Reason id: %d", sig);
    clean_up_gracefully();
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params,
                      XmlRpc::XmlRpcValue& result) {
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
    if (num_params > 1) {
        std::string reason = params[1];
        ROS_WARN(
            "[NETCAM_STREAM] Shutdown request received. Reason: [%s]",
            reason.c_str());
        clean_up_gracefully();
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

void publish_image_callback(const ros::TimerEvent&) {
    to_publish = true;
}

void ReadPoseFile(string pose_file_name, string CAMERA_ID)
{
    std::ifstream fin(pose_file_name.c_str());
    YAML::Node doc = YAML::Load(fin);
        
    doc = doc["transform"];
    YAML::Node sub_node = doc["translation"];
    camera_pose.transform.translation.x = sub_node["x"].as<double>();
    camera_pose.transform.translation.y = sub_node["y"].as<double>();
    camera_pose.transform.translation.z = sub_node["z"].as<double>();

    sub_node = doc["rotation"];
    camera_pose.transform.rotation.x = sub_node["x"].as<double>();
    camera_pose.transform.rotation.y = sub_node["y"].as<double>();
    camera_pose.transform.rotation.z = sub_node["z"].as<double>();
    camera_pose.transform.rotation.w = sub_node["w"].as<double>();

    camera_pose.header.stamp = ros::Time::now();
    camera_pose.header.frame_id = "InSpace_Lab";
    camera_pose.child_frame_id = CAMERA_ID;

    pose_calibrated = true;

    //ROS_INFO_STREAM("\n\n[NETCAM_STREAM_" << CAMERA_ID << "]: Reading file and setting pose. " << pose_calibrated);

}

bool SetCameraPose(netcam_stream::SetPose::Request  &req, netcam_stream::SetPose::Response &res)
{
    string path = ros::package::getPath("netcam_stream");
    ofstream myfile;
    myfile.open (path + "/extrinsic_pose/pose_" + CAMERA_ID + ".yaml");
    myfile << req.pose;
    myfile.close();
    res.calibrated = true;
    //ROS_INFO_STREAM("\n\n\nPose written to file!: " << CAMERA_ID << "\n" << req.pose );
    pose_calibrated = true;
    camera_pose = req.pose;
    camera_pose.header.frame_id = "InSpace_Lab";
    camera_pose.child_frame_id = CAMERA_ID;
    return true;
}


int main(int argc, char** argv) {
    // Initialize ROS, handles, etc
    ros::init(argc, argv, "netcam_stream", ros::init_options::NoSigintHandler);

    // Create a handler for signals (Ctrl+C)
    signal(SIGINT, mySigIntHandler);
    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    if (!ros::master::check()) {
        ROS_FATAL("[NETCAM_STREAM] Cannot detect ROS master!");
        clean_up_gracefully();
        return 1;
    }

    ros::NodeHandle node("~");

    int id;
    if (!node.getParam("camera_id", id)) {
        ROS_FATAL("[NETCAM_STREAM] Cannot read 'camera_id' from param server.");
        clean_up_gracefully();
        return 0;
    }
    std::stringstream ss;
    ss << id;
    CAMERA_ID = ss.str();

    std::string user, pass;
    if (!node.getParam("/cam_user", user)) {
        if (!node.getParam("/launch_user", user) || (user == "user")) {
            ROS_FATAL(
                "[NETCAM_STREAM_%d] Cannot read username from param server "
                "(cam_user) or launch file is not changed (launch_user).",
                id);
            clean_up_gracefully(CAMERA_ID);
            return 0;
        }
        ROS_INFO("The user is: %s", user.c_str());
    }
    if (!node.getParam("/cam_pass", pass)) {
        if (!node.getParam("/launch_pass", pass) || (pass == "pass")) {
            ROS_FATAL(
                "[NETCAM_STREAM_%d] Cannot read password from param server "
                "(cam_pass) or launch file is not changed (launch_pass).",
                id);
            clean_up_gracefully(CAMERA_ID);
            return 0;
        }
    }

    int frame_rate;
    if (!node.getParam("/frame_rate", frame_rate)) {
        ROS_WARN(
            "[NETCAM_STREAM_%d] Camera frequency ('frame_rate') "
            "cannot be read. Using 30 Hz.",
            id);
        frame_rate = 30;
    }

    ROS_INFO("[NETCAM_STREAM_%d] Advertising.", id);
    image_transport::ImageTransport it(node);
    image_transport::CameraPublisher cam_pub = it.advertiseCamera("image_raw", 1);

    std::string path = ros::package::getPath("netcam_stream");
    ROS_INFO("[NETCAM_STREAM_%d] Path to calibration files: %s", id, path.c_str());
    camera_info_manager::CameraInfoManager cim(
        node, "netcam_stream_" + CAMERA_ID,
        "file://" + path + "/calibration/cam" + CAMERA_ID + ".yaml");

    if (cim.isCalibrated())
        ROS_INFO("[NETCAM_STREAM_%d] Camera is calibrated.", id);
    else
        ROS_WARN("[NETCAM_STREAM_%d] Camera is NOT calibrated!", id);

    std::string cam_url = "http://" + user + ":" + pass + "@192.168.107." +
                          CAMERA_ID + "/mjpg/video.mjpg";
    cv::VideoCapture net_cam;
    bool isOpened = net_cam.open(cam_url);
    if (!isOpened) {
        ROS_FATAL(
            "[NETCAM_STREAM_%d] Network camera stream is not opened! "
            "Have you checked the username and password. Trying to access '%s'",
            id,
            ("http://" + user + ":{the_password}@192.168.107." + CAMERA_ID +
             "/mjpg/video.mjpg").c_str());
        clean_up_gracefully(CAMERA_ID);
        return 0;
    }

    ros::Rate loop_rate(30);
    ros::Timer timer =
        node.createTimer(ros::Duration(1.0 / frame_rate), publish_image_callback);

    static tf2_ros::TransformBroadcaster tfb;
    // Checking if the pose file exists.
    ROS_INFO_STREAM("\n\n[NETCAM_STREAM_" << CAMERA_ID << "]: Checking file");
    string pose_file_name = path + "/extrinsic_pose/pose_" + CAMERA_ID + ".yaml";
    if(access( pose_file_name.c_str(), F_OK ) != -1)
    {
        ROS_INFO_STREAM("\n\n[NETCAM_STREAM_" << CAMERA_ID << "]: File exists!");
        pose_calibrated = true;
        ReadPoseFile(pose_file_name,CAMERA_ID);
    }

    ros::ServiceServer pose_service = node.advertiseService("/netcam_stream_" 
        + CAMERA_ID + "/set_pose", &SetCameraPose);

    while (node.ok()) {
        /*
        if (cam_pub.getNumSubscribers() == 0) {
            loop_rate.sleep();
            continue;
        }*/
        cv::Mat frame;
        net_cam >> frame;
        if (frame.empty()) {
            ROS_WARN("[NETCAM_STREAM_%d] Camera returned empty frame! Skipping.", id);
            ROS_INFO("[NETCAM_STREAM_%d] Reopening stream...", id);

            if (!net_cam.open(cam_url))
                ROS_FATAL("[NETCAM_STREAM_%d] Network camera stream cannot be opened!", id);
            continue;
        }
        if (to_publish) {
            sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                             frame).toImageMsg();

            image->header.frame_id = "cam_" + CAMERA_ID;
            image->header.stamp = ros::Time::now();

            // Get current CameraInfo data
            sensor_msgs::CameraInfo ci = cim.getCameraInfo();
            ci.header = image->header;

            cam_pub.publish(*image, ci);

            to_publish = false;
        }

        if(pose_calibrated)
        {
            camera_pose.header.stamp = ros::Time::now();
            tfb.sendTransform(camera_pose);
            //ROS_INFO_STREAM("\n\n[NETCAM_STREAM_" << CAMERA_ID << "]: Publishing transform");
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    clean_up_gracefully(CAMERA_ID);
    return 0;
}