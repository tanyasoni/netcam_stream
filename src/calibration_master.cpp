#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include "image_geometry/pinhole_camera_model.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/console.h>
#include <ros/package.h>
#include "std_msgs/String.h"

#include <cmath>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <list>
#include <fstream>
#include <sstream>

#include <netcam_stream/ChessPose.h>
#include <netcam_stream/SetPose.h>

using namespace cv;
using namespace std;


struct CamCalibInfo{
  string camera_id;
  geometry_msgs::TransformStamped camera_transform;
};

list<CamCalibInfo> calibrated_cameras; // List to store info for calibrated cameras.
list<CamCalibInfo> current_cam_info;   // List to store the cameras currently bein calibrated.
netcam_stream::ChessPose chess_srv;    // Get pose from the camera node.
netcam_stream::SetPose set_pose_srv;   // Set pose of camera through the main node.

string base_url;
list<string> cam_id_list;
map<string,ros::ServiceClient> client_map;

boost::mutex mtx;

string cam_shut_nodes; // Camera nodes to be shut down.

// Returns true for empty strings
bool IsEmpty(const string& s){
  if(s.size()==0)
    return true;
  return false;
}

// Create list<string> for camera IDs
void CreateList(string id_str, list<string>& cam_id_list){
  int id_start = 0;
  int id_end = id_str.find(',');
  int str_size = id_str.size();
  list<string>::iterator it;
  string temp;
  
  // Split string by ',' and get the camera IDs
  while (id_end > -1) {
      cam_id_list.push_back(id_str.substr(id_start, id_end-id_start));
      id_start = ++id_end;
      id_end = id_str.find(',', id_end);
     }
  cam_id_list.push_back(id_str.substr(id_start, id_str.length()));

  // Remove whitespaces
  for(it=cam_id_list.begin(); it!=cam_id_list.end(); it++){
    temp = string(*it);
    while(temp[0] == ' ')
      temp = temp.substr(1,temp.size());
    while(temp[temp.size()-1] == ' ')
      temp = temp.substr(0,temp.size()-2);
    *it = temp;
  }

  // Remove empty strings from the list
  cam_id_list.remove_if(IsEmpty);

  // Remove duplicate entries
  cam_id_list.unique();
}

void CallTransformService(string camera_id)
{
  ROS_INFO_STREAM("Call service for: " << camera_id);
  if(!ros::service::waitForService(base_url + camera_id + "/chessboard_pose", 1))
  {
    ROS_ERROR_STREAM("\n\n[CALIBRATION_MASTER_NODE]: Failed Service call for " << camera_id<< "! Shutting down the node.");
    cam_shut_nodes += camera_id + ",";
    cam_id_list.remove(camera_id);
    return;
  }
  mtx.lock();
  
  // Call the service to get chessboard transform.
  if(ros::service::call(base_url + camera_id + "/chessboard_pose",chess_srv))
  {
    // Error encountered!
    if(chess_srv.response.fatal_error_id != 0)
    {
      string problem_desc;
      switch(chess_srv.response.fatal_error_id)
      {
        case 1: problem_desc = "Missing Camera Matrix. Please calibrate the camera.";
        break;
        case 2: problem_desc = "Missing no of squares in the width.";
        break;
        case 3: problem_desc = "Missing no of squares in the height.";
        break;
        case 4: problem_desc = "Missing length of Squares(metres).";
        break;
      }
      ROS_ERROR_STREAM("\n\n\n[CAMERA_NODE_" + camera_id + "]: " + problem_desc + " Shutting down the node.");
      // Remove the erroneous camera along with it's service client and calibrated camera map entry.
      cam_id_list.remove(camera_id);
      cam_shut_nodes += camera_id + ",";
      mtx.unlock();
      return;
    }
    // Check if the camera detected the chessboard.
    if(chess_srv.response.board_found)
    {
      // Push the pose into the list of currently detected camera transforms.
      CamCalibInfo temp_info;
      temp_info.camera_id = camera_id;
      temp_info.camera_transform = chess_srv.response.pose;
      current_cam_info.push_back(temp_info);
    }
  }
  else
  {
    ROS_ERROR_STREAM("\n[CAMERA_NODE_" + camera_id + "]: Error encountered while calling service. Shutting down the node.\n");
    cam_shut_nodes += camera_id + ",";
    cam_id_list.remove(camera_id);
  }
  mtx.unlock();
}


void CallSetPoseService(string camera_id, geometry_msgs::TransformStamped pose)
{
  set_pose_srv.request.pose = pose;
  
  if(!ros::service::call(base_url + camera_id + "/set_pose", set_pose_srv))
  {
    ROS_ERROR_STREAM("[CALIBRATION_MASTER_NODE]: Error in calling service to set pose. Pose not set for camera: " << camera_id);
  }
}

// Returns true of camera is calibrated.
bool CheckCameraCalibrated(string camera_id)
{
  bool result = false;
  for(list<CamCalibInfo>::iterator iter = calibrated_cameras.begin(); iter != calibrated_cameras.end(); iter++)
  {
    if(camera_id.compare(iter->camera_id) == 0)
    {
      result = true;
      break;
    }
  }
  return result;
}


void CalculateCamPose()
{
  mtx.lock();
  Eigen::Affine3d pose_aff, calib_cam_pose, temp_pose;
  CamCalibInfo cur;
  list<CamCalibInfo> calib_camera_list, uncalib_camera_list;
  if(current_cam_info.size() == 0) // No camera transform received.
  {
    mtx.unlock();
    return;
  }
  
  // We are starting calibration by keeping chessboard at the origin of the lab.
  if(calibrated_cameras.size() == 0)
  {
    for(list<CamCalibInfo>::iterator cam_iter = current_cam_info.begin(); cam_iter != current_cam_info.end(); cam_iter++)
    {
      CamCalibInfo cam_transform;
      cam_transform.camera_id = cam_iter->camera_id;
      pose_aff = tf2::transformToEigen(cam_iter->camera_transform);
      cam_transform.camera_transform = tf2::eigenToTransform(pose_aff);
      calibrated_cameras.push_back(cam_transform);
      
      CallSetPoseService(cam_iter->camera_id,cam_iter->camera_transform);
    }
    mtx.unlock();
    return;
  }

  // Some of the cameras have already been calibrated.
  // Separating the calibrated and uncalibrated camera lists.
  int list_size = current_cam_info.size();
  list<CamCalibInfo>::iterator cam_iter = current_cam_info.begin();
  do
  {

    if(CheckCameraCalibrated(cam_iter->camera_id)) // Camera is calibrated.
    {
      calib_camera_list.push_back(CamCalibInfo(*cam_iter));
    }

    else // Camera is not calibrated.
    {
      uncalib_camera_list.push_back(CamCalibInfo(*cam_iter));
    }

    cam_iter++;
    list_size--;
    
  }while(list_size > 0);

  // Either there are no cameras to be calibrated OR 
  // there are no transforms from a calibrated camera for propagation.
  if(uncalib_camera_list.size() == 0 || calib_camera_list.size() == 0)
  {
    mtx.unlock();
    return;
  }
  
  static tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transform_stamped;
  CamCalibInfo cam_pose;
  double x,y,z,rx,ry,rz;
  int counter = 0;
  for(list<CamCalibInfo>::iterator uncalib_iter = uncalib_camera_list.begin(); 
    uncalib_iter != uncalib_camera_list.end(); uncalib_iter++)
  {
    counter = 0;
    x = 0;
    y = 0;
    z = 0;
    rx = 0;
    ry = 0;
    rz = 0;
    cur = CamCalibInfo(*uncalib_iter);
    pose_aff = tf2::transformToEigen(cur.camera_transform);
    for(list<CamCalibInfo>::iterator calib_iter = calib_camera_list.begin(); calib_iter != calib_camera_list.end(); calib_iter++)
    {
      calib_cam_pose = tf2::transformToEigen(calib_iter->camera_transform);
      calib_cam_pose = calib_cam_pose.inverse();
      temp_pose = calib_cam_pose * pose_aff;
      for(list<CamCalibInfo>::iterator it = calibrated_cameras.begin(); it != calibrated_cameras.end(); it++)
      {
        if(CamCalibInfo(*it).camera_id.compare(calib_iter->camera_id) == 0)
        {
          temp_pose = tf2::transformToEigen(it->camera_transform) * temp_pose;
          cam_pose.camera_id = cur.camera_id;
          cam_pose.camera_transform = tf2::eigenToTransform(temp_pose);
          x += cam_pose.camera_transform.transform.translation.x;
          y += cam_pose.camera_transform.transform.translation.y;
          z += cam_pose.camera_transform.transform.translation.z;
          rx += cam_pose.camera_transform.transform.rotation.x;
          ry += cam_pose.camera_transform.transform.rotation.y;
          rz += cam_pose.camera_transform.transform.rotation.z;
          counter++;
          break;
        }
      }
    }
    
    if(counter > 1)
    {
      cam_pose.camera_transform.transform.translation.x = x/counter;
      cam_pose.camera_transform.transform.translation.y = y/counter;
      cam_pose.camera_transform.transform.translation.z = z/counter;
      cam_pose.camera_transform.transform.rotation.x = rx/counter;
      cam_pose.camera_transform.transform.rotation.y = ry/counter;
      cam_pose.camera_transform.transform.rotation.z = rz/counter; 
    }
    
    calibrated_cameras.push_back(cam_pose);
    CallSetPoseService(cam_pose.camera_id,cam_pose.camera_transform);
  }
  mtx.unlock();
}

// Removes entries that are more than 100ms apart.
void RemoveOutdatedTransform()
{
  mtx.lock();
  CamCalibInfo last = current_cam_info.back(); // Latest entry in the list.
  double threshold, cur_stamp, last_stamp = last.camera_transform.header.stamp.toSec();
  for(list<CamCalibInfo>::iterator cam_iter = current_cam_info.begin(); cam_iter != current_cam_info.end(); cam_iter++)
    {
      cur_stamp = cam_iter->camera_transform.header.stamp.toSec();
      threshold = abs(cur_stamp - last_stamp);

      // Remove the transform if the time difference is more than 1 minute.
      if(threshold > 60)
      {
        ROS_INFO_STREAM("\n\n[CALIBRATION_MASTER_NODE]: Erased old transform: " << CamCalibInfo(*cam_iter).camera_id << ": " 
          << abs(cur_stamp - last_stamp));
        //current_cam_info.erase(cam_iter);
        //cam_iter--;
      }
    }
  mtx.unlock();
}

void ListenCameraTransform(string origin)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  for(list<string>::iterator it = cam_id_list.begin(); it != cam_id_list.end(); it++)
  {
    try
    {
      CamCalibInfo cam_temp;
      listener.waitForTransform("/" + origin,"/" + string(*it),ros::Time(0),ros::Duration(10));
      listener.lookupTransform("/" + origin,"/" + string(*it),ros::Time(0), transform);
      
      cam_temp.camera_id = string(*it);
      geometry_msgs::Transform geo_trans;
      tf::transformTFToMsg(transform,geo_trans);
      geometry_msgs::TransformStamped transform_temp;
      transform_temp.transform = geo_trans;
      transform_temp.header.frame_id = origin;
      transform_temp.child_frame_id = cam_temp.camera_id;
      cam_temp.camera_transform = transform_temp;

      calibrated_cameras.push_back(cam_temp);
    }
    catch (tf::TransformException ex)
    {
      //ROS_INFO_STREAM("[CALIBRATION_MASTER_NODE]: Transform not found: " << string(*it));
    }
  }
}

void PrintCalibratedNodes()
{
  string calib_cam = "";
  list<CamCalibInfo>::iterator i;
  for(i = calibrated_cameras.begin(); i != calibrated_cameras.end(); i++)
  {
    calib_cam += i->camera_id + " ";
  }
  ROS_INFO_STREAM("[CALIBRATION_MASTER_NODE]: Calibrated cameras: " << calib_cam);
}

void CalibrateCamSystem()
{
  // Create thread for each camera transform subscriber.
  boost::thread_group cam_thread_list;
  for(list<string>::iterator cam_id_iter = cam_id_list.begin(); cam_id_iter != cam_id_list.end(); cam_id_iter++)
  {
    cam_thread_list.create_thread(boost::bind(CallTransformService,string(*cam_id_iter)));
  }

  // Wait for all the threads to get their respective camera transforms.
  cam_thread_list.join_all();

  if(current_cam_info.size() > 0) // Atleast 1 camera found the chessboard.
  {
    // Remove old camera transform entries.
    RemoveOutdatedTransform();

    // Calculate the transform of the cameras which detected the chessboard.
    CalculateCamPose();

    PrintCalibratedNodes();
  }

  ROS_INFO_STREAM("[CALIBRATION_MASTER_NODE]: The Chessboard can be moved now within the next 10 seconds.");
  for(int n = 10; n > 0; n--)
  {
    ROS_INFO_STREAM("[CALIBRATION_MASTER_NODE]: " << n);
    ros::Duration(1).sleep();
  }
  ROS_INFO_STREAM("[CALIBRATION_MASTER_NODE]: Do not move the chessboard.");
}

int CamListCompare()
{
  int count = 0;
  int x;
  for(list<string>::iterator i = cam_id_list.begin(); i != cam_id_list.end(); i++)
  {
    x = 0;
    for(list<CamCalibInfo>::iterator t = calibrated_cameras.begin(); t != calibrated_cameras.end(); t++)
    {
      if(string(*i).compare(t->camera_id) == 0)
      {
        x = 1;
        count++;
        break;
      }
    }
  }
  if(count == cam_id_list.size())
    return 1;
  return 0;
}

int main(int argc, char** argv) 
{
    //Initializes ROS. The node name is replaced by the one from launch file.
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle node_handle("~");
    vector<boost::thread> cam_thread;
    string origin;
    
    // Publisher to indicate camera node shutdown.
    ros::Publisher camera_node_shutdown = node_handle.advertise<std_msgs::String>("camera_node_shutdown", 10);
    std_msgs::String msg;
    
    // Get string containing camera IDs
    string all_camera_id;
    if(!node_handle.getParam("all_camera_id", all_camera_id)){
      ros::shutdown();
      ROS_INFO("[CALIBRATION_NODE]: Camera ID not available!");
    }
    node_handle.getParam("base_url",base_url);

    if(!node_handle.getParam("origin",origin))
    {
      origin = "origin";
    }
    
    // Convert string to list of camera IDs
    CreateList(all_camera_id,cam_id_list);

    list<string>::iterator cam_id_iter;

    ListenCameraTransform(origin);

    for(cam_id_iter = cam_id_list.begin(); cam_id_iter != cam_id_list.end(); cam_id_iter++)
    {
      ros::ServiceClient client_temp = ros::ServiceClient(node_handle.serviceClient<netcam_stream::ChessPose>(base_url 
        + string(*cam_id_iter) + "/chessboard_pose"));
      client_map[string(*cam_id_iter)] = client_temp;
    }

    ros::Duration(3).sleep();
    while(CamListCompare() != 1 && ros::ok())
    {
      CalibrateCamSystem();
      current_cam_info.clear();
      msg.data = cam_shut_nodes;
      camera_node_shutdown.publish(msg);
      PrintCalibratedNodes();
    }
    msg.data = all_camera_id;

    ROS_INFO_STREAM("\n\n[CALIBRATION_MASTER_NODE]: All the cameras have been calibrated!\n\n");
    while(ros::ok())
    {
      camera_node_shutdown.publish(msg);
      ros::Duration(0.5).sleep();
    }

    ros::spinOnce();
    return 0;
}
