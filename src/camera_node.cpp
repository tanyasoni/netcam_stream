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

using namespace cv;
using namespace std;

const int kMissingCamId = 1;
const int kMissingWidthSquares = 2;
const int kMissingHeightSquares = 3;
const int kMissingSquareLength = 4;
int fatal_error = 0;
// Num of squares in width and height of chessboard and length of a square in mm.
int width_squares, height_squares;
double square_length;

Size board_size;
vector<Point3f> board_points; // Coordinates of squares on chessboard(default).
vector<Point2f> corners; // Coordinates of squares on chessboard(computed).

string base_url = "";
string camera_id = "";
bool cam_init = false; // Checks if camera matrices have been initialised.
bool chessboard_found = false, board_init = false;
Mat intrinsics, distortion; // Stores camera intrinsic and distortion matrix.
Mat img, rot_vec, trans_vec; // Image, Rotation and Translation Matrix.
Mat old_rot_vec = Mat::ones(3, 1, CV_64F);
Mat old_trans_vec = Mat::ones(3, 1, CV_64F);

Eigen::Affine3d pose;
geometry_msgs::TransformStamped chess_pose_tf;

int steady_count;
cv_bridge::CvImagePtr img_converter;
Mat mask_sub_pixel;
Size zeroZone = Size( -1, -1 );
TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );

boost::mutex mtx;

// Display the problem encountered and shutdown the node.
void InitFatalError(int problem)
{
	string problem_desc;
	fatal_error = problem;
	switch(problem)
	{
		case 1: problem_desc = "Missing Camera Matrix. Please calibrate the camera.";
		break;
		case 2: problem_desc = "Missing no of squares in the width.";
		break;
		case 3: problem_desc = "Missing no of squares in the height.";
		break;
		case 4: problem_desc = "Missing length of Squares(metres).";
		break;
		case 5: problem_desc = "Missing Camera ID.";
		break;
	}
	ROS_INFO_STREAM("[CAMERA_NODE_" + camera_id + "]: " + problem_desc);
}


// Initialise camera intrinsic and distortion matrices.
void InitCamInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	if (fatal_error != 0)
		return;
	if (cam_init)
		return;
	intrinsics = Mat::ones(3, 3, CV_64F);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			intrinsics.at<double>(i, j) = msg->K[i * 3 + j];
		distortion = Mat::ones(5, 1, CV_64F);
		for (int i = 0; i < 5; ++i)
			distortion.at<double>(i, 1) = msg->D[i];
		cam_init = true;
	ROS_INFO_STREAM("[CAMERA_NODE_" << camera_id << "]: CAMERA MATRIX INITIALISED");
}

// Calculate the difference between new & old pose.
// Return true if values are within a threshold.
bool DiffThreshold()
{
	if(abs(trans_vec.at<double>(0,0) - old_trans_vec.at<double>(0,0)) < 20 && 
		abs(trans_vec.at<double>(0,1) - old_trans_vec.at<double>(0,1)) < 20 && 
		abs(trans_vec.at<double>(0,2) - old_trans_vec.at<double>(0,2)) < 20 &&
		abs(rot_vec.at<double>(0,0) - old_rot_vec.at<double>(0,0)) < 0.1 &&
		abs(rot_vec.at<double>(0,1) - old_rot_vec.at<double>(0,1)) < 0.1 &&
		abs(rot_vec.at<double>(0,2) - old_rot_vec.at<double>(0,2)) < 0.1){
		return true;
	}
	/*else
	{
		ROS_INFO_STREAM("\n\n[CAMERA_NODE_" << camera_id << "]: ");

		ROS_INFO_STREAM("\n\nTrans_vec: " << trans_vec.at<double>(0,0) << " " << trans_vec.at<double>(0,1) << " " << 
		trans_vec.at<double>(0,2));
		ROS_INFO_STREAM("\n\nold_Trans_vec: " << old_trans_vec.at<double>(0,0) << " " << 
		old_trans_vec.at<double>(0,1) << " " << old_trans_vec.at<double>(0,2));

		ROS_INFO_STREAM("\nTrans_vec Diff: " << abs(trans_vec.at<double>(0,0) - old_trans_vec.at<double>(0,0))
		 << " " << abs(trans_vec.at<double>(0,1) - old_trans_vec.at<double>(0,1)) << " " << 
		 abs(trans_vec.at<double>(0,2) - old_trans_vec.at<double>(0,2)));

		ROS_INFO_STREAM("\n\nRot_vec: " << rot_vec.at<double>(0,0) << " " << rot_vec.at<double>(0,1) << " " << 
		rot_vec.at<double>(0,2));
		ROS_INFO_STREAM("\n\nold_Rot_vec: " << old_rot_vec.at<double>(0,0) << " " << old_rot_vec.at<double>(0,1) << " " << 
		old_rot_vec.at<double>(0,2));

		ROS_INFO_STREAM("\nRot_vec Diff:" << abs(rot_vec.at<double>(0,0) - old_rot_vec.at<double>(0,0)) << " " <<
			abs(rot_vec.at<double>(0,1) - old_rot_vec.at<double>(0,1)) << " " << 
			abs(rot_vec.at<double>(0,2) - old_rot_vec.at<double>(0,2)));
	}*/

	return false;
}

// Calculate pose of chesssboard as seen by the camera.
void ChessboardPoseEstimate(const sensor_msgs::ImageConstPtr& msg)
{
	if (fatal_error != 0)
		return;
	if(!cam_init || !board_init)
	{
		return;
	}
	try
	{
		img_converter = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	mtx.lock();
	img = img_converter->image;
	chessboard_found = findChessboardCorners( img, board_size, corners);
	//drawChessboardCorners(img, board_size, corners, chessboard_found);
	ROS_INFO_STREAM(camera_id);
	if(corners.size() == board_size.height * board_size.width && chessboard_found)
	{
		// Refine the corner coordinates.
		img.convertTo(img, CV_8U);
		cvtColor(img, mask_sub_pixel, CV_BGR2GRAY);
		cornerSubPix( mask_sub_pixel, corners, board_size, zeroZone, criteria );
		chessboard_found = false;
		
		ROS_INFO_STREAM(camera_id);
		// Calculate the transform
		try
		{
			solvePnP(Mat(board_points), Mat(corners), intrinsics, distortion, rot_vec, trans_vec, false, CV_EPNP);
		}
		catch(Exception e)
		{
			ROS_WARN_STREAM("Exception in solvePnP: " << e.what());
			return;
		}
		//ROS_INFO_STREAM("[CAMERA_NODE_" << camera_id << "]: After solvePnP");
		//ROS_INFO_STREAM("\n\n\nTrans_vec: " << trans_vec.at<double>(0,0) << " " << trans_vec.at<double>(0,1) 
		//	<< " " << trans_vec.at<double>(0,2));
		//ROS_INFO_STREAM("\nRot_vec: " << rot_vec.at<double>(0,0) << " " << rot_vec.at<double>(0,1) << " " << 
		//	rot_vec.at<double>(0,2));

		// Check if the chessboard is steady among frames.
		if(DiffThreshold())
			steady_count++;
		else
		{
			steady_count = 0;
			chessboard_found = false;
		}
		old_rot_vec = rot_vec;
		old_trans_vec = trans_vec;
		
		if(steady_count > 5) // Chessboard steady for atleast 5 frames.
		{
			// Initialize the chessboard transform.
			pose = Eigen::Translation3d(trans_vec.at<double>(0,0), trans_vec.at<double>(0,1), trans_vec.at<double>(0,2)) *
			Eigen::AngleAxisd(rot_vec.at<double>(0,0), Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxisd(rot_vec.at<double>(0,1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(rot_vec.at<double>(0,2), Eigen::Vector3d::UnitZ());
			chess_pose_tf = tf2::eigenToTransform(pose);
			chess_pose_tf.header.stamp = ros::Time::now();
			chessboard_found = true;
		}
		
	}
	else
	{
		chessboard_found = false;
		steady_count = 0;
	}
	mtx.unlock();
	//cv::imshow(camera_id, img_converter->image);
    //cv::waitKey(3);
}

// Service to send pose of chessboard.
bool ChessboardPoseService(netcam_stream::ChessPose::Request  &req, netcam_stream::ChessPose::Response &res)
{
	mtx.lock();
	res.fatal_error_id = fatal_error;
	if(fatal_error != 0)
	{
		mtx.unlock();
		return true;
	}
	if(steady_count > 5)
	{
		res.pose = chess_pose_tf;
		res.board_found = chessboard_found;
	}
	else
		res.board_found = false;
	mtx.unlock();
	return true;
}

// Shutdown the camera node.
void check_shutdown(const std_msgs::String::ConstPtr& msg)
{
	string nodes = msg->data.c_str();
	std::size_t pos = nodes.find(camera_id);
	if(pos!=std::string::npos)
	{
		ROS_INFO_STREAM("[CAMERA_NODE_" + camera_id + "]: Shutting down.");
		ros::shutdown();
	}
}

int main(int argc, char** argv) 
{
    //Initialize ROS. The node name is replaced by the one from launch file to avoid duplicate names.
	ros::init(argc, argv, "camera_node");
	ros::NodeHandle node_handle("~");

	if (!node_handle.getParam("camera_id", camera_id)) {
		camera_id = "X";
		InitFatalError(kMissingCamId);
	}
 
	if (!node_handle.getParam("width_squares", width_squares)){
		InitFatalError(kMissingWidthSquares);
	}

	if (!node_handle.getParam("height_squares", height_squares)){
		InitFatalError(kMissingHeightSquares);
	}

	if (!node_handle.getParam("square_length", square_length)){
		InitFatalError(kMissingSquareLength);
	}

	node_handle.getParam("base_url",base_url);

	board_size.width = width_squares;
	board_size.height = height_squares;
	
    //Initialize the coordinates for chessboard points.
	for(int i = 0; i<height_squares; i++)
	{
		for (int j=0; j<width_squares; j++)
		{
			board_points.push_back(cv::Point3f(i*square_length, (width_squares - 1 - j)*square_length, 0));
		}
	}
	
	board_init = true;
	steady_count = 0; // Ensures that the chessboard is stationary.

    // Creating subscribers to image and camera info.
	ros::Subscriber camera_info_sub = node_handle.subscribe(base_url 
		+ camera_id + "/camera_info", 1, &InitCamInfo);

	image_transport::ImageTransport img_trans(node_handle);
	image_transport::Subscriber img_sub = img_trans.subscribe(base_url 
		+ camera_id + "/image_rect_color", 1, &ChessboardPoseEstimate);

	// Server to send pose to calibration master node.
	ros::ServiceServer pose_service = node_handle.advertiseService(base_url 
		+ camera_id + "/chessboard_pose", &ChessboardPoseService);

	// Subscriber for shutdown information.
	ros::Subscriber sub = node_handle.subscribe("/netcam_extrinsic_calibration_master_node/camera_node_shutdown",
	 10, &check_shutdown);

	//cv::namedWindow(camera_id);

	while(ros::ok())
	{
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}
  	return 0;
}