#include <string>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h> 
#include <ros/console.h>

using namespace cv;
using namespace ros;

int CAMERA_ID; 
Size BOARD_SIZE; 
double SQUARE_SIZE; 
Point2f BOARD_OFFSET;

template <typename T>
std::string to_str(T a)
{
    std::ostringstream temp;
    temp<<a;
    return temp.str();
}

bool to_process = false;
bool got_cam_params = false;
cv::Mat frame, K, distcoeff, H;


static void calcChessboardCorners2D(Size boardSize, float squareSize, vector<Point2f>& corners)
{
    corners.resize(0);
    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(Point2f(-float(j*squareSize),
                                      -float(i*squareSize)) + BOARD_OFFSET);
}

void calc_homography(Mat& image)
{
    vector<Point2f> real_corners;
    calcChessboardCorners2D(BOARD_SIZE, SQUARE_SIZE, real_corners);

    vector<Point2f> pointBuf;
    bool found = findChessboardCorners(image, BOARD_SIZE, pointBuf,
                CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK  | CV_CALIB_CB_FILTER_QUADS); // CALIB_CB_NORMALIZE_IMAGE

    // if (found)
    // {
    //     // May lead to floating corners
    //     Mat viewGray;
    //     cvtColor(image, viewGray, CV_BGR2GRAY);
    //     cornerSubPix(viewGray, pointBuf, Size(11,11),
    //           Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
    // }
    drawChessboardCorners(image, BOARD_SIZE, Mat(pointBuf), found);

    if (found)
    {
        std::cout << pointBuf.size() << pointBuf << std::endl;
        std::cout << real_corners.size() << real_corners << std::endl;

        H = cv::findHomography(pointBuf, real_corners);

        ROS_INFO_STREAM("[CALC_HOMOGRAPHY] H: " << H);

        // Save to file
        FileStorage fs("homography.yaml", FileStorage::WRITE);
        fs << "H_" + to_str(CAMERA_ID) << H;
        fs.release();

        // Add to params
        // std::vector<float> Harr;
        // Harr.assign((float*)H.datastart, (float*)H.dataend);
    }
    imshow("Calc Board", image);
    waitKey(10);
    ROS_INFO("[CALC_HOMOGRAPHY] Found board: %d", found);

}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    frame = cv_ptr->image;

    imshow("Board", frame);
}

void camera_info(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (!got_cam_params)
    {
        ROS_INFO("[CALC_HOMOGRAPHY] Camera info has been received.");
        got_cam_params = true;
        K = Mat::ones(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                K.at<double>(i,j) = msg->K[i*3+j];
        distcoeff = Mat::ones(5, 1, CV_64F);
        for (int i = 0; i < 5; ++i) distcoeff.at<double>(i, 1) = msg->D[i];
        ROS_INFO_STREAM("K: " << K);
        ROS_INFO_STREAM("D: " << distcoeff);
    }
}

void chessboardMouseCallback(int event, int x, int y, int flags, void* userdata)
{
    if(event == EVENT_LBUTTONDOWN)
    {
        if (H.empty())
        {
            ROS_WARN("[CALC_HOMOGRAPHY] You need to find chessboard first. Use 'h'.");
            return;
        }
        ROS_INFO_STREAM("[CALC_HOMOGRAPHY] Left button of the mouse is clicked - position (" << x << ", " << y << ")");

        std::vector<Point2f> img_pnt(1, Point2f(x, y)), scene_pnt(1);

        perspectiveTransform(img_pnt, scene_pnt, H);

        ROS_INFO_STREAM("[CALC_HOMOGRAPHY] Transformed point: " << scene_pnt[0]);

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calc_homography");
    if (!ros::master::check())
    {
        ROS_ERROR("[CALC_HOMOGRAPHY] Cannot detect ROS master!");
        return 1;
    }
    
    ros::NodeHandle node("~"); 
    // Get params
    node.getParam("camera_id", CAMERA_ID);
    int w, h;
    node.getParam("board_size_w", w); node.getParam("board_size_h", h);
    BOARD_SIZE = Size(w, h);
    node.getParam("square_size", SQUARE_SIZE);
    double x, y;  
    node.getParam("board_offset_x", x); node.getParam("board_offset_y", y);
    BOARD_OFFSET = Point2f(x, y);

    // Set up topics
    ROS_INFO("[CALC_HOMOGRAPHY] Setting up topics.");
    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe("/netcam_stream_" + to_str(CAMERA_ID) + "/image_rect_color", 1, &imageCb);
    ros::Subscriber cam_info_sub = node.subscribe("/netcam_stream_" + to_str(CAMERA_ID) + "/camera_info", 1, &camera_info);

    ros::Rate loop_rate(30);
    unsigned int frame_id = 0;

    namedWindow("Board");
    setMouseCallback("Board", chessboardMouseCallback, NULL);
    ROS_INFO("Place the calibration board on the desired position (described by the offset params)"
                "and press 'h' on the image to calculate the homography.");
    while (node.ok()) {
        if (to_process){
            calc_homography(frame);
            to_process = false;
        }
        
        char k = waitKey(10);
        if (k == 'h')
            to_process = true;
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}

