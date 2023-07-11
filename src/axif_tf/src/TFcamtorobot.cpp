/**********************ROS****************************************/
#include <ros/ros.h>
#include <axif_tf/getPoint.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
/*********************EUGEN***************************************/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
/********************OPENCVLIBRARY********************************/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
/*******************PACKAGE_HEADER*******************************/
#include <opencvtest/pixel_point0.h>
#include <dobot/GetPose.h>
/**************************************计算过程中用到的变量
***************************************************/
using namespace std;
axif_tf::getPoint msg1;                             // 自定义 msg 存储的是红色木块中心的像素坐标
axif_tf::getPoint msg2; // 绿色木块中心的像素坐标
axif_tf::getPoint msg3; // 蓝色木块中心的像素坐标
axif_tf::getPoint msg4; // huang色木块中心的像素坐标
axif_tf::getPoint msg5; // zi色木块中心的像素坐标
axif_tf::getPoint msg6; // ju色木块中心的像素坐标


Eigen::Matrix<double, 4, 3> *pointer_camera_matrix; // 相机内参*Zc
ros::NodeHandle *n_p = NULL;
ros::Publisher *pointer_result_1_pub = NULL; // 红色
ros::Publisher *pointer_result_2_pub = NULL; // 绿色
ros::Publisher *pointer_result_3_pub = NULL; // 蓝色
ros::Publisher *pointer_result_4_pub = NULL; // 色
ros::Publisher *pointer_result_5_pub = NULL; // 色
ros::Publisher *pointer_result_6_pub = NULL; // 色


// tf::TransformListener* listener_ptr;
double Zc = 1.0;                       // 坐标变换因子(相机坐标系原点到世界坐标平面的距离)
const string camera_name = "logitech"; // 相机坐标系的名称
// 手动输入标定好的相机内参
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1042.300116166677, 0, 238.3667675097579, 0, 1043.257207991183, 147.074867651602, 0, 0, 1);
// 相机畸变参数
cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 0.0684211758324593, 0.419412673131594, -0.0314044701662507, -0.01692326508172931, 0); // matrix_num2
// Aruco 二维码的编码代号
cv::Ptr<cv::aruco::Dictionary> dictionary;
// 二维码中心.即世界坐标系的中心点像素坐标
vector<cv::Point2f> marker_center;
// 相机坐标系乘以尺度因子 Zc
Eigen::Matrix<double, 3, 3> camera_matrix_Zc_temp;
Eigen::Matrix<double, 3, 3> camera_matrix_Zc_inver;
Eigen::Matrix<double, 4, 3> camera_matrix_Zc;
// 齐次像素坐标初始化,用来储存接收到的木块中心像素
Eigen::Vector3d pixel_vec(1.0, 1.0, 1.0);
Eigen::Vector3d pixel_vec_transpose(1.0, 1.0, 1.0);
Eigen::Vector3d green_pixel_vec(1.0, 1.0, 1.0);
Eigen::Vector3d green_pixel_vec_transpose(1.0, 1.0, 1.0);

Eigen::Vector3d blue_pixel_vec(1.0, 1.0, 1.0);
Eigen::Vector3d blue_pixel_vec_transpose(1.0, 1.0, 1.0);

Eigen::Vector3d yellow_pixel_vec(1.0, 1.0, 1.0);
Eigen::Vector3d yellow_pixel_vec_transpose(1.0, 1.0, 1.0);
Eigen::Vector3d purple_pixel_vec(1.0, 1.0, 1.0);
Eigen::Vector3d purple_pixel_vec_transpose(1.0, 1.0, 1.0);
Eigen::Vector3d orange_pixel_vec(1.0, 1.0, 1.0);
Eigen::Vector3d orange_pixel_vec_transpose(1.0, 1.0, 1.0);
// 中心点在相机坐标系下的坐标计算结果
Eigen::Vector4d result_1;
Eigen::Vector4d result_2;
Eigen::Vector4d result_3;
Eigen::Vector4d result_4;
Eigen::Vector4d result_5;
Eigen::Vector4d result_6;
// 世界坐标系下的坐标
/**********************************************TF 变量
******************************************************/
tf::TransformBroadcaster *pointer_marker_position_broadcaster;
tf::TransformBroadcaster *pointer_dobot_base_bro;
tf::TransformBroadcaster *pointer_dobot_effector_bra;
/******************************CALLBACK_FUNCTION*********************/
void callbackCalculateAxis(opencvtest::pixel_point0::ConstPtr message);
void callbackImage(const sensor_msgs::ImageConstPtr &msg);
/**********************************FUNCTION**************************/
void loadCalibrationFiles(string &input_path, cv::Mat &camera_matrix, cv::Mat &distcoeffs, double scale);
void getMarker(cv::Mat &marker_image, vector<cv::Point2f> &marker_center, bool key);
// 得到世界坐标系的中心点,useless
// void getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids,vector<cv::Point2f>& marker_center);
void sendMarkerTf(vector<cv::Vec3d> &marker_vecs, vector<cv::Vec3d> &marker_rvecs);
void sendDobotTf();
void sendDobotEffectorTF();
/********************************************************************/
/********************************************************************/
/******************************主函数*********************************/
/********************************************************************/
/********************************************************************/
int main(int argc, char **argv)
{
    // 手动输入标定好的相机内参
    camera_matrix_Zc_temp << 1042.300116166677, 0, 238.3667675097579, 0, 1043.257207991183, 147.074867651602, 0, 0, 1;
    camera_matrix_Zc_inver = camera_matrix_Zc_temp.inverse(); // 逆矩阵
    camera_matrix_Zc(0, 0) = camera_matrix_Zc_inver(0, 0) * Zc;
    camera_matrix_Zc(0, 1) = camera_matrix_Zc_inver(0, 1) * Zc;
    camera_matrix_Zc(0, 2) = camera_matrix_Zc_inver(0, 2) * Zc;
    camera_matrix_Zc(1, 0) = camera_matrix_Zc_inver(1, 0) * Zc;
    camera_matrix_Zc(1, 1) = camera_matrix_Zc_inver(1, 1) * Zc;
    camera_matrix_Zc(1, 2) = camera_matrix_Zc_inver(1, 2) * Zc;
    camera_matrix_Zc(2, 0) = camera_matrix_Zc_inver(2, 0) * Zc;
    camera_matrix_Zc(2, 1) = camera_matrix_Zc_inver(2, 1) * Zc;
    camera_matrix_Zc(2, 2) = camera_matrix_Zc_inver(2, 2) * Zc;
    ros::init(argc, argv, "axif_tf");
    ros::NodeHandle n;
    n_p = &n;
    tf::TransformBroadcaster marker_position_broadcaster;
    tf::TransformBroadcaster dobot_base_bro;
    tf::TransformBroadcaster dobot_effector_bra;
    pointer_marker_position_broadcaster = &marker_position_broadcaster; ////
    pointer_dobot_base_bro = &dobot_base_bro;                           /////
    pointer_dobot_effector_bra = &dobot_effector_bra;                   ////
    ros::ServiceClient client_pose = n.serviceClient<dobot::GetPose>("dobot/GetPose");
    // 订阅机器人末端的姿态
    image_transport::ImageTransport it_(n);
    image_transport::Subscriber image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, callbackImage);
    // 发布相机坐标系下的红色物块中心坐标
    ros::Publisher result_1_pub = n.advertise<axif_tf::getPoint>("result_1", 1000);
    // 取地址发布
    pointer_result_1_pub = &result_1_pub;

    ros::Publisher result_2_pub = n.advertise<axif_tf::getPoint>("result_2", 1000);
    pointer_result_2_pub = &result_2_pub;
    ros::Publisher result_3_pub = n.advertise<axif_tf::getPoint>("result_3", 1000);
    pointer_result_3_pub = &result_3_pub;
    ros::Publisher result_4_pub = n.advertise<axif_tf::getPoint>("result_4", 1000);
    pointer_result_4_pub = &result_4_pub;
    ros::Publisher result_5_pub = n.advertise<axif_tf::getPoint>("result_5", 1000);
    pointer_result_5_pub = &result_5_pub;
    ros::Publisher result_6_pub = n.advertise<axif_tf::getPoint>("result_6", 1000);
    pointer_result_6_pub = &result_6_pub;

    


    ros::Subscriber pixel_sub = n.subscribe("pixel_center_axis", 100, callbackCalculateAxis); // 订阅木块中心的像素坐标
    // 每秒接受 30 次回调函数
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
} //使用像素坐标计算相机坐标的回调函数
void callbackCalculateAxis(opencvtest::pixel_point0::ConstPtr message)
{
    // Zc 更新， 所以刷新赋值
    camera_matrix_Zc(0, 0) = camera_matrix_Zc_inver(0, 0) * Zc;
    camera_matrix_Zc(0, 1) = camera_matrix_Zc_inver(0, 1) * Zc;
    camera_matrix_Zc(0, 2) = camera_matrix_Zc_inver(0, 2) * Zc;
    camera_matrix_Zc(1, 0) = camera_matrix_Zc_inver(1, 0) * Zc;
    camera_matrix_Zc(1, 1) = camera_matrix_Zc_inver(1, 1) * Zc;
    camera_matrix_Zc(1, 2) = camera_matrix_Zc_inver(1, 2) * Zc;
    camera_matrix_Zc(2, 0) = camera_matrix_Zc_inver(2, 0) * Zc;
    camera_matrix_Zc(2, 1) = camera_matrix_Zc_inver(2, 1) * Zc;
    camera_matrix_Zc(2, 2) = camera_matrix_Zc_inver(2, 2) * Zc;
    // 处理红色工件
    for (int i = 0; i < message->red_u.size(); i++)
    {
        if ((pixel_vec[0] != message->red_u[i]) || (pixel_vec[1] != message->red_v[i]))
        { // 计算公式
            cout << "1" << endl;
            pixel_vec[0] = message->red_u[i];
            pixel_vec[1] = message->red_v[i];
            pixel_vec_transpose = pixel_vec.transpose();
            result_1 = camera_matrix_Zc * pixel_vec_transpose;
            msg1.x1.push_back(result_1[0]);
            msg1.x2.push_back(result_1[1]);
            msg1.x3.push_back(result_1[2]);
        }
    }
    cout << msg1 << endl;
    pointer_result_1_pub->publish(msg1); // 发布出来
    msg1.x1.clear();
    msg1.x2.clear();
    msg1.x3.clear();


    // 处理绿色工件
    for (int i = 0; i < message->green_u.size(); i++)
    {
        if ((green_pixel_vec[0] != message->green_u[i]) || (green_pixel_vec[1] != message->green_v[i]))
        {
            cout << "2" << endl;
            green_pixel_vec[0] = message->green_u[i];
            green_pixel_vec[1] = message->green_v[i];
            green_pixel_vec_transpose = green_pixel_vec.transpose();
            result_2 = camera_matrix_Zc * green_pixel_vec_transpose;
            msg2.x1.push_back(result_2[0]);
            msg2.x2.push_back(result_2[1]);
            msg2.x3.push_back(result_2[2]);
        }
    }
    cout << msg2 << endl;
    pointer_result_2_pub->publish(msg2); // 发布出来
    msg2.x1.clear();
    msg2.x2.clear();
    msg2.x3.clear();

    // 处理蓝色工件
    for (int i = 0; i < message->blue_u.size(); i++)
    {
        if ((blue_pixel_vec[0] != message->blue_u[i]) || (blue_pixel_vec[1] != message->blue_v[i]))
        {
            cout << 3 << endl;
            blue_pixel_vec[0] = message->blue_u[i];
            blue_pixel_vec[1] = message->blue_v[i];
            blue_pixel_vec_transpose = blue_pixel_vec.transpose();
            result_3 = camera_matrix_Zc * blue_pixel_vec_transpose;
            msg3.x1.push_back(result_3[0]);
            msg3.x2.push_back(result_3[1]);
            msg3.x3.push_back(result_3[2]);
        }
    }
    cout << msg3 << endl;
    pointer_result_3_pub->publish(msg3); // 发布出来
    msg3.x1.clear();
    msg3.x2.clear();
    msg3.x3.clear();

    for (int i = 0; i < message->yellow_u.size(); i++)
    {
        if ((yellow_pixel_vec[0] != message->yellow_u[i]) || (yellow_pixel_vec[1] != message->yellow_v[i]))
        {
            cout << 4 << endl;
            yellow_pixel_vec[0] = message->yellow_u[i];
            yellow_pixel_vec[1] = message->yellow_v[i];
            yellow_pixel_vec_transpose = yellow_pixel_vec.transpose();
            result_4 = camera_matrix_Zc * yellow_pixel_vec_transpose;
            msg4.x1.push_back(result_4[0]);
            msg4.x2.push_back(result_4[1]);
            msg4.x3.push_back(result_4[2]);
        }
    }
    cout << msg4 << endl;
    pointer_result_4_pub->publish(msg4); // 发布出来
    msg4.x1.clear();
    msg4.x2.clear();
    msg4.x3.clear();

    for (int i = 0; i < message->purple_u.size(); i++)
    {
        if ((purple_pixel_vec[0] != message->purple_u[i]) || (purple_pixel_vec[1] != message->purple_v[i]))
        {
            cout << 5 << endl;
            purple_pixel_vec[0] = message->purple_u[i];
            purple_pixel_vec[1] = message->purple_v[i];
            purple_pixel_vec_transpose = purple_pixel_vec.transpose();
            result_5 = camera_matrix_Zc * purple_pixel_vec_transpose;
            msg5.x1.push_back(result_5[0]);
            msg5.x2.push_back(result_5[1]);
            msg5.x3.push_back(result_5[2]);
        }
    }
    cout << msg5 << endl;
    pointer_result_5_pub->publish(msg5); // 发布出来
    msg5.x1.clear();
    msg5.x2.clear();
    msg5.x3.clear();

    for (int i = 0; i < message->orange_u.size(); i++)
    {
        if ((orange_pixel_vec[0] != message->orange_u[i]) || (orange_pixel_vec[1] != message->orange_v[i]))
        {
            cout << 6 << endl;
            orange_pixel_vec[0] = message->orange_u[i];
            orange_pixel_vec[1] = message->orange_v[i];
            orange_pixel_vec_transpose = orange_pixel_vec.transpose();
            result_6 = camera_matrix_Zc * orange_pixel_vec_transpose;
            msg6.x1.push_back(result_6[0]);
            msg6.x2.push_back(result_6[1]);
            msg6.x3.push_back(result_6[2]);
        }
    }
    cout << msg6 << endl;
    pointer_result_6_pub->publish(msg6); // 发布出来
    msg6.x1.clear();
    msg6.x2.clear();
    msg6.x3.clear();
} //接受传来的图像, 目的是为了检测二维码, 并发布世界坐标系, 相机坐标系, 机器人坐标系, 机器人 末端坐标系的 tf 变换关系 
void callbackImage(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // 转化为opencv 格式图像， 返回指针
    }
    catch(cv_bridge::Exception & e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } //获取二维码信息, 并发布坐标系之间的关系
    getMarker(cv_ptr->image, marker_center, 0);
    cv::imshow("callbackImage", cv_ptr->image);
    // cout<<result_1<<endl;
    cv::waitKey(1);
}

void getMarker(cv::Mat &marker_image, vector<cv::Point2f> &marker_center, bool key)
{
    // 从而二维码中获取相机和二维码(世界坐标系)的变化关系
    vector<int> ids;
    vector<vector<cv::Point2f>> corners;
    vector<cv::Vec3d> rvecs, tvecs;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    // 需要根据实际的二维码输入相应的码制
    if (!marker_image.empty())
    {
        cv::aruco::detectMarkers(marker_image, dictionary, corners, ids); // 侦测到角点以备姿态检测使用
        cv::aruco::drawDetectedMarkers(marker_image, corners, ids);
        cv::aruco::estimatePoseSingleMarkers(corners, 0.10, camera_matrix, dist_coeffs, rvecs,tvecs); // marker 大小要根据实际码的大小改写
        if (rvecs.empty() && tvecs.empty())
        {
            cout << "no trans" << endl;
        }
        else
        {
            cv::aruco::drawAxis(marker_image, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1);
            // 画坐标轴
            Zc = tvecs[0][2] - 0.044;
            cout << "深度输出： " << Zc << endl;
            sendMarkerTf(rvecs, tvecs); // 发布 tf
        }
    }
}
void sendMarkerTf(vector<cv::Vec3d> &marker_rvecs, vector<cv::Vec3d> &marker_tvecs)
{
    if (marker_rvecs.size() == 0 && marker_rvecs.size() == 0)
    {
        cout << "haven't received any vecs yet" << endl;
    }
    else
    {
        cv::Mat rotated_matrix(3, 3, CV_64FC1); // 储存旋转矩阵  

        tf::TransformBroadcaster marker_position_broadcaster;  
       
        // 从相机坐标系与世界坐标系中心的旋转向量转换为旋转矩阵  
        cv::Rodrigues(marker_rvecs[0], rotated_matrix);
        tf::Matrix3x3 tf_rotated_matrix(rotated_matrix.at<float>(0,0), rotated_matrix.at<float>(0,1), rotated_matrix.at<float>(0,2),
                                       rotated_matrix.at<float>(1,0), rotated_matrix.at<float>(1,1), rotated_matrix.at<float>(1,2), 
                                       rotated_matrix.at<float>(2,0), rotated_matrix.at<float>(2,1), rotated_matrix.at<float>(2,2));

        // 调用相机坐标系与世界坐标系中心的平移向量  
        tf::Vector3 tf_tvecs(marker_tvecs[0][0], marker_tvecs[0][1], marker_tvecs[0][2]);

        // 建立相机坐标系和世界参考坐标系的变换关系  
        tf::Transform transform(tf_rotated_matrix, tf_tvecs);  

        // 发布相机坐标系和世界参考坐标系的坐标变换  
        pointer_marker_position_broadcaster -> sendTransform(tf::StampedTransform(transform, ros::Time::now(), "logitech", "world"));
  
        // 发布机器人和世界坐标系之间的关系
        sendDobotTf();
    }
}
void sendDobotTf()
{
    cv::Mat rotated_matrix(3, 3, CV_64FC1);
    // 定义 rotated_matrix3X3
    rotated_matrix.at<float>(0, 0) = 1.0;
    rotated_matrix.at<float>(0, 1) = 0.0;
    rotated_matrix.at<float>(0, 2) = 0.0;
    rotated_matrix.at<float>(1, 0) = 0.0;
    rotated_matrix.at<float>(1, 1) = 1.0;
    rotated_matrix.at<float>(1, 2) = 0.0;
    rotated_matrix.at<float>(2, 0) = 0.0;
    rotated_matrix.at<float>(2, 1) = 0.0;
    rotated_matrix.at<float>(2, 2) = 1.0;
    // 机器人基座标系中心和世界坐标系中心的旋转矩阵(3X3)
    tf::Matrix3x3 tf_rotated_matrix(
        rotated_matrix.at<float>(0, 0), rotated_matrix.at<float>(0, 1), rotated_matrix.at<float>(0, 2),
        rotated_matrix.at<float>(1, 0), rotated_matrix.at<float>(1, 1), rotated_matrix.at<float>(1, 2),
        rotated_matrix.at<float>(2, 0), rotated_matrix.at<float>(2, 1), rotated_matrix.at<float>(2, 2));
    // 测量机器人基座标系中心和世界坐标系中心的平移向量
    tf::Vector3 tf_tvecs(-0.115, 0.251, 0.138);  // 测量改这里
    // 建立世界参考坐标系和基座标系的变换关系
    tf::Transform transform(tf_rotated_matrix, tf_tvecs);
    // 发布世界参考坐标系和基座标系的坐标变换
    pointer_dobot_base_bro -> sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dobot_base"));  

    // 发布机器人末端坐标系和基座标系的变化关系
    sendDobotEffectorTF();
}
void sendDobotEffectorTF()
{
    ros::ServiceClient client_getpose = n_p->serviceClient<dobot::GetPose>("/DobotServer/GetPose");
    cv::Mat rotated_matrix(3, 3, CV_64FC1);
    rotated_matrix.at<float>(0, 0) = 1.0;
    rotated_matrix.at<float>(0, 1) = 0.0;
    rotated_matrix.at<float>(0, 2) = 0.0;
    rotated_matrix.at<float>(1, 0) = 0.0;
    rotated_matrix.at<float>(1, 1) = 1.0;
    rotated_matrix.at<float>(1, 2) = 0.0;
    rotated_matrix.at<float>(2, 0) = 0.0;
    rotated_matrix.at<float>(2, 1) = 0.0;
    rotated_matrix.at<float>(2, 2) = 1.0;
    tf::Matrix3x3 tf_rotated_matrix(
        rotated_matrix.at<float>(0, 0), rotated_matrix.at<float>(0, 1), rotated_matrix.at<float>(0, 2),
        rotated_matrix.at<float>(1, 0), rotated_matrix.at<float>(1, 1), rotated_matrix.at<float>(1, 2),
        rotated_matrix.at<float>(2, 0), rotated_matrix.at<float>(2, 1), rotated_matrix.at<float>(2, 2));
    dobot::GetPose srv;
    client_getpose.call(srv);
    cout << srv.response.x / 1000 << "," << srv.response.y / 1000 << "," << srv.response.z / 1000 << endl;
    tf::Vector3 tf_tvecs(srv.response.x / 1000, srv.response.y / 1000, srv.response.z / 1000);
    tf::Transform transform(tf_rotated_matrix, tf_tvecs);
    pointer_dobot_effector_bra->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                                   "dobot_base", "dobot_effector"));
}