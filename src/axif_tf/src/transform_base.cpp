

/**********************ROS**********************************/

#include <ros/ros.h>

#include <axif_tf/getPoint.h>

#include <geometry_msgs/PointStamped.h>

#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>

#include <tf/transform_datatypes.h>

#include <tf_conversions/tf_eigen.h>

#include <tf/transform_listener.h>

/*********************EUGEN*********************************/

#include <eigen3/Eigen/Core>

#include <eigen3/Eigen/Dense>

#include <eigen3/Eigen/Geometry>

#include <cmath>

/********************OPENCVLIBRARY***************************/

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

/*******************PACKAGE_HEADER***************************/

#include <opencvtest/pixel_point0.h>

// #include <dobot/mypose.h>

#include <dobot/GetPose.h>

using namespace std;

axif_tf::getPoint msg10; // 存储物块在机械臂基座坐标系下的中心坐标

axif_tf::getPoint msg20; // 存储物块在机械臂基座坐标系下的中心坐标

axif_tf::getPoint msg30; // 存储物块在机械臂基座坐标系下的中心坐标

axif_tf::getPoint msg40; // 存储物块在机械臂基座坐标系下的中心坐标

axif_tf::getPoint msg50; // 存储物块在机械臂基座坐标系下的中心坐标

axif_tf::getPoint msg60; // 存储物块在机械臂基座坐标系下的中心坐标

void callbackCalculateAxif1(axif_tf::getPoint::ConstPtr message);

void callbackCalculateAxif2(axif_tf::getPoint::ConstPtr message);

void callbackCalculateAxif3(axif_tf::getPoint::ConstPtr message);

void callbackCalculateAxif4(axif_tf::getPoint::ConstPtr message);

void callbackCalculateAxif5(axif_tf::getPoint::ConstPtr message);

void callbackCalculateAxif6(axif_tf::getPoint::ConstPtr message);

ros::NodeHandle *n_p = NULL;

ros::Publisher *pointer_result_10_pub = NULL; // 红

ros::Publisher *pointer_result_20_pub = NULL; // blue

ros::Publisher *pointer_result_30_pub = NULL; // blue

ros::Publisher *pointer_result_40_pub = NULL; // blue

ros::Publisher *pointer_result_50_pub = NULL; // blue

ros::Publisher *pointer_result_60_pub = NULL; // blue

geometry_msgs::PointStamped result_in;

geometry_msgs::PointStamped result_out;

tf::TransformListener *listener_ptr;

int main(int argc, char **argv)

{

    ros::init(argc, argv, "transform_base");

    ros::NodeHandle n;

    n_p = &n;

    tf::TransformListener listener;

    listener_ptr = &listener;

    ros::Subscriber pixel_sub1 = n.subscribe("result_1", 100, callbackCalculateAxif1);

    ros::Subscriber pixel_sub2 = n.subscribe("result_2", 100, callbackCalculateAxif2);

    ros::Subscriber pixel_sub3 = n.subscribe("result_3", 100, callbackCalculateAxif3);

    ros::Subscriber pixel_sub4 = n.subscribe("result_4", 100, callbackCalculateAxif4);

    ros::Subscriber pixel_sub5 = n.subscribe("result_5", 100, callbackCalculateAxif5);

    ros::Subscriber pixel_sub6 = n.subscribe("result_6", 100, callbackCalculateAxif6);

    ros::Publisher result_1_pub = n.advertise<axif_tf::getPoint>("result_10", 1); // 发布相机坐标系下的红色物块中心坐标

    ros::Publisher result_2_pub = n.advertise<axif_tf::getPoint>("result_20", 1); // 发布相机坐标系下的green物块中心坐标

    ros::Publisher result_3_pub = n.advertise<axif_tf::getPoint>("result_30", 1); // 发布相机坐标系下的blue物块中心坐标

    ros::Publisher result_4_pub = n.advertise<axif_tf::getPoint>("result_40", 1); // 发布相机坐标系下的yellow物块中心坐标

    ros::Publisher result_5_pub = n.advertise<axif_tf::getPoint>("result_50", 1); // 发布相机坐标系下的purple物块中心坐标

    ros::Publisher result_6_pub = n.advertise<axif_tf::getPoint>("result_60", 1); // 发布相机坐标系下的orange物块中心坐标

    pointer_result_10_pub = &result_1_pub;

    pointer_result_20_pub = &result_2_pub;

    pointer_result_30_pub = &result_3_pub;

    pointer_result_40_pub = &result_4_pub;

    pointer_result_50_pub = &result_5_pub;

    pointer_result_60_pub = &result_6_pub;

    ros::Rate loop_rate(30);

    while (ros::ok())

    {

        ros::spinOnce();

        loop_rate.sleep();
    }
}

void callbackCalculateAxif1(axif_tf::getPoint::ConstPtr message)

{

    for (int i = 0; i < message->x1.size(); i++)

    {

        result_in.point.x = message->x1[i];

        result_in.point.y = message->x2[i];

        result_in.point.z = message->x3[i];

        result_in.header.frame_id = "logitech";

        result_out.header.frame_id = "dobot_base";

        try

        {

            listener_ptr->transformPoint("dobot_base", ros::Time(0), result_in, "logitech", result_out); // 相机坐标系到机械臂坐标系下的变换
        }

        catch (tf::TransformException &ex)

        {

            ROS_ERROR("%s", ex.what());

            ros::Duration(1.0).sleep();

            continue;
        }
        if(result_out.point.x > 0.1) return ;
        msg10.x1.push_back(result_out.point.x + 0.01); // 单位为 m,可加减调参

        msg10.x2.push_back(result_out.point.y + 0.005); // 单位为 m,可加减调参

        msg10.x3.push_back(-0.040); // 获得坐标 Z
    }

    cout << "red" << endl;

    cout << msg10 << endl;

    pointer_result_10_pub->publish(msg10); // 发布出来

    msg10.x1.clear();

    msg10.x2.clear();

    msg10.x3.clear();
}

void callbackCalculateAxif2(axif_tf::getPoint::ConstPtr message)

{

    for (int i = 0; i < message->x1.size(); i++)

    {

        result_in.point.x = message->x1[i];

        result_in.point.y = message->x2[i];

        result_in.point.z = message->x3[i];

        result_in.header.frame_id = "logitech";

        result_out.header.frame_id = "dobot_base";

        try

        {

            listener_ptr->transformPoint("dobot_base", ros::Time(0), result_in, "logitech", result_out); // 相机坐标系到机械臂坐标系下的变换
        }

        catch (tf::TransformException &ex)

        {

            ROS_ERROR("%s", ex.what());

            ros::Duration(1.0).sleep();

            continue;
        }
                if(result_out.point.x > 0.1) return ;

        msg20.x1.push_back(result_out.point.x + 0.01); // 单位为 m,可加减调参

        msg20.x2.push_back(result_out.point.y + 0.000); // 单位为 m,可加减调参

        msg20.x3.push_back(-0.040); // 获得坐标 Z
    }

    cout << "green" << endl;

    cout << msg20 << endl;

    pointer_result_20_pub->publish(msg20); // 发布出来

    msg20.x1.clear();

    msg20.x2.clear();

    msg20.x3.clear();
}

void callbackCalculateAxif3(axif_tf::getPoint::ConstPtr message)

{

    for (int i = 0; i < message->x1.size(); i++)

    {

        result_in.point.x = message->x1[i];

        result_in.point.y = message->x2[i];

        result_in.point.z = message->x3[i];

        result_in.header.frame_id = "logitech";

        result_out.header.frame_id = "dobot_base";

        try

        {

            listener_ptr->transformPoint("dobot_base", ros::Time(0), result_in, "logitech", result_out); // 相机坐标系到机械臂坐标系下的变换
        }

        catch (tf::TransformException &ex)

        {

            ROS_ERROR("%s", ex.what());

            ros::Duration(1.0).sleep();

            continue;
        }
        if(result_out.point.x > 0.1) return ;

        msg30.x1.push_back(result_out.point.x + 0.01); // 单位为 m,可加减调参

        msg30.x2.push_back(result_out.point.y + 0.000); // 单位为 m,可加减调参

        msg30.x3.push_back(-0.040); // 获得坐标 Z
    }

    cout << "blue" << endl;

    cout << msg30 << endl;

    pointer_result_30_pub->publish(msg30); // 发布出来

    msg30.x1.clear();

    msg30.x2.clear();

    msg30.x3.clear();
}

void callbackCalculateAxif4(axif_tf::getPoint::ConstPtr message)

{

    for (int i = 0; i < message->x1.size(); i++)

    {

        result_in.point.x = message->x1[i];

        result_in.point.y = message->x2[i];

        result_in.point.z = message->x3[i];

        result_in.header.frame_id = "logitech";

        result_out.header.frame_id = "dobot_base";

        try

        {

            listener_ptr->transformPoint("dobot_base", ros::Time(0), result_in, "logitech", result_out); // 相机坐标系到机械臂坐标系下的变换
        }

        catch (tf::TransformException &ex)

        {

            ROS_ERROR("%s", ex.what());

            ros::Duration(1.0).sleep();

            continue;
        }
        if(result_out.point.x > 0.1) return ;

        msg40.x1.push_back(result_out.point.x + 0.018); // 单位为 m,可加减调参

        msg40.x2.push_back(result_out.point.y + 0.002); // 单位为 m,可加减调参

        msg40.x3.push_back(-0.040); // 获得坐标 Z
    }

    cout << "yellow" << endl;

    cout << msg40 << endl;

    pointer_result_40_pub->publish(msg40); // 发布出来

    msg40.x1.clear();

    msg40.x2.clear();

    msg40.x3.clear();
}

void callbackCalculateAxif5(axif_tf::getPoint::ConstPtr message)

{

    for (int i = 0; i < message->x1.size(); i++)

    {

        result_in.point.x = message->x1[i];

        result_in.point.y = message->x2[i];

        result_in.point.z = message->x3[i];

        result_in.header.frame_id = "logitech";

        result_out.header.frame_id = "dobot_base";

        try

        {

            listener_ptr->transformPoint("dobot_base", ros::Time(0), result_in, "logitech", result_out); // 相机坐标系到机械臂坐标系下的变换
        }

        catch (tf::TransformException &ex)

        {

            ROS_ERROR("%s", ex.what());

            ros::Duration(1.0).sleep();

            continue;
        }
        if(result_out.point.x > 0.1) return ;

        msg50.x1.push_back(result_out.point.x + 0.015); // 单位为 m,可加减调参

        msg50.x2.push_back(result_out.point.y + 0.002); // 单位为 m,可加减调参

        msg50.x3.push_back(-0.040); // 获得坐标 Z
    }

    cout << "purple" << endl;

    cout << msg50 << endl;

    pointer_result_50_pub->publish(msg50); // 发布出来

    msg50.x1.clear();

    msg50.x2.clear();

    msg50.x3.clear();
}

void callbackCalculateAxif6(axif_tf::getPoint::ConstPtr message)

{

    for (int i = 0; i < message->x1.size(); i++)

    {

        result_in.point.x = message->x1[i];

        result_in.point.y = message->x2[i];

        result_in.point.z = message->x3[i];

        result_in.header.frame_id = "logitech";

        result_out.header.frame_id = "dobot_base";

        try

        {

            listener_ptr->transformPoint("dobot_base", ros::Time(0), result_in, "logitech", result_out); // 相机坐标系到机械臂坐标系下的变换
        }

        catch (tf::TransformException &ex)

        {

            ROS_ERROR("%s", ex.what());

            ros::Duration(1.0).sleep();

            continue;
        }
        if(result_out.point.x > 0.1) return ;

        msg60.x1.push_back(result_out.point.x + 0.008); // 单位为 m,可加减调参

        msg60.x2.push_back(result_out.point.y + 0.00); // 单位为 m,可加减调参

        msg60.x3.push_back(-0.040); // 获得坐标 Z
    }

    cout << "orange" << endl;

    cout << msg60 << endl;

    pointer_result_60_pub->publish(msg60); // 发布出来

    msg60.x1.clear();

    msg60.x2.clear();

    msg60.x3.clear();
}
