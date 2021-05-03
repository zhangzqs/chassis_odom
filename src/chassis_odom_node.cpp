//
// Created by sit on 2021/4/10.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

double vx = 0;
double vy = 0;
double vth = 0;

double x = 0.0;
double y = 0.0;
double th = 0.0;
geometry_msgs::Quaternion odom_quat;

ros::Time current_time;
ros::Time last_time;
/**
 * 从底盘获得的速度消息
 * @param msg 速度消息
 */
void speedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    current_time = msg->header.stamp;
    vx = msg->twist.linear.x;
    vy = msg->twist.linear.y;
    //vth = msg->twist.angular.z;

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    //double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    //th += delta_th;

    last_time = current_time;
}



/**
 * 从IMU获取的航偏角消息
 * @param msg 速度消息
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg){
    static bool first = true;
    static double firstTh = 0;

    auto orientation = msg->orientation;
    double yaw = tf::getYaw(orientation);   //获取yaw角信息

    //ROS_INFO("Current Imu Yaw: %f",yaw);
    if(first){
        first = false;
        firstTh = yaw;
    }
    vth = msg->angular_velocity.z;
    th = -(yaw - firstTh);
}

/**
 * 获取里程计的tf消息
 * @return 里程计tf消息
 */
geometry_msgs::TransformStamped getOdomTfData(){
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
}

/**
 * 获取里程计消息
 * @return 里程计消息
 */
nav_msgs::Odometry getOdomMsg(){
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    return odom;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"odom_publisher");  //初始化里程计发布器
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/real_speed", 10, speedCallback); //订阅速度信息
    ros::Subscriber imu_sub = n.subscribe("/imu_data",10,imuCallback);  //订阅IMU数据
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10); //发布里程计信息
    tf2_ros::TransformBroadcaster odom_broadcaster;  //里程计tf发布

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    for(ros::Rate rate(30);ros::ok();rate.sleep()){
        ros::spinOnce();
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf::createQuaternionMsgFromYaw(th);

        //发布tf
        odom_broadcaster.sendTransform(getOdomTfData());

        //发布里程计信息
        odom_pub.publish(getOdomMsg());
        ROS_INFO("odom-->x:%f y:%f th:%f", x,y,th);

    }
    return 0;
}
