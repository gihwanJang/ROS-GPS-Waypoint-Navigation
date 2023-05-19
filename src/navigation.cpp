#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/bind.hpp>

#define GO 1
#define LEFT 2
#define RIGHT 3
#define STOP 4

// laser topic callback
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan, float&min_distance){
    for (int i = 0; i < scan->ranges.size(); ++i) {
        float distance = scan->ranges[i];

        if (std::isnan(distance)) 
            continue;

        if (distance < min_distance)
            min_distance = distance;
    }
}

// angle topci callback
void angle_callback(const std_msgs::Float64::ConstPtr& angle, float&angle_vel){
    angle_vel = angle->data;
}

// lane detect callback
void lane_callback(const std_msgs::Float64::ConstPtr& lane_pub, float&lane_vel){
    lane_vel = lane_pub->data;
}

// decision direction
int getDirection(float min_distance, float angle){
    // distance (1m)
    float search_distance = 1.0;

    // stop && detect object distance
//    if (min_distance < search_distance){
//        ROS_INFO("Object detected within %f meters", search_distance);
//        return 4;
//    }
//    else{
//        ROS_INFO("object detected within %f meters", search_distance);
//    }

    // right && left
    if(30 < angle && angle < 90)
        return 3;
    else if(270 < angle && angle < 330)
        return 2;

    return 1;
}

double median_correction(geometry_msgs::Twist&msg, float lane){

}

int main(int argc, char** argv) {
    geometry_msgs::Twist msg;
    float distance = 0;
    float angle = 0;
    float lane = 0;

    ros::init(argc, argv, "lidar_object_detection");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber sub_laser = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(&laser_callback, _1, boost::ref(distance)));
    ros::Subscriber sub_angle = nh.subscribe<std_msgs::Float64>("/angle", 1, boost::bind(&angle_callback, _1, boost::ref(angle)));

    //publisher
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Rate rate(5);
    while(ros::ok()){
        ros::spinOnce();

        int direction = getDirection(distance, angle);

        if(direction == GO){
            msg.linear.x = 70;
            msg.angular.x = 0;
            ROS_INFO("Go Straight\n");
        }
        else if(direction == LEFT){
            msg.linear.x = 70;
            msg.angular.x = -11;

            ROS_INFO("Go Left\n");
        }
        else if(direction == RIGHT){
            msg.linear.x = 70;
            msg.angular.x = 11;
            ROS_INFO("Go right\n");
        }
        else if(direction == STOP){
            msg.linear.x = 0;
            msg.angular.x = 0;
            ROS_INFO("Stop\n");
        }
        else{
            msg.linear.x = 0;
            msg.angular.x = 0;
            ROS_INFO("Direction Error\n");
        }
	
	pub_cmd_vel.publish(msg);

        rate.sleep();
    }
    return 0;
}
