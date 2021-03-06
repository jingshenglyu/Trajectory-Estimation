

/*
 * @Author       : Jingsheng Lyu
 * @Date         : 2021-05-29 14:50:43
 * @LastEditors  : Jingsheng Lyu
 * @LastEditTime : 2021-09-25 17:51:58
 * @FilePath     : /undefined/home/jingsheng/catkin_ws/src/mbot_vision/src/move_to_target.cpp
 * @Github       : https://github.com/jingshenglyu
 * @Web          : https://jingshenglyu.github.io/
 * @E-Mail       : jingshenglyu@gmail.com
 */
#include<math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#define STATUS_EXPLORING    (0)
#define STATUS_CLOSE_TARGET (1)
#define STATUS_GO_HOME      (2)

#define GET_TARGET_SIZE     (11000)

ros::Publisher vel_pub;
ros::Publisher cmd_pub;
ros::Publisher voice_pub;

int status_flag = STATUS_EXPLORING;

// When a subscribed message is received, the message callback function is entered
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // Print out the incoming message
    ROS_INFO("Target pose: x:%0.6f, y:%0.6f, z:%0.6f", msg->position.x, msg->position.y, msg->position.z);


    // Move function   
    if(status_flag==STATUS_EXPLORING)
    {
        status_flag=STATUS_CLOSE_TARGET;
        std_msgs::Int8 cmd;
        cmd.data=STATUS_CLOSE_TARGET;
        cmd_pub.publish(cmd);
        std_msgs::String msg;
        msg.data="Discover the target and move forward";
        voice_pub.publish(msg);
        
    }
    else if(status_flag == STATUS_CLOSE_TARGET && msg->position.z > GET_TARGET_SIZE)
    {
        status_flag = STATUS_GO_HOME;
        std_msgs::Int8 cmd;
        cmd.data=STATUS_GO_HOME;
        cmd_pub.publish(cmd);
        
        std_msgs::String msg;
        msg.data="Reaching the target";
        voice_pub.publish(msg);
    }
    else if(status_flag==STATUS_CLOSE_TARGET)
    { 
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x=(150000 - msg->position.z)/150000*0.6;
        //vel_msg.angular.z=0;
        vel_msg.angular.z=(640-msg->position.x)/640*0.1;

        vel_pub.publish(vel_msg);
        ROS_INFO("Publish velocity command[%0.2f m/s,%0.2f rad/s]",vel_msg.linear.x,vel_msg.angular.z);

    }


}

int main(int argc, char **argv)
{
	// ROS node initialisation
	ros::init(argc, argv, "move_to_target");

	// Creating node handles
	ros::NodeHandle n;
    // Create a Subscriber, subscribe to a topic named /turtle1/pose and register the callback function poseCallback
    ros::Subscriber pose_sub = n.subscribe("/object_detect_pose", 10, poseCallback);
	// Create a Publisher that publishes a topic named cmd_vel with message type geometry_msgs::Twist and queue length 10
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	// Create a Publisher to publish a topic named cmd_vel with message type std_msgs::Int8 and queue length 10
	cmd_pub = n.advertise<std_msgs::Int8>("/exploring_cmd", 10);
    // Publish output content    
    voice_pub = n.advertise<std_msgs::String>("voiceWords", 1000); 

    // Loop to wait for callback functions
    ros::spin();

	return 0;
}
