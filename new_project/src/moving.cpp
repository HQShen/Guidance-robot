/****************************************
 * Filename: moveforward.cpp
* Student: Di Gu, Huaqing Shen, Zhao Zhang
* HW #6.1: Multimodal Controller
*
* Description: This is the main control file for HW 6.1,
* which will subscribe data from both RGB-D camera and
* cmvision blob detection nodes. With provided point cloud 
* and color blobs information, the turtlebot will make 
* decision either to move forward or avoid obstacles. 
*
* How to use:
* Usage:
* roscore
* roslaunch turtlebot_bringup minimal.launch
* roslaunch astra_launch astra_pro.launch
* roscd cmvision
* roslaunch cmvision cmvision.launch image:=/camera/rgb/image_raw
* rosparam set /cmvision/color_file home/catkin_ws/src/cmvision/colors.txt
* rosrun cmvision cmvision image:=/camera/rgb/image_raw
* cd ~/catkin_ws
* catkin_make
* roslaunch moveforward moveforward
****************************************/
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <depth_image_proc/depth_traits.h>
#include "opencv_apps/PeopleDetectConfig.h"
#include "opencv_apps/Rect.h"
#include "opencv_apps/RectArrayStamped.h"
#include "sound_play/sound_play.h"

using namespace std; 

ros::Publisher cmdpub; 
ros::Publisher cloud;

/* boolean that flags if target color has been detected  */
bool colorDetected = false;
/* distance from the centroid of point cloud to the camera */
float objDistance;
std_msgs::String msg_c;
float scan_filtered[4];
float ave_people_x;
int num_close[4];
int num_sum = 0;
int unsigned long people_ahead;
int human_ahead = 0;
int stage;
int navigate = 0;
bool flag0 = true;
string command_v;
/************************************************
* Name: void goforward(double speed, double angle)
* Purpose: This function will publish movement command to the ROS
* @input double forward: forward speed, double angle: angular speed
* @return No return, will only publish command
****************************************/
void goforward(double speed, double angle){
    /* Generate geometry Twist and publish to the ROS*/
	geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	cmd->linear.x = speed;
	cmd->angular.z = angle;
	cmdpub.publish(cmd);
}

void voice_callback(const std_msgs::String::ConstPtr& msg){
	command_v = msg->data.c_str();
	ROS_INFO("I heard: [%s]", msg->data.c_str());

}

void people_callback(const opencv_apps::RectArrayStamped::ConstPtr& people_msg){
	ave_people_x = 300;

	int unsigned long people_num = people_msg->rects.size();
	float people_x = 0;
	if(!people_msg->rects.empty()){
		people_ahead = 1;
		for(int i = 0; i < people_num; ++i){
			people_x += people_msg->rects[i].x;
		}
	}
	else{
		people_ahead = 0;
	}
	if(people_num != 0){
		ave_people_x = people_x / people_num;
	}
}  

void laserscan_callback(const sensor_msgs::LaserScan& input_scan){
	float scan_len = (input_scan.angle_max - input_scan.angle_min)/input_scan.angle_increment;
	float quarter = scan_len/4;
	human_ahead = 0;
	for(int j = 0; j < 4; ++j){
		scan_filtered[j] = 0;
		for(int i = (int) quarter * j; i < quarter * (j+1); ++i){
			if(input_scan.ranges[i] < input_scan.range_max && input_scan.ranges[i] > input_scan.range_min){
				scan_filtered[j] += input_scan.ranges[i];
			}
		}
		scan_filtered[j] /= quarter;
	}
	num_sum = 0;
    for(int i = 0; i < 4; ++i){
    	if (scan_filtered[i] < 2.0){
    		human_ahead += 1;
    	}

    	if(scan_filtered[i]<0.5){
    		num_close[i] = 1;
    	}
    	else{
    		num_close[i] = 0;
    	}
    	num_sum += num_close[i];
    }
}

/************************************************
* Name: int main(int argc, char **argv)
* Purpose: This is the main function loop.
****************************************/ 

int main(int argc, char **argv)
{
	do {
		cout << '\n' << "Press a key to continue...";
	} while (cin.get() != '\n');

    /* Initialize ROS */ 
	ros::init(argc, argv, "moveforward");
    /* Initialize node */ 
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	cmdpub = nh.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/navi", 1);
    /* Subscribe blobs and point cloud data */
	ros::Subscriber depthSub = nh.subscribe("/scan", 1000, laserscan_callback);
	ros::Subscriber people = nh.subscribe("/people_detect/found", 1000, people_callback);
	ros::Subscriber voice = nh.subscribe<std_msgs::String>("/recognizer/output", 1000, voice_callback);
	ros::Publisher command = nh.advertise<std_msgs::String>("moveforward", 1);
    /* Setting loop rate */
	ros::Rate r(10);
    /* Warm-up for 5 seconds */
	ros::Duration(5.0).sleep();
	flag0 = true;
    navigate = 0;
    msg_c.data = "zero";
    command.publish(msg_c);

	std::cout << "Finding..." << std::endl;

    while(people_ahead == 0) ros::spinOnce();

	if (flag0){
  		sc.say("Hello, this is Doby. Nice to see you. Do you need me to guide you somewhere?");
  		std::cout<<"Hello, this is Doby. Nice to see you. Do you need me to guide you somewhere?"<<std::endl;
  		ros::Duration(10.0).sleep();
  		flag0 = false;
  		stage = 0;
  	}

  	do {
		if (people_ahead == 1 && human_ahead > 0){
			ros::Duration(2);
			if(ave_people_x<250){
			  	goforward(0, 0.5);
			  }
			  else if(ave_people_x>350){
			  	goforward(0, -0.5);
			  }
			  else{
			  	goforward(0, 0);
			  }	
		}
		if (command_v == "yes" and stage == 0){
			sc.say("Tell me where do you want to go, you have three options: one, room 3219; two, room 3220; and three, room 3246.");
	  		std::cout<<"Tell me where do you want to go, you have three options: one, room 3219; two, room 3220; and three, room 3246."<<std::endl;
			ros::Duration(15.0).sleep();
			stage += 1;
			navigate = 1;
		} else if (command_v == "no" and stage == 0){
			sc.say("OK. Then have a good day.");
			ROS_INFO("Saying: [OK. Then have a good day.]");
			ros::Duration(5.0).sleep();
			flag0 = true;
			navigate = 1;
			stage += 1;
		}
		ros::spinOnce();
	} while (navigate != 1);

	if (flag0 && navigate == 1) {
		msg_c.data = "origin";
		command.publish(msg_c);
		stage = 3;
	}

	while (stage == 1 || stage == 2) {
		if (command_v == "one" and stage == 1){
			sc.say("Just to make sure you want to go to room 3219 right?");
			ROS_INFO("Saying: [Just to make sure you want to go to room 3219 right?]");
			ros::Duration(5.0).sleep();
			msg_c.data = "one";
			stage += 1;
		}
		else if (command_v == "two" and stage == 1){
			sc.say("Just to make sure you want to go to room 3220 right?");
			ROS_INFO("Saying: [Just to make sure you want to go to room 3220 right?]");
			ros::Duration(5.0).sleep();
			msg_c.data = "two";
			stage += 1;
		}
		else if (command_v == "three" and stage == 1){
			sc.say("Just to make sure you want to go to room 3246 right?");
			ROS_INFO("Saying: [Just to make sure you want to go to room 3246 right?]");
			ros::Duration(5.0).sleep();
			msg_c.data = "three";
			stage += 1;
		}
		if (command_v == "yes" && stage == 2){
			sc.say("Ok, please follow me.");
			ROS_INFO("Saying: [Ok, please follow me.]");
			ros::Duration(5.0).sleep();
			stage = 3;
			navigate = 1;
		}
		else if (command_v == "no" && stage == 2){
			sc.say("Sorry please tell me your choice again.");
			ROS_INFO("Saying: [Sorry please tell me your choice again, one is room 3219; two is room 3220; and three is room 3246.]");
			ros::Duration(15.0).sleep();
			stage = 1;
		}
		ros::spinOnce();
	}
	
	ros::Time startTime = ros::Time::now();
	ros::Duration loopDuration(5.0); // 5 seconds
    while(ros::Time::now() < startTime + loopDuration) goforward(0,0.7);
    
	command.publish(msg_c);

	while(ros::ok()){	
		if (navigate == 1){
			if (people_ahead == 1 && human_ahead > 0){
			  	sc.say("Excuse me");
				ros::Duration(5);
			}
		}
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}




