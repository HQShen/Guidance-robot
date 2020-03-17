#include <ros/ros.h>
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sound_play/sound_play.h"

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal, double zGoal, double wGoal);

std::string choice;

void command_callback(const std_msgs::String::ConstPtr& msg) {
	choice = msg->data.c_str();
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/** declare the coordinates of interest **/
double xorigin = 6.56 ;
double yorigin = 4.74;
double zorigin = -0.91;
double worigin = 0.40;

double xOffice3219 = 26.81 ;
double yOffice3219 = 25.61;
double zOffice3219 = 0.28;
double wOffice3219 = 0.95;

double xOffice3220 = 30.36 ;
double yOffice3220 = 29.66;
double zOffice3220 = 0.68;
double wOffice3220 = 0.73;

double xOffice3246 = 22.96 ;
double yOffice3246 = 10.20;
double zOffice3246 = 0.11;
double wOffice3246 = 0.99;

double xOffice3234 = 37.85;
double yOffice3234 = 25.34;
double zOffice3234 = -0.80;
double wOffice3234 = 0.59;

double xOffice3258 = 8.53; //3258, 3260,3256
double yOffice3258 = -3.65;
double zOffice3258 = -0.93;
double wOffice3258 = 0.35;

//std::set<std::string> choices = { "origin", "3219", "3220" , "3246" , "3234", "3258", "quit"};

bool goalReached = false;

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::Subscriber command_sub = n.subscribe("/moveforward", 1000, command_callback);
	sound_play::SoundClient sc;
	ros::spinOnce();
	
	while(!goalReached){
		// choice =choose();
		if (choice == "origin"){
			goalReached = moveToGoal(xorigin, yorigin, zorigin, worigin);
		}else if (choice == "one"){
			goalReached = moveToGoal(xOffice3219, yOffice3219, zOffice3219, wOffice3219);
		}else if (choice == "two"){
			goalReached = moveToGoal(xOffice3220, yOffice3220, zOffice3220, wOffice3220);
		}else if (choice == "three"){
			goalReached = moveToGoal(xOffice3246, yOffice3246, zOffice3246, wOffice3246);
		}else if (choice == "four"){
			goalReached = moveToGoal(xOffice3234, yOffice3234, zOffice3234, wOffice3234);
		}else if (choice == "five"){
			goalReached = moveToGoal(xOffice3258, yOffice3258, zOffice3258, wOffice3258);
		}
		ros::spinOnce();
	}

	sc.say("you reached your destination, have a nice day");
	ros::Duration(10.0).sleep();
	goalReached = moveToGoal(xorigin, yorigin, zorigin, worigin);

	return 0;
}

bool moveToGoal(double xGoal, double yGoal, double zGoal, double wGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = zGoal;
	goal.target_pose.pose.orientation.w = wGoal;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}
