
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>

///// Recebe pose do robo e map
///// Envia goal e out_map

///////PRÓXIMO PASSO : colocar um BVP simples de exploração. Ver código do Jaci TCC.

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define SEND_TEST_GOALS false

// Define a client for to send goal requests to the move base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using std::string;

void estimated_pose_callback (const geometry_msgs::PoseWithCovarianceStamped & pose)
{
  printf ("Received pose %f, %f, %f\n", pose.pose.pose.position.x,
          pose.pose.pose.position.y, pose.pose.pose.position.z);
}


geometry_msgs::Pose robot_pose;
nav_msgs::OccupancyGrid map_recevied, out_map;

void odom_callback(const nav_msgs::Odometry& msg){
	robot_pose = msg.pose.pose;
	// ROS_INFO("FOI %f em x", robot_pose.position.x);
}

void map_callback(const nav_msgs::OccupancyGrid& msg){
	map_recevied = msg;
	// ROS_INFO("MAPA seq %d", map_recevied.header.seq);//, map_recevied.info.width);
}

int main(int argc, char** argv){
	// Initialize the simple navigation goals node
	ros::init(argc, argv, "move_robot");

  	ros::NodeHandle n;
  	ros::Subscriber odom_sub = n.subscribe("odom", 1, odom_callback);
	ros::Subscriber map_sub = n.subscribe("map", 1, map_callback);
	ros::Publisher out_map_pub = n.advertise<nav_msgs::OccupancyGrid>("out_map", 1);

	//////////ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> poseSub ("estimated_pose", &estimated_pose_callback);
  	//////////n.subscribe(poseSub);
	
	nav_msgs::Odometry odometria;
	

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	// Wait 5 sec for move base action server to come up
	while(!ac.waitForServer (ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	///ros::Subscriber sub = n.subscribe("/odom", 1, odometry);

	move_base_msgs::MoveBaseGoal goal;
	// set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	if(SEND_TEST_GOALS){
		float goals[2][3] =  {{0.5, 0.0, 0}, {0.5, 0.5 , 1.57}}; // {{2.0, 0.0, 0}, {2.0, 2.0 , 1.57}};
		for (int i =0; i< 2; i++){
			// Define a position and orientation for the robot to reach
			goal.target_pose.pose.position.x = goals[i][0];
			goal.target_pose.pose.position.y = goals[i][1];
			goal.target_pose.pose.orientation.w = goals[i][2];
			// Send the goal position and orientation for the robot to reach
			ROS_INFO ("Sending goal");
			ros::spinOnce();//////////PARA RECEBER E MANDAR MSGS
			///////ROS_INFO("FOI %f em x", robot_pose.position.x);
			ROS_INFO("MAPA de %d por %d ", map_recevied.info.height, map_recevied.info.width);
			ac.sendGoal (goal);
			// Wait an infinite time for the results
			ac.waitForResult();
			ros::spinOnce();
			ROS_INFO("MAPA de %d por %d ", map_recevied.info.height, map_recevied.info.width);
			//////////ROS_INFO("FOI %f em x", robot_pose.position.x);
			ros::Duration(5.0).sleep();
		}
		// Check if the robot reached ts goal
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO( "Hooray, reached drop off zone");
		else
			ROS_INFO("The base failed to move forward");
		/////return 0;
	}

	while (ros::ok()){
		out_map.info=map_recevied.info;
		out_map.data=map_recevied.data;
		out_map.header.frame_id="map";
		out_map.header.stamp=ros::Time::now();
		out_map_pub.publish(out_map);
		ros::spinOnce();
	}
	
}

int mainTESTE_BASICO(int argc, char** argv){
	// Initialize the simple navigation goals node
	ros::init(argc, argv, "move_robot");
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	// Wait 5 sec for move base action server to come up
	while(!ac.waitForServer (ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	move_base_msgs::MoveBaseGoal goal;
	// set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	float goals[2][3] =  {{0.5, 0.0, 0}, {0.5, 0.5 , 1.57}}; // {{2.0, 0.0, 0}, {2.0, 2.0 , 1.57}};
	for (int i =0; i< 2; i++){
		// Define a position and orientation for the robot to reach
		goal.target_pose.pose.position.x = goals[i][0];
		goal.target_pose.pose.position.y = goals[i][1];
		goal.target_pose.pose.orientation.w = goals[i][2];
		// Send the goal position and orientation for the robot to reach
		ROS_INFO ("Sending goal");
		ac.sendGoal (goal);
		// Wait an infinite time for the results
		ac.waitForResult();
		ros::Duration(5.0).sleep();
	}
	// Check if the robot reached ts goal
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO( "Hooray, reached drop off zone");
	else
		ROS_INFO("The base failed to move forward");
	return 0;
}