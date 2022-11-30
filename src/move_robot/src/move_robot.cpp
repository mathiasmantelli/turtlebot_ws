#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <move_robot/move_robot.h> /////////// FUNFA PARA COMPILAR, FALTA testar

#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#include <sstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using std::vector;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using std::string;

geometry_msgs::Pose robot_pose, old_robot_on_formation, robot_on_formation;
float grad_desc, grad_desc_mag;
bool finished_operation;

float initial_robot_orientation, initial_robot_x, initial_robot_y;

#define DIST_MIN 0.15

///// fonte: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
float constrainAngle(float x){
    float new_x = fmod(x + M_PI, 2.0 * M_PI);
    if (new_x < 0.0)
        new_x += 2.0 * M_PI;
    return new_x - M_PI;
}

///////////////////////////////////
/////                         /////
///// COMUNICATION FUNCTIONS  /////
/////                         /////
///////////////////////////////////

void odom_callback(const nav_msgs::Odometry& msg){
	robot_pose = msg.pose.pose;
  robot_pose.position.x += initial_robot_x;
  robot_pose.position.y += initial_robot_y;
	// ROS_INFO("FOI %f em x", robot_pose.position.x);
}

void robot_on_formation_callback(const geometry_msgs::PoseStamped& msg){
  old_robot_on_formation = robot_on_formation;
  robot_on_formation = msg.pose;
}

void grad_desc_callback(const std_msgs::Float64& msg){
  grad_desc = msg.data;
}

void grad_desc_mag_callback(const std_msgs::Float64& msg){
  grad_desc_mag = msg.data;
}

void finished_operation_callback(const std_msgs::Bool& msg){
  finished_operation = msg.data;
}

class Robot{
  public:
    char name[7] = "robot0";
    move_base_msgs::MoveBaseGoal goal, old_goal;
    int num;

    void SetupVariables();
    bool UpdateGoalPosition();
    Robot(int n){
      name[5] += n;
      num = n;
      vel_max = 0.5;////////PARA ACHAR SIG FAULT DO CENTRAL0.5;//////0.4;///////2;//0.22 for turtlebot

      //pesos
      mi_Bi = 0.65;///0.65; 0.5 testado tambem
      n = 0.4; 

      // make_plan_topic[] = "/robot3/move_base/NavfnROS/make_plan";
      strcat(make_plan_topic, "/");
      strcat(make_plan_topic, name);
      strcat(make_plan_topic, "/move_base/NavfnROS/make_plan");
      test_goal_msg.request.start.header.frame_id = "map";
      test_goal_msg.request.goal.header.frame_id = "map";
      test_goal_msg.request.tolerance = 0.2;

      goal.target_pose.header.frame_id = "map";

      // contador = 0;
    }

  private:
    // int contador;
    float vel_max;
    float old_robot_orientation, old_force_z_orientation;
    float robot_orientation, force_z_orientation;
    float mi_Bi, n; //pesos
    nav_msgs::GetPlan test_goal_msg;
    char make_plan_topic[37] = {};

    float Psi(float x);
    std::tuple<float, float> CalculateForce_z();

};

///////////////////////////////////
/////                         /////
/////    CENTRAL FUNCTIONS    /////
/////                         /////
///////////////////////////////////

void Robot::SetupVariables(){
  goal.target_pose.pose = robot_pose;

  old_robot_orientation = constrainAngle(ComputeYaw(robot_pose.orientation) + initial_robot_orientation);
  old_robot_on_formation = robot_pose;

  float z_x, z_y;
  std::tie(z_x, z_y) = CalculateForce_z();

  old_force_z_orientation = atan2(z_y, z_x);
}

float Robot::Psi(float x){
  if(x > (M_PI/2.0))  return    0.0;
  else              return cos(x);
}

std::tuple<float, float> Robot::CalculateForce_z(){
  float x, y;

  x = robot_on_formation.position.x - robot_pose.position.x;
  y = robot_on_formation.position.y - robot_pose.position.y;

  return std::make_tuple(x, y);
}



bool Robot::UpdateGoalPosition(){

  ros::Time stamp_comum;
  // std_msgs::Header stamp_comum;

  old_robot_orientation = constrainAngle(ComputeYaw(robot_pose.orientation) + initial_robot_orientation);

  float x_k, y_k; //robot actual position

  x_k = robot_pose.position.x;
  y_k = robot_pose.position.y;

  float alfa;

  float environment, formation;

  float grad_orientation = grad_desc;
  float grad_mag = grad_desc_mag;

  robot_orientation = constrainAngle(n*old_robot_orientation + (1.0-n)*grad_orientation);

  float delta_r_x, delta_r_y;

  float z_x, z_y;
  std::tie(z_x, z_y) = CalculateForce_z();

  force_z_orientation = atan2(z_y, z_x);
  float z_mag = sqrt(pow(z_x, 2.0) + pow(z_y, 2.0));

  alfa = (z_mag < 1.0? z_mag : 1.0);
  // if(z_mag < 1.0)
  //   alfa = z_mag;
  // else
  //   alfa = 1.0;

  ///////////////x
  environment = Psi(abs(constrainAngle(old_robot_orientation - grad_orientation)))        * cos(robot_orientation);
  formation =   Psi(abs(constrainAngle(  force_z_orientation - old_force_z_orientation))) * cos(force_z_orientation);
  delta_r_x =  vel_max * (mi_Bi * environment + (1.0-mi_Bi) * alfa * formation);
    
  //////////////y
  environment = Psi(abs(constrainAngle(old_robot_orientation - grad_orientation)))        * sin(robot_orientation);
  formation =   Psi(abs(constrainAngle(  force_z_orientation - old_force_z_orientation))) * sin(force_z_orientation);
  delta_r_y =  vel_max * (mi_Bi * environment + (1.0-mi_Bi) * alfa * formation);

  old_force_z_orientation = force_z_orientation;

  goal.target_pose.pose.position.x = x_k + delta_r_x;
  goal.target_pose.pose.position.y = y_k + delta_r_y;
  goal.target_pose.pose.position.z = 0.0;

  goal.target_pose.pose.orientation = computeQuaternionFromYaw(robot_orientation);

  stamp_comum = ros::Time::now();

	goal.target_pose.header.stamp = stamp_comum;
  // old_robot_orientation = robot_orientation;

  // if(num == 3){
  test_goal_msg.request.start.pose = robot_pose;
  test_goal_msg.request.start.header.stamp = stamp_comum;
  test_goal_msg.request.goal = goal.target_pose;

  bool resul = ros::service::call(make_plan_topic, test_goal_msg);
    
  // if(num == 3) ROS_WARN("robot3 -> %s plano de %ld poses", (resul? "OK": "ERRO"), test_goal_msg.response.plan.poses.size());

  return (test_goal_msg.response.plan.poses.size() > 0);
  // }

  // return true;
  // usar make_plan

  ///////COLOCAR LIMITE NO GOAL PRA ELE N SAIR DO CAMP POT
  ////// OU IR PRA AREAS TIPO ENTRE OS PES DA CADEIRA
}

///////////////////////////////////
/////                         /////
/////      MAIN FUNCTION      /////
/////                         /////
///////////////////////////////////

int main(int argc, char** argv){
	// Initialize
	ros::init(argc, argv, "move_robot", ros::init_options::AnonymousName);

  ros::NodeHandle n("~");

  int i;// = -1;
  // n.param("robot_num", i, 0);

  std::string param_i;
  if (n.searchParam("robot_num", param_i)){
    n.getParam(param_i, i);
  }else{
    ROS_WARN("No param 'robot_num' found in an upward search");
    return 0;
  }

  Robot robot(i);

  initial_robot_x = 0.0;
  if (n.searchParam("initial_robot_x", param_i)){
    n.getParam(param_i, initial_robot_x);
  }else{
    ROS_WARN("No param 'initial_robot_x' found in an upward search");
  }

  initial_robot_y = 0.0;
  if (n.searchParam("initial_robot_y", param_i)){
    n.getParam(param_i, initial_robot_y);
  }else{
    ROS_WARN("No param 'initial_robot_y' found in an upward search");
  }

  float dist_robot_goal=DIST_MIN + 1.0;

  auto node_name = ros::this_node::getName();
  ROS_WARN("ROBOT: %s on the node %s", robot.name, node_name.c_str());

  std_srvs::Empty emptymsg;
  char clear_costmaps_topic[33]={};
  strcat(clear_costmaps_topic, "/");
  strcat(clear_costmaps_topic, robot.name);
  strcat(clear_costmaps_topic, "/move_base/clear_costmaps");
  // ros::service::call(clear_costmaps_topic, emptymsg);

  // // //nav_msgs/GetPlan
  // // nav_msgs::GetPlan test_goal_msg;
  // char make_plan_topic[28]={};
  // strcat(make_plan_topic, "/");
  // strcat(make_plan_topic, robot.name);
  // strcat(make_plan_topic, "/move_base/make_plan");

  ros::Subscriber odom_sub, map_sub, robot_on_formation_sub, robot_grad_sub, robot_grad_mag_sub, finished_operation_sub;

  char odom_topic[1+6+1+4+1] = {};
  strcat(odom_topic, "/");
  strcat(odom_topic, robot.name);
  strcat(odom_topic, "/odom");
  odom_sub = n.subscribe(odom_topic, 1, odom_callback);
 
  char move_base_topic[1+6+1+9+1] = {};
  strcat(move_base_topic, "/");
  strcat(move_base_topic, robot.name);
  strcat(move_base_topic, "/move_base");
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac(move_base_topic, true);
  // ROS_INFO("TOPICO %s", move_base_topic);

  char robot_on_formation_topic[1+5+1+14] = {};
  strcat(robot_on_formation_topic, "/");
  strcat(robot_on_formation_topic, robot.name);
  strcat(robot_on_formation_topic, "_on_formation");
	robot_on_formation_sub = n.subscribe(robot_on_formation_topic, 1, robot_on_formation_callback);

  char robot_grad_topic[1+5+1+11] = {};
  strcat(robot_grad_topic, "/");
  strcat(robot_grad_topic, robot.name);
  strcat(robot_grad_topic, "_grad_desc");
  robot_grad_sub = n.subscribe(robot_grad_topic, 1, grad_desc_callback);
  grad_desc = 0.0;
  strcat(robot_grad_topic, "_mag");
  robot_grad_mag_sub = n.subscribe(robot_grad_topic, 1, grad_desc_mag_callback);
  grad_desc_mag = 0.0;

  finished_operation_sub = n.subscribe("/finished_operation", 1, finished_operation_callback);
  finished_operation = false;

  ros::spinOnce();

	// Wait 5 sec for move base action server to come up
	while(!ac.waitForServer (ros::Duration(5.0))){
		ROS_WARN("Waiting for the move_base action server: %s :for %s to come up", move_base_topic, robot.name);
    if(!ros::ok()){
      return 0;
    }
  }

	ROS_WARN("OK move_base action server: %s :for %s", move_base_topic, robot.name);

  ros::spinOnce();
	  
	ros::Duration(3.0).sleep();
  ros::spinOnce();

  
  // if(robot.num == 1){
  //   robot_pose.position.x += 0.537;
  //   robot_pose.position.y += 3.897;
  // }

  // if(robot.num == 2){
  //   robot_pose.position.x += 0.537;
  //   robot_pose.position.y += 2.897;
  // }
  robot_pose.position.z = 0.0;
  initial_robot_orientation = 0.0;

  robot.SetupVariables();

  ros::Duration(3.0).sleep();

  int cont = 0;
// finished_operation = false;
	while (ros::ok() && !finished_operation){
		ros::spinOnce();

  // if(robot.num == 1){
  //   robot_pose.position.x += 0.537;
  //   robot_pose.position.y += 3.897;
  // }


  // if(robot.num == 2){
  //   robot_pose.position.x += 0.537;
  //   robot_pose.position.y += 2.897;
  // }
  
    if(robot.UpdateGoalPosition()){
      ac.sendGoal (robot.goal);
    }

    cont++;
    //fonte https://answers.ros.org/question/267485/local_costmap-ghost-objects-restarting-move_base/
    if(cont%2 == 0)
      ros::service::call(clear_costmaps_topic, emptymsg);
    if(cont >= 1000000000) cont = 0;

    ros::Duration(1.0).sleep();
  }
  
  ROS_WARN("ROBOT: %s terminou de operar", robot.name);
  return 0;
}


/*
TENTATIVA DENTRO DO WHILE
		ros::spinOnce();
    // cont++;
    // //fonte https://answers.ros.org/question/267485/local_costmap-ghost-objects-restarting-move_base/
    // if(cont%400 == 0)
    //   ros::service::call(clear_costmaps_topic, emptymsg);
    // if(cont >= 1000000000) cont = 0;
    
    if(robot.UpdateGoalPosition()){
      ac.sendGoal (robot.goal);
      ac.waitForResult();

      // do{
      //   ros::spinOnce();
      //   dist_robot_goal = sqrt(pow(robot.goal.target_pose.pose.position.x - robot_pose.position.x, 2.0) + pow(robot.goal.target_pose.pose.position.y - robot_pose.position.y, 2.0));
      // }while (dist_robot_goal > DIST_MIN && ros::ok() && !finished_operation);
    
    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      if(!ros::ok() || finished_operation)
        break;
    }

    }
    
    // cont++;
    //fonte https://answers.ros.org/question/267485/local_costmap-ghost-objects-restarting-move_base/
    // if(cont%400 == 0)
      ros::service::call(clear_costmaps_topic, emptymsg);
    // if(cont >= 1000000000) cont = 0;

    // ros::Duration(1.0).sleep();

    // while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
    //   if(!ros::ok() || finished_operation)
    //     break;
    // }

    ROS_WARN("%s OK", robot.name);*/