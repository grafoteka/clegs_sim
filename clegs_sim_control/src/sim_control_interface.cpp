//=====================================================================
//	Author:	Raúl Cebolla Arroyo
//	File:
//	Version:
//	Description:
//	Changelog:
//=====================================================================

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <clhero_gait_controller/LegCommand.h>
#include <clhero_gait_controller/LegState.h>
#include <controller_manager_msgs/SwitchController.h>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <cmath>
#include <thread>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define LOOP_RATE 100 //Rate at which the node checks for callbacks
#define LEG_NUMBER 6 //Number of legs of the robot
#define CONTROL_RATE 100 //Rate at which the node sends the control of each leg
#define PI 3.14159265359
#define VEL_THR 0.5235987755982988 // = 5 [rpm]
#define ANG_P_THR 0.12217304763960307 // 7[º]
#define ANG_V_THR 0.6108652381980153 // 35 [º]
#define REF_ANG_RESOLUTION_LIMIT 0.003490658503988659 // 0.2[º]
#define POSITION_CONTROL 107
#define VELOCITY_CONTROL 108

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------

//Class that keeps the info of the last msg
class Command {

  public:

    float pos[LEG_NUMBER];
    float vel[LEG_NUMBER];
    float acel[LEG_NUMBER];
    float decel[LEG_NUMBER];
    bool new_acel_profile[LEG_NUMBER];
    bool position_command[LEG_NUMBER];

    //Default constructor
    Command (){
      for (int i = 0; i<LEG_NUMBER; i++){
        pos[i] = 0;
        vel[i] = 0;
        acel[i] = 0;
        decel[i] = 0;
        new_acel_profile[i] = false;
        return;
      }
    }

    //Method that updates the com
    void updateCommand (const clhero_gait_controller::LegCommand::ConstPtr& msg){
      for(int i=0; i<LEG_NUMBER; i++){
        if( (pos[i] != msg->pos[i]) || (vel[i] != msg->vel[i]) ){
          pos[i] = msg->pos[i];
          vel[i] = msg->vel[i];
          acel[i] = msg->acel[i];
          decel[i] = msg->decel[i];
          new_acel_profile[i] = msg->new_acel_profile[i].data;
          position_command[i] = msg->position_command[i].data;
        }
      }
      return;
    }

};

struct Leg_state {
  float position [LEG_NUMBER];
  float raw_position [LEG_NUMBER];
  float velocity [LEG_NUMBER];
  float effort [LEG_NUMBER];
};

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

//Publisher of the legs_state_msg
ros::Publisher legs_state_pub;

//Client to switch the controller type
ros::ServiceClient switch_controller_cli;

//Publisher of each of the command msgs
std::vector<ros::Publisher> position_controller_command_pub;
std::vector<ros::Publisher> velocity_controller_command_pub;

//Legs'command
Command command;

//Legs'command mutex
std::mutex command_mtx;

//State of the legs
Leg_state leg_state;

//Legs state mutex
std::mutex leg_state_mtx;

//Controller's namespace
const std::string controller_namespace = "/hexapodo";

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that performs the controllers'switch
bool changeControllerType (int leg, int controller_type){
  
  controller_manager_msgs::SwitchController msg;
  std::string controller_type_name;

  if(controller_type == POSITION_CONTROL){

    controller_type_name = "position controller";
    //msg.request.start_controllers.push_back(controller_namespace + "/motor" + std::to_string(leg) + "_position_controller");
    msg.request.start_controllers.push_back("motor" + std::to_string(leg) + "_position_controller");
    //msg.request.stop_controllers.push_back(controller_namespace + "/motor" + std::to_string(leg) + "_velocity_controller");
    msg.request.stop_controllers.push_back("motor" + std::to_string(leg) + "_velocity_controller");
  
  }else if(controller_type == VELOCITY_CONTROL){

    controller_type_name = "velocity controller";
    //msg.request.start_controllers.push_back(controller_namespace + "/motor" + std::to_string(leg) + "_velocity_controller");
    msg.request.start_controllers.push_back("motor" + std::to_string(leg) + "_velocity_controller");
    //msg.request.stop_controllers.push_back(controller_namespace + "/motor" + std::to_string(leg) + "_position_controller");
    msg.request.stop_controllers.push_back("motor" + std::to_string(leg) + "_position_controller");
  
  }else{

    return false;

  }

  msg.request.strictness = 1;

  //sends the request for the change
  if(!switch_controller_cli.call(msg)){
    ROS_ERROR("[clhero_simulation_interface]: Could not call switch_controller service.");
  }

  if(!msg.response.ok){
    std::string error_msg = "[clhero_simulation_interface]: Could not switch actuator " + std::to_string(leg) + " to " + controller_type_name;
    ROS_ERROR(error_msg.c_str());
  }

  return true;

}

//Function that calcs the velocity for position control
double posControlVel (double angle, double angle_ref, double vel_ref){
  
  double vel;

  //The difference between the position and the reference is set according to the sign
  //of the velocity
  double dif_ang;
  if(vel_ref > 0){
    dif_ang = angle_ref - angle;
  }else{
    dif_ang = angle - angle_ref;
  }

  //Checks if the difference is negative and changes it in that case
  if(dif_ang < 0){
    dif_ang = 2*PI + dif_ang;
  }

  //calcs the velcity
  if(dif_ang > ANG_V_THR){
    //If the difference is great enough, the velocity is the referenced
    vel = vel_ref;
  }else if(fabs(angle - angle_ref) < ANG_P_THR){
    vel = 0;
  }else{
    //vel = (dif_ang - angle_ref)/ANG_V*vel_ref;
    vel = vel_ref/(ANG_V_THR - 0)*(dif_ang - 0);
  }

  return vel;
}

//Function that turns the readings of position into a range of [0, 360]
double fixAngle (double angle){
  double fixed_angle = (angle - 2*PI*trunc(angle/(2*PI)));
  if(fixed_angle < 0){
    fixed_angle = 2*PI + fixed_angle;
  }
  return fixed_angle;
}

//Thread for each leg command control
void control_leg (int leg){

  //Rate of the control
  ros::Rate loop_rate (CONTROL_RATE);

  //Message to be sent
  std_msgs::Float64 command_msg;

  //Movement control variables
  double ang, ang_ref, vel_ref, prev_ang_ref;

  leg_state_mtx.lock();
  prev_ang_ref = ang = leg_state.position[leg-1];
  leg_state_mtx.unlock();

  command_mtx.lock();
  ang_ref = command.pos[leg-1];
  vel_ref = command.vel[leg-1];
  command_mtx.unlock();

  //Type of the current controller
  int controller_type = VELOCITY_CONTROL;

  while(ros::ok()){
    //The behaviour of the control shall change whether it is a position command or not
    if(command.position_command[leg-1]){
      //Position command
      
      leg_state_mtx.lock();
      ang = leg_state.position[leg-1];
      leg_state_mtx.unlock();

      command_mtx.lock();
      ang_ref = command.pos[leg-1];
      vel_ref = command.vel[leg-1];
      command_mtx.unlock();

      //Checks if the position angle is in the threshold for the position cotroller 
      if(fabs(ang - ang_ref) < ANG_P_THR){
        
        if(controller_type == VELOCITY_CONTROL){
          //If the velocity controller is still active, request the switch into position controller
          if(changeControllerType(leg, POSITION_CONTROL)){
            controller_type = POSITION_CONTROL;
          }
        }

        if(controller_type == POSITION_CONTROL){
          //With the position controller now running, the command msg is built
          command_msg.data = ang_ref;
        }        

      }else{

        if(controller_type == POSITION_CONTROL){
          //The controller shall change into velocity again only if the reference angle is a new one
          if(fabs(prev_ang_ref - ang_ref) > REF_ANG_RESOLUTION_LIMIT){
            //If the position controller is still active, request the switch into velocity controller
            if(changeControllerType(leg, VELOCITY_CONTROL)){
              controller_type = VELOCITY_CONTROL;
            }
          }else{
            if(controller_type == POSITION_CONTROL){
              //With the position controller now running, the command msg is built
              command_msg.data = ang_ref;
            }  
          }
        }

        if(controller_type == VELOCITY_CONTROL){
          //With the position controller now running, the command msg is built
          command_msg.data = posControlVel(ang, ang_ref, vel_ref);
        } 
      }

    }else{
      //Velocity command
      //In velocity command, just the velocity shall be sent
      command_mtx.lock();
      command_msg.data = command.vel[leg-1];
      command_mtx.unlock();
    }

    
    //Sends the msg command depending on whether the control type
    if(controller_type == POSITION_CONTROL){
      position_controller_command_pub[leg-1].publish(command_msg);
    }else if(controller_type == VELOCITY_CONTROL){
      velocity_controller_command_pub[leg-1].publish(command_msg);
    }

    prev_ang_ref = ang_ref;

    loop_rate.sleep();
    
  }

  return;
  
}

//Callback for joint states msgs
void jointStatesCallback (const sensor_msgs::JointState::ConstPtr& msg){

  clhero_gait_controller::LegState leg_state_msg;

  leg_state_mtx.lock();
  for(int i=0; i < LEG_NUMBER; i++){
    leg_state_msg.pos.push_back(fixAngle(msg->position[i]));
    leg_state_msg.vel.push_back(msg->velocity[i]);
    leg_state_msg.torq.push_back(msg->effort[i]);
    leg_state.position[i] = leg_state_msg.pos[i];
    leg_state.raw_position[i] = msg->position[i];
    leg_state.velocity[i] = leg_state_msg.vel[i];
    leg_state.effort[i] = leg_state_msg.torq[i];
  }
  leg_state_mtx.unlock();

  legs_state_pub.publish(leg_state_msg);

  return;
}

//Callback for leg command msgs
void legCommandCallback (const clhero_gait_controller::LegCommand::ConstPtr& msg){
  command_mtx.lock();
  command.updateCommand(msg);
  command_mtx.unlock();
  return;
}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main(int argc, char **argv){

  //----------------------------------------------------
  //    ROS starting statements
  //----------------------------------------------------

  ros::init(argc, argv, "clhero_simulation_interface");
  ros::NodeHandle nh;

  ros::Rate loop_rate (LOOP_RATE);

  //Publishers
  legs_state_pub = nh.advertise<clhero_gait_controller::LegState>("legs_state", 1000);
  for(int i=0; i < LEG_NUMBER; i++){
    velocity_controller_command_pub.push_back(nh.advertise<std_msgs::Float64>(controller_namespace + "/motor" + std::to_string(i+1) + "_velocity_controller/command", 1000));
    position_controller_command_pub.push_back(nh.advertise<std_msgs::Float64>(controller_namespace + "/motor" + std::to_string(i+1) + "_position_controller/command", 1000));
  }

  //Topics subscription
  ros::Subscriber leg_command_sub = nh.subscribe("legs_command", 1000, legCommandCallback);
  ros::Subscriber joint_states_sub = nh.subscribe("/hexapodo/joint_states", 1000, jointStatesCallback);

  //Client declaration
  switch_controller_cli = nh.serviceClient<controller_manager_msgs::SwitchController>(controller_namespace + "/controller_manager/switch_controller");

  //threads with the control of each leg
  std::thread control_leg_1_thr (control_leg, 1);
  std::thread control_leg_2_thr (control_leg, 2);
  std::thread control_leg_3_thr (control_leg, 3);
  std::thread control_leg_4_thr (control_leg, 4);
  std::thread control_leg_5_thr (control_leg, 5);
  std::thread control_leg_6_thr (control_leg, 6);

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  control_leg_1_thr.join();
  control_leg_2_thr.join();
  control_leg_3_thr.join();
  control_leg_4_thr.join();
  control_leg_5_thr.join();
  control_leg_6_thr.join();

  return 0;

}
