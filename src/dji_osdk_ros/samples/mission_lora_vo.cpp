/** @file mission_node.cpp
 *  @version 4.0
 *  @date June 2020
 *
 *  @brief node of hotpoint 1.0/waypoint 1.0.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <std_srvs/SetBool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>

#include <dji_osdk_ros/mission_node.h>
#include <dji_osdk_ros/FlightTaskControl.h>

#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/MissionWpGetInfo.h>
#include <dji_osdk_ros/CameraFocusPoint.h>

#include <opencv2/core/core.hpp>

#include <sstream>
#include <mutex>

#include <csignal>
#include <ros/xmlrpc_manager.h>

using namespace DJI::OSDK;
using namespace dji_osdk_ros;

// global variables
ros::ServiceClient     waypoint_upload_client;
ros::ServiceClient     waypoint_action_client;
ros::ServiceClient     hotpoint_upload_client;
ros::ServiceClient     hotpoint_action_client;
ros::ServiceClient     hotpoint_update_yawRate_client;
ros::ServiceClient     hotpoint_updateRadius_client;
ros::ServiceClient     flight_control_client;
sensor_msgs::NavSatFix gps_pos;

char                   keyboard_input;
ros::Subscriber        gps_pos_subscriber;
ros::Subscriber        force_landing_subscriber;
ros::Subscriber        pos_offsets_subscriber;
ros::Publisher         land_confirmation_email;
ros::Publisher         correctedGPS_publisher_;
bool uploadedMission = false;
bool activatedLanding = false;
short focus_trigger = 4;
float altitude = 0.0f;
float corr = 0.0f;
float x_offset = 0.0f;
float y_offset = 0.0f;
float z_offset = 0.0f;
geometry_msgs::Point32 offsets;
bool offset_completed = false;
bool firstGPS = true;
std::mutex mOff;
std::mutex mCorr;
std::mutex mGPS;

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(std::string mission_file_path);
void setWaypointInitDefaults(dji_osdk_ros::MissionWaypointTask& waypointTask, float idleVelocity);
bool moveByPosOffset(MoveOffset&& move_offset);

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){

  gps_pos = *msg;

  mGPS.lock();
  altitude = gps_pos.altitude;
  mGPS.unlock();

  if(firstGPS){
    mCorr.lock();
    corr = corr - gps_pos.altitude;
    mCorr.unlock();
    firstGPS = !firstGPS;
  }
}

void posOffsetsCallback(const geometry_msgs::Point32::ConstPtr& msg){

  offsets = *msg;

  mOff.lock();
  x_offset = offsets.x;
  y_offset = offsets.y;
  z_offset = offsets.z;
  mOff.unlock();

}

void forceLandingCallback(const std_msgs::String::ConstPtr& msg){

  keyboard_input = msg->data.c_str()[0];
  if (keyboard_input == 'L' || keyboard_input == 'l')
  {
    //Stop current missions and force landing
    bool land_flag = false;

    ROS_INFO("Stop");
    if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                      DJI::OSDK::MISSION_ACTION::STOP).result)
    {
      ROS_INFO("Mission stop command sent successfully");
      land_flag = true;
    }
    else if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT,
                      DJI::OSDK::MISSION_ACTION::STOP).result)
    {
      ROS_INFO("Mission stop command sent successfully");
      land_flag = true;
    }
    else
    {
      ROS_WARN("Failed sending mission stop command");
    }

    if (land_flag)
    {
      ROS_INFO("Land");
      if (land())
      {
        ROS_INFO("Land command sent successfully");
      }
      else
      {
        ROS_WARN("Failed sending land command");
      }
    }
  }
}

bool runWaypointMission(std::string mission_file_path, int responseTimeout){

  ros::spinOnce();

  // Waypoint Mission : Initialization
  ROS_INFO("Setting Waypoint Init Defaults..\n");
  dji_osdk_ros::MissionWaypointTask waypointTask;
  auto file = cv::FileStorage(mission_file_path, cv::FileStorage::READ);
  if(!file.isOpened()){
    std::cout << "Failed to parse config file" << std::endl;
    exit(0);
  }
  float idleVelocity = (float)file["idle_velocity"]; 
  ROS_INFO("Setting iddle velocity to %s m/s.\n", (std::to_string(idleVelocity).c_str()));
  setWaypointInitDefaults(waypointTask, idleVelocity);
  ROS_INFO("Creating Waypoints..\n");
  std::vector<WayPointSettings> generatedWaypts = createWaypoints(mission_file_path);

  // Waypoint Mission: Upload the waypoints
  ROS_INFO("Uploading Waypoints..\n");
  uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

  // Waypoint Mission: Init mission
  ROS_INFO("Initializing Waypoint Mission..\n");
  if (initWaypointMission(waypointTask).result)
  {
    ROS_INFO("Waypoint upload command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending waypoint upload command");
    return false;
  }

  // Waypoint Mission: Start
  if (missionAction(DJI_MISSION_TYPE::WAYPOINT,
                    MISSION_ACTION::START)
        .result)
  {
    ROS_INFO("Mission start command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission start command");
    return false;
  }
  uploadedMission = true;
  return true;
}

void setWaypointDefaults(WayPointSettings* wp){

  wp->damping         = 1;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

void setWaypointInitDefaults(dji_osdk_ros::MissionWaypointTask& waypointTask, float idleVelocity){

  waypointTask.velocity_range     = 10;
  waypointTask.idle_velocity      = idleVelocity;
  waypointTask.action_on_finish   = dji_osdk_ros::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_osdk_ros::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = dji_osdk_ros::MissionWaypointTask::TRACE_COORDINATED;
  waypointTask.action_on_rc_lost  = dji_osdk_ros::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_osdk_ros::MissionWaypointTask::GIMBAL_PITCH_AUTO;
}

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(std::string mission_file_path){

  auto file = cv::FileStorage(mission_file_path, cv::FileStorage::READ);
  if(!file.isOpened()){
    std::cout << "Failed to parse config file" << std::endl;
    exit(0);
  }

  // std::cout << "Parsing yaml" << std::endl;
  std::vector<DJI::OSDK::WayPointSettings> wp_list;
  int nWaypoint = (int)file["wp_number"]; 
  auto waypointList = file["wp_list"]; 
  // std::cout << "Number of waypoints: " << nWaypoint << std::endl;
  WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);
  std::string wp_name = "wp0";
  auto wp0 = waypointList[wp_name];
  start_wp.index = 0;
  start_wp.latitude  = (float)wp0["latitude"];
  start_wp.longitude = (float)wp0["longitude"];
  start_wp.altitude  = (float)wp0["altitude"];
  start_wp.damping =   (float)wp0["damping"];
  ROS_INFO("First waypoint damping to %s m.\n", (std::to_string(start_wp.damping).c_str()));

  wp_list.push_back(start_wp);
  // std::cout << "wp0: " << "LLA: " <<  start_wp.latitude << " " << start_wp.longitude << " " << start_wp.altitude << std::endl;

  for (int i = 1; i < nWaypoint; i++){
    std::string wp_name = "wp" + std::to_string(i);
    auto wpN = waypointList[wp_name];
    WayPointSettings  wp;
    setWaypointDefaults(&wp);
    wp.index     = i;
    wp.latitude  = (float)wpN["latitude"];
    wp.longitude = (float)wpN["longitude"];
    wp.altitude  = (float)wpN["altitude"]; 
    wp.damping =   (float)wpN["damping"];
    // std::cout << "wp"<< i <<": " << "LLA" <<  wp.latitude << " " << wp.longitude << " " << wp.altitude << std::endl;
    wp_list.push_back(wp);
  }
  return wp_list;
}

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                     int responseTimeout,
                     dji_osdk_ros::MissionWaypointTask& waypointTask){

  dji_osdk_ros::MissionWaypoint waypoint;
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
             wp->longitude, wp->altitude);
    waypoint.latitude            = wp->latitude;
    waypoint.longitude           = wp->longitude;
    waypoint.altitude            = wp->altitude;
    waypoint.damping_distance    = 1;
    waypoint.target_yaw          = 0;
    waypoint.target_gimbal_pitch = -900;
    waypoint.turn_mode           = 0;
    waypoint.has_action          = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
  }
}

ServiceAck initWaypointMission(dji_osdk_ros::MissionWaypointTask& waypointTask){

  dji_osdk_ros::MissionWpUpload missionWpUpload;
  missionWpUpload.request.waypoint_task = waypointTask;
  waypoint_upload_client.call(missionWpUpload);
  if (!missionWpUpload.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
             missionWpUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
  }
  return ServiceAck(
    missionWpUpload.response.result, missionWpUpload.response.cmd_set,
    missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type, DJI::OSDK::MISSION_ACTION   action){

  dji_osdk_ros::MissionWpAction missionWpAction;
  dji_osdk_ros::MissionHpAction missionHpAction;
  switch (type)
  {
    case DJI::OSDK::WAYPOINT:
      missionWpAction.request.action = action;
      waypoint_action_client.call(missionWpAction);
      if (!missionWpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
                 missionWpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
      }
      return { missionWpAction.response.result,
               missionWpAction.response.cmd_set,
               missionWpAction.response.cmd_id,
               missionWpAction.response.ack_data };
    case DJI::OSDK::HOTPOINT:
      missionHpAction.request.action = action;
      hotpoint_action_client.call(missionHpAction);
      if (!missionHpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionHpAction.response.cmd_set,
                 missionHpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionHpAction.response.ack_data);
      }
      return ServiceAck(
        missionHpAction.response.result, missionHpAction.response.cmd_set,
        missionHpAction.response.cmd_id, missionHpAction.response.ack_data);
  }
}

bool takeoff(){

  dji_osdk_ros::FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_TAKEOFF;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

bool land(){

  dji_osdk_ros::FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_LAND;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

// Function used to move to a position specified as an offset from the current drone's position
bool moveByPosOffset(MoveOffset&& move_offset){

  dji_osdk_ros::FlightTaskControl task;
  task.request.task = FlightTaskControl::Request::TASK_GO_LOCAL_POS;
  // pos_offset: A vector contains that position_x_offset, position_y_offset, position_z_offset in order
  task.request.pos_offset.clear();
  task.request.pos_offset.push_back(move_offset.x);
  task.request.pos_offset.push_back(move_offset.y);
  task.request.pos_offset.push_back(move_offset.z);
  // yaw_params: A vector contains that yaw_desired, position_threshold(Meter), yaw_threshold(Degree)
  task.request.yaw_params.clear();
  task.request.yaw_params.push_back(move_offset.yaw);
  task.request.yaw_params.push_back(move_offset.pos_threshold);
  task.request.yaw_params.push_back(move_offset.yaw_threshold);
  flight_control_client.call(task);
  return task.response.result;
}

void set_focus_point(){

  ros::NodeHandle n;
  auto camera_set_focus_point_client = n.serviceClient<dji_osdk_ros::CameraFocusPoint>("camera_task_set_focus_point");
  dji_osdk_ros::CameraFocusPoint cameraFocusPoint;
  cameraFocusPoint.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
  cameraFocusPoint.request.x = 0.5;
  cameraFocusPoint.request.y = 0.5;
  camera_set_focus_point_client.call(cameraFocusPoint);
  std::cout << "Changed focus point" << std::endl;
}

void ModeCallback(const std_msgs::UInt8::ConstPtr& msg){

  ros::NodeHandle n;
  dji_osdk_ros::MissionWpGetInfo mission_get_info;
  auto get_mission_info_client = n.serviceClient<dji_osdk_ros::MissionWpGetInfo>("dji_osdk_ros/mission_waypoint_getInfo");
  get_mission_info_client.call(mission_get_info);
  dji_osdk_ros::MissionWaypointTask waypointTask = mission_get_info.response.waypoint_task;

  if(waypointTask.mission_waypoint.empty() && uploadedMission && offset_completed && !activatedLanding){
    std_srvs::SetBool vs_msg;
    vs_msg.request.data = true;
    auto vs_client = n.serviceClient<std_srvs::SetBool>("autonomous_landing/activate_visualservoing");
    vs_client.call(vs_msg);
    std::cout << "Activated autonomous landing" << std::endl;
    activatedLanding = true;
    set_focus_point();
  }
}

void HeightCallback(const std_msgs::Float32::ConstPtr& msg){

  auto height = msg->data;
  mGPS.lock();
  float gpsAltitude = altitude;
  mGPS.unlock();
  if(height > 0.0){
    mCorr.lock();
    corr = height - gpsAltitude;
    mCorr.unlock();
  }
     
  std_msgs::Float32 corrected_msg;
  mCorr.lock();
  corrected_msg.data = gpsAltitude + corr;
  mCorr.unlock();
  correctedGPS_publisher_.publish(corrected_msg);

  if(activatedLanding){
    if(height < 2 && focus_trigger){
      std::cout << "Height under 2m -> set focus point" << std::endl; 
      set_focus_point();
      focus_trigger--;
    } else if(height < 5 && focus_trigger == 2){
      std::cout << "Height under 5m -> set focus point" << std::endl; 
      set_focus_point();
      focus_trigger--;
    } else if(height < 8 && focus_trigger == 3){
      std::cout << "Height under 8m -> set focus point" << std::endl; 
      set_focus_point();
      focus_trigger--;
    } else if(height < 10 && focus_trigger == 4){
      std::cout << "Height under 10m -> set focus point" << std::endl; 
      set_focus_point();
      focus_trigger--;
    }
  } 
}

void FlightStatusCallback(const std_msgs::UInt8::ConstPtr& msg){
  
  auto status = msg->data;
  if(activatedLanding && status == DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND)
  {
    ros::NodeHandle n;
    std_srvs::SetBool landing_confirmation;
    landing_confirmation.request.data = true;
    auto land_confirmation_client = n.serviceClient<std_srvs::SetBool> ("landing_confirmation");
    land_confirmation_client.call(landing_confirmation);
    std::cout << "Taking confirmation picture and sending it." << std::endl;
  }
}

void finishHandler(int _sig){
  std::cout << "Finish Handler catch" << _sig << std::endl;
}

void shutDownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result){
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1){
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
  }
  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char** argv){  
  signal(SIGINT, finishHandler);
  signal(SIGTERM, finishHandler);
  ros::init(argc, argv, "missions_node");
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutDownCallback);
  
  ros::NodeHandle nh;

  if(argc < 2){
    std::cout << "Wrong argument. Please provide a yaml file to start. Usage: rosrun dji_osdk_ros missions_yaml mission.yaml" << std::endl;
    return -1;
  }
  std::string mission_file_path = argv[1];
  ROS_INFO("Input yaml file: %s\n", mission_file_path.c_str());
  
  waypoint_upload_client =          nh.serviceClient<dji_osdk_ros::MissionWpUpload>("dji_osdk_ros/mission_waypoint_upload");
  waypoint_action_client =          nh.serviceClient<dji_osdk_ros::MissionWpAction>("dji_osdk_ros/mission_waypoint_action");
  hotpoint_upload_client =          nh.serviceClient<dji_osdk_ros::MissionHpUpload>("dji_osdk_ros/mission_hotpoint_upload");
  flight_control_client =           nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");
  gps_pos_subscriber =              nh.subscribe<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", 10, &gpsPosCallback);
  pos_offsets_subscriber =          nh.subscribe<geometry_msgs::Point32>("position_offsets",10, &posOffsetsCallback);

  force_landing_subscriber =        nh.subscribe<std_msgs::String>("/force_landing", 10, &forceLandingCallback);

  ros::Subscriber flight_mode_subscriber = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/display_mode", 1, &ModeCallback);
  correctedGPS_publisher_ = nh.advertise<std_msgs::Float32>("dji_osdk_ros/correctedGPSHeight",10);
  ros::Subscriber height_subscriber =      nh.subscribe<std_msgs::Float32>("dji_osdk_ros/height_above_takeoff", 1, &HeightCallback);
  ros::Subscriber flight_status_subscriber = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/flight_status", 1, &FlightStatusCallback);

  int     responseTimeout = 1;
  // Waypoint mission
  ROS_INFO("Executing approximation sequence.");
  runWaypointMission(mission_file_path, responseTimeout);
  ros::Duration(10).sleep();
  ROS_INFO("First approximation completed.");

  // LoRa Navigation (offset)
  ROS_INFO("Switching to radio/offset navigation.");
  //bool move_result = moveByPosOffset(MoveOffset(x_offset, y_offset, z_offset, 0.0f));
  //if (move_result)
  if (moveByPosOffset(MoveOffset(x_offset, y_offset, z_offset, 0.0f)))
  {
    ROS_INFO("Switching to radio/offset navigation.");
    ros::Duration(10).sleep();
    offset_completed = true;
  }

  ros::spin();
  ROS_INFO("Mission completed.");
  return 0;
}
