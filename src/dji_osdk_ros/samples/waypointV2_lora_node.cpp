/** @file waypointV2_lora_node.cpp
 *  @version 1.0
 *  @date July 2022
 *
 *  @brief node of waypoint V2.0.
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

#include <dji_osdk_ros/waypointV2_node.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>

#include <dji_osdk_ros/FlightTaskControl.h>

#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/CameraFocusPoint.h>

#include <opencv2/core/core.hpp>
#include <sstream>
#include <mutex>
#include <csignal>
#include <ros/xmlrpc_manager.h>

#include <dji_osdk_ros/SetJoystickMode.h>
#include <dji_osdk_ros/JoystickAction.h>

#include <math.h>
#include <chrono>
#include <time.h>
#include <boost/bind.hpp>

using namespace DJI::OSDK;
using namespace dji_osdk_ros;

//global variables

std::string mission_file_path;
uint8_t mission_state = 0x00;
bool userInterrupt = false; //user input on joystick detected
//mission state of 
bool uploadedMission = false;
bool activatedLanding = false;
short focus_trigger = 4;
float altitude = 0.0f;
float corr = 0.0f;
float x_offset = 0.0f;
float y_offset = 0.0f;
float z_offset = 0.0f;
float yaw_offset = 0.0f;
float pos_thr = 0.5f;
float yaw_thr = 1.0f;

float rel_alt = 0.0f;
geometry_msgs::Point32 offsets;

bool received_offsets = false;

bool offset_completed = false;
bool firstGPS = true;

int action_id = 0;

enum Mode {mode_waypoints, mode_offsets};

std::mutex mOff;
std::mutex mCorr;
std::mutex mGPS;
std::mutex mState;

sensor_msgs::NavSatFix gps_corr;
sensor_msgs::NavSatFix lat_lon_offsets; // relative beacon offset from drone
geometry_msgs::Quaternion imu_quat;

ros::ServiceClient flight_control_client;
ros::ServiceClient obtain_release_control_client;
ros::Publisher correctedGPS_publisher_;

bool moveByPosOffset(const JoystickCommand &offsetDesired,
                    float posThresholdInM,
                    float yawThresholdInDeg);

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
    gps_position_ = *gpsPosition;

    // Adapted based on our missions node
    mGPS.lock();
    altitude = gps_position_.altitude;
    mGPS.unlock();

    if(firstGPS){
        mCorr.lock();
        corr = corr - gps_position_.altitude;
        mCorr.unlock();
        firstGPS = !firstGPS;
    }
}

void waypointV2MissionEventSubCallback(const dji_osdk_ros::WaypointV2MissionEventPush::ConstPtr& waypointV2MissionEventPush)
{
  waypoint_V2_mission_event_push_ = *waypointV2MissionEventPush;

  ROS_INFO("waypoint_V2_mission_event_push_.event ID :0x%x\n", waypoint_V2_mission_event_push_.event);

  if(waypoint_V2_mission_event_push_.event == 0x01)
  {
    ROS_INFO("interruptReason:0x%x\n", waypoint_V2_mission_event_push_.interruptReason);
  }
  if(waypoint_V2_mission_event_push_.event == 0x02)
  {
    ROS_INFO("recoverProcess:0x%x\n", waypoint_V2_mission_event_push_.recoverProcess);
  }
  if(waypoint_V2_mission_event_push_.event== 0x03)
  {
    ROS_INFO("finishReason:0x%x\n", waypoint_V2_mission_event_push_.finishReason);
  }

  if(waypoint_V2_mission_event_push_.event == 0x10)
  {
    ROS_INFO("current waypointIndex:%d\n", waypoint_V2_mission_event_push_.waypointIndex);
  }

  if(waypoint_V2_mission_event_push_.event == 0x11)
  {
    ROS_INFO("currentMissionExecNum:%d\n", waypoint_V2_mission_event_push_.currentMissionExecNum);
  }
}

void waypointV2MissionStateSubCallback(const dji_osdk_ros::WaypointV2MissionStatePush::ConstPtr& waypointV2MissionStatePush)
{
  waypoint_V2_mission_state_push_ = *waypointV2MissionStatePush;

  //ROS_INFO("waypointV2MissionStateSubCallback");
  //ROS_INFO("missionStatePushAck->commonDataVersion:%d\n",waypoint_V2_mission_state_push_.commonDataVersion);
  //ROS_INFO("missionStatePushAck->commonDataLen:%d\n",waypoint_V2_mission_state_push_.commonDataLen);
  ROS_INFO("missionStatePushAck->data.state:0x%x\n",waypoint_V2_mission_state_push_.state);
  //ROS_INFO("missionStatePushAck->data.curWaypointIndex:%d\n",waypoint_V2_mission_state_push_.curWaypointIndex);
  //ROS_INFO("missionStatePushAck->data.velocity:%d\n",waypoint_V2_mission_state_push_.velocity);

  mState.lock();
  mission_state = waypoint_V2_mission_state_push_.state;
  ROS_INFO("Mission State: %i", mission_state);
  mState.unlock();
}

void setWaypointV2Defaults(dji_osdk_ros::WaypointV2& waypointV2)
{
  waypointV2.waypointType = dji_osdk_ros::DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  waypointV2.headingMode = dji_osdk_ros::DJIWaypointV2HeadingModeAuto;
  waypointV2.config.useLocalCruiseVel = 0;
  waypointV2.config.useLocalMaxVel = 0;

  waypointV2.dampingDistance = 1;
  waypointV2.heading = 0;
  waypointV2.turnMode = dji_osdk_ros::DJIWaypointV2TurnModeClockwise;

  waypointV2.positionX = 0;
  waypointV2.positionY = 0;
  waypointV2.positionZ = 0;
  waypointV2.maxFlightSpeed= 9;
  waypointV2.autoFlightSpeed = 2;
}

std::vector<dji_osdk_ros::WaypointV2> generatePolygonWaypoints(ros::NodeHandle &nh, float32_t radius, uint16_t polygonNum)
{
    auto file = cv::FileStorage(mission_file_path, cv::FileStorage::READ);
    if(!file.isOpened())
    {
        std::cout << "Failed to parse mission file" << std::endl;
        exit(0);
    }

    // Let's create a vector to store our waypoints in.
    std::vector<dji_osdk_ros::WaypointV2> waypointList;
    int nWaypoint = (int)file["wp_number"]; 
    auto wpList = file["wp_list"]; 
  
    dji_osdk_ros::WaypointV2 startPoint;
    std::string wp_name = "wp0";
    auto wp0 = wpList[wp_name];

    setWaypointV2Defaults(startPoint);
    startPoint.latitude  = (float)wp0["latitude"] * C_PI / 180.0;
    startPoint.longitude = (float)wp0["longitude"] * C_PI / 180.0;
    //startPoint.latitude  = (float)wp0["latitude"];
    //startPoint.longitude = (float)wp0["longitude"];
    startPoint.relativeHeight = (float)wp0["altitude"];
    startPoint.dampingDistance = (float)wp0["damping"];

    std::cout << "wp0 lat: " << startPoint.latitude << "lon: " << startPoint.longitude << std::endl;
    
    waypointList.push_back(startPoint);

    for (int i = 1; i < nWaypoint; i++)
    {
        std::string wp_name = "wp" + std::to_string(i);
        auto wpN = wpList[wp_name];
        dji_osdk_ros::WaypointV2 waypointV2;
        setWaypointV2Defaults(waypointV2);

        waypointV2.latitude = (float)wpN["latitude"] * C_PI / 180.0;
        waypointV2.longitude = (float)wpN["longitude"] * C_PI / 180.0;
        //waypointV2.latitude = (float)wpN["latitude"];
        //waypointV2.longitude = (float)wpN["longitude"];
        std::cout << "wp" << i << " lat: " << waypointV2.latitude << " lon: " << waypointV2.longitude << std::endl;
        waypointV2.relativeHeight = (float)wpN["altitude"]; 
        waypointV2.dampingDistance = (float)wpN["damping"];
        waypointList.push_back(waypointV2);
    }

    return waypointList;
}
// alternate waypointv2 mission. Creates a goal coordinate based on received lat lon offsets and an intermediate coordinate (to satisfy the two waypoint minimum requirement)
std::vector<dji_osdk_ros::WaypointV2> generateOffsetWaypoints(ros::NodeHandle &nh)
{
    auto Start = std::chrono::high_resolution_clock::now();
    while(!received_offsets)
    {
        auto End = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> Elapsed = End - Start;
        if (Elapsed.count() >= 5000.00)
            break;
    } // wait up to 5 sec for offset data

    // Vector to store waypoints in
    std::vector<dji_osdk_ros::WaypointV2> waypointList;

    if(received_offsets)
    {
            dji_osdk_ros::WaypointV2 endpoint;
	    setWaypointV2Defaults(endpoint);
	    dji_osdk_ros::WaypointV2 midpoint;
	    setWaypointV2Defaults(midpoint);
	    //get current drone position
            
	    //float curr_latitude = gps_corr.latitude;
	    //float curr_longitude = gps_corr.longitude;
	    //float curr_altitude = gps_corr.altitude;

            float curr_latitude = gps_position_.latitude;
	    float curr_longitude = gps_position_.longitude;
	    //float curr_altitude = gps_position_.altitude;
            float curr_altitude = rel_alt; //corrected gps height
            
            float lat_offset = lat_lon_offsets.latitude;
            float lon_offset = lat_lon_offsets.longitude;

	    endpoint.latitude  = (curr_latitude + lat_offset) * C_PI / 180.0;
	    endpoint.longitude  = (curr_longitude + lon_offset) * C_PI / 180.0;
	    endpoint.relativeHeight  = rel_alt;
	    endpoint.dampingDistance = 0;

            std::cout << "lat offset:" << lat_offset << std::endl;
            std::cout << "lon offset:" << lon_offset << std::endl;

            std::cout << "current lat:" << curr_latitude << std::endl;
            std::cout << "current lon:" << curr_longitude << std::endl;
            std::cout << "current alt:" << curr_altitude << std::endl;

            std::cout << "goal lat:" << curr_latitude + lat_offset << std::endl;
            std::cout << "goal lon:" << curr_longitude + lon_offset << std::endl;
            std::cout << "goal alt:" << endpoint.relativeHeight << std::endl;
	    
	    // simple subdivision of trajectory. Does not necessarily yield straight line
	    midpoint.latitude  = (curr_latitude + (lat_offset / 2)) * C_PI / 180.0;
	    midpoint.longitude  = (curr_longitude + (lon_offset / 2)) * C_PI / 180.0;
	    midpoint.relativeHeight  = curr_altitude;
	    midpoint.dampingDistance = 0;

	    waypointList.push_back(midpoint);
	    waypointList.push_back(endpoint);
    }
    return waypointList;
}

//Currently Unused. 
std::vector<float> metres_to_lon_lat_offset(float orig_lon, float orig_lat, float dx, float dy)
{
    std::vector<float> dlon_dlat;
    float dlon = orig_lon + dy/111111.00;
    float orig_lat_rad = orig_lat*C_PI/180;
    float dlat = orig_lat + float(dx / 111111.00) * cos(orig_lat_rad);
    dlon_dlat.push_back(dlon);
    dlon_dlat.push_back(dlat);
    ROS_INFO("Offset Coords: Lon: %f, Lat: %f", dlon, dlat);
    return dlon_dlat;
}
//mode 0, read from yaml file. Mode 1, offset navigation
bool initWaypointV2Setting(ros::NodeHandle &nh, Mode mode) 
{
    waypointV2_init_setting_client = nh.serviceClient<dji_osdk_ros::InitWaypointV2Setting>("dji_osdk_ros/waypointV2_initSetting");
    //These two parameters will be ignored when passed to the function, they were kept just to preserve the function prototype
    dji_osdk_ros::InitWaypointV2Setting initWaypointV2Setting_;
    initWaypointV2Setting_.request.polygonNum = 6;
    initWaypointV2Setting_.request.radius = 6;
    //this changed to the fit the number of waypoints in the file; the generation of action is moved to after we know the mission size
    //initWaypointV2Setting_.request.actionNum = 5;

    initWaypointV2Setting_.request.waypointV2InitSettings.repeatTimes = 1;
    //initWaypointV2Setting_.request.waypointV2InitSettings.finishedAction = initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionFinishedNoAction; // Drone just hovers and awaits further input
    initWaypointV2Setting_.request.waypointV2InitSettings.finishedAction = initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionFinishedContinueUntilStop; // Drone just hovers and awaits further input
    initWaypointV2Setting_.request.waypointV2InitSettings.maxFlightSpeed = 10;
    initWaypointV2Setting_.request.waypointV2InitSettings.autoFlightSpeed = 2;
    initWaypointV2Setting_.request.waypointV2InitSettings.exitMissionOnRCSignalLost = 1;
    initWaypointV2Setting_.request.waypointV2InitSettings.gotoFirstWaypointMode = initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionGotoFirstWaypointModePointToPoint;
    // The function generatePolygon has been changed to populate the waypoint list 
    // with the content of the mission fila and match the waypoint V2 definitions
    if(mode == mode_waypoints)
    {
        initWaypointV2Setting_.request.waypointV2InitSettings.mission = generatePolygonWaypoints(nh, initWaypointV2Setting_.request.radius, initWaypointV2Setting_.request.polygonNum);
        initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen = initWaypointV2Setting_.request.waypointV2InitSettings.mission.size();
        /*! Generate actions. Changed */
        generateWaypointV2Actions(nh, initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen);
    }
    else if(mode == mode_offsets)
    {
        initWaypointV2Setting_.request.waypointV2InitSettings.mission = generateOffsetWaypoints(nh);
	initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen = initWaypointV2Setting_.request.waypointV2InitSettings.mission.size();
        /*! Generate actions. Changed */
        generateWaypointV2Actions(nh, initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen);
    }
     
    waypointV2_init_setting_client.call(initWaypointV2Setting_);
    if (initWaypointV2Setting_.response.result)
    {
      ROS_INFO("Init mission setting successfully!\n");
    }
    else
    {
std::cout << "response: " << initWaypointV2Setting_.response << std::endl;
      ROS_ERROR("Init mission setting failed!\n");
    }

    return initWaypointV2Setting_.response.result;

}

bool uploadWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_upload_mission_client = nh.serviceClient<dji_osdk_ros::UploadWaypointV2Mission>("dji_osdk_ros/waypointV2_uploadMission");
    waypointV2_upload_mission_client.call(uploadWaypointV2Mission_);

    if(uploadWaypointV2Mission_.response.result)
    {
      ROS_INFO("Upload waypoint v2 mission successfully!\n");
      uploadedMission = true;
    }
    else
    {
      ROS_ERROR("Upload waypoint v2 mission failed!\n");
    }

    return uploadWaypointV2Mission_.response.result;
}

bool uploadWaypointV2Action(ros::NodeHandle &nh)
{
    waypointV2_upload_action_client = nh.serviceClient<dji_osdk_ros::UploadWaypointV2Action>("dji_osdk_ros/waypointV2_uploadAction");
    waypointV2_upload_action_client.call(uploadWaypointV2Action_);

    if(uploadWaypointV2Action_.response.result)
    {
      ROS_INFO("Upload waypoint v2 actions successfully!\n");
    }
    else
    {
      ROS_ERROR("Upload waypoint v2 actions failed!\n");
    }

    return uploadWaypointV2Action_.response.result;
}

bool downloadWaypointV2Mission(ros::NodeHandle &nh, std::vector<dji_osdk_ros::WaypointV2> &mission)
{
    waypointV2_download_mission_client = nh.serviceClient<dji_osdk_ros::DownloadWaypointV2Mission>("dji_osdk_ros/waypointV2_downloadMission");
    waypointV2_download_mission_client.call(downloadWaypointV2Mission_);
    mission = downloadWaypointV2Mission_.response.mission;

    if(downloadWaypointV2Mission_.response.result)
    {
      ROS_INFO("Download waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Download waypoint v2 mission failed!\n");
    }

    return downloadWaypointV2Mission_.response.result; 
}

bool startWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_start_mission_client = nh.serviceClient<dji_osdk_ros::StartWaypointV2Mission>("dji_osdk_ros/waypointV2_startMission");
    waypointV2_start_mission_client.call(startWaypointV2Mission_);

    if(startWaypointV2Mission_.response.result)
    {
      ROS_INFO("Start waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Start waypoint v2 mission failed!\n");
    }

    return startWaypointV2Mission_.response.result;
}

bool stopWaypointV2Mission()
{
    waypointV2_stop_mission_client.call(stopWaypointV2Mission_);

    if(stopWaypointV2Mission_.response.result)
    {
      ROS_INFO("Stop waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Stop waypoint v2 mission failed!\n");
    }

    return stopWaypointV2Mission_.response.result;
}

bool pauseWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_pause_mission_client = nh.serviceClient<dji_osdk_ros::PauseWaypointV2Mission>("dji_osdk_ros/waypointV2_pauseMission");
    waypointV2_pause_mission_client.call(pauseWaypointV2Mission_);

    if(pauseWaypointV2Mission_.response.result)
    {
      ROS_INFO("Pause waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Pause waypoint v2 mission failed!\n");
    }

    return pauseWaypointV2Mission_.response.result;
}

bool resumeWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_resume_mission_client = nh.serviceClient<dji_osdk_ros::ResumeWaypointV2Mission>("dji_osdk_ros/waypointV2_resumeMission");
    waypointV2_resume_mission_client.call(resumeWaypointV2Mission_);

    if(resumeWaypointV2Mission_.response.result)
    {
      ROS_INFO("Resume Waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Resume Waypoint v2 mission failed!\n");
    }

    return resumeWaypointV2Mission_.response.result;
}

bool generateWaypointV2Actions(ros::NodeHandle &nh, uint16_t actionNum)
{
    waypointV2_generate_actions_client = nh.serviceClient<dji_osdk_ros::GenerateWaypointV2Action>("dji_osdk_ros/waypointV2_generateActions");
    std::cout << "action num: " << actionNum << std::endl;
    dji_osdk_ros::WaypointV2Action actionVector;
    for (uint16_t i = 0; i < actionNum; i++)
    {
      actionVector.actionId  = action_id++;
      std::cout << "action ID: " << action_id << std::endl;
      actionVector.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
      actionVector.waypointV2SampleReachPointTrigger.waypointIndex = i;
      actionVector.waypointV2SampleReachPointTrigger.terminateNum = 0;
      actionVector.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
      actionVector.waypointV2CameraActuator.actuatorIndex = 0;
      actionVector.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto;
      generateWaypointV2Action_.request.actions.push_back(actionVector);
    }

    waypointV2_generate_actions_client.call(generateWaypointV2Action_);

    return generateWaypointV2Action_.response.result;
}

bool setGlobalCruiseSpeed(ros::NodeHandle &nh, float32_t cruiseSpeed)
{
    waypointV2_set_global_cruisespeed_client = nh.serviceClient<dji_osdk_ros::SetGlobalCruisespeed>("dji_osdk_ros/waypointV2_setGlobalCruisespeed");
    setGlobalCruisespeed_.request.global_cruisespeed = cruiseSpeed;
    waypointV2_set_global_cruisespeed_client.call(setGlobalCruisespeed_);

    if(setGlobalCruisespeed_.response.result)
    {
      ROS_INFO("Current cruise speed is: %f m/s\n", cruiseSpeed);
    }
    else
    {
      ROS_ERROR("Set glogal cruise speed failed\n");
    }

    return setGlobalCruisespeed_.response.result;
}

float32_t getGlobalCruiseSpeed(ros::NodeHandle &nh)
{
    waypointV2_get_global_cruisespeed_client = nh.serviceClient<dji_osdk_ros::GetGlobalCruisespeed>("dji_osdk_ros/waypointV2_getGlobalCruisespeed");
    waypointV2_get_global_cruisespeed_client.call(getGlobalCruisespeed_);

    ROS_INFO("Current cruise speed is: %f m/s\n", getGlobalCruisespeed_.response.global_cruisespeed);

    return getGlobalCruisespeed_.response.global_cruisespeed;
}

bool runWaypointV2Mission(ros::NodeHandle &nh, Mode mode)
{
    std::cout << "mission_state: " << int(mission_state) << std::endl;
    sleep(1);
    //while(!(mission_state == 0x0) && !userInterrupt) //wait for mission paused or exited state before trying to run new mission
    //{
      //if(mission_state == 0x0)
      //{
      //  stopWaypointV2Mission();
      //}
      //ROS_INFO_THROTTLE(5,"waiting for mission to end");
    //}
    srand(time(NULL));
    action_id = rand();

    int timeout = 1;
    bool result = false;

    get_drone_type_client = nh.serviceClient<dji_osdk_ros::GetDroneType>("get_drone_type");
    waypointV2_mission_state_push_client = nh.serviceClient<dji_osdk_ros::SubscribeWaypointV2State>("dji_osdk_ros/waypointV2_subscribeMissionState");
    waypointV2_mission_event_push_client = nh.serviceClient<dji_osdk_ros::SubscribeWaypointV2Event>("dji_osdk_ros/waypointV2_subscribeMissionEvent");

    waypointV2EventSub = nh.subscribe("dji_osdk_ros/waypointV2_mission_event", 10, &waypointV2MissionEventSubCallback);
    waypointV2StateSub = nh.subscribe("dji_osdk_ros/waypointV2_mission_state", 10, &waypointV2MissionStateSubCallback);

    subscribeWaypointV2Event_.request.enable_sub = true;
    subscribeWaypointV2State_.request.enable_sub = true;
    
    get_drone_type_client.call(drone_type);
    if (drone_type.response.drone_type != static_cast<uint8_t>(dji_osdk_ros::Dronetype::M300))
    {
        ROS_DEBUG("This sample only supports M300!\n");
        return false;
    }

    waypointV2_mission_state_push_client.call(subscribeWaypointV2State_);
    waypointV2_mission_event_push_client.call(subscribeWaypointV2Event_);

    /*! init mission */        
    result = initWaypointV2Setting(nh, mode);
    if(!result)
    {
        return false;
    }
    //sleep(timeout);

    /*! upload mission */
    result = uploadWaypointV2Mission(nh);
    if(!result)
    {
        return false;
    }
    //sleep(timeout);

    /*! upload  actions */
    //result = uploadWaypointV2Action(nh);
    //if(!result)
    //{
    //    return false;
    //}
    //sleep(timeout);

    /*! set global cruise speed */
    //We used to read this from the yaml file, for simplicity let's set it here for now.
    result = setGlobalCruiseSpeed(nh, 3); 
    if(!result)
    {
        return false;
    }
    //sleep(timeout);

    /*! start mission */
    result = startWaypointV2Mission(nh);
    if(!result)
    {
        return false;
    }
    //sleep(100);

    return true;
}


bool joystickActivityCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    //std::vector<float> joyInputs = *msg.axes;
    sensor_msgs::Joy joyInputs = *msg;
    //userInterrupt is true if any axis of the joystick shows nonzero value. Index 4 is not included as it is not an axis
    bool nz_axis = false;
    nz_axis = (!(abs(joyInputs.axes[0]) < 1e-9) | !(abs(joyInputs.axes[1]) < 1e-9) |
               !(abs(joyInputs.axes[2]) < 1e-9) | !(abs(joyInputs.axes[3]) < 1e-9) |
               !(abs(joyInputs.axes[5]) < 1e-9));
    if(nz_axis) //set flag to true permanently if user input detected
        userInterrupt = true;
    
    if(userInterrupt & (mission_state > 0x0))
        stopWaypointV2Mission();

    //ROS_INFO("axes[0]: %f", joyInputs.axes[0]);
    ROS_INFO("RC Joystick Input: %i\n", nz_axis);

}
/********************** Added functions *********************/

void posOffsetsCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
    std::cout << "pos offsets received!" << std::endl; 
    offsets = *msg;

    mOff.lock();
    /*
    std::cout << "Mission State: " << DJIWaypointV2MissionState::DJIWaypointV2MissionStateUnWaypointActionActuatorknown << std::endl;
    std::cout << "Mission State: " << DJIWaypointV2MissionState::DJIWaypointV2MissionStateDisconnected << std::endl;
    std::cout << "Mission State: " << DJIWaypointV2MissionState::DJIWaypointV2MissionStateReadyToExecute << std::endl;
    std::cout << "Mission State: " << DJIWaypointV2MissionState::DJIWaypointV2MissionStateExecuting << std::endl;
    std::cout << "Mission State: " << DJIWaypointV2MissionState::DJIWaypointV2MissionStateInterrupted << std::endl;
    std::cout << "Mission State: " << DJIWaypointV2MissionState::DJIWaypointV2MissionStateResumeAfterInterrupted << std::endl;
    std::cout << "Mission State: " << DJIWaypointV2MissionState::DJIWaypointV2MissionStateExitMission << std::endl;
    std::cout << "Mission State: " << DJIWaypointV2MissionState::DJIWaypointV2MissionStateFinishedMission << std::endl;
    */
    dji_osdk_ros::ObtainControlAuthority authority_request;
    authority_request.request.enable_obtain = false;
    

    if (!userInterrupt & (mission_state == 0x0)){
      JoystickCommand js;
      js.x = offsets.x;
      js.y = offsets.y;
      js.z = offsets.z;
      js.yaw = 0.0;
      //obtain control
      authority_request.request.enable_obtain = true;
      obtain_release_control_client.call(authority_request);
      moveByPosOffset(js, pos_thr, yaw_thr);
      //ROS_INFO
      //release control
      authority_request.request.enable_obtain = false;
      obtain_release_control_client.call(authority_request);
    }

    mOff.unlock();
}

void latLonOffsetsCallback(ros::NodeHandle &node_handle, const sensor_msgs::NavSatFix ::ConstPtr& msg)
{
    received_offsets = true;
    std::cout << "lat lon offsets received!" << std::endl; 
    lat_lon_offsets = *msg;
    if(mission_state == 0x0 & !userInterrupt)
    {
      ROS_INFO("Executing approximation sequence.");
      // run mission in mode 1: offset mission
      Mode mode = mode_waypoints;
      int result = 0;
      result = runWaypointV2Mission(node_handle, mode_offsets); 
      ROS_INFO("Waypoint Offset Mission 1 run %s",(result == 0 ? "failure" : "success"));
      std::cout << "mission state after upload:" << int(mission_state) << std::endl;
    }
}

// Function used to move to a position specified as an offset from the current drone's position
bool moveByPosOffset(const JoystickCommand &offsetDesired,
                    float posThresholdInM,
                    float yawThresholdInDeg)
{
	dji_osdk_ros::FlightTaskControl task;
	task.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
	task.request.joystickCommand.x = offsetDesired.x;
	task.request.joystickCommand.y = offsetDesired.y;
	task.request.joystickCommand.z = offsetDesired.z;
	task.request.joystickCommand.yaw = offsetDesired.yaw;
	task.request.velocityControlTimeMs = 10000;
	task.request.posThresholdInM   = posThresholdInM;
	task.request.yawThresholdInDeg = yawThresholdInDeg;

	flight_control_client.call(task);
	return task.response.result;
}

void set_focus_point()
{
    ros::NodeHandle n;
    auto camera_set_focus_point_client = n.serviceClient<dji_osdk_ros::CameraFocusPoint>("camera_task_set_focus_point");
    dji_osdk_ros::CameraFocusPoint cameraFocusPoint;
    cameraFocusPoint.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
    cameraFocusPoint.request.x = 0.5;
    cameraFocusPoint.request.y = 0.5;
    camera_set_focus_point_client.call(cameraFocusPoint);
    std::cout << "Changed focus point" << std::endl;
}

void ModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    ros::NodeHandle n;

    // We were checking if the waypoint mission list was empty by calling a service. 
    // Now we can check if we exited the mission in the mission state callback and save it in a global variable.

    //dji_osdk_ros::MissionWpGetInfo mission_get_info;
    //auto get_mission_info_client = n.serviceClient<dji_osdk_ros::MissionWpGetInfo>("dji_osdk_ros/mission_waypoint_getInfo");
    //get_mission_info_client.call(mission_get_info);
    //dji_osdk_ros::MissionWaypointTask waypointTask = mission_get_info.response.waypoint_task;

    //if(waypointTask.mission_waypoint.empty() && uploadedMission && offset_completed && !activatedLanding)
    if(mission_state==0x6 && uploadedMission && offset_completed && !activatedLanding)
    {
        std_srvs::SetBool vs_msg;
        vs_msg.request.data = true;
        auto vs_client = n.serviceClient<std_srvs::SetBool>("autonomous_landing/activate_visualservoing");
        vs_client.call(vs_msg);
        std::cout << "Activated autonomous landing" << std::endl;
        activatedLanding = true;
        set_focus_point();
    }
}


void HeightCallback(const std_msgs::Float32::ConstPtr& msg)
{
    auto height = msg->data;
    rel_alt = height;
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

//subscribe to corrected GPS topic
void CorrGpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
    gps_corr = *gpsPosition; //copy to global GPS position variable
}


void FlightStatusCallback(const std_msgs::UInt8::ConstPtr& msg)
{
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

void finishHandler(int _sig)
{
    std::cout << "Finish Handler catch" << _sig << std::endl;
}

void shutDownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
    if (num_params > 1){
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    }
    result = ros::xmlrpc::responseInt(1, "", 0);
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_quat = msg->orientation;
}
/********************** End of Added functions *********************/

int main(int argc, char** argv)
{
    signal(SIGINT, finishHandler);
    signal(SIGTERM, finishHandler);
    ros::init(argc, argv, "waypointV2_lora_node");
    
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutDownCallback);
    ros::NodeHandle nh;
    ros::Subscriber gpsPositionSub            = nh.subscribe<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
    ros::Subscriber lonlatOffsetsSub          = nh.subscribe<sensor_msgs::NavSatFix>("/lon_lat_offsets", 1, boost::bind(&latLonOffsetsCallback, boost::ref(nh), _1));
    flight_control_client                     = nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");
    ros::Subscriber pos_offsets_subscriber    = nh.subscribe<geometry_msgs::Point32>("position_offsets",10, &posOffsetsCallback);
    ros::Subscriber flight_mode_subscriber    = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/display_mode", 1, &ModeCallback);
    correctedGPS_publisher_                   = nh.advertise<std_msgs::Float32>("dji_osdk_ros/correctedGPSHeight",10);
    ros::Subscriber height_subscriber         = nh.subscribe<std_msgs::Float32>("dji_osdk_ros/height_above_takeoff", 1, &HeightCallback);
    ros::Subscriber flight_status_subscriber  = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/flight_status", 1, &FlightStatusCallback);
    ros::Subscriber joySubscriber             = nh.subscribe<sensor_msgs::Joy>("dji_osdk_ros/rc", 1, &joystickActivityCallback);
    ros::Subscriber imu_subscriber            = nh.subscribe<sensor_msgs::Imu>("dji_osdk_ros/imu", 1, &ImuCallback);
    waypointV2_stop_mission_client            = nh.serviceClient<dji_osdk_ros::StopWaypointV2Mission>("dji_osdk_ros/waypointV2_stopMission");
    obtain_release_control_client             = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("/obtain_release_control_authority");

    if(argc < 2)
    {
        std::cout << "Wrong argument. Please provide a yaml file to start. Usage: rosrun dji_osdk_ros waypointV2_lora_node mission_filename.yaml" << std::endl;
        return -1;
    }
    mission_file_path = argv[1];
    ROS_INFO("Input yaml file: %s\n", mission_file_path.c_str());
    

    ros::Duration(1).sleep();
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Waypoint mission
    ROS_INFO("Executing approximation sequence.");
    // run mission in mode 1: offset mission
    Mode mode = mode_waypoints;
    int result = 0;
    result = runWaypointV2Mission(nh, mode_offsets); 
    ROS_INFO("Waypoint Offset Mission 1 run %s",(result == 0 ? "failure" : "success"));
    std::cout << "mission state after upload:" << int(mission_state) << std::endl;

    

   
    //sleep(100);
    

    /*result = runWaypointV2Mission(nh, mode_waypoints);
    ROS_INFO("Waypoint Mission run %s",(result == 0 ? "failure" : "success"));*/
    //metres_to_lon_lat_offset(-8.89, 52.71, 10, 10);
    //ros::Duration(10).sleep();
    //ROS_INFO("First approximation completed.");

    // LoRa Navigation (offset)
    //ROS_INFO("Switching to radio/offset navigation.");

    ros::waitForShutdown();
}
