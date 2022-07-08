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

using namespace DJI::OSDK;
using namespace dji_osdk_ros;

//global variables

std::string mission_file_path;
uint8_t mission_state = 0;
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
std::mutex mState;

bool moveByPosOffset(MoveOffset&& move_offset);

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

  ROS_INFO("waypointV2MissionStateSubCallback");
  ROS_INFO("missionStatePushAck->commonDataVersion:%d\n",waypoint_V2_mission_state_push_.commonDataVersion);
  ROS_INFO("missionStatePushAck->commonDataLen:%d\n",waypoint_V2_mission_state_push_.commonDataLen);
  ROS_INFO("missionStatePushAck->data.state:0x%x\n",waypoint_V2_mission_state_push_.state);
  ROS_INFO("missionStatePushAck->data.curWaypointIndex:%d\n",waypoint_V2_mission_state_push_.curWaypointIndex);
  ROS_INFO("missionStatePushAck->data.velocity:%d\n",waypoint_V2_mission_state_push_.velocity);

  mState.lock();
  mission_state = waypoint_V2_mission_state_push_.state;
  mState.unlock();
}

void setWaypointV2Defaults(dji_osdk_ros::WaypointV2& waypointV2)
{
  waypointV2.waypointType = dji_osdk_ros::DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  waypointV2.headingMode = dji_osdk_ros::DJIWaypointV2HeadingModeAuto;
  waypointV2.config.useLocalCruiseVel = 0;
  waypointV2.config.useLocalMaxVel = 0;

  waypointV2.dampingDistance = 40;
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

    auto wp0 = wpList[wp_name];

    setWaypointV2Defaults(startPoint);
    startPoint.latitude  = (float)wp0["latitude"];
    startPoint.longitude = (float)wp0["longitude"];
    startPoint.relativeHeight = (float)wp0["altitude"];
    startPoint.dampingDistance = (float)wp0["damping"];
    
    waypointList.push_back(startPoint);

    for (int i = 1; i < nWaypoint; i++)
    {
        std::string wp_name = "wp" + std::to_string(i);
        auto wpN = wpList[wp_name];
        dji_osdk_ros::WaypointV2 waypointV2;
        setWaypointV2Defaults(waypointV2);

        waypointV2.latitude = (float)wpN["latitude"];
        waypointV2.longitude = (float)wpN["longitude"];
        waypointV2.relativeHeight = (float)wpN["altitude"]; 
        waypointV2.dampingDistance = (float)wpN["damping"];
        waypointList.push_back(waypointV2)
    }

    return waypointList;
}

bool initWaypointV2Setting(ros::NodeHandle &nh)
{
    waypointV2_init_setting_client = nh.serviceClient<dji_osdk_ros::InitWaypointV2Setting>("dji_osdk_ros/waypointV2_initSetting");
    //These two parameters will be ignored when passed to the function, they were kept just to preserve the function prototype
    initWaypointV2Setting_.request.polygonNum = 6;
    initWaypointV2Setting_.request.radius = 6;
    //this changed to the fit the number of waypoints in the file; the generation of action is moved to after we know the mission size
    //initWaypointV2Setting_.request.actionNum = 5;

    initWaypointV2Setting_.request.waypointV2InitSettings.repeatTimes = 1;
    initWaypointV2Setting_.request.waypointV2InitSettings.finishedAction = initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionFinishedGoHome;
    initWaypointV2Setting_.request.waypointV2InitSettings.maxFlightSpeed = 10;
    initWaypointV2Setting_.request.waypointV2InitSettings.autoFlightSpeed = 2;
    initWaypointV2Setting_.request.waypointV2InitSettings.exitMissionOnRCSignalLost = 1;
    initWaypointV2Setting_.request.waypointV2InitSettings.gotoFirstWaypointMode = initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionGotoFirstWaypointModePointToPoint;
    // The function generatePolygon has been changed to populate the waypoint list 
    // with the content of the mission fila and match the waypoint V2 definitions
    initWaypointV2Setting_.request.waypointV2InitSettings.mission = generatePolygonWaypoints(nh, initWaypointV2Setting_.request.radius, initWaypointV2Setting_.request.polygonNum);
    initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen = initWaypointV2Setting_.request.waypointV2InitSettings.mission.size();

    /*! Generate actions. Changed */
    generateWaypointV2Actions(nh, initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen);
    
    waypointV2_init_setting_client.call(initWaypointV2Setting_);
    if (initWaypointV2Setting_.response.result)
    {
      ROS_INFO("Init mission setting successfully!\n");
    }
    else
    {
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

bool stopWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_stop_mission_client = nh.serviceClient<dji_osdk_ros::StopWaypointV2Mission>("dji_osdk_ros/waypointV2_stopMission");
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
    dji_osdk_ros::WaypointV2Action actionVector;
    for (uint16_t i = 0; i < actionNum; i++)
    {
      actionVector.actionId  = i;
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

bool runWaypointV2Mission(ros::NodeHandle &nh)
{
    int timeout = 1;
    bool result = false;

    get_drone_type_client = nh.serviceClient<dji_osdk_ros::GetDroneType>("get_drone_type");
    waypointV2_mission_state_push_client = nh.serviceClient<dji_osdk_ros::SubscribeWaypointV2Event>("dji_osdk_ros/waypointV2_subscribeMissionState");
    waypointV2_mission_event_push_client = nh.serviceClient<dji_osdk_ros::SubscribeWaypointV2State>("dji_osdk_ros/waypointV2_subscribeMissionEvent");

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
    result = initWaypointV2Setting(nh);
    if(!result)
    {
        return false;
    }
    sleep(timeout);

    /*! upload mission */
    result = uploadWaypointV2Mission(nh);
    if(!result)
    {
        return false;
    }
    sleep(timeout);

    /*! upload  actions */
    result = uploadWaypointV2Action(nh);
    if(!result)
    {
        return false;
    }
    sleep(timeout);

    /*! set global cruise speed */
    //We used to read this from the yaml file, for simplicity let's set it here for now.
    result = setGlobalCruiseSpeed(nh, 3); 
    if(!result)
    {
        return false;
    }
    sleep(timeout);

    /*! start mission */
    result = startWaypointV2Mission(nh);
    if(!result)
    {
        return false;
    }
    sleep(20);

    return true;
}

/********************** Added functions *********************/

void posOffsetsCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
    offsets = *msg;

    mOff.lock();
    x_offset = offsets.x;
    y_offset = offsets.y;
    z_offset = offsets.z;
    mOff.unlock();
}

// Function used to move to a position specified as an offset from the current drone's position
bool moveByPosOffset(MoveOffset&& move_offset)
{
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

/********************** End of Added functions *********************/

int main(int argc, char** argv)
{
    signal(SIGINT, finishHandler);
    signal(SIGTERM, finishHandler);
    ros::init(argc, argv, "waypointV2_lora_node");
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutDownCallback);
    
    ros::NodeHandle nh;

    if(argc < 2)
    {
        std::cout << "Wrong argument. Please provide a yaml file to start. Usage: rosrun dji_osdk_ros waypointV2_lora_node mission_filename.yaml" << std::endl;
        return -1;
    }
    mission_file_path = argv[1];
    ROS_INFO("Input yaml file: %s\n", mission_file_path.c_str());

    ros::Subscriber gpsPositionSub            = nh.subscribe<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
    ros::ServiceClient flight_control_client  = nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");
    ros::Subscriber pos_offsets_subscriber    = nh.subscribe<geometry_msgs::Point32>("position_offsets",10, &posOffsetsCallback);
    ros::Subscriber flight_mode_subscriber    = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/display_mode", 1, &ModeCallback);
    ros::Publisher correctedGPS_publisher_    = nh.advertise<std_msgs::Float32>("dji_osdk_ros/correctedGPSHeight",10);
    ros::Subscriber height_subscriber         = nh.subscribe<std_msgs::Float32>("dji_osdk_ros/height_above_takeoff", 1, &HeightCallback);
    ros::Subscriber flight_status_subscriber  = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/flight_status", 1, &FlightStatusCallback);

    ros::Duration(1).sleep();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Waypoint mission
    ROS_INFO("Executing approximation sequence.");
    runWaypointV2Mission(nh);
    ros::Duration(10).sleep();
    ROS_INFO("First approximation completed.");

    // LoRa Navigation (offset)
    ROS_INFO("Switching to radio/offset navigation.");

    if (moveByPosOffset(MoveOffset(x_offset, y_offset, z_offset, 0.0f)))
    {
        ROS_INFO("Switching to radio/offset navigation.");
        ros::Duration(5).sleep();
        offset_completed = true;
    }

    ros::waitForShutdown();
}