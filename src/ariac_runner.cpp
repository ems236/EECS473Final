#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Kit.h"
#include "osrf_gear/KitObject.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
// MoveIt header files
// Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "ur_kinematics/ur_kin.h" 
#include "trajectory_msgs/JointTrajectory.h"

#include <cstdlib>
#include <string>
#include <vector>
#include <map>

using namespace std;

bool has_started_competition = false;
vector<osrf_gear::Order> current_orders;
osrf_gear::LogicalCameraImage camera_data;
//sensor_msgs::JointState joint_states;

map<string, float> joint_state_map;

//Kinemtatics info
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_sols[8][6];
double best_solution[6];

int current_kit_index = 0;
int current_kit_object_index = 0;
string current_model_type = "";
int camera_model_index = -1;
int sequence_number = 0;

bool is_current_object_known()
{
    return !current_model_type.empty();
}

void try_start_competition(ros::ServiceClient& begin_client, ros::Rate* loop_rate)
{
    if(!has_started_competition)
    {
		ROS_INFO("Calling Service...");
        std_srvs::Trigger begin_comp;
        if(begin_client.call(begin_comp))
        {
            ROS_INFO("Service was called");
            if(begin_comp.response.success)
            {
                ROS_INFO("Successful service call");
                has_started_competition = true;
                *loop_rate = ros::Rate(10);
            }
            else
            {
                ROS_WARN("Start competition unsuccessful. Message %s", begin_comp.response.message.c_str());
            }
        }
        else
        {
            ROS_ERROR("Error contacting competition service.");
            exit(0);
        }
    }
}

void current_kit_location(ros::ServiceClient& kit_lookup_client)
{
    if(is_current_object_known() || current_orders.size() == 0 || current_orders.front().kits.size() == 0)
    {
        //ROS_INFO("There is no need to lookup the current order locaiton again.");
        return;
    }
    
    //ROS_INFO("Order has %d kits", current_orders.front().kits.size());
    
    for(vector<osrf_gear::KitObject>::iterator current = current_orders.front().kits.front().objects.begin(); current != current_orders.front().kits.front().objects.end(); ++current)
    {
        //ROS_INFO("Kit types: %s", current->type.c_str());
    }
    
    osrf_gear::Kit& current_kit = current_orders.front().kits.at(current_kit_index);
    current_model_type = current_kit.objects.at(current_kit_object_index).type;
    osrf_gear::GetMaterialLocations kit_lookup;
    kit_lookup.request.material_type = current_model_type;
    
    
    //ROS_INFO("Looking up location for kit type %s", current_kit.kit_type.c_str());
    if(kit_lookup_client.call(kit_lookup))
    {
        //Iterate t
        ROS_INFO("Called kit type lookup service found %d", kit_lookup.response.storage_units.size());
        
        
        for(vector<osrf_gear::StorageUnit>::iterator current = kit_lookup.response.storage_units.begin(); current != kit_lookup.response.storage_units.end(); ++current)
        {
            ROS_INFO("Found kit type %s in storage unit %s", current_kit.kit_type.c_str(), current->unit_id.c_str());
        }
    }
    else
    {
        ROS_ERROR("Looking up kit location falied.");
    }
}

void print_pose(string message, geometry_msgs::Pose& pose)
{
    geometry_msgs::Point position = pose.position;
    geometry_msgs::Quaternion orientation = pose.orientation;
    ROS_INFO("%s \r\n at position %f %f %f \r\n and orientation %f %f %f %f", 
                message.c_str(),
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w);
}

geometry_msgs::Pose lookup_object_location(string type)
{
    for(int i = 0; camera_data.models.size(); i++)
    {
        osrf_gear::Model current = camera_data.models.at(i);
        if(current.type.compare(type) == 0)
        {
            camera_model_index = i;
            print_pose("Object of type" + type, current.pose); 
            return current.pose;
        }
    }
}

bool lookup_next_object(int index, geometry_msgs::Pose* pose)
{
    if(index < camera_data.models.size())
    {
        osrf_gear::Model current = camera_data.models.at(index);
        *pose = current.pose; 
        return true;
    }

    return false;
}

void new_order_callback(const osrf_gear::Order::ConstPtr& new_order)
{
    ROS_INFO("Received order");
    current_orders.push_back(*new_order);
}

void camera_callback(const osrf_gear::LogicalCameraImage& camera_info)
{
    camera_data = camera_info;
}

void joint_state_listener(const sensor_msgs::JointState& joint_state)
{
    for(int joint_index = 0; joint_index < joint_state.name.size(); joint_index++)
    {
        joint_state_map[joint_state.name[joint_index]] = joint_state.position[joint_index];
    }
}

bool have_valid_orders(ros::ServiceClient& begin_client, ros::Rate* loop_rate, ros::ServiceClient& kit_lookup_client)
{
    try_start_competition(begin_client, loop_rate);
    if(has_started_competition)
    {
        //ROS_INFO("Competition is started, looking up orders.");
        current_kit_location(kit_lookup_client);
        return is_current_object_known();
    }

    return false;
}

string remove_first_char(string string_val)
{
    if(string_val.size() > 1 && string_val.substr(0, 1).compare("/") == 0)
    {
        return string_val.substr(1, string_val.size() - 1);
    }
    else
    {
        return string_val;
    }
}

void offset_target_position(geometry_msgs::PoseStamped* goal_pose)
{
    goal_pose->pose.position.z += 0.10;
    goal_pose->pose.orientation.w = 0.707;
    goal_pose->pose.orientation.x = 0.0;
    goal_pose->pose.orientation.y = 0.707;
    goal_pose->pose.orientation.z = 0.0;
}

geometry_msgs::PoseStamped logical_camera_to_base_link(tf2_ros::Buffer& tfBuffer, geometry_msgs::Pose& logical_pose)
{
    //ROS_INFO("Converting the logical camera pose to world coordinates.");
    // Retrieve the transformation
    geometry_msgs::TransformStamped tf_logical_to_world, tf_world_to_base_link;
	
    try 
    {
        tf_logical_to_world = tfBuffer.lookupTransform(
            "world",
            "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0)
        );

        tf_world_to_base_link = tfBuffer.lookupTransform(
            "base_link",
            "world", ros::Time(0.0), ros::Duration(1.0)
        );
		
        /*
        ROS_INFO("Transform to [%s] from [%s]", 
            tfStamped.header.frame_id.c_str(),
            tf_world_to_base_link.child_frame_id.c_str()
        );
        */
    } 
    catch (tf2::TransformException &ex) 
    {
        ROS_ERROR("%s", ex.what());
        exit(1);
    }

    //Do the actual transform.
    geometry_msgs::PoseStamped local_pose, world_pose, base_link_pose;
    local_pose.pose = logical_pose;

    tf2::doTransform(local_pose, world_pose, tf_logical_to_world);
    //offset_target_position(&world_pose);
    tf2::doTransform(world_pose, base_link_pose, tf_world_to_base_link);

    return base_link_pose;
}


void populate_forward_kinematics()
{
    q_pose[0] = joint_state_map["shoulder_pan_joint"];
    q_pose[1] = joint_state_map["shoulder_lift_joint"];
    q_pose[2] = joint_state_map["elbow_joint"];
    q_pose[3] = joint_state_map["wrist_1_joint"];
    q_pose[4] = joint_state_map["wrist_2_joint"];
    q_pose[5] = joint_state_map["wrist_3_joint"];
    ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);
    
}

void inverse_desired_pos(geometry_msgs::PoseStamped& desired_pose)
{
    T_des[0][3] = desired_pose.pose.position.x;
    T_des[1][3] = desired_pose.pose.position.y;
    T_des[2][3] = desired_pose.pose.position.z + 0.3; // above part
    T_des[3][3] = 1.0;
    // The orientation of the end effector so that the end effector is down.
    T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
    T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
    T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
    T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_sols);
    best_solution = q_sols[0];
}

void move_to_best_position(const ros::Publisher& command_publisher)
{
    trajectory_msgs::JointTrajectory joint_trajectory;
    // Fill out the joint trajectory header.
    joint_trajectory.header.seq = sequence_number++; // Each joint trajectory should have anincremented sequence number
    joint_trajectory.header.stamp = ros::Time::now(); // When was this messagecreated.
    joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
    // Set the names of the joints being used. All must be present.
    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");
    // Set a start and end point.
    joint_trajectory.points.resize(2);
    // Set the start point to the current position of the joints from joint_states.
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());

    
    for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) 
    {
        string current_name = joint_trajectory.joint_names[indy];
        joint_trajectory.points[0].positions[indy] = joint_state_map[current_name];
        /*
        for (int indz = 0; indz < joint_states.name.size(); indz++) 
        {
            if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) 
            {
                joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
        */
    }
    

    // When to start (immediately upon receipt).
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
    // Must select which of the num_sols solution to use. Just start with the first.
    // Set the end point for the movement
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
    // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    joint_trajectory.points[1].positions[0] = joint_state_map["linear_arm_actuator_joint"];
    // The actuators are commanded in an odd order, enter the joint positions in the correct positions
    for (int indy = 0; indy < 6; indy++) 
    {
        joint_trajectory.points[1].positions[indy + 1] = best_solution[indy];
    }
    // How long to take for the movement.
    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
    // Publish the specified trajectory.
    command_publisher.publish(joint_trajectory);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab_3_ariac");
    ros::NodeHandle node_handle;
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    ros::ServiceClient begin_client = node_handle.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient kit_lookup_client = node_handle.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    ros::Subscriber order_subscriber = node_handle.subscribe("/ariac/orders", 200, new_order_callback);
    ros::Subscriber logical_camera_subscriber = node_handle.subscribe("/ariac/logical_camera", 200, camera_callback);
    ros::Subscriber joint_state_subscriber = node_handle.subscribe("/ariac/joint_states", 10, joint_state_listener);
    
    ros::Publisher trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("ariac/arm/command", 200);

    //Spin slow until competition starts
    ros::Rate loop_rate(0.2);

    //Main loop
    while(ros::ok())
    {
        
        if(have_valid_orders(begin_client, &loop_rate, kit_lookup_client))
        {
            for(int logical_object_index = 0; logical_object_index < camera_data.models.size(); logical_object_index++)
            {
                geometry_msgs::Pose object_pose_local; 
                if(lookup_next_object(logical_object_index, &object_pose_local))
                {
                    geometry_msgs::PoseStamped goal_pose = logical_camera_to_base_link(tfBuffer, object_pose_local);
                    print_pose("Object location in base_link", goal_pose.pose);
                    inverse_desired_pos(goal_pose);
                    move_to_best_position(trajectory_publisher);

                    return 0;
                }
            }

            /*
            move_group.setPoseTarget(object_pose_world);
            //moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
            ROS_INFO("Starting to plan...");
			
			ros::AsyncSpinner spinner(1);
            spinner.start();
            if(move_group.plan(movement_plan))
            {
                ROS_INFO("Planning successful, trying to execute...");
                if(move_group.execute(movement_plan))
                {
                    ROS_INFO("Execution successful. Exitting...");
                    exit(0);
                }
				else
				{
					ROS_INFO("Execution failed");
				}
            }
            else
            {
                ROS_INFO("Planning failed");
            }

            spinner.stop();
            */
        }
        //process all callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;   
}
