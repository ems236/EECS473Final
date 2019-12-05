#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Kit.h"
#include "osrf_gear/KitObject.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/AVGControl.h"
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

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include "angles/angles.h"

#include <cstdlib>
#include <string>
#include <vector>
#include <map>

#define PI 3.14159

using namespace std;

bool has_started_competition = false;
vector<osrf_gear::Order> current_orders;
osrf_gear::LogicalCameraImage camera_data;
osrf_gear::LogicalCameraImage agv_camera_data;

//sensor_msgs::JointState joint_states;

map<string, float> joint_state_map;
bool has_found_joint_states = false;

//Kinemtatics info
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_sols[8][6];
double best_solution[6];
double home_position[7] {0.03874, 3.1, -0.9, 1.5, 3.022, -1.615, 0.0445};

/*
//Old lookup data
int current_kit_index = 0;
int current_kit_object_index = 0;
string current_model_type = "";
*/

int camera_model_index = -1;
int sequence_number = 0;

/*
bool is_current_object_known()
{
    return !current_model_type.empty();
}*/

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

/*
void current_kit_location(ros::ServiceClient& kit_lookup_client)
{
    if(is_current_object_known() || current_orders.size() == 0 || current_orders.front().kits.size() == 0)
    {
        //ROS_INFO("There is no need to lookup the current order locaiton again.");
        return;
    }
    
    //ROS_INFO("Order has %d kits", current_orders.front().kits.size());
    
    /*
    for(vector<osrf_gear::KitObject>::iterator current = current_orders.front().kits.front().objects.begin(); current != current_orders.front().kits.front().objects.end(); ++current)
    {
        //ROS_INFO("Kit types: %s", current->type.c_str());
    }*/
    /*
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
}*/

bool have_valid_orders(ros::ServiceClient& begin_client, ros::Rate* loop_rate, ros::ServiceClient& kit_lookup_client)
{
    try_start_competition(begin_client, loop_rate);
    if(has_started_competition)
    {
        return current_orders.size() != 0 && current_orders.front().kits.size() != 0;
        /*
        //ROS_INFO("Competition is started, looking up orders.");
        current_kit_location(kit_lookup_client);
        return is_current_object_known();
        */
    }

    return false;
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

bool lookup_object_location(string type, geometry_msgs::Pose* part_pose)
{
    //Only cares about the single logical camera
    for(int i = 0; camera_data.models.size(); i++)
    {
        osrf_gear::Model current = camera_data.models.at(i);
        if(current.type.compare(type) == 0)
        {
            //camera_model_index = i;
            //print_pose("Object of type" + type, current.pose);
            *part_pose = current.pose; 
            return true;
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

void agv_camera_callback(const osrf_gear::LogicalCameraImage& camera_info)
{
    agv_camera_data = camera_info;
}

bool lookup_agv_tray_position(geometry_msgs::Pose* pose)
{
    for(osrf_gear::Model current : agv_camera_data.models)
    {
        if(current.type == "kit_tray")
        {
            *pose = current.pose;
            return true;
        }
    }
    return false;
}

void joint_state_listener(const sensor_msgs::JointState& joint_state)
{
    for(int joint_index = 0; joint_index < joint_state.name.size(); joint_index++)
    {
        joint_state_map[joint_state.name[joint_index]] = joint_state.position[joint_index];
    }
    has_found_joint_states = true;
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

geometry_msgs::PoseStamped logical_camera_to_base_link(tf2_ros::Buffer& tfBuffer, geometry_msgs::Pose& logical_pose, string current_frame)
{
    //ROS_INFO("Converting the logical camera pose to world coordinates.");
    // Retrieve the transformation
    geometry_msgs::TransformStamped tf_logical_to_world, tf_world_to_base_link;
	
    try 
    {
        tf_logical_to_world = tfBuffer.lookupTransform(
            "world",
            //"logical_camera_frame", ros::Time(0.0), ros::Duration(1.0)
            current_frame, ros::Time(0.0), ros::Duration(1.0)
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

void apply_solution_constraints(int num_sols)
{
    //best_solution = q_sols[0];    
    //return;

    int best_index = 0;
    int best_heuristic_count = 0;

    bool should_normalize_positive[6] = {true, false, false, false, false, false};

    int heuristic_weight[6] = {10, 10, 10, 0, 0, 0};
    float lowerBounds[6] = {-1.0 * PI / 2, -1.0f * PI, PI / 6.0, PI, PI, PI};
    float higherBound[6] = {PI / 2.0, 0, PI, 2.0f * PI, 2.0f * PI, 2.0f * PI};
    for(int solution_index = 0; solution_index < num_sols; solution_index++)
    {
        int heuristics_satisfied = 0;
        for(int angle_index = 0; angle_index < 6; angle_index++)
        {
            float current_angle = angles::normalize_angle(q_sols[solution_index][angle_index]);
            if(lowerBounds[angle_index] < current_angle && current_angle < higherBound[angle_index])
            {
                heuristics_satisfied += heuristic_weight[angle_index] * 1;
            }
        }

        //ROS_INFO("index %i satisfies %i heuristics", solution_index, heuristics_satisfied);
        if(heuristics_satisfied > best_heuristic_count)
        {
            best_index = solution_index;
            best_heuristic_count = heuristics_satisfied;
        }
    }

    //ROS_INFO("Picking index %i as the best solution", best_index);
    for(int joint_index = 0; joint_index < 6; joint_index++)
    {
        float non_normalized_angle = q_sols[best_index][joint_index];
        if(should_normalize_positive[joint_index])
        {
            best_solution[joint_index] = angles::normalize_angle_positive(non_normalized_angle);
        }
        else
        {
            best_solution[joint_index] = angles::normalize_angle(non_normalized_angle);
        }
    }
    //best_solution = q_sols[0];    
}

void inverse_desired_pos(float x, float y, float z)
{
    T_des[0][3] = x;
    T_des[1][3] = y;
    T_des[2][3] = z; // above part
    T_des[3][3] = 1.0;
    // The orientation of the end effector so that the end effector is down.
    T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
    T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
    T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
    T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_sols);
    apply_solution_constraints(num_sols);
}

void inverse_desired_pos(geometry_msgs::PoseStamped& desired_pose)
{
    inverse_desired_pos(desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z);
}

void initialize_trajectory(control_msgs::FollowJointTrajectoryAction& trajectory_action)
{
    trajectory_msgs::JointTrajectory joint_trajectory;
    // Fill out the joint trajectory header.
    trajectory_action.action_goal.header.seq = sequence_number++; // Each joint trajectory should have anincremented sequence number
    trajectory_action.action_goal.header.stamp = ros::Time::now(); // When was this messagecreated.
    trajectory_action.action_goal.header.frame_id = "/world"; // Frame in which this is specified.
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
    joint_trajectory.points.resize(1);
    // Set the start point to the current position of the joints from joint_states.
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    //joint_trajectory.points[0].positions.clear();
    //ROS_INFO("elbow is at %f", joint_state_map["elbow_joint"]);
    
    for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) 
    {
        string current_name = joint_trajectory.joint_names[indy];
        joint_trajectory.points[0].positions[indy] = joint_state_map[current_name];
        //ROS_INFO("%s is at point %f", current_name.c_str(), joint_state_map[current_name]);
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
    trajectory_action.action_goal.goal.trajectory = joint_trajectory;
}

void add_point_to_trajectory(control_msgs::FollowJointTrajectoryAction& trajectory_action, const ros::Duration& time_from_start, double* joint_positions, double arm_position)
{
    trajectory_msgs::JointTrajectory joint_trajectory = trajectory_action.action_goal.goal.trajectory;
    joint_trajectory.points.resize(joint_trajectory.points.size() + 1);
    int last_index = joint_trajectory.points.size() - 1;

    joint_trajectory.points[last_index].positions.resize(joint_trajectory.joint_names.size());
    //joint_trajectory.points[last_index].positions.clear();
    // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    joint_trajectory.points[last_index].positions[0] = arm_position;
    //ROS_INFO("%s is moving to point %f", joint_trajectory.joint_names[0].c_str(), arm_position);

    // The actuators are commanded in an odd order, enter the joint positions in the correct positions
    for (int indy = 0; indy < 6; indy++) 
    {
        //ROS_INFO("%s is moving to point %f", joint_trajectory.joint_names[indy + 1].c_str(), joint_positions[indy]);
        joint_trajectory.points[last_index].positions[indy + 1] = joint_positions[indy];
    }
    // How long to take for the movement.
    joint_trajectory.points[last_index].time_from_start = time_from_start;
    trajectory_action.action_goal.goal.trajectory = joint_trajectory;
}

void add_linear_move_to_trajectory(control_msgs::FollowJointTrajectoryAction& trajectory_action, const ros::Duration& time_from_start, double linear_position)
{
    trajectory_msgs::JointTrajectory joint_trajectory = trajectory_action.action_goal.goal.trajectory;
    joint_trajectory.points.resize(joint_trajectory.points.size() + 1);
    int last_index = joint_trajectory.points.size() - 1;
    joint_trajectory.points[last_index].positions.resize(joint_trajectory.joint_names.size());

    joint_trajectory.points[last_index].positions[0] = linear_position;
    for (int indy = 1; indy < joint_trajectory.joint_names.size(); indy++) 
    {
        joint_trajectory.points[last_index].positions[indy] = joint_trajectory.points[last_index - 1].positions[indy];
    }

    joint_trajectory.points[last_index].time_from_start = time_from_start;
    trajectory_action.action_goal.goal.trajectory = joint_trajectory;
}

void add_point_to_trajectory(control_msgs::FollowJointTrajectoryAction& trajectory_action, const ros::Duration& time_from_start, double* joint_positions)
{
    add_point_to_trajectory(trajectory_action, time_from_start, joint_positions, joint_state_map["linear_arm_actuator_joint"]);   
}

void add_best_point_to_trajectory(control_msgs::FollowJointTrajectoryAction& trajectory_action, const ros::Duration& time_from_start)
{
    add_point_to_trajectory(trajectory_action, time_from_start, best_solution);
}

void add_home_point_to_trajectory(control_msgs::FollowJointTrajectoryAction& trajectory_action, const ros::Duration& time_from_start)
{
    float home_arm_position = home_position[0];
    add_point_to_trajectory(trajectory_action, time_from_start, &home_position[1], home_arm_position);   
}

void set_suction(ros::ServiceClient& begin_client, bool is_enabled)
{
    osrf_gear::VacuumGripperControl vaccuum_call;
    vaccuum_call.request.enable = is_enabled;
    begin_client.call(vaccuum_call);
}

void submit_kit(ros::ServiceClient& submit_client, string& kit_type)
{
    osrf_gear::AGVControl avgRequest;
    avgRequest.request.kit_type = kit_type;
    submit_client.call(avgRequest);
}

void add_world_point_to_trajectory(control_msgs::FollowJointTrajectoryAction& trajectory_action, const ros::Duration& time_from_start, geometry_msgs::PoseStamped& goal_pose)
{
    inverse_desired_pos(goal_pose);
    add_best_point_to_trajectory(trajectory_action, time_from_start);
}

void add_world_point_to_trajectory(control_msgs::FollowJointTrajectoryAction& trajectory_action, const ros::Duration& time_from_start, float x, float y, float z)
{
    inverse_desired_pos(x, y, z);
    add_best_point_to_trajectory(trajectory_action, time_from_start);
}

void move_to_dropoff(
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& trajectory_as
    , tf2_ros::Buffer &tfBuffer
    , ros::ServiceClient& grip_client
    , geometry_msgs::Pose& target_pose
)
{
    double dropoff_orientation[6] {1.57, -1.57, 1.5, 3.022, -1.65, 0.0445};
    double dropoff_linear_position = 2.1;

    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
    initialize_trajectory(joint_trajectory_as);
    add_point_to_trajectory(joint_trajectory_as, ros::Duration(1.0), dropoff_orientation);
    add_point_to_trajectory(joint_trajectory_as, ros::Duration(5.0), dropoff_orientation, dropoff_linear_position);

    actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(3.0));
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());

    ros::spinOnce();

    geometry_msgs::Pose tray_pose_local; 
    lookup_agv_tray_position(&tray_pose_local);
    geometry_msgs::PoseStamped tray_home_pose = logical_camera_to_base_link(tfBuffer, tray_pose_local, "logical_camera_over_agv1_frame");
    //print_pose("Agv kit pose in world", goal_pose.pose);

    tray_home_pose.pose.position.z += 0.1;

    control_msgs::FollowJointTrajectoryAction over_tray_joint_trajectory_as;
    initialize_trajectory(over_tray_joint_trajectory_as);
    add_world_point_to_trajectory(over_tray_joint_trajectory_as, ros::Duration(2.0), tray_home_pose);

    geometry_msgs::PoseStamped goal_pose = logical_camera_to_base_link(tfBuffer, target_pose, "agv1_load_point_frame");
    //print_pose("Agv kit pose in world", goal_pose.pose);
    goal_pose.pose.position.z += 0.1;
    add_world_point_to_trajectory(over_tray_joint_trajectory_as, ros::Duration(4.0), goal_pose);


    state = trajectory_as.sendGoalAndWait(over_tray_joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(3.0));
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
    ROS_INFO("Moved to dropoff");

    set_suction(grip_client, false);

    control_msgs::FollowJointTrajectoryAction return_home_joint_trajectory_as;
    initialize_trajectory(return_home_joint_trajectory_as);
    add_point_to_trajectory(return_home_joint_trajectory_as, ros::Duration(2.0), dropoff_orientation);
    add_linear_move_to_trajectory(return_home_joint_trajectory_as, ros::Duration(4.0), home_position[0]);
    add_home_point_to_trajectory(return_home_joint_trajectory_as, ros::Duration(7.0));
    
    state = trajectory_as.sendGoalAndWait(return_home_joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(3.0));
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
    ROS_INFO("Moved to home");
}

void move_to_point_and_grip(
    geometry_msgs::PoseStamped& goal_pose
    , actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& trajectory_as
    , ros::ServiceClient& grip_client)
{
    //offset so above
    goal_pose.pose.position.z += 0.3; 
    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
    initialize_trajectory(joint_trajectory_as);
    add_world_point_to_trajectory(joint_trajectory_as, ros::Duration(3.0), goal_pose);

    goal_pose.pose.position.z -= 0.28;
    add_world_point_to_trajectory(joint_trajectory_as, ros::Duration(4.0), goal_pose);


    actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(3.0));
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
    
    set_suction(grip_client, true);
    //move_to_best_position(joint_trajectory_as);
    ros::Duration(1.0).sleep();

    control_msgs::FollowJointTrajectoryAction joint_trajectory_drop;
    initialize_trajectory(joint_trajectory_drop);
    goal_pose.pose.position.z += 0.28;
    add_world_point_to_trajectory(joint_trajectory_drop, ros::Duration(1.0), goal_pose);


    state = trajectory_as.sendGoalAndWait(joint_trajectory_drop.action_goal.goal, ros::Duration(30.0), ros::Duration(3.0));
    ROS_INFO("finished moving");
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());


    //GRIP
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
    ros::ServiceClient vacuum_client = node_handle.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    ros::ServiceClient agv1_submit_client = node_handle.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");

    ros::Subscriber order_subscriber = node_handle.subscribe("/ariac/orders", 200, new_order_callback);
    ros::Subscriber logical_camera_subscriber = node_handle.subscribe("/ariac/logical_camera", 200, camera_callback);
    ros::Subscriber agv_logical_camera_subscriber = node_handle.subscribe("/ariac/logical_camera_over_agv1", 200, agv_camera_callback);
    ros::Subscriber joint_state_subscriber = node_handle.subscribe("/ariac/joint_states", 10, joint_state_listener);
    
    ros::Publisher trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("ariac/arm/command", 200);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm/follow_joint_trajectory", true);

    // It is possible to reuse the JointTrajectory from above
    
    // The header and goal (not the tolerances) of the action must be filled out as well.
    // (rosmsg show control_msgs/FollowJointTrajectoryAction)

    //Spin slow until competition starts
    ros::Rate loop_rate(0.2);
    while(!has_found_joint_states)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
    initialize_trajectory(joint_trajectory_as);
    add_linear_move_to_trajectory(joint_trajectory_as, ros::Duration(2.0), home_position[0]);
    add_home_point_to_trajectory(joint_trajectory_as, ros::Duration(5.0));
    actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(3.0));
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
    ROS_INFO("Moved home");

    ros::Duration(1.0).sleep();
    ros::spinOnce();

    //move_to_dropoff(trajectory_as, tfBuffer);

    ros::Duration(1.0).sleep();
    
    
    while(!have_valid_orders(begin_client, &loop_rate, kit_lookup_client))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    //ROS_INFO("Moved home");

    
    //Main loop
    //while(ros::ok())
    //{   
        ROS_INFO("%d orders", current_orders.size());
        for(osrf_gear::Order current_order : current_orders)
        {
            ROS_INFO("order contains %d kits", current_order.kits.size());
            for(osrf_gear::Kit current_kit : current_order.kits)
            {
                ROS_INFO("Kit contains %d objects", current_kit.objects.size());
                int kit_object_index = 0;
                while(kit_object_index < current_kit.objects.size())
                {
                    osrf_gear::KitObject current_object = current_kit.objects.at(kit_object_index);
                    ROS_INFO("Kit type: %s", current_object.type.c_str());

                    bool was_successful = false;
                    geometry_msgs::Pose object_pose_local; 
                    if(lookup_object_location(current_object.type, &object_pose_local))
                    {
                        geometry_msgs::PoseStamped goal_pose = logical_camera_to_base_link(tfBuffer, object_pose_local, "logical_camera_frame");
                        print_pose("Object location in base_link", goal_pose.pose);
                        
                        move_to_point_and_grip(goal_pose, trajectory_as, vacuum_client);
                        ros::Duration(0.5).sleep();
                        move_to_dropoff(trajectory_as, tfBuffer, vacuum_client, current_object.pose);

                        was_successful = true;
                    }
                    //lookup / grab object
                    
                    if(was_successful)
                    {
                        kit_object_index++;
                    }
                    else
                    {
                        ROS_WARN("Unable to find current kit object, trying again");
                    }
                    
                    ros::spinOnce();
                    loop_rate.sleep();
                }

                submit_kit(agv1_submit_client, current_kit.kit_type);
            }
            
              
        }

        ros::spinOnce();
        loop_rate.sleep();

        /*
        for(int logical_object_index = 0; logical_object_index < camera_data.models.size(); logical_object_index++)
        {
            geometry_msgs::Pose object_pose_local; 
            if(lookup_next_object(logical_object_index, &object_pose_local))
            {
                geometry_msgs::PoseStamped goal_pose = logical_camera_to_base_link(tfBuffer, object_pose_local, "logical_camera_frame");
                print_pose("Object location in base_link", goal_pose.pose);
                
                move_to_point_and_grip(goal_pose, trajectory_as, vacuum_client);
                ros::Duration(0.5).sleep();
                move_to_dropoff(trajectory_as, tfBuffer, vacuum_client);
            }
        }*/

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

        */
    //}
    
    //process all callbacks
    
    
    spinner.stop();

    return 0;   
}
