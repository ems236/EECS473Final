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
#include <cstdlib>
#include <string>
#include <vector>

using namespace std;

bool has_started_competition = false;
vector<osrf_gear::Order> current_orders;
osrf_gear::LogicalCameraImage camera_data;

int current_kit_index = 0;
int current_kit_object_index = 0;
string current_model_type = "";
int camera_model_index = -1;

bool is_current_object_known()
{
    return current_model_type.empty();
}

void try_start_competition(ros::ServiceClient& begin_client, ros::Rate* loop_rate)
{
    if(!has_started_competition)
    {
        ROS_INFO("got here ");
        std_srvs::Trigger begin_comp;
        if(begin_client.call(begin_comp))
        {
            ROS_INFO("Service was called");
            if(begin_comp.response.success)
            {
                ROS_INFO("successful service call");
                has_started_competition = true;
                *loop_rate = ros::Rate(10);
                ROS_INFO("successful service call 2");
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
        return;
    }
    
    ROS_INFO("Order has %d kits", current_orders.front().kits.size());
    
    for(vector<osrf_gear::KitObject>::iterator current = current_orders.front().kits.front().objects.begin(); current != current_orders.front().kits.front().objects.end(); ++current)
    {
        ROS_INFO("kit types: %s", current->type.c_str());
    }
    
    osrf_gear::Kit& current_kit = current_orders.front().kits.at(current_kit_index);
    current_model_type = current_kit.objects.at(current_kit_object_index).type;
    osrf_gear::GetMaterialLocations kit_lookup;
    kit_lookup.request.material_type = current_model_type;
    
    
    ROS_INFO("Looking up location for kit type %s", current_kit.kit_type.c_str());
    if(kit_lookup_client.call(kit_lookup))
    {
        //Iterate t
        ROS_INFO("Called kit type lookup service found %d", kit_lookup.response.storage_units.size());
        
        
        for(vector<osrf_gear::StorageUnit>::iterator current = kit_lookup.response.storage_units.begin(); current != kit_lookup.response.storage_units.end(); ++current)
        {
            ROS_INFO("Found kit type %s in storage unit %s", current_kit.kit_type.c_str(), current->unit_id.c_str());
            //ROS_INFO("Service replied");
        }
    }
    else
    {
        ROS_ERROR("Looking up kit location falied.");
    }
}

geometry_msgs::Pose lookup_object_location(string type)
{
    for(int i = 0; camera_data.models.size(); i++)
    {
        osrf_gear::Model current = camera_data.models.at(i);
        if(current.type.compare(type) == 0)
        {
            camera_model_index = i;
            geometry_msgs::Point position = current.pose.position;
            geometry_msgs::Quaternion orientation = current.pose.orientation; 
            ROS_INFO("object of type %s is \r\n at position %f %f %f \r\n and orientation %f %f %f %f", 
                type.c_str(),
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w);
            return current.pose;
        }
    }
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

bool have_valid_orders(ros::ServiceClient& begin_client, ros::Rate* loop_rate, ros::ServiceClient& kit_lookup_client)
{
    try_start_competition(begin_client, &loop_rate);
    if(has_started_competition)
    {
        current_kit_location(kit_lookup_client);
        return is_current_object_known()
    }

    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab_3_ariac");
    ros::NodeHandle node_handle;
    
    //string ns = ros::this_node::getNamespace();
    //ROS_INFO("Detected namespace %s", ns.c_str());

    ros::ServiceClient begin_client = node_handle.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient kit_lookup_client = node_handle.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    ros::Subscriber order_subscriber = node_handle.subscribe("/ariac/orders", 200, new_order_callback);
    ros::Subscriber logical_camera_subscriber = node_handle.subscribe("/ariac/logical_camera", 200, camera_callback);
    
    //Spin slow until competition starts
    ros::Rate loop_rate(0.2);

    //Main loop
    while(ros::ok())
    {
        if(have_valid_orders(begin_client, &loop_rate, kit_lookup_client))
        {
            geometry_msgs::Pose& object_pose = lookup_object_location(current_model_type);
        }

        //process all callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;   
}
