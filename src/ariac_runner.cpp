#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Kit.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations"
#include <cstdlib>
#include <string>
#include <vector>

using namespace std;

bool has_started_competition = false;
vector<osrf_gear::Order> current_orders;
int current_kit_index = 0;

void try_start_competition(ros::ServiceClient& begin_client, ros::Rate* loop_rate)
{
    if(!has_started_competition)
    {
        std_srvs::Trigger begin_comp;
        if(begin_client.call(begin_comp))
        {
            if(begin_comp.response.success)
            {
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

string current_kit_location(&ros::ServiceClient kit_lookup_client)
{
     
    osrf_gear::Kit current_kit = current_orders.first().kits.at(current_kit_index);
    osrf_gear::GetMaterialLocations kit_lookup;
    kit_lookup.request.material_type = current_kit.kit_type;

    ROS_INFO("Looking up location for kit type %s", current_kit.kit_type.c_str());
    if(kit_lookup_client.call(current_kit))
    {
        //Iterate t
        for(vector<osrf_gearm::StorageUnit>::iterator current = current_kit.response.storage_units.begin(); current != current_kit.response.storage_units.end(); ++current)
        {
            ROS_INFO("Found kit type %s in storage unit %s", current_kit.kit_type.c_str(), current->c_string());
        }
    }
    else
    {
        ROS_ERROR("Looking up kit location falied.");
    }
    
}


void new_order_callback(const osrf_gear::Order::ConstPtr& new_order)
{
    current_orders.push_back(*new_order);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab_3_ariac");
    ros::NodeHandle node_handle;
    
    string ns = ros::this_node::getNamespace();
    ROS_INFO("Detected namespace %s", ns.c_str());

    ros::ServiceClient begin_client = node_handle.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient kit_lookup_client = node_handle.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    ros::Subscriber order_subscriber = node_handle.subscribe("/ariac/orders", 200, new_order_callback);
    
    //Spin slow until competition starts
    ros::Rate loop_rate(0.2);

    //Main loop
    while(ros::ok())
    {
        try_start_competition(&begin_client, &loop_rate)
        if(has_started_competition)
        {

        }
        

        //process all callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;   
}