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

bool try_start_competition(ros::ServiceClient& begin_client)
{

    std_srvs::Trigger begin_comp;
    if(begin_client.call(begin_comp))
    {
        if(begin_comp.response.success)
        {
            has_started_competition = true;
            return true;
        }
        else
        {
            ROS_WARN("Start competition unsuccessful. Message %s", begin_comp.response.message.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Error contacting competition service.");
        exit(0);
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
    ros::Subscriber order_subscriber = node_handle.subscribe("/ariac/orders", 200, new_order_callback);
    
    //Spin slow until competition starts
    ros::Rate loop_rate(0.2);

    //Main loop
    while(ros::ok())
    {
        if(!has_started_competition)
        {
            if(try_start_competition(begin_client))
            {
                loop_rate = ros::Rate(10);
            }
        }
        else
        {
            
        }
        

        //process all callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;   
}