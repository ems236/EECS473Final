#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <cstdlib>
#include <string>
#include <vector>

using namespace std;

bool has_started_competition = false;

void try_start_competition(ros::ServiceClient begin_client)
{
    std_srvs::Trigger begin_comp;
    if(begin_client.call(begin_comp))
    {
        if(begin_comp.response.success)
        {
            has_started_competition = true;
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab_3_ariac");
    ros::NodeHandle node_handle;
    
    string ns = ros::this_node::getNamespace();
    ROS_INFO("Detected namespace %s", ns.c_str());

    ros::ServiceClient begin_client = node_handle.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    //ros::Subscriber velocity_subscriber = node_handle.subscribe(read_velocity_topic, 200, desired_velocity_callback);
    
    ros::Rate loop_rate(10);

    //Main loop
    while(ros::ok())
    {
        if(!has_started_competition)
        {

        }

        //process all callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;   
}