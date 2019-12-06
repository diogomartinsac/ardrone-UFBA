#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "ardrone_controller/pid.hpp"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"

using namespace std;



// void visionCallback(const std_msgs::String::ConstPtr& msg){

// }



int main(int argc, char **argv)
{
    ros::init(argc, argv,"motionPlanner");
    
    ros::NodeHandle node;
    ros::Rate(10);
    
    // ros::Subscriber visual_data_sub = node.subscribe("visual_data", 1000, visionCallback);
    ros::Subscriber vision_sub = node.subscribe("", 1000, visionCallback);
    //ros::Subscriber sonar_height_sub = node.subscribe("sonar_height", 1000, sonarCallback);
    
    ros::Publisher cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    ros::Time timeNow(0), timePrev(0);
    // ros::Duration dt(0);
    timePrev = ros::Time::now();

    geometry_msgs::Twist mv;
    
    

    while(ros::ok()){

        if ((ros::Time::now() - timePrev).toSec() >= 0.06){
            dt = (ros::Time::now() - timePrev).toSec();
            timePrev = ros::Time::now();
            

            //mv.linear.x = xControl.calculate(dt, 0, vxMeasured);
            //mv.linear.y = yControl.calculate(dt, 1000, vyMeasured);
            mv.linear.z = zControl.calculate(dt, 0.5, height);
            
            cout<< height<<endl;
            
            //cmd_vel.publish(mv);
        }
        
        ros::spinOnce();
        
    }
    return 0;
}
