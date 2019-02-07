

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

mavros_msgs::State current_state;
double local_pose_enu_x = 0;
double local_pose_enu_y = 0;
double local_pose_enu_z = 0;

double local_pose_ned_x = 0;
double local_pose_ned_y = 0;
double local_pose_ned_z = 0;

// setpoints
double acceptance_radius = 0.25;
const int num_of_wps = 4;

double setpoint_array[num_of_wps][3] = {
    {0.8, 0.7, 0.7},
    {2.2, 0.7, 0.7},
    {2.2, 1.4, 0.7},
    {0.8, 1.4, 0.7},
};

int wp_counter = 0;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

    local_pose_enu_x = msg->pose.position.x;
    local_pose_enu_y = msg->pose.position.y;
    local_pose_enu_z = msg->pose.position.z;

    // Transform position to NED
    local_pose_ned_x = local_pose_enu_y;
    local_pose_ned_y = local_pose_enu_x;
    local_pose_ned_z = -local_pose_enu_z;

    //ROS_INFO("Current position: (%g %g %g)", local_pose_x, local_pose_y, local_pose_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher target_local_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);


    // ros::Publisher target_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 2);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",1,local_pos_callback);

    ros::Publisher wp_pub = nh.advertise<std_msgs::Int64>("/wp_num", 1);
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float64>("/wp_dist", 1);
    ros::Publisher wp_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/wp_pose_ned", 1);
    ros::Publisher lpose_ned_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_pose_ned", 1);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(4.0);

    // wait for FCU connection
  while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connection established");
    geometry_msgs::PoseStamped pose;
    mavros_msgs::PositionTarget target;

  //  mavros_msgs::GlobalPositionTarget globalTarget;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL srv_takeoff{};
    srv_takeoff.request.altitude = 1;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;


    ros::Time last_request = ros::Time::now();
 float counter=0;
    while(ros::ok()){

    	if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
         }  else {
            true;
            //if( !current_state.armed &&
              //  (ros::Time::now() - last_request > ros::Duration(5.0))){
               // if( arming_client.call(arm_cmd) &&
               //     arm_cmd.response.success){
               //     ROS_INFO("Vehicle armed");
               // }
                //last_request = ros::Time::now();

        }
    	ros::Rate rate(4.0);
     	pose.header.stamp=ros::Time::now();
        pose.pose.position.x = 2+counter;
        pose.pose.position.y = 4+counter;
        pose.pose.position.z = 1+counter;
       // pose.pose.orientation.w= NAN;
       // pose.pose.orientation.x= NAN;
       // pose.pose.orientation.y= NAN;
       // pose.pose.orientation.z= NAN;


        //r_T = Vector3f(setpoint_array[wp_counter][0], setpoint_array[wp_counter][1], setpoint_array[wp_counter][2]);
        double setpoint_x = setpoint_array[wp_counter][0];
        double setpoint_y = setpoint_array[wp_counter][1];
        double setpoint_z = setpoint_array[wp_counter][2];

        double wp_distance = sqrt((local_pose_ned_x-setpoint_x) *  (local_pose_ned_x-setpoint_x) + (local_pose_ned_y-setpoint_y) * (local_pose_ned_y-setpoint_y));


        if( wp_distance < acceptance_radius) {

            wp_counter++;
            wp_counter %= num_of_wps; //wrap around to go from last setpoint -> first setpoint

        }

        target.coordinate_frame=mavros_msgs::PositionTarget::FRAME_BODY_NED;
        target.type_mask= mavros_msgs::PositionTarget::IGNORE_VX |
                		  mavros_msgs::PositionTarget::IGNORE_VY |
						  mavros_msgs::PositionTarget::IGNORE_VZ |
						  mavros_msgs::PositionTarget::IGNORE_AFX |
						  mavros_msgs::PositionTarget::IGNORE_AFY |
						  mavros_msgs::PositionTarget::IGNORE_AFZ |
						  mavros_msgs::PositionTarget::FORCE |
						  mavros_msgs::PositionTarget::IGNORE_YAW |
						  mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;

        target.header.stamp=ros::Time::now();
        target.position.x= setpoint_x;
        target.position.y= setpoint_y;
        target.position.z= setpoint_z;

        //publish next waypoint
        target_local_pub.publish(target);

        //publish number of waypoint
        std_msgs::Int64 wp_number;
        wp_number.data = wp_counter;
        wp_pub.publish(wp_number);

        //publish distance to waypoint
        std_msgs::Float64 distance;
        distance.data = wp_distance;
        distance_pub.publish(distance);

        //publish next waypoint transformed in ned for visualization
        geometry_msgs::PoseStamped wp_pose;
        wp_pose.position.x = setpoint_y;
        wp_pose.position.y = setpoint_x;
        wp_pose.position.z = -setpoint_z;
        wp_pose_pub.publish(wp_pose);

        //publish local position in ned for visualization
        geometry_msgs::PositionStamped lpos_ned_msg;
        lpos_ned_msg.position.x = local_pose_ned_x;
        lpos_ned_msg.position.y = local_pose_ned_y;
        lpos_ned_msg.position.z = local_pose_ned_z;
        lpose_ned_pub.publish(lpos_ned_msg);

        //   counter=counter+0.02;

         //Global Test
      /*    globalTarget.coordinate_frame=mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
          globalTarget.type_mask= mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                                      mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                                                      mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                                                      mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                                                      mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                                                      mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                                                      mavros_msgs::GlobalPositionTarget::FORCE |
                                                      mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
                                                      mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE ;

          globalTarget.header.stamp=ros::Time::now();
          globalTarget.latitude =2.00;
          globalTarget.longitude= 5.00;
          globalTarget.altitude= 1.00;

          target_global_pub.publish(globalTarget);*/
         // local_pos_pub.publish(pose);

      //trajectory_desired_pub(traject);

    //takeoff_client.call(srv_takeoff);
     //   if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
       //    return true;
        // else
       //return false;




        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}




