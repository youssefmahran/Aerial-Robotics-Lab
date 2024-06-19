#include <ros/ros.h>  // ROS library

#include <mavros_msgs/CommandBool.h>  // For arming/disarming the drone

#include <mavros_msgs/CommandTOL.h>  // For takeoff and landing commands

#include <mavros_msgs/State.h>  // For drone state information

#include <mavros_msgs/SetMode.h>  // For setting drone mode

#include <mavros_msgs/PositionTarget.h>  // For setting position targets

#include <geometry_msgs/PoseStamped.h>  // For getting position information

#include <cmath>  // For mathematical functions

/*#####################################################*/

// Global variables for storing the drone's state and position

mavros_msgs::State current_state;   // Message containing the current state of the drone

ros::Subscriber state_sub;  // Subscriber to the MAVROS topic for the drone state

ros::Subscriber pose_sub;   // Subscriber to the MAVROS topic for the drone's position

ros::ServiceClient setModeClient;   // Service client to set the drone mode

ros::ServiceClient arming_client;   // Service client to arm or disarm the drone

ros::ServiceClient takeoff_client;  // Service client to takeoff the drone

ros::ServiceClient landing_client;  // Service client to land the drone

ros::Publisher setpoint_pub;    // Publisher to publish the drone's target position

bool isVehicleArmed = false;    // Flag to indicate if the drone is armed

float curr_x;   // Drone's current X position

float curr_y;   // Drone's current Y position

float curr_z;   // Drone's current Z position

float des_x;    // Drone's target X position

float des_y;    // Drone's target Y position

float des_z;    // Drone's target Z position

/*#####################################################*/

// Callback function that stores the drone's state
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    
    current_state = *msg;  // Update the current state
    
    isVehicleArmed = msg->armed;  // Update the armed state
}

/*#####################################################*/

// Callback function that stores the drone's position
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    
    curr_x = (*msg).pose.position.x;  // Update the current X position
    
    curr_y = (*msg).pose.position.y;  // Update the current Y position
    
    curr_z = (*msg).pose.position.z;  // Update the current Z position
}

/*#####################################################*/

// Function to convert degrees to radians
double rad(double degrees) {
    
    return degrees * M_PI / 180.0;
}

/*#####################################################*/

// Function to calculate the Euclidean distance between current and target position
double distance() {
    
    double dx = curr_x - des_x;  // Difference in X position
    
    double dy = curr_y - des_y;  // Difference in Y position
    
    double dz = curr_z - des_z;  // Difference in Z position
    
    return std::sqrt(dx * dx + dy * dy + dz * dz);  // Calculate the Euclidean distance
}

/*#####################################################*/

// Function to make the drone fly in a circle
void flyInSquare(const float x, const float y, const float z, ros::Rate& rate) {
    
    mavros_msgs::PositionTarget setpoint;  // Create a position target message
    
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;  // Set coordinate frame to local NED
    
    setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                         mavros_msgs::PositionTarget::IGNORE_VY |
                         mavros_msgs::PositionTarget::IGNORE_VZ |
                         mavros_msgs::PositionTarget::IGNORE_AFX |
                         mavros_msgs::PositionTarget::IGNORE_AFY |
                         mavros_msgs::PositionTarget::IGNORE_AFZ;  // Ignore velocity and acceleration
    
    for(int i = 0; i < 4; i++){  // Loop to create waypoints for a square path

        switch(i){ 
            case 0: 
            des_x = 1;
            des_y = 0; 
            des_z = z;
            break;

            case 1: 
            des_x = 1;
            des_y = 1; 
            des_z = z;
            break;

            case 2: 
            des_x = 0;
            des_y = 1;
            des_z = z; 
            break;

            case 3: 
            des_x = 0;
            des_y = 0;
            des_z = z;
            break;
             
            default: 
            break; 
        } 
        
        setpoint.position.x = des_x;  // Set desired X position
        
        setpoint.position.y = des_y;  // Set desired Y position
        
        setpoint.position.z = des_z;  // Set desired Z position
        
        setpoint_pub.publish(setpoint);  // Publish the setpoint
        
        while (ros::ok() && distance() > 0.1) {  // Wait until the drone reaches the setpoint
            
            ros::spinOnce();  // Process callbacks
            
            rate.sleep();  // Sleep for the specified rate
        } 
    } 
}

/*#####################################################*/

// Function to set the drone's mode
bool setMode(const std::string& mode) {
    
    mavros_msgs::SetMode set_mode;  // Create a set mode message
    
    set_mode.request.custom_mode = mode;  // Set the desired mode

    if (setModeClient.call(set_mode)) {  // Call the set mode service
        
        if (set_mode.response.mode_sent) {  // Check if the mode change was successful
            
            ROS_INFO_STREAM("Mode changed to " << mode);  // Inform the user
            
            return true;  // Return true if successful
        } else {
            
            ROS_ERROR_STREAM("Failed to set mode to " << mode);  // Inform the user
            
            return false;  // Return false if unsuccessful
        }
    } else {
        
        ROS_ERROR("Failed to call set_mode service");  // Inform the user
        
        return false;  // Return false if the service call failed
    }
}

/*#####################################################*/

// Function to arm or disarm the drone
bool arm(ros::Rate& rate, bool f){
    
    mavros_msgs::CommandBool arm_cmd;  // Create an arming message
    
    arm_cmd.request.value = f;  // Set arming to true (ARM) or false (DISARM)
    
    if (arming_client.call(arm_cmd) && arm_cmd.response.success ) {  // Call the arming service
        
        if(f)
            ROS_INFO("Vehicle armed");  // Inform the user if the drone is armed
        else
            ROS_INFO("Vehicle disarmed");  // Inform the user if the drone is disarmed
    } else {
        
        return false;  // Return false if the service call failed
    }
    
    // Wait until the vehicle is successfully armed
    
    int armTimeout = 300;  // Timeout in seconds
    
    while (ros::ok() && !isVehicleArmed && armTimeout > 0) {  // While the vehicle is still not armed
        
        ros::spinOnce();  // Process callbacks
        
        rate.sleep();  // Sleep for the specified rate
        
        armTimeout--;  // Decrease the timeout counter
        
        ROS_INFO("Arming");  // Inform the user
    }

    if(armTimeout == 0){
        
        ROS_ERROR("Failed to arm in time");  // Inform the user if arming failed
        
        return false;  // Return false if arming failed
    }
    else
        
        ROS_INFO("Armed");  // Inform the user if arming was successful
    
    return true;  // Return true if arming was successful
}

/*#####################################################*/

// Function to takeoff the drone
bool takeoff(float z, ros::Rate& rate){ 
    
    mavros_msgs::CommandTOL takeoff_cmd;  // Create a takeoff message
    
    takeoff_cmd.request.altitude = z;  // Set the takeoff altitude

    if (takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success) {  // Call the takeoff service
        
        ROS_INFO("Takeoff command sent");  // Inform the user
    } 
    else{
        
        ROS_INFO("Takeoff command not sent");  // Inform the user if the takeoff command failed
        
        return false;  // Return false if the takeoff command failed
    }

    // Wait until the drone reaches the takeoff altitude
    
    double takeoffAltitude = takeoff_cmd.request.altitude;  // Get the takeoff altitude
    
    while (ros::ok() && current_state.armed && takeoffAltitude - curr_z > 0.3) {  // While the drone has not reached the altitude
        
        ros::spinOnce();  // Process callbacks
        
        rate.sleep();  // Sleep for the specified rate
    }
    
    return true;  // Return true if takeoff was successful
}

/*#####################################################*/

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "simple_takeoff");  // Initialize the ROS node
    
    ros::NodeHandle nh;  // Create a NodeHandle

    /*#####################################################*/

    // Subscribe to the state and position topics
    
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);  // Subscribe to the state topic
    
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, poseCallback);  // Subscribe to the position topic
    
    /*#####################################################*/

    // Advertise the service clients and publisher
    
    setModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");  // Create a service client for setting the mode
    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");  // Create a service client for arming/disarming
    
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");  // Create a service client for takeoff
    
    landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");  // Create a service client for landing
    
    setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);  // Create a publisher for setpoints

    /*#####################################################*/

    // Wait for the connection to be established
    
    ros::Rate rate(10.0);  // Set the loop rate to 10 Hz
    
    int connectTimeout = 30;  // Timeout in seconds
    
    while (ros::ok() && !current_state.connected && connectTimeout > 0) {  // While not connected
        
        ros::spinOnce();  // Process callbacks
        
        rate.sleep();  // Sleep for the specified rate
        
        connectTimeout--;  // Decrease the timeout counter
    }
    
    if (!current_state.connected) {  // Check if connection was not established
        
        ROS_ERROR("Failed to connect to the vehicle. Check your MAVROS connection.");  // Inform the user
        
        return -1;  // Return an error code
    }

    /*#####################################################*/
    
    if (!setMode("GUIDED")) {  // Set the drone mode to GUIDED
        
        ROS_ERROR("Failed to set GUIDED mode");  // Inform the user if setting mode failed
        
        return -1;  // Return an error code
    }
    
    if (!arm(rate, true)) {  // Arm the drone
        
        ROS_ERROR("Failed to arm");  // Inform the user if arming failed
        
        return -1;  // Return an error code
    }

    if(!takeoff(1 , rate)){  // Takeoff to 1 meter altitude
        
        ROS_ERROR("Failed to takeoff");  // Inform the user if takeoff failed
        
        return -1;  // Return an error code
    }

    flyInSquare(curr_x, curr_y, curr_z, rate);  // Fly in a square path
    
    if (!setMode("LAND")) {  // Set the drone mode to LAND
        
        ROS_ERROR("Failed to set LAND mode");  // Inform the user if setting mode failed
        
        return -1;  // Return an error code
    }

    return 0;  // Return success
}
