import rospy  # Import the rospy library for ROS functionalities

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL  # Import required services from mavros_msgs

from mavros_msgs.msg import State, PositionTarget  # Import required messages from mavros_msgs

from geometry_msgs.msg import PoseStamped  # Import PoseStamped message from geometry_msgs

import math  # Import the math module for mathematical functions


class DroneController:  # Define the DroneController class

    def __init__(self):  # Define the constructor method

        rospy.init_node('drone_controller', anonymous=True)  # Initialize the ROS node

        self.current_state = State()  # Initialize current_state as an empty State message

        self.arm_state = False  # Initialize arm_state as False

        self.rate = rospy.Rate(10)  # Set the loop rate to 10 Hz

        #######################################################

        # ROS subscribers

        rospy.Subscriber('mavros/state', State, self.state_callback)  # Subscribe to the 'mavros/state' topic

        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)  # Subscribe to the 'mavros/local_position/pose' topic

        #######################################################

        # ROS publishers

        self.pose_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)  # Create a publisher for setpoints

        #######################################################

        # ROS services

        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)  # Create a service client for setting the mode

        self.arm_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)  # Create a service client for arming/disarming

        self.takeoff_srv = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)  # Create a service client for takeoff

        #######################################################

        self.curr_x = 0  # Initialize current x position

        self.curr_y = 0  # Initialize current y position

        self.curr_z = 0  # Initialize current z position

        self.des_x = 0  # Initialize desired x position

        self.des_y = 0  # Initialize desired y position

        self.des_z = 0  # Initialize desired z position

    #######################################################

    def state_callback(self, state):  # Define the state callback method

        self.current_state = state  # Update current_state with the self.des state data

    #######################################################

    def pose_callback(self, msg: PoseStamped):  # Define the pose callback method

        self.curr_x = msg.pose.position.x  # Update current x position

        self.curr_y = msg.pose.position.y  # Update current y position

        self.curr_z = msg.pose.position.z  # Update current z position

    #######################################################

    def connect(self):  # Define the connect method

        while not rospy.is_shutdown():  # Loop until ROS is shut down

            if self.current_state.connected:  # Check if connected to MAVROS

                rospy.loginfo("Connected to MAVROS!")  # Log info message when connected

                return True  # Return True if connected

            self.rate.sleep()  # Sleep for the specified rate

        rospy.logerr("Failed to connect to MAVROS!")  # Log error message if connection fails

        return False  # Return False if connection fails

    #######################################################

    def set_mode(self, mode):  # Define the set_mode method

        while not rospy.is_shutdown():  # Loop until ROS is shut down

            if self.current_state.mode == mode:  # Check if the mode is already set

                rospy.loginfo("Mode changed to " + mode)  # Log info message when mode is changed

                return True  # Return True if mode is changed

            if rospy.Time.now() - self.current_state.header.stamp > rospy.Duration(5.0):  # Check if mode setting is taking too long

                rospy.logerr("Failed to set mode to " + mode)  # Log error message if setting mode fails

                return False  # Return False if setting mode fails

            self.set_mode_srv(custom_mode=mode)  # Call the set_mode service

            self.rate.sleep()  # Sleep for the specified rate

    #######################################################

    def arm(self, arm: bool):  # Define the arm method

        self.arm_srv(arm)  # Call the arm/disarm service

        while not rospy.is_shutdown():  # Loop until ROS is shut down

            if self.current_state.armed:  # Check if the vehicle is armed

                rospy.loginfo("Vehicle armed!")  # Log info message when vehicle is armed

                return True  # Return True if vehicle is armed

            if rospy.Time.now() - self.current_state.header.stamp > rospy.Duration(5.0):  # Check if arming is taking too long

                rospy.logerr("Failed to arm vehicle!")  # Log error message if arming fails

                return False  # Return False if arming fails

            self.rate.sleep()  # Sleep for the specified rate

    #######################################################

    def takeoff(self, altitude):  # Define the takeoff method

        try:

            resp = self.takeoff_srv(altitude=altitude)  # Call the takeoff service with the specified altitude

            if resp.success:  # Check if the takeoff service call was successful

                rospy.loginfo("Takeoff command sent successfully!")  # Log info message when takeoff command is sent

                while not rospy.is_shutdown():  # Loop until ROS is shut down

                    if altitude - self.curr_z <= 0.3:  # Check if the drone has reached the desired altitude

                        rospy.loginfo("Reached altitude {:.2f} meters".format(altitude))  # Log info message when altitude is reached

                        return True  # Return True if the desired altitude is reached

                    self.rate.sleep()  # Sleep for the specified rate

            else:  # If the takeoff service call was not successful

                rospy.logerr("Failed to send takeoff command!")  # Log error message if takeoff command fails

                return False  # Return False if takeoff command fails

        except rospy.ServiceException as e:  # Catch any service exceptions

            rospy.logerr("Service call failed: %s" % e)  # Log error message if service call fails

    #######################################################

    def flyInCircle(self, x, y, z):  # Define the flyInCircle method

        setpoint = PositionTarget()  # Create a PositionTarget message

        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # Set the coordinate frame

        setpoint.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ  # Set the type mask to ignore velocities and accelerations

        for i in range(1, 101):  # Loop 100 times to create a circular path

            self.des_x = x + 1 * math.cos(math.radians(i * 3.6))  # Calculate the desired x position

            self.des_y = y + 1 * math.sin(math.radians(i * 3.6))  # Calculate the desired y position

            self.des_z = z  # Calculate the desired z position

            setpoint.position.x = self.des_x  # Set the desired position in the setpoint message

            setpoint.position.y = self.des_y  # Set the desired y position in the setpoint message

            setpoint.position.z = self.des_z  # Set the desired z position in the setpoint message

            self.pose_pub.publish(setpoint)  # Publish the setpoint message

            while not rospy.is_shutdown() and self.distance() > 0.1:  # Loop until ROS is shut down or the distance is within the threshold

                self.rate.sleep()  # Sleep for the specified rate
            
        # Return to center
        self.des_x = 0  # Calculate the desired x position

        self.des_y = 0  # Calculate the desired y position

        self.des_z = 1  # Calculate the desired z position

        setpoint.position.x = self.des_x  # Set the desired position in the setpoint message

        setpoint.position.y = self.des_y  # Set the desired y position in the setpoint message

        setpoint.position.z = self.des_z  # Set the desired z position in the setpoint message

        self.pose_pub.publish(setpoint)  # Publish the setpoint message

        while not rospy.is_shutdown() and self.distance() > 0.1:  # Loop until ROS is shut down or the distance is within the threshold

            self.rate.sleep()  # Sleep for the specified rate

    #######################################################

    def distance(self):  # Define the distance method

        dx = self.curr_x - self.des_x  # Calculate the difference in x position

        dy = self.curr_y - self.des_y  # Calculate the difference in y position

        dz = self.curr_z - self.des_z  # Calculate the difference in z position

        return math.sqrt(dx * dx + dy * dy + dz * dz)  # Return the Euclidean distance

###########################################################

if __name__ == "__main__":  # Main script execution starts here

    controller = DroneController()  # Create an instance of DroneController

    if not controller.connect():  # Connect to MAVROS

        rospy.logerr("Exiting...")  # Log error message if connection fails

        exit()  # Exit the program

    if not controller.set_mode('GUIDED'):  # Set the mode to GUIDED

        rospy.logerr("Exiting...")  # Log error message if setting mode fails

        exit()  # Exit the program

    if not controller.arm(True):  # Arm the vehicle

        rospy.logerr("Exiting...")  # Log error message if arming fails

        exit()  # Exit the program

    if not controller.takeoff(1.0):  # Take off to 1 meter altitude

        rospy.logerr("Exiting...")  # Log error message if takeoff fails

        exit()  # Exit the program

    controller.flyInCircle(controller.curr_x, controller.curr_y, controller.curr_z) # Fly in a circular path

    if not controller.set_mode('LAND'):  # Set the mode to LAND

        rospy.logerr("Exiting...")  # Log error message if setting mode fails

        exit()  # Exit the program

    exit() # Exit the program
