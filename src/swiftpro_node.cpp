/*
 * Software License Agreement:
 * do whatever teh crap you want with this software
 * just mention my name if you use it, bitte
 * 
 * Author: Joshua Petrin <joshua.m.petrin@vanderbilt.edu>     
 */
#include <cstdlib>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
// #include <geometry_msgs/Twist.h>

#include <pnr_swiftpro/uSwiftState.h>

/*
 * For reference...
 * The uSwift's error codes are the following:
 * 
 * E20 = Command does not exist
 * E21 = Parameter error
 * E22 = Location out of range
 * E23 = Command buffer full
 * E24 = Power unconnected
 * E25 = Operation failure
 */


// number of seconds the uSwift has to say 'ok' in response to a command
const double USB_RESPONSE_TIME = 0.001;

// time delay between ROS spins
const int ROS_SPIN_PERIOD = 0.001;

// TODO: these are probably wrong
const double JOINT0_INITIAL = 90.00;
const double JOINT1_INITIAL = 130.00;
const double JOINT2_INITIAL = 45.00;
const double JOINT3_INITIAL = 45.00;
double joint0 = JOINT0_INITIAL;
double joint1 = JOINT1_INITIAL;
double joint2 = JOINT2_INITIAL;
double joint3 = JOINT3_INITIAL;


// rosparam-dependent values
// -----

// Too many vector commands will flood the uSwift.
// This variable is set to 'true' every vector_flood_delay seconds.
bool accept_vector = true;

const double VECTOR_FLOOD_DELAY_DEFAULT = 0.2;
double vector_flood_delay = VECTOR_FLOOD_DELAY_DEFAULT;


// how fast the move commands are executed on the uSwift
// (10000 is fastest, 0 is slowest I guess)
const int MOVE_SPEED_DEFAULT = 8000;
int move_speed;

// time between how often this node publishes a new state (secs)
const double POSE_UPDATE_PERIOD_DEFAULT = 0.3;
double pose_update_period = POSE_UPDATE_PERIOD_DEFAULT;

// publishing clearence, true => ready to publish new state
bool publish_uswift_state = false;

// usb port pointing to the swift
const std::string USB_PORT_DEFAULT = "/dev/ttyACM0";
std::string usb_port = USB_PORT_DEFAULT;


// published values
pnr_swiftpro::uSwiftState us_state;
geometry_msgs::Point us_pos;
std_msgs::Bool us_actuator_on;


// serial object, from the amazing ROS serial package
serial::Serial *usb;

// generic function for writing a string to the serial
void usbWrite(std::string Gcode) 
{
    usb->flush();
    usb->write(Gcode.c_str());
    ros::Duration(0.01).sleep();      // wait 0.01s (?)
    usb->flush();
    std::string result = usb->read(usb->available());
    if (result[0] == 'E')
    {
        ROS_WARN("Received error from uSwift after the command %s:\n"
         "%s", Gcode.c_str(), result.c_str());
    }
}

void position_write_callback(const geometry_msgs::Point& msg_in)
{
    char x[8];
    char y[8];
    char z[8];
    char ms[8];
    sprintf(x, "%.2f", msg_in.x);
    sprintf(y, "%.2f", msg_in.y);
    sprintf(z, "%.2f", msg_in.z);
    sprintf(ms, "%i", move_speed);

    std::string Gcode = std::string("G0 X") 
        + x + " Y" + y + " Z" + z + " F" + ms + "\n";

    ROS_DEBUG("Sending a position command to the uSwift.\n"
        "Gcode: %s\n", Gcode.c_str());

    usbWrite(Gcode);
}


void cyl_position_write_callback(const geometry_msgs::Point& msg_in)
{
    // stretch, rotation, and height (aka: r, theta, z)
    char s[8];
    char r[8];
    char h[8];
    char ms[8];
    sprintf(s, "%.2f", msg_in.x);
    sprintf(r, "%.2f", msg_in.y);
    sprintf(h, "%.2f", msg_in.z);
    sprintf(ms, "%i", move_speed);


    std::string Gcode = std::string("G2201 S")
        + s + " R" + r + " H" + h + " F" + ms + "\n";

    ROS_DEBUG("Sending a cylindrical pos command to the uSwift.\n"
        "Gcode: %s\n", Gcode.c_str());

    usbWrite(Gcode);
}


void joint0_write_callback(const std_msgs::Float64& msg_in)
{
    if (accept_vector && abs(msg_in.data) > 0.001)
    {
        char degree[8];
        sprintf(degree, "%.2f", msg_in.data);
        std::string Gcode = std::string("G2202 N0 V") 
            + (joint0 += degree) + " F" + move_speed + "\n";
        ROS_DEBUG("Sending joint0 command to the uSwift.\n"
            "Gcode: %s\n", Gcode.c_str());
        usbWrite(Gcode);

        accept_vector = false;
    }
}
void joint1_write_callback(const std_msgs::Float64& msg_in)
{
    if (accept_vector && abs(msg_in.data) > 0.001)
    {
        char degree[8];
        sprintf(degree, "%.2f", msg_in.data);
        std::string Gcode = std::string("G2202 N1 V")
            + (joint1 += degree) + " F" + move_speed + "\n";
        ROS_DEBUG("Sending joint1 command to the uSwift.\n"
            "Gcode: %s\n", Gcode.c_str());
        usbWrite(Gcode);

        accept_vector = false;
    }
}
void joint2_write_callback(const std_msgs::Float64& msg_in)
{
    if (accept_vector && abs(msg_in.data) > 0.001)
    {
        char degree[8];
        sprintf(degree, "%.2f", msg_in.data);
        std::string Gcode = std::string("G2202 N2 V")
            + (joint2 += degree) + " F" + move_speed + "\n";
        ROS_DEBUG("Sending joint2 command to the uSwift.\n"
            "Gcode: %s\n", Gcode.c_str());
        usbWrite(Gcode);

        accept_vector = false;
    }
}
void joint3_write_callback(const std_msgs::Float64& msg_in)
{
    if (accept_vector && abs(msg_in.data) > 0.001)
    {
        char degree[8];
        sprintf(degree, "%.2f", msg_in.data);
        std::string Gcode = std::string("G2202 N3 V")
            + (joint3 += degree) + " F" + move_speed + "\n";
        ROS_DEBUG("Sending joint3 command to the uSwift.\n"
            "Gcode: %s\n", Gcode.c_str());
        usbWrite(Gcode);

        accept_vector = false;
    }
}


void vector_write_callback(const geometry_msgs::Vector3& msg_in)
{
    if (accept_vector && 
        (abs(msg_in.x) > 0.001 || 
         abs(msg_in.y) > 0.001 || 
         abs(msg_in.z) > 0.001))
    {
        char x[8];
        char y[8];
        char z[8];
        char ms[8];
        sprintf(x, "%.2f", msg_in.x);
        sprintf(y, "%.2f", msg_in.y);
        sprintf(z, "%.2f", msg_in.z);
        sprintf(ms, "%i", move_speed);


        std::string Gcode = std::string("G2204 X") + x + " Y" + y + " Z" + z + " F" +
            ms + "\r\n";
        ROS_DEBUG("Sending vector command to the uSwift.\n"
            "Gcode: %s", Gcode.c_str());

        usbWrite(Gcode);

        accept_vector = false;
    }
}


void cyl_vector_write_callback(const geometry_msgs::Vector3& msg_in)
{
    if (accept_vector && 
        (abs(msg_in.x) > 0.001 || 
         abs(msg_in.y) > 0.001 || 
         abs(msg_in.z) > 0.001))
    {
        char s[8];
        char r[8];
        char h[8];
        char ms[8];
        sprintf(s, "%.2f", msg_in.x);
        sprintf(r, "%.2f", msg_in.y);
        sprintf(h, "%.2f", msg_in.z);
        sprintf(ms, "%i", move_speed);

        std::string Gcode = std::string("G2205 S")
            + s + " R" + r + " H" + h + " F" + ms + "\n";
        ROS_DEBUG("Sending cyl.vector command to the uSwift.\n"
            "Gcode: %s", Gcode.c_str());
        
        usbWrite(Gcode);

        accept_vector = false;
    }
}


void actuator_write_callback(const std_msgs::Bool& msg_in)
{
    us_actuator_on.data = msg_in.data;

    std::string Gcode = "M2232 ";
    if (us_actuator_on.data)
        Gcode += "V1";
    else
        Gcode += "V0";
    Gcode += "\n"; // this is very important!!

    ROS_DEBUG("Sending actuator command to the uSwift.\n"
        "Gcode: %s", Gcode.c_str());
        
    usbWrite(Gcode);
}

/**
 * TODO: Document this confusing document (lol) 
 */
void state_read_callback(const ros::TimerEvent&)
{
    // I'm not sure if it's possible to get the state of 
    // all the joints from the uSwift, so for now, we'll just
    // do the actuator angle and the end effector position.

    usb->flush();
    std::string data = usb->read(usb->available());
    ROS_DEBUG_STREAM("Read from USB: " << data);
    // DEBUG
    // data = usb->read(usb->available());
    // ROS_INFO_STREAM("Read from USB: " << data);

    // In case the buffer fills up, this array needs to be reeaally big
    char str[1000];
    if (data.length() > 999)
    {
        // ensure stack smashing does not occur
        ROS_DEBUG_STREAM("Read malformed uSwift data.");
        return;
    }
    strcpy(str, data.c_str());
    char* pch = strtok(str, " ");
    float x, y, z, r;

    int vals = 0;

    while (vals < 4)
    {
        if (pch == NULL)
        {
            ROS_DEBUG_STREAM("Read malformed uSwift data.");
            return;
        }
        switch (pch[0])
        {
            case 'X':
                x = atof(pch+1);
                ++vals;
                break;
            case 'Y':
                y = atof(pch+1);
                ++vals;
                break;
            case 'Z':
                z = atof(pch+1);
                ++vals;
                break;
            case 'R':
                r = atof(pch+1);
                ++vals;
                break;
            case '@':
                break;
            default:
                ROS_DEBUG_STREAM("Read malformed uSwift data.");
                return;
        }
        pch = strtok(NULL, " ");
    }

    // pnr_swiftpro::uswift_state us_state
    us_state.x = x;
    us_state.y = y;
    us_state.z = z;
    us_state.angle_actuator = r;
    us_state.actuator = us_actuator_on.data;

    // geometry_msgs::Point us_pos
    us_pos.x = x;
    us_pos.y = y;
    us_pos.z = z;

    // std_msgs::Bool us_actuator_on
    // we assume that the state of the actuator is equal to our stored state

    // we can publish now
    publish_uswift_state = true;
}


void vector_accept_callback(const ros::TimerEvent&)
{
    // ROS_DEBUG("Opening the uSwift floodgate");
    accept_vector = true;
}


/* 
 * Node name: pnr_core
 *   
 *
 * Topics published: (rate = 20Hz, queue size = 1)
 *   
 *
 * Topics subscribed: (queue size = 1)
 *   
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pnr_swiftpro");
    ros::NodeHandle nh("pnr_swiftpro");

    // set defaults for the actuator
    us_actuator_on.data = false;

    std::string usb_port;

    // complete state of the robot
    ros::Publisher us_state_pub
        = nh.advertise<pnr_swiftpro::uSwiftState>("state", 1);
    ros::Publisher us_pos_pub = 
        nh.advertise<geometry_msgs::Point>("position", 1);
    ros::Publisher us_actuator_on_pub = 
        nh.advertise<std_msgs::Bool>("actuator", 1);

    ros::Subscriber pos_sub = 
        nh.subscribe("position_write", 1, position_write_callback);
    ros::Subscriber pcy_sub = 
        nh.subscribe("cyl_position_write", 1, cyl_position_write_callback);
    ros::Subscriber vec_sub = 
        nh.subscribe("vector_write", 1, vector_write_callback);
    ros::Subscriber vcy_sub = 
        nh.subscribe("cyl_vector_write", 1, cyl_vector_write_callback);
    ros::Subscriber atr_sub = 
        nh.subscribe("actuator_write", 1, actuator_write_callback);
    ros::Subscriber j0_sub = 
        nh.subscribe("joint0_write", 1, joint0_write_callback);
    ros::Subscriber j1_sub = 
        nh.subscribe("joint1_write", 1, joint1_write_callback);
    ros::Subscriber j2_sub = 
        nh.subscribe("joint2_write", 1, joint2_write_callback);
    ros::Subscriber j3_sub = 
        nh.subscribe("joint3_write", 1, joint3_write_callback);
    // ros::Rate loop_rate(20);

    if (!nh.hasParam("port"))
        ROS_WARN("One-time defaulting to /pnr_swiftpro/port = '%s'.",
            USB_PORT_DEFAULT.c_str());
    nh.param<std::string>("port", usb_port, USB_PORT_DEFAULT);

    if (!nh.hasParam("move_speed"))
        ROS_WARN("Defaulting to /pnr_swiftpro/move_speed = %d.",
            MOVE_SPEED_DEFAULT);

    if (!nh.hasParam("pose_update_period"))
        ROS_WARN("One-time defaulting to /pnr_swiftpro/pose_update_period = %.2f.",
            POSE_UPDATE_PERIOD_DEFAULT);
    nh.param<double>(
        "pose_update_period", 
        pose_update_period, 
        POSE_UPDATE_PERIOD_DEFAULT);

    if (!nh.hasParam("vector_flood_delay"))
        ROS_WARN("Defaulting to /pnr_swiftpro/vector_flood_delay = %.2f.", 
            VECTOR_FLOOD_DELAY_DEFAULT);

    ros::Duration(3.5).sleep();  // wait 3.5s (???)

    try
    {
        usb = new serial::Serial(
            usb_port, // TODO: Make a program that will test the ports for the uSwift
            115200, 
            serial::Timeout::simpleTimeout(250));
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open USB port for the uSwift!");
        ROS_ERROR_STREAM("Trace: " << e.what());
        return -1;
    }
    
    if (usb->isOpen())
    {
        ROS_INFO_STREAM("uSwift's USB port has been opened successfully");

        ros::Duration(3.5).sleep();     // wait 3.5s
        // usb->write("M2120 V0\r\n");  // stop report position
        ros::Duration(0.1).sleep();     // wait 0.1s
        usb->write("M17\n");            // attach
        ROS_INFO_STREAM("uSwift attached and waiting for commands.");
        ros::Duration(1.0).sleep();     // wait 1.0s

        std::string result = usb->read(usb->available());
        ROS_INFO("uSwift startup message: \n%s", result.c_str());

        ros::Duration(0.5).sleep();       // wait 0.5s
        usb->write("G0 X50 Y0 Z50 F1000\n"); // move to the uncalibrated "home" position
                                          // TODO: Learn how to calibrate this mugger
                                          // Maybe have a command that sets home position??
        ros::Duration(USB_RESPONSE_TIME).sleep();      // wait some
        usb->flush();
        result = usb->read(usb->available());
        if (result[0] == 'E')
        {
            ROS_WARN("Received error from uSwift after the command %s:\n"
                "%s", "G0 X50 Y0 Z50 F1000\n", result.c_str());
        }
        usb->flush();

        ros::Duration(0.1).sleep();     // wait 0.1s

        // Ask for position updates every pose_update_period seconds.
        char pus_msg[50];
        sprintf(pus_msg, "M2120 V%.3f\n", pose_update_period);

        ROS_INFO("Sending feedback command to the uSwift.\n"
            "Gcode: %s", pus_msg);
        usb->write(pus_msg);
        ros::Duration(pose_update_period).sleep(); // wait for an update?
        // ros::Duration(0.01).sleep();     // wait 0.01s
        result = usb->read(usb->available());
        if (result[0] == 'E')
        {
            ROS_WARN("Received error from uSwift after the command %s:\n"
             "%s", pus_msg, result.c_str());
        }


        // initialize and start timers

        // the uSwift's position update timer
        ros::Timer pos_update_timer 
            = nh.createTimer(
                ros::Duration(pose_update_period), 
                state_read_callback);

        // the vector timer update manager (ensure the vectors don't flood)
        ros::Timer vec_timer_manager
            = nh.createTimer(
                ros::Duration(vector_flood_delay), 
                vector_accept_callback);

        // program loop: wait for callbacks and publish when we are ready
        while (ros::ok())
        {
            // update rosparams
            nh.param<int>("move_speed", move_speed, MOVE_SPEED_DEFAULT);
            nh.param<double>("vector_flood_delay", 
                vector_flood_delay, 
                VECTOR_FLOOD_DELAY_DEFAULT);
            
            if (publish_uswift_state)
            {
                us_state_pub.publish(us_state);
                us_pos_pub.publish(us_pos);
                us_actuator_on_pub.publish(us_actuator_on);
                publish_uswift_state = false;
            }
            
            ros::spinOnce();

            ros::Duration(ROS_SPIN_PERIOD).sleep();
        }
    }

    
    return 0;  // Трахни грудь моей женщины! 
}
