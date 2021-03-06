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

#include <std_msgs/Empty.h>
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


// empirical home position
// these MUST be changed every time the arm is calibrated.
const double HOME_X = 50.0;
const double HOME_Y = 0.0;
const double HOME_Z = 50.0;

// number of seconds the uSwift has to say 'ok' in response to a command
const double USB_RESPONSE_TIME = 0.001;

// TODO: these are probably wrong
const double JOINT0_INITIAL = 85.0;
const double JOINT1_INITIAL = 117.0;
const double JOINT2_INITIAL = 45.0;
const double JOINT3_INITIAL = 90.0;
double joint0 = JOINT0_INITIAL;
double joint1 = JOINT1_INITIAL;
double joint2 = JOINT2_INITIAL;
double joint3 = JOINT3_INITIAL;


// rosparam-dependent values
// -----


// how fast the move commands are executed on the uSwift
// (10000 is fastest, 0 is slowest I guess)
const int MOVE_SPEED_DEFAULT = 25;
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
int usbWrite(std::string Gcode)
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
        return 1;
    }
    return 0;
}


// an 'empty' command to go to home at HOME_X, HOME_Y, HOME_Z
void home_callback(const std_msgs::Empty msg_in)
{
    char x[8];
    char y[8];
    char z[8];
    char ms[8];
    sprintf(x, "%.2f", HOME_X);
    sprintf(y, "%.2f", HOME_Y);
    sprintf(z, "%.2f", HOME_Z);
    sprintf(ms, "%i", 400*move_speed); // ?

    std::string Gcode = std::string("G0 X")
        + x + " Y" + y + " Z" + z + " F" + ms + "\n";

    ROS_DEBUG("Sending the uSwift to home.\n"
              "Gcode: %s\n", Gcode.c_str());

    usbWrite(Gcode);
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
    if (abs(msg_in.data) > 0.001)
    {
        char degree[8];
        char joint[8];
        char ms[8];
        joint0 += msg_in.data;
        sprintf(degree, "%.2f", msg_in.data);
        sprintf(joint, "%.2f", joint0);
        sprintf(ms, "%i", move_speed);
        std::string Gcode = std::string("G2202 N0 V")
            + joint + " F" + ms + "\n";
        ROS_DEBUG("Sending joint0 command to the uSwift.\n"
            "Gcode: %s\n", Gcode.c_str());
        usbWrite(Gcode);
    }
}

void joint1_write_callback(const std_msgs::Float64& msg_in)
{
    if (abs(msg_in.data) > 0.001)
    {
        char degree[8];
        char joint[8];
        char ms[8];
        joint1 += msg_in.data;
        sprintf(degree, "%.2f", msg_in.data);
        sprintf(joint, "%.2f", joint1);
        sprintf(ms, "%i", move_speed);
        std::string Gcode = std::string("G2202 N1 V")
          + joint + " F" + ms + "\n";
        ROS_DEBUG("Sending joint1 command to the uSwift.\n"
            "Gcode: %s\n", Gcode.c_str());
        usbWrite(Gcode);
    }
}

void joint2_write_callback(const std_msgs::Float64& msg_in)
{
    if (abs(msg_in.data) > 0.001)
    {
        char degree[8];
        char joint[8];
        char ms[8];
        joint2 += msg_in.data;
        sprintf(degree, "%.2f", msg_in.data);
        sprintf(joint, "%.2f", joint2);
        sprintf(ms, "%i", move_speed);
        std::string Gcode = std::string("G2202 N2 V")
          + joint + " F" + ms + "\n";
        ROS_DEBUG("Sending joint2 command to the uSwift.\n"
            "Gcode: %s\n", Gcode.c_str());
        usbWrite(Gcode);
    }
}

void joint3_write_callback(const std_msgs::Float64& msg_in)
{
    char degree[8];
    char joint[8];
    char ms[8];
    joint3 += msg_in.data;
    sprintf(degree, "%.2f", msg_in.data);
    sprintf(joint, "%.2f", joint3);
    sprintf(ms, "%i", move_speed);
    std::string Gcode = std::string("G2202 N3 V")
        + joint + " F" + ms + "\n";
    ROS_DEBUG("Sending joint3 command to the uSwift.\n"
              "Gcode: %s\n", Gcode.c_str());
    usbWrite(Gcode);
}


void vector_write_callback(const geometry_msgs::Vector3& msg_in)
{
    if (abs(msg_in.x) > 0.001 ||
        abs(msg_in.y) > 0.001 ||
        abs(msg_in.z) > 0.001)
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
    }
}


void cyl_vector_write_callback(const geometry_msgs::Vector3& msg_in)
{
    if (abs(msg_in.x) > 0.001 || 
        abs(msg_in.y) > 0.001 || 
        abs(msg_in.z) > 0.001)
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
 * TODO: Document this confusing document
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


/*
 * Node name: pnr_core
 *
 * Topics published: (rate = 20Hz, queue size = 1)
 * - /pnr_swiftpro/
 *     /state - position info, plus actuator status
 *     /position - position of the end-effector
 *     /actuator - actuator status (true = open, false = closed)
 *
 * Topics subscribed: (queue size = 1 for all)
 * - /pnr_swiftpro
 *     /position_write
 *     /cyl_position_write
 *     /vector_write
 *     /cyl_vector_write
 *     /actuator_write
 *     /joint0_write
 *     /joint1_write
 *     /joint2_write
 *     /joint3_write
 *
 * Parameters accepted:
 * - /pnr_swiftpro
 *     /port = (string) the port of the swiftpro
 *       (default = "/dev/ttyACM0")
 *     /move_speed (int) the speed of the move commands
 *       (default = 25, range = [1, 10000])
 *     /pose_update_period = (double) state publish update period
 *       (default = 0.2)
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

    ros::Subscriber home_sub = 
        nh.subscribe("home", 1, home_callback);
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

        // perform set up operations
        ros::Duration(0.5).sleep();        // wait 0.5s
        home_callback(std_msgs::Empty());  // move to the "home"
        ros::Duration(0.1).sleep();        // wait 0.1s
        std_msgs::Float64 angle_tmp;
        angle_tmp.data = 0.0;
        joint3_write_callback(angle_tmp);
        ros::Duration(0.1).sleep();        // wait 0.1s

        // Ask for position updates every pose_update_period seconds.
        char pus_msg[50];
        sprintf(pus_msg, "M2120 V%.3f\n", pose_update_period);

        ROS_INFO("Sending feedback command to the uSwift.\n"
            "Gcode: %s", pus_msg);
        usbWrite(std::string(pus_msg));

        // ---
        // initialize and start timers

        // the uSwift's position update timer
        // this is a 'best-effort' timing strategy
        ros::Timer pos_update_timer
            = nh.createTimer(
                ros::Duration(pose_update_period),
                state_read_callback);

        // program loop: wait for callbacks and publish when we are ready
        while (ros::ok())
        {
            if (publish_uswift_state)
            {
                // update rosparams in time to the state publisher
                nh.param<int>("move_speed", move_speed, MOVE_SPEED_DEFAULT);

                us_state_pub.publish(us_state);
                us_pos_pub.publish(us_pos);
                us_actuator_on_pub.publish(us_actuator_on);
                publish_uswift_state = false;
            }
            // you spin me right round baby right round
            ros::spinOnce();
        }
    }

    return 0;  // Трахни грудь моей женщины!
}
