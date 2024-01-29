//Author :Zahir Castrejon
//Date : 4/20/2023
//Title : 2023 Tesla Proof-of-concept code one by one. This Script has a acceleration profile for the dynamxile motors. 



#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds   

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include "dynamixel_sdk.h" // Uses Dynamixel SDK library

// Control table address
//#define ADDR_XM_TORQUE_ENABLE 24 //Control table address is different in Dynamixel model
//#define ADDR_XM_GOAL_POSITION 30
//#define ADDR_XM_GOAL_VELOCITY 32

//Creates a macros and CANNOT be changed unlike variables 
#define xl320_TORQUE_ENABLE 24 //Control table address is different in Dynamixel model
#define xl320_GOAL_VELOCITY 32
#define xl320_GOAL_POSITION 30 // control talb eaddress to get position 
// Protocol version
//#define PROTOCOL_VERSION 2.0
#define PROTOCOL_VERSION 2.0

#define PROTOCOL_VERSION1 1.0
// Default setting
#define DXL_ID1 1 //  Dynamixel ID: 1
#define DXL_ID2 2 //  Dynamixel ID: 2
#define DXL_ID3 3 // Motor for tilt Mechanism. 
#define DXL_ID4 4
#define DXL_ID5 5


#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0" // Check which port is being used on your controller, if you need to give permission, type "sudo chmod 777 /dev/ttyUSB0"
#define TORQUE_ENABLE 1           // Value for enabling the torque
#define TORQUE_DISABLE 0          // Value for disabling the torque
#define ACCELERATION_VALUE 15      // Value of Acceleration of motors in [rev/min^2]

#define ESC_ASCII_VALUE                 0x1b

//#define VELOCITY_INPUT			40

double joy_leftright;
double joy_fwdbwd;
double joy_up;
double joy_down;
double joy_end;
double joy_speedup;
double buttonA;
double buttonB;

// unsigned 8-bit integer type and only represents non-negative numbers 
uint8_t dxl_error = 0;             // Dynamixel error

//The next line creates a pointer called portHandler of the PortHandler & PacketHandler Class 
// this is all within the namespace called dynamixel. 
// to return the pointer, getPortHanler() & getPacketHandle() are used. 
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
//dynamixel::PacketHandler *packetHandler1= dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);

// Class created called DA_Manipulator
class DA_Manipulator{
public:
    DA_Manipulator(); // Constructor is a special member function that is executed to create a new object of that class. NO PARAMETERS
    ~DA_Manipulator(); // Deconstrucor cleans up any resorces that the bject may have acquired during its lifetime. 
    bool dxl_comm_result; // bool variable 
    int get_position();
    void set_position(int position); 
};

//From the DA_Manipulator, the constructor DA_Manipulator is used to open the port and set the BaudRate/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DA_Manipulator::DA_Manipulator() {
    ROS_INFO("Tesla Skycam is initialized");
    portHandler->openPort(); //openport
    portHandler->setBaudRate(BAUDRATE); //setsthe baudrate

    // Enable Dynamixel Torque
    ///// motor 1
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, xl320_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL_ID1);
    }
    ////////////////////////////////////// motor 2
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, xl320_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL_ID2);
    } 
    //////////////////////////////////////// motor 3
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID3, xl320_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL_ID3);
    }

//////////////////////////////////////// motor 4
   




}
// From the Class we use the Deconstructor to Disable the Dynamixdel Torque ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DA_Manipulator::~DA_Manipulator() {

    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, xl320_TORQUE_ENABLE ,TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    //////////////////////////////
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, xl320_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    //////////////////////////////////
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID3, xl320_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }


    // Close port
    portHandler->closePort();
    printf("PinchNode\n");
}

int DA_Manipulator::get_position(){
    uint16_t position; 
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID3, xl320_GOAL_POSITION, &position,&dxl_error );
    if (dxl_comm_result != COMM_SUCCESS) { 
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    return position;
}
void DA_Manipulator::set_position(int position) { //accepts position
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID3, xl320_GOAL_POSITION, position, &dxl_error); // writes the position needed
}

void joystick_input_sub(const sensor_msgs::Joy msg) //message callback function
{
    joy_leftright = msg.axes[4];
    joy_fwdbwd = msg.axes[5];
    joy_up = msg.buttons[4];
    joy_down = msg.buttons[5];
    joy_end = msg.buttons[6];
    joy_speedup = msg.buttons[7];
    buttonA = msg.buttons[0];
    buttonB = msg.buttons[1];
}


// Main function script  ********************************* 
int main(int argc, char *argv[]) 
{
    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    bool dxl_getdata_result = false;                  // GetParam result

    // Initial Parameters

    int pose1 = 0;
    int pose2 = 0;
    int pose3 = 0;
    int pose4 = 0;
    int pose5 = 0;

    DA_Manipulator da_manipulator;

    //ROS node creation
    ros::init(argc, argv, "Pinch");
    ros::NodeHandle n;

    ros::Subscriber joystick_sub = n.subscribe("/joy", 100, joystick_input_sub); // This subscriber is used to use the joy node and controls and calls the Void Call-Back function called joystick_input_sub
    //ros::Publisher da_current_pub = n.advertise<geometry_msgs::Pose>("/DA22/torque", 10);


    // 20Hz control loop
    ros::Rate rate(20.0);

    
    while (ros::ok())
    {
        // printf("Choose Direction: ");
        //scanf("%d", &skycam_direction);

        if(joy_fwdbwd == 1){
        int motor3 = da_manipulator.get_position();
          printf("position of motor 3 %d\n", motor3);
          int motor_right = 100 + motor3;
          da_manipulator.set_position(motor_right);

        }else if(joy_fwdbwd == -1){
            printf("NODE MOVE 1 \n");
            //pose3 = 1023;
          int motor3 = da_manipulator.get_position();
          printf("position of motor 3 %d\n", motor3);
          int motor_left = motor3 -100;
          da_manipulator.set_position(motor_left);

        }else if(joy_leftright == 1){
            printf("Moving left \n");
            pose1 = 2047;
            pose2 = 1023;

        }else if(joy_leftright == -1){
            printf("Moving right \n");
            pose1 = 1023;
            pose2 = 2047;

        }else if(joy_up == 1){
          int motor3 = da_manipulator.get_position();
          printf("position of motor 3 %d\n", motor3);
          int motor_middle = 500;
          da_manipulator.set_position(motor_middle);


        }
        // when button a is pressed down 
        else if(buttonA){
            pose4 = 1023;
            pose5 = 1023;
        }
        // when button b is press down
        else if(buttonB){
            pose4 = 2047;
            pose5 = 2047;
        }
        else if(joy_end == 1){
            printf("Program Ended \n");
            break;
        }

        packetHandler->write2ByteTxRx(portHandler, DXL_ID1, xl320_GOAL_VELOCITY, pose1, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_ID2, xl320_GOAL_VELOCITY, pose2, &dxl_error);

        // ax 12 motor Main reel in and out
        packetHandler->write2ByteTxRx(portHandler, DXL_ID1, xl320_GOAL_VELOCITY, pose4, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_ID2, xl320_GOAL_VELOCITY, pose5, &dxl_error);

        std::this_thread::sleep_for (std::chrono::milliseconds(20));

        if(joy_fwdbwd == 0){
            pose1 = 0;
            pose2 = 0;



        }else if(joy_fwdbwd == 0){
            pose1 = 0;
            pose2 = 0;
      
   

        }else if(joy_leftright == 0){
            pose1 = 0;
            pose2 = 0;
            pose3 = 0;


        }else if(joy_leftright == 0){
            pose1 = 0;
            pose2 = 0;
            pose3 = 0;
     

        }else if(joy_up == 0){
            pose1 = 0;
            pose2 = 0;
            pose3 = 0;


        }else if(joy_down == 0){
            pose1 = 0;
            pose2 = 0;
            pose3 = 0;

        }


        rate.sleep();

        ros::spinOnce();
    }

    return 0;
}
