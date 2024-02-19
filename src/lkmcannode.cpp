#include <cstdint> // Include for data types like uint8_t
#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include "std_msgs/msg/string.hpp" // Modify as necessary for actual CAN message type
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"


#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
// #include <geometry_msgs/msg/transform_stamped.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>


#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>

#include <rclcpp/time.hpp>
#include "include/bldcMotor.hpp"


class DifferentialDriveNode : public rclcpp::Node {
public:

    double wheelbase = 0.290; // Example wheelbase in meters
    double x = 0.0; // Current x position of the robot in meters
    double y = 0.0; // Current y position of the robot in meters
    double theta = 0.0; // Current orientation of the robot in radians
    const double wheelDiameterMm = 175.0; // Wheel diameter in mm
    const double pi = 3.14159265358979323846;
    const double wheelCircumferenceMm = pi * wheelDiameterMm;
    const double ticksPerRotation = 65535.0; // Number of encoder ticks per full rotation
    const double distancePerTickMm = wheelCircumferenceMm / ticksPerRotation;
    double leftSpeedSetpoint = 0; // commanded speed set point
    double rightSpeedSetpoint = 0; // commanded speed set point
    double leftSpeedError = 0;
    double rightSpeedError = 0;
    double errorScaleFactor = 0.5;
    rclcpp::Time lastLoopTimestamp_;
    bool isFirstLoop = true; // bool for setting the last loop time stamp to a default value.



    DifferentialDriveNode() : Node("differential_drive_node") {

        // Initialize CAN publisher


        leftMotor = std::make_shared<BLDCMotor>(1);
        rightMotor = std::make_shared<BLDCMotor>(2);
        
         if (!initCanSocket()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket");
            rclcpp::shutdown();
        } else {
            keep_reading_ = true;
            can_read_thread_ = std::thread(&DifferentialDriveNode::readCanMessages, this);
        }
        
        // Subscriber for CMD_VEL
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
             "cmd_vel", 10, std::bind(&DifferentialDriveNode::velocityCallback, this, std::placeholders::_1));
         // Publisher for Odometry
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        
         // Timer to run loop at 100Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&DifferentialDriveNode::loop_callback, this));

        
    }

    // New method for startup sequence
    void startupProcedure() {

    }
     ~DifferentialDriveNode() {
        keep_reading_ = false;
        // leftMotor = std::make_shared<BLDCMotor>(1);
        // rightMotor = std::make_shared<BLDCMotor>(2);
        if (can_read_thread_.joinable()) {
            can_read_thread_.join();
        }
        if (close(can_socket_) < 0) {
            perror("Close");
        }
    }
    // Other public methods
private:
    
     std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    int can_socket_;
    std::thread can_read_thread_;
    std::atomic<bool> keep_reading_;    

    std::shared_ptr<BLDCMotor> leftMotor;
    std::shared_ptr<BLDCMotor> rightMotor;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    
    // rclcpp::Publisher<your_custom_msg::msg::MotorStats>::SharedPtr stats_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool initCanSocket() {
        struct sockaddr_can addr;
        struct ifreq ifr;

        if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Socket");
            return false;
        }

        strcpy(ifr.ifr_name, "can0");
        ioctl(can_socket_, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Bind");
            return false;
        }

        return true;
    }

    void readCanMessages() {
        struct can_frame frame;
        while (keep_reading_) {
            int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                perror("CAN read");
                continue;
            }
            if (nbytes == sizeof(struct can_frame)) {
                // Process the received frame
                onCanMessageReceived(frame);
            }
        }
    }

void onCanMessageReceived(const struct can_frame &frame) {
    // Log the CAN ID and DLC (Data Length Code)
    //RCLCPP_INFO(this->get_logger(), "Received CAN message with ID: 0x%X, DLC: %d", frame.can_id, frame.can_dlc);
    bool printDebug = false;

    if(printDebug){
    // Prepare strings for hex and int values
    std::stringstream hexStream;
    std::stringstream intStream;

    hexStream << "Hex: ";
    intStream << "Int: ";

    // Iterate through the data bytes of the frame
    for (int i = 0; i < frame.can_dlc; ++i) {
        hexStream << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << " ";
        intStream << std::dec << std::setw(3) << std::setfill(' ') << static_cast<int>(frame.data[i]) << " ";
    }

    // Log the hex and int values
    RCLCPP_INFO(this->get_logger(), "%s", hexStream.str().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", intStream.str().c_str());
    }

    if(frame.can_id == 0x141)// right motor, come back and fix this when I figure out whats causing the fault
    {
         leftMotor->parseResponse(frame.data);
    }
    if(frame.can_id == 0x142) // right motor, come back and fix this when I figure out whats causing the fault
    {
        rightMotor->parseResponse(frame.data); 
    }
    
}

void calculateSpeedError() {
    if(isFirstLoop)
    {
        leftSpeedError = 0;
        rightSpeedError = 0;
        return;
    }
    // Get the elapsed time since the last loop iteration
    const rclcpp::Time now = this->get_clock()->now();
    const auto elapsedTime = now - lastLoopTimestamp_;
    const double dt = elapsedTime.seconds(); // Use elapsedTime directly


    //RCLCPP_INFO(this->get_logger(), "Current Delta %f", dt);
    // Calculate the commanded distance each wheel should have moved in the elapsed time
    double leftCommandedDistance = leftSpeedSetpoint * dt;
    double rightCommandedDistance = rightSpeedSetpoint * dt;

    // Calculate the distance moved by each wheel based on their encoder ticks
    int32_t leftdelta = leftMotor->calculateDelta();
    int32_t rightdelta = -1 * rightMotor->calculateDelta(); // flip this one to be the opposite
    double leftDistanceMovedMm = leftdelta * distancePerTickMm;
    double rightDistanceMovedMm = rightdelta * distancePerTickMm;

    // Convert mm to meters for calculation
    double leftDistanceMoved = leftDistanceMovedMm / 1000.0;
    double rightDistanceMoved = rightDistanceMovedMm / 1000.0;

    // Calculate the percentage error for each motor for distance moved
    double leftDistanceError = (leftDistanceMoved - leftCommandedDistance) / leftCommandedDistance * 100;
    double rightDistanceError = (rightDistanceMoved - rightCommandedDistance) / rightCommandedDistance * 100;

    //RCLCPP_INFO(this->get_logger(), "Current Delta %f", dt);

    // Convert the distance error to a velocity error percentage
    leftSpeedError = leftDistanceError / dt;
    rightSpeedError = rightDistanceError / dt;

    RCLCPP_INFO(this->get_logger(), "Current Speed Error Left %f", leftSpeedError);
    RCLCPP_INFO(this->get_logger(), "Current Speed Error Right %f", rightSpeedError);


    // Calculate the percentage error for each motor for distance moved
    double leftDistanceErrorPercent = (leftDistanceMoved - leftCommandedDistance) / leftSpeedSetpoint * 100;
    double rightDistanceErrorPercent = (rightDistanceMoved - rightCommandedDistance) / rightSpeedSetpoint * 100;

    // Optionally, you can also scale the error by a factor before publishing it as feedback
    leftSpeedError = leftDistanceErrorPercent;// * errorScaleFactor;
    rightSpeedError = rightDistanceErrorPercent;// * errorScaleFactor;
    RCLCPP_INFO(this->get_logger(), "Current Speed Error Left %f", leftSpeedError);
    RCLCPP_INFO(this->get_logger(), "Current Speed Error Right %f", rightSpeedError);

    lastLoopTimestamp_ = now;
}

   void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Calculate RPM from setpoint observation
    double rotationsPerSecondObservation = 1 / 28.6; // One rotation per 28.6 seconds
    double rpmObservation = rotationsPerSecondObservation * 60; // Convert to RPM

    // Since 1200 setpoint gives rpmObservation, find conversion factor
    double setpointPerRPM = 1200 / rpmObservation;

    // Assuming msg->linear.x is in m/s and converting it to RPM
    // First, calculate the wheel circumference
    constexpr double wheelDiameterMm = 183.0; // Wheel diameter in mm
    constexpr double wheelCircumferenceM = (wheelDiameterMm / 1000.0) * M_PI; // Convert mm to meters and calculate circumference

    // Convert m/s to RPM
    double linearSpeedRPM = (msg->linear.x / wheelCircumferenceM) * 60;

    // Adjust for robot's differential drive
    double leftSpeedRPM = linearSpeedRPM - msg->angular.z;
    double rightSpeedRPM = -(linearSpeedRPM + msg->angular.z);

    // Convert RPM to setpoint
    leftSpeedSetpoint = leftSpeedRPM * setpointPerRPM;
    rightSpeedSetpoint = rightSpeedRPM * setpointPerRPM;

    // leftMotor->setSpeed(leftSpeedSetpoint);
    // rightMotor->setSpeed(rightSpeedSetpoint);

    RCLCPP_INFO(this->get_logger(), "Set left motor speed setpoint: %f", leftSpeedSetpoint);
    RCLCPP_INFO(this->get_logger(), "Set right motor speed setpoint: %f", rightSpeedSetpoint);
}

     void loop_callback() {
            if (isFirstLoop) {
           lastLoopTimestamp_ = this->get_clock()->now();
        isFirstLoop = false;
        }
        
        
        uint8_t leftSerializedData[8]; // Ensure the buffer is appropriately sized
        uint8_t rightSerializedData[8];
        
        // Assuming readEncoderValuesCommand() returns a ReadEncoderCommand instance
        auto leftEncoderCommand = leftMotor->readEncoderValuesCommand();
        auto rightEncoderCommand = rightMotor->readEncoderValuesCommand();

        // Populate local buffers with serialized data
        leftEncoderCommand.serialize(leftSerializedData);
        rightEncoderCommand.serialize(rightSerializedData);

        // Send the serialized data over CAN // to do come back and fix this, so no segmentation fault on get ID 
        sendCanMessage(0x141, leftSerializedData, sizeof(leftSerializedData));
        sendCanMessage(0x142, rightSerializedData, sizeof(rightSerializedData));

        calculateSpeedError();
        double errorLeftCorrectedSpeedSetPoint = 0;
        double errorRightCorrectedSpeedSetPoint = 0;
        
        // Disabling this for the moment since it adds no value (other thank taking off when it shouldnt)
        if(leftSpeedError != NAN)
        {
            
            errorLeftCorrectedSpeedSetPoint = leftSpeedSetpoint ;//* std::abs(leftSpeedError);
        }
        else
        {
            errorLeftCorrectedSpeedSetPoint = leftSpeedSetpoint;
        }

        
        if(leftSpeedError != NAN)
        {
            errorRightCorrectedSpeedSetPoint = rightSpeedSetpoint ;//* std::abs(rightSpeedError);
        }
        else
        {
            errorRightCorrectedSpeedSetPoint = rightSpeedSetpoint;
        }
        //leftSpeedSetpoint * leftSpeedError;
        //rightSpeedSetpoint;

        // RCLCPP_INFO(this->get_logger(), "Current Speed Error Left %f, commanded speed %f ", leftSpeedError, leftSpeedSetpoint);
        // RCLCPP_INFO(this->get_logger(), "Current Speed Error Right %f, commanded speed %f ", rightSpeedError, rightSpeedSetpoint);

         

        auto leftspeedCommand =  leftMotor->createSpeedClosedLoopControlCommand(leftMotor->getMotorId(),errorLeftCorrectedSpeedSetPoint );
        uint8_t leftspeedmessageData[8];
        leftspeedCommand.serialize(leftspeedmessageData);        
        sendCanMessage(leftMotor->getMotorCANId(), leftspeedmessageData, sizeof(leftspeedmessageData));

        auto rightspeedCommand =  rightMotor->createSpeedClosedLoopControlCommand(rightMotor->getMotorId(),errorRightCorrectedSpeedSetPoint);
        uint8_t rightspeedmessageData[8];
        rightspeedCommand.serialize(rightspeedmessageData);        
        sendCanMessage(rightMotor->getMotorCANId(), rightspeedmessageData, sizeof(rightspeedmessageData));

        // Calculate distance based on delta
        int32_t leftdelta = leftMotor->calculateDelta();
        int32_t rightdelta = -1 * rightMotor->calculateDelta(); // flip this one to be the opposite
        bool encoderDebug = true;
        
        double leftDistanceMovedMm = leftdelta * distancePerTickMm;
        double rightDistanceMovedMm = rightdelta * distancePerTickMm;
       

        if (!leftMotor && !rightMotor){
             std::cerr << "Motors are uninitialized or nullptr!" << std::endl;             
        }
        else{        

        }
         // Convert mm to meters for calculation
        double leftDistanceMoved = leftDistanceMovedMm / 1000.0;
        double rightDistanceMoved = rightDistanceMovedMm / 1000.0;

        // Calculate the change in orientation
        double dTheta = (rightDistanceMoved - leftDistanceMoved) / wheelbase;
        
        if(encoderDebug){
            if(leftdelta != 0 || rightdelta != 0){
                RCLCPP_INFO(this->get_logger(), "Robot encoder Delta - LA: %u, Ld: %i, LDM: %f, RA: %u, rD: %i, RDM: %f, t: %f", leftMotor->getEncoderPosition(), leftdelta, rightMotor->getEncoderPosition(), rightdelta,leftDistanceMoved, rightDistanceMoved, dTheta);
            }
            }
        
        // Calculate the average distance moved
        //double dDistance = (leftDistanceMoved + rightDistanceMoved) / 2.0;
        double dDistance = (rightDistanceMoved + leftDistanceMoved) / 2.0;

        // Improved pose calculation
        if (dTheta != 0) {
            double radius = dDistance / dTheta;
            double centerX = x - radius * sin(theta);
            double centerY = y + radius * cos(theta);
            x = centerX + radius * sin(theta + dTheta);
            y = centerY - radius * cos(theta + dTheta);
        } else {
            // If dTheta is zero, the robot is moving straight
            x += dDistance * cos(theta);
            y += dDistance * sin(theta);
        }
        theta += dTheta;

        // Normalize theta to the range [-pi, pi)
        theta = atan2(sin(theta), cos(theta));
        
        // TODO: Calculate and publish odometry data

        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta); // Roll, Pitch, Yaw
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        if (!tfBroadcaster_) {
            tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
        }
        //tfBroadcaster_.sendTransform(transformStamped);
        tfBroadcaster_->sendTransform(transformStamped);
        //.sendTransform(transformStamped);
        // TODO: Publish other motor statistics

    }
    
    void sendCanMessage(uint32_t can_id, uint8_t* data, size_t size) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame)); // Clear the frame structure
    frame.can_id = can_id;
    frame.can_dlc = size;
    memcpy(frame.data, data, size);

    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write CAN message failed");
        // Handle error
    }
}


};

int main(int argc, char **argv) {   
    
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialDriveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

