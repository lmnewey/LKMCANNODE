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

// Enum to represent different motor types
enum class MotorType {
    MF,
    MH,
    MG,
    MS
};
// Define a class to represent a BLDC motor
class BLDCMotor {

private:
    MotorType motorType_; // Type of the motor
    uint8_t motorId_; // Unique ID for the motor
   
    int8_t temperature_  =0;   // Motor temperature (1℃/LSB)
    int16_t motorPower_  =0;   // Motor power (-850~850 or -2048~2048 depending on the command)
    int16_t motorSpeed_ =0;   // Motor speed (1dps/LSB)
    uint16_t encoderPosition_ =0; // Encoder position (14-bit, 15-bit, or 18-bit depending on the command)
    int16_t torqueCurrent_ =0;

    double anglePidKp_;
    double anglePidKi_;
    double speedPidKp_;
    double speedPidKi_;
    double iqPidKp_;
    double iqPidKi_;

    int32_t acceleration_;

    // Raw Position Data
    uint16_t encoderRaw_;
    uint16_t encoderOffset_;

    int64_t motorAngle_;
    uint32_t cicleAngle_;

    
    uint16_t voltage_  =0 ;  // Assuming little-endian architecture
    int8_t errorState_; // will be parsed to errors below

    bool isVoltageNormal_;// = (errorState & 0x01) != 0;
    bool isUnderVoltageProtect_;// = (errorState & 0x02) != 0;
    bool isTemperatureNormal_;// = (errorState & 0x04) != 0;
    bool isOverTemperatureProtect_;// = (errorState & 0x08) != 0;

    int16_t torque_ =0;
    int16_t speed_ =0;
    uint16_t encoder_ =0;

    // Per Phase Current 
    int16_t iA_ =0;
    int16_t iB_ =0;
    int16_t iC_ =0;

    // startup boolean, for calculating the delta tick change
    bool initialTicksSet;

    uint16_t deltaTicks = 0;

    // ticks for encoders
    uint16_t LastTicks = 0;
public:
// Define a struct to represent the Motor Off command
struct MotorOffCommand {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes

        uint8_t commandByte = 0x80;    // Data field: 8 bytes

            void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        // Set the encoderOffset bytes
        buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
        
    }
    // Constructor to initialize the struct
    MotorOffCommand(uint8_t motorId) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Motor Off command format
        data[0] = 0x80;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
    }
};

// Define a struct to represent the Motor On command
struct MotorOnCommand {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0x88;    // Data field: 8 bytes

    void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        // Set the encoderOffset bytes
        buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
        
    }
    // Constructor to initialize the struct
    MotorOnCommand(uint8_t motorId) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Motor On command format
        data[0] = 0x88;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
    }
};


// Define a struct to represent the Motor Stop command
struct MotorStopCommand {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0x81;    // Data field: 8 bytes

    void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        // Set the encoderOffset bytes
        buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
        
    }
    // Constructor to initialize the struct
    MotorStopCommand(uint8_t motorId) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Motor Stop command format
        data[0] = 0x81;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
    }
};

// Define a struct to represent the Open Loop Control command - command can only be applied to MS series motor
struct OpenLoopControlCommand {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0xA0;
        void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
        
    }

    // Constructor to initialize the struct
    OpenLoopControlCommand(int16_t powerControl) {
        // Set the data fields as per the Open Loop Control command format
        identifier = 0xA0;
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = static_cast<uint8_t>(powerControl & 0xFF);         // Low byte
        data[4] = static_cast<uint8_t>((powerControl >> 8) & 0xFF);  // High byte
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
    }
};

// Define a struct to represent the Torque Closed Loop Control command - command can only be applied to MF,MH,MG series
struct TorqueClosedLoopControlCommand {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0xA1;

    void serialize(uint8_t* buffer) {
        buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
    }

    // Constructor to initialize the struct
    TorqueClosedLoopControlCommand(uint8_t motorId, int16_t torqueControl) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Torque Closed Loop Control command format
        data[0] = 0xA1;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = static_cast<uint8_t>(torqueControl & 0xFF);         // Low byte
        data[5] = static_cast<uint8_t>((torqueControl >> 8) & 0xFF);  // High byte
        data[6] = 0x00;
        data[7] = 0x00;
    }
};

// Define a struct to represent the Speed Closed Loop Control command
struct SpeedClosedLoopControlCommand {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0xA2;
    void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
    }
    // Constructor to initialize the struct
    SpeedClosedLoopControlCommand(uint8_t motorId_, int32_t speedControl) {
        // Set the data fields as per the Speed Closed Loop Control command format
        identifier = 0x141 + motorId_;
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = static_cast<uint8_t>(speedControl & 0xFF);         // Low byte
        data[5] = static_cast<uint8_t>((speedControl >> 8) & 0xFF);  // Middle-low byte
        data[6] = static_cast<uint8_t>((speedControl >> 16) & 0xFF); // Middle-high byte
        data[7] = static_cast<uint8_t>((speedControl >> 24) & 0xFF); // High byte
    }
};

   SpeedClosedLoopControlCommand createSpeedClosedLoopControlCommand(uint8_t motorId_, int32_t speed) {
        return SpeedClosedLoopControlCommand(motorId_,speed);
    }


// Define a struct to represent Multi Loop Angle Control Command 1
struct MultiLoopAngleControl1Command {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0xA3;

        void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
    }
    // Constructor to initialize the struct
    MultiLoopAngleControl1Command(uint8_t motorId, int32_t angleControl, int16_t maxSpeed) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Multi Loop Angle Control Command 1 format
        data[0] = 0xA3;
        data[1] = 0x00; // Placeholder for temperature
        data[2] = static_cast<uint8_t>(maxSpeed & 0xFF);          // Max Speed low byte
        data[3] = static_cast<uint8_t>((maxSpeed >> 8) & 0xFF);   // Max Speed high byte
        data[4] = static_cast<uint8_t>(angleControl & 0xFF);      // Angle Control byte 0
        data[5] = static_cast<uint8_t>((angleControl >> 8) & 0xFF); // Angle Control byte 1
        data[6] = static_cast<uint8_t>((angleControl >> 16) & 0xFF); // Angle Control byte 2
        data[7] = static_cast<uint8_t>((angleControl >> 24) & 0xFF); // Angle Control byte 3
    }
};

// Define a struct to represent Multi Loop Angle Control Command 2
struct MultiLoopAngleControl2Command {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0xA4;
    
    void serialize(uint8_t* buffer) {
     buffer[0] = commandByte;
     buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
    }
    // Constructor to initialize the struct
    MultiLoopAngleControl2Command(uint8_t motorId, int32_t angleControl, uint16_t maxSpeed) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Multi Loop Angle Control Command 2 format
        data[0] = 0xA4;
        data[1] = 0x00; // Placeholder for temperature
        data[2] = static_cast<uint8_t>(maxSpeed & 0xFF);          // Max Speed low byte
        data[3] = static_cast<uint8_t>((maxSpeed >> 8) & 0xFF);   // Max Speed high byte
        data[4] = static_cast<uint8_t>(angleControl & 0xFF);      // Angle Control byte 0
        data[5] = static_cast<uint8_t>((angleControl >> 8) & 0xFF); // Angle Control byte 1
        data[6] = static_cast<uint8_t>((angleControl >> 16) & 0xFF); // Angle Control byte 2
        data[7] = static_cast<uint8_t>((angleControl >> 24) & 0xFF); // Angle Control byte 3
    }
};

// Define a struct to represent the Single Loop Angle Control 1 command
struct SingleLoopAngleControl1Command {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
        uint8_t commandByte = 0xA5;
    void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;        
        buffer[1] = data[1] ;
        buffer[2] = data[2];
        buffer[3] = data[3];
        buffer[4] = data[4];
        buffer[5] = data[5];
        buffer[6] = data[6];
        buffer[7] = data[7];
        
    }
    
    // Constructor to initialize the struct
    SingleLoopAngleControl1Command(uint8_t motorId, uint8_t spinDirection, uint32_t angleControl) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Single Loop Angle Control 1 command format
        data[0] = 0xA5;
        data[1] = spinDirection;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = static_cast<uint8_t>(angleControl & 0xFF);         // Byte 0
        data[5] = static_cast<uint8_t>((angleControl >> 8) & 0xFF);  // Byte 1
        data[6] = static_cast<uint8_t>((angleControl >> 16) & 0xFF); // Byte 2
        data[7] = static_cast<uint8_t>((angleControl >> 24) & 0xFF); // Byte 3
    }
};

// Define a struct to represent the Single Loop Angle Control 2 command
struct SingleLoopAngleControl2Command {     // not fully implemented do not use. Need to do the encoder offset

    uint8_t encoderOffset = 0;
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0xA6;
        void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        // Set the encoderOffset bytes
        buffer[6] = static_cast<uint8_t>(encoderOffset);
        buffer[7] = static_cast<uint8_t>(encoderOffset >> 8);
    }
    // Constructor to initialize the struct
    SingleLoopAngleControl2Command(uint8_t motorId, uint8_t spinDirection, uint32_t angleControl, uint16_t maxSpeed) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Single Loop Angle Control 2 command format
        data[0] = 0xA6;
        data[1] = spinDirection;
        data[2] = static_cast<uint8_t>(maxSpeed & 0xFF);         // Byte 0 (Speed limit low byte)
        data[3] = static_cast<uint8_t>((maxSpeed >> 8) & 0xFF);  // Byte 1 (Speed limit high byte)
        data[4] = static_cast<uint8_t>(angleControl & 0xFF);         // Byte 2 (Angle control 1)
        data[5] = static_cast<uint8_t>((angleControl >> 8) & 0xFF);  // Byte 3 (Angle control 2)
        data[6] = static_cast<uint8_t>((angleControl >> 16) & 0xFF); // Byte 4 (Angle control 3)
        data[7] = static_cast<uint8_t>((angleControl >> 24) & 0xFF); // Byte 5 (Angle control 4)
    }
};

// Define a struct to represent the Increment Angle Control command. Not fully implemented
struct IncrementAngleControlCommand {

    uint8_t encoderOffset = 0; // 
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t data[8];    // Data field: 8 bytes
    uint8_t commandByte = 0xA7;

        void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        // Set the encoderOffset bytes
        buffer[6] = static_cast<uint8_t>(encoderOffset);
        buffer[7] = static_cast<uint8_t>(encoderOffset >> 8);
    }
    // Constructor to initialize the struct
    IncrementAngleControlCommand(uint8_t motorId, int32_t angleIncrement) {
        // Calculate the identifier based on the motor ID (1~32)
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        // Set the data fields as per the Increment Angle Control command format
        data[0] = 0xA7;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = static_cast<uint8_t>(angleIncrement & 0xFF);         // Low byte
        data[5] = static_cast<uint8_t>((angleIncrement >> 8) & 0xFF);  // Middle low byte
        data[6] = static_cast<uint8_t>((angleIncrement >> 16) & 0xFF); // Middle high byte
        data[7] = static_cast<uint8_t>((angleIncrement >> 24) & 0xFF); // High byte
    }
};

// Command message to read PID parameters
struct ReadPIDParametersCommand {
    
    uint8_t commandByte = 0x00; // not implemented
    ReadPIDParametersCommand() : commandByte(0x30) {}

    std::vector<uint8_t> serialize() const {
        std::vector<uint8_t> data;
        data.push_back(commandByte);
        for (int i = 1; i < 8; i++) {
            data.push_back(0x00); // NULL bytes
        }
        return data;
    }
};


// Command message to write PID parameters to RAM // not fully implemented, Just what the script did. DO NOT USE
struct WritePIDParametersToRAMCommand {
    uint8_t commandByte = 0x31;
    double anglePidKp;
    double anglePidKi;
    double speedPidKp;
    double speedPidKi;
    double iqPidKp;
    double iqPidKi;

    WritePIDParametersToRAMCommand(double angleKp, double angleKi, double speedKp, double speedKi, double iqKp, double iqKi)
        : commandByte(0x31), anglePidKp(angleKp), anglePidKi(angleKi), speedPidKp(speedKp), speedPidKi(speedKi), iqPidKp(iqKp), iqPidKi(iqKi) {}

    std::vector<uint8_t> serialize() const {
        std::vector<uint8_t> data;
        data.push_back(commandByte);
        data.push_back(0x00); // NULL byte
        data.push_back(static_cast<uint8_t>(anglePidKp));
        data.push_back(static_cast<uint8_t>(anglePidKi));
        data.push_back(static_cast<uint8_t>(speedPidKp));
        data.push_back(static_cast<uint8_t>(speedPidKi));
        data.push_back(static_cast<uint8_t>(iqPidKp));
        data.push_back(static_cast<uint8_t>(iqPidKi));
        return data;
    }
};

struct WritePIDParametersToROMCommand {
    uint8_t commandByte = 0x32;
    uint16_t anglePidKp;
    uint16_t anglePidKi;
    uint16_t speedPidKp;
    uint16_t speedPidKi;
    uint16_t iqPidKp;
    uint16_t iqPidKi;
};

struct ReadAccelerationCommand {
    uint8_t commandByte = 0x33;
};

// Define a struct for WriteEncoderValueToROM command
struct WriteEncoderValueToROMCommand {
    uint8_t commandByte = 0x91;
    uint16_t encoderOffset;

    // Constructor to initialize the command with encoderOffset value
    WriteEncoderValueToROMCommand(uint16_t offset) : encoderOffset(offset) {}

    // Function to serialize the command data into an array
    void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        // Set the encoderOffset bytes
        buffer[6] = static_cast<uint8_t>(encoderOffset);
        buffer[7] = static_cast<uint8_t>(encoderOffset >> 8);
    }
};

// Define a struct for WriteCurrentPositionToROM command
struct WriteCurrentPositionToROMCommand {
    uint8_t commandByte = 0x19;

    // Function to serialize the command data into an array
    void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        // Set the remaining bytes to NULL
        for (int i = 1; i < 8; ++i) {
            buffer[i] = 0x00;
        }
    }
};

// Define an enum to represent different message types
enum class MessageType {
    GeneralStatus,
    TorqueClosedLoopControl,
    SpeedClosedLoopControl,
    MultiLoopAngleControl1,
    MultiLoopAngleControl2,
    SingleLoopAngleControl1,
    SingleLoopAngleControl2,
    IncrementAngleControl1,
    IncrementAngleControl2,
    ReadPIDParameters,
    WritePIDParametersToRAM,
    WritePIDParametersToROM,
    ReadAcceleration,
    WriteAccelerationToRAM,
    ReadEncoder,
    WriteEncoderValueToROM, // New message type
    WriteCurrentPositionToROM, // New message type
    ReadMultiAngleLoopCommand,
    ReadSingleAngleLoopCommand,
    ClearMotorAngleLoopCommand,
    ReadMotorState1AndErrorCommand,
    ClearMotorState1Error,
    ReadMotorState2Command,
    ReadMotorState3Command,
    Unknown,
    // Add more message types as needed
};

// Define a struct for WriteAccelerationToRAM command
struct WriteAccelerationToRAMCommand {
    uint8_t commandByte = 0x34;
    uint32_t acceleration;

    // Constructor to initialize the command with acceleration value
    WriteAccelerationToRAMCommand(uint32_t accel) : acceleration(accel) {}

    // Function to serialize the command data into an array
    void serialize(uint8_t* buffer) {
        buffer[0] = commandByte;
        // Set the acceleration bytes
        buffer[4] = static_cast<uint8_t>(acceleration);
        buffer[5] = static_cast<uint8_t>(acceleration >> 8);
        buffer[6] = static_cast<uint8_t>(acceleration >> 16);
        buffer[7] = static_cast<uint8_t>(acceleration >> 24);
    }
};

int32_t calculateDelta() {
    uint16_t currentTicks = encoderPosition_;
    if (!initialTicksSet) {
        LastTicks = currentTicks;
        initialTicksSet = true;
        return 0; // No delta on the first read
    }

    // Calculate the raw delta considering the unsigned nature of encoder values
    int32_t rawDelta = static_cast<int32_t>(currentTicks) - static_cast<int32_t>(LastTicks);

    // Correct the delta for underflow or overflow
    if (rawDelta < -32768) {
        // Underflow correction
        rawDelta += 65536;
    } else if (rawDelta > 32767) {
        // Overflow correction
        rawDelta -= 65536;
    }

    // Update LastTicks for the next calculation
    LastTicks = currentTicks;

    // Return the signed delta
    return rawDelta;
}
//  // Call this method with the current encoder ticks
//  uint16_t calculateDelta() {
//         uint16_t currentTicks = encoderPosition_;//encoderRaw_;
//         if (!initialTicksSet) {
//             // First time receiving ticks, set LastTicks and mark as set            
//             LastTicks = currentTicks;
//             initialTicksSet = true;
//             return 0; // Delta is 0 since this is the first measurement
//         } else {
//             // Calculate the raw delta difference, taking potential overflow into account
//             uint16_t delta;
            
//             if(currentTicks >= LastTicks) {
//                 // No overflow, or overflow in positive direction
//                 delta = currentTicks - LastTicks;
//             } else {
//                 // Overflow occurred in negative direction
//                 delta = static_cast<uint16_t>((65536 + currentTicks) - LastTicks);
//             }

//             // Update LastTicks for the next calculation
//             LastTicks = currentTicks;
//             deltaTicks = delta;
//             return delta;
//         }
//     }
// Define a struct for ReadEncoder command
struct ReadEncoderCommand {
    uint8_t commandByte = 0x90; // Command byte
    uint8_t data[7]; // Adjusted to fit within an 8-byte limit including the command byte

    // Corrected constructor to initialize data array
    ReadEncoderCommand(uint8_t initialDataValue = 0) {
        std::fill(std::begin(data), std::end(data), initialDataValue);
    }

    // Serialize the command data into the provided buffer, respecting the 8-byte limit
    void serialize(uint8_t* buffer) const {
        buffer[0] = commandByte; // Place the command byte at the beginning
        memcpy(buffer + 1, data, sizeof(data)); // Copy the 7 bytes of data following the command byte
    }
};

// Command struct for Read Multi Angle Loop command
struct ReadMultiAngleLoopCommand {
    uint8_t commandByte = 0x92;
    uint8_t data[8];
    // Constructor
    ReadMultiAngleLoopCommand() = default;

    // Function to serialize the command into a data buffer
    void serialize(uint8_t* buffer) const {
        buffer[0] = commandByte;
        // Fill remaining bytes with NULL (0x00)
        for (int i = 1; i < 8; ++i) {
            buffer[i] = 0x00;
        }
    }
};

// Command struct for Read Single Angle Loop command
struct ReadSingleAngleLoopCommand {
    uint8_t commandByte = 0x94;
    uint8_t data[8];
    // Constructor
    ReadSingleAngleLoopCommand() = default;

    // Function to serialize the command into a data buffer
    void serialize(uint8_t* buffer) const {
        buffer[0] = commandByte;
        // Fill remaining bytes with NULL (0x00)
        for (int i = 1; i < 8; ++i) {
            buffer[i] = 0x00;
        }
    }
};

// Command struct for Clear Motor Angle Loop command
struct ClearMotorAngleLoopCommand {
    uint8_t commandByte = 0x95;
    uint8_t data[8];
    // Constructor
    ClearMotorAngleLoopCommand() = default;

    // Function to serialize the command into a data buffer
    void serialize(uint8_t* buffer) const {
        buffer[0] = commandByte;
        // Fill remaining bytes with NULL (0x00)
        for (int i = 1; i < 8; ++i) {
            buffer[i] = 0x00;
        }
    }
};

// Command struct for Read Motor State 1 and Error State command
struct ReadMotorState1AndErrorStateCommand {
    uint8_t commandByte = 0x9A;
    uint8_t data[8];
    // Constructor
    ReadMotorState1AndErrorStateCommand() = default;

    // Function to serialize the command into a data buffer
    void serialize(uint8_t* buffer) const {
        buffer[0] = commandByte;
        // Fill remaining bytes with NULL (0x00)
        for (int i = 1; i < 8; ++i) {
            buffer[i] = 0x00;
        }
    }
};

struct ClearErrorStateCommand {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t commandByte = 0x9A; 
    uint8_t data[8];
        
        ClearErrorStateCommand(uint8_t motorId){
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }

        data[0] = 0x81;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;// Fill with NULL bytes
        }
};

struct ReadMotorState2Command {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t commandByte = 0x9C;
    uint8_t data[8];

    ReadMotorState2Command(uint8_t motorId){
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }
        data[0] = 0x81;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;  // Fill with NULL bytes
    }
};

// Define a command structure for the "Read motor state 3" command
struct ReadMotorState3Command {
    uint32_t identifier; // Identifier: 0x140 + ID (1~32)
    uint8_t commandByte = 0x9D;
    uint8_t data[8];

    ReadMotorState3Command(uint8_t motorId){
        if (motorId >= 1 && motorId <= 32) {
            identifier = 0x140 + motorId;
        } else {
            // Handle an invalid motor ID (out of range)
            // You can set a default value or raise an error here
            identifier = 0x140; // Default to motor ID 1
        }
        data[0] = 0x81;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;  // Fill with NULL bytes
    }
};

// I extraced the description of the command messages and their responses to text and ran a script against it to generate the parser, But its resulted in some duplicates.. They will
// need to be removed potentially to neaten up the code. but will be left in for the moment as that wont hurt anything

// Function to parse a General Status message response and update member variables
void parseGeneralStatusResponse(const uint8_t* response) {    
    setTemperature(static_cast<int8_t>(response[1]));
    setMotorPower(static_cast<int16_t>(response[2] | (response[3] << 8)));
    setMotorSpeed(static_cast<int16_t>(response[4] | (response[5] << 8)));
    setEncoderPosition(static_cast<uint16_t>(response[6] | (response[7] << 8)));
}

// Function to parse a Torque Closed Loop Control message response and update member variables
void parseTorqueClosedLoopControlResponse(const uint8_t* response) {
    setTemperature(static_cast<int8_t>(response[1]));    
    setMotorPower(static_cast<int16_t>(response[2] | (response[3] << 8)));    
    setMotorSpeed(static_cast<int16_t>(response[4] | (response[5] << 8)));    
    setEncoderPosition(static_cast<uint16_t>(response[6] | (response[7] << 8)));
}

// Function to parse a Speed Closed Loop Control message response
void parseSpeedClosedLoopControlResponse(const uint8_t* response) {
    setTemperature(static_cast<int8_t>(response[1]));    
    setMotorSpeed(static_cast<int32_t>(response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24)));
    setMotorSpeed(static_cast<int16_t>(response[2] | (response[3] << 8)));
    setEncoderPosition(static_cast<uint16_t>(response[8] | (response[9] << 8)));
}

// Function to parse a Multi Loop Angle Control 1 response
void parseMultiLoopAngleControl1Response(const uint8_t* response) {
    setTemperature(static_cast<int8_t>(response[1]));
    setMotorSpeed(static_cast<int16_t>(response[2] | (response[3] << 8)));
    setEncoderPosition(static_cast<uint16_t>(response[8] | (response[9] << 8)));
}

// Function to parse a Multi Loop Angle Control 2 response
void parseMultiLoopAngleControl2Response(const uint8_t* response) {
    setTemperature(static_cast<int8_t>(response[1]));
    setTorqueCurrent(static_cast<int16_t>(response[2] | (response[3] << 8)));
    setMotorSpeed(static_cast<int16_t>(response[4] | (response[5] << 8)));
    setEncoderPosition(static_cast<uint16_t>(response[10] | (response[11] << 8)));
}

// Function to parse the Single Loop Angle Control 1 command response
void parseSingleLoopAngleControl1Response(const uint8_t* response) {    
    setTemperature(static_cast<int8_t>(response[1]));
    setTorqueCurrent(static_cast<int16_t>(response[2] | (response[3] << 8)));
    setMotorSpeed(static_cast<int16_t>(response[4] | (response[5] << 8)));
    setEncoderPosition(static_cast<uint16_t>(response[6] | (response[7] << 8)));
}

// Function to parse the Single Loop Angle Control 2 command response
void parseSingleLoopAngleControl2Response(const uint8_t* response) {    
    setTemperature(static_cast<int8_t>(response[1]));
    setTorqueCurrent(static_cast<int16_t>(response[2] | (response[3] << 8)));
    setMotorSpeed(static_cast<int16_t>(response[4] | (response[5] << 8)));
    setEncoderPosition(static_cast<uint16_t>(response[6] | (response[7] << 8)));
    
}

// Function to parse an Increment Angle Control response
void parseIncrementAngleControlResponse(const uint8_t* response) {
    
    setTemperature(static_cast<int8_t>(response[1]));    
    setTorqueCurrent(static_cast<int16_t>(response[2] | (response[3] << 8)));
    setMotorSpeed(static_cast<int16_t>(response[4] | (response[5] << 8)));
    setEncoderPosition(static_cast<uint16_t>(response[6] | (response[7] << 8)));
}

void parseWritePIDParametersToROMResponse(const uint8_t* response){ //} uint16_t& anglePidKp, uint16_t& anglePidKi, uint16_t& speedPidKp, uint16_t& speedPidKi, uint16_t& iqPidKp, uint16_t& iqPidKi) {
    // Parse the response based on the data field structure
    // anglePidKp = static_cast<uint16_t>(response[2] | (response[3] << 8));
    // anglePidKi = static_cast<uint16_t>(response[4] | (response[5] << 8));
    // speedPidKp = static_cast<uint16_t>(response[6] | (response[7] << 8));
    // speedPidKi = static_cast<uint16_t>(response[8] | (response[9] << 8));
    // iqPidKp = static_cast<uint16_t>(response[10] | (response[11] << 8));
    // iqPidKi = static_cast<uint16_t>(response[12] | (response[13] << 8));
}

// Parser for the response of the "Read PID parameter" message
void parseReadPIDParametersResponse(const uint8_t* response) {
    // motor.anglePidKp_ = static_cast<double>(response[2]);
    // motor.anglePidKi_ = static_cast<double>(response[3]);
    // speedPidKp_ = static_cast<double>(response[4]);
    // speedPidKi_ = static_cast<double>(response[5]);
    // motor.iqPidKp_ = static_cast<double>(response[6]);
    // motor.iqPidKi_ = static_cast<double>(response[7]);

    setAnglePidKp(static_cast<double>(response[2]));
    setAnglePidKi(static_cast<double>(response[3]));
    
    
    setSpeedPidKp(static_cast<double>(response[4]));
    setSpeedPidKi(static_cast<double>(response[5]));
    
    setIqPidKp(static_cast<double>(response[6]));
    setIqPidKi(static_cast<double>(response[7]));
}

void parseReadAccelerationResponse(const uint8_t* response) {
    // Parse the response based on the data field structure
    //motor.acceleration_ = static_cast<int32_t>(response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24));
    setAcceleration(static_cast<int32_t>(response[4] | (response[5] << 8) | (response[6] << 16) | (response[7] << 24)));
}

// Function to parse WriteAccelerationToRAM response
void parseWriteAccelerationToRAMResponse(const uint8_t* response) {
    // Check if the response length is valid
    if (response[0] == 0x34 && response[1] == 0x00 && response[2] == 0x00 && response[3] == 0x00) {
        // Extract acceleration from the response bytes
        acceleration_ = static_cast<uint32_t>(response[4]) |
                      (static_cast<uint32_t>(response[5]) << 8) |
                      (static_cast<uint32_t>(response[6]) << 16) |
                      (static_cast<uint32_t>(response[7]) << 24);
    } else {
        // Handle invalid response
        acceleration_ = 0;
    }
}

// Function to parse ReadEncoder response
void parseReadEncoderResponse(const uint8_t* response){ //} uint16_t& encoder, uint16_t& encoderRaw, uint16_t& encoderOffset) {
    // Check if the response length is valid
    
    
        // Extract encoder values from the response bytes
        //motor.encoderPosition_ = static_cast<uint16_t>(response[2]) | (static_cast<uint16_t>(response[3]) << 8);
        setEncoderPosition(static_cast<uint16_t>(response[2]) | (static_cast<uint16_t>(response[3]) << 8));
        
        //motor.encoderRaw_ = static_cast<uint16_t>(response[4]) | (static_cast<uint16_t>(response[5]) << 8);
        setEncoderRaw(static_cast<uint16_t>(response[4]) | (static_cast<uint16_t>(response[5]) << 8)); 
        
        //motor.encoderOffset_ = static_cast<uint16_t>(response[6]) | (static_cast<uint16_t>(response[7]) << 8);
        setEncoderOffset(static_cast<uint16_t>(response[6]) | (static_cast<uint16_t>(response[7]) << 8));

}

// Function to parse WriteEncoderValueToROM response
void parseWriteEncoderValueToROMResponse(const uint8_t* response, uint16_t& encoderOffset) {
    // // Check if the response length is valid
    // if (response[0] == 0x91 && response[1] == 0x00) {
    //     // Extract encoderOffset from the response bytes
    //     encoderOffset = static_cast<uint16_t>(response[6]) | (static_cast<uint16_t>(response[7]) << 8);
    // } else {
    //     // Handle invalid response
    //     encoderOffset = 0;
    // }

}

// Function to parse WriteCurrentPositionToROM response
void parseWriteCurrentPositionToROMResponse(const uint8_t* response, uint16_t& encoderOffset) {
    // // Check if the response length is valid
    // if (response[0] == 0x19 && response[1] == 0x00) {
    //     // Extract encoderOffset from the response bytes
    //     encoderOffset = static_cast<uint16_t>(response[6]) | (static_cast<uint16_t>(response[7]) << 8);
    // } else {
    //     // Handle invalid response
    //     encoderOffset = 0;
    // }
}

// Function to parse the response for Read Multi Angle Loop command
void parseReadMultiAngleLoopResponse(const uint8_t* response) {
    // Extract the motorAngle from the response bytes
    //motor.motorAngle_ = *(int64_t*)(response + 1);  // Assuming little-endian architecture
    setMotorAngle(*(int64_t*)(response + 1));
}

// Function to parse the response for Read Single Angle Loop command
void parseReadSingleAngleLoopResponse(const uint8_t* response) {
    // Extract the circleAngle from the response bytes
    //motor.circleAngle_ = *((uint32_t*)(response + 4));  // Assuming little-endian architecture
    setCicleAngle(*((uint32_t*)(response + 4)));
}


// Function to parse the response for Read Motor State 1 and Error State command // the documentation states there is a differnce in the response message but it only appears to be the command byte, Since i scripted the creation of this, there is now two. I will look at getting rid of one of them
void parseReadMotorState1AndErrorStateResponse(const uint8_t* response) {
    // Extract temperature, voltage, and errorState from the response bytes
    // motor.temperature = static_cast<int8_t>(response[1]);
    // motor.voltage = *(uint16_t*)(response + 3);  // Assuming little-endian architecture
    // motor.errorState = response[7];
    // motor.isVoltageNormal_ = (motor.errorState & 0x01) != 0;
    // motor.isUnderVoltageProtect_ = (errorState & 0x02) != 0;
    // motor.isTemperatureNormal_ = (errorState & 0x04) != 0;
    // motor.isOverTemperatureProtect_ = (errorState & 0x08) != 0;
    setTemperature(static_cast<int8_t>(response[1]));
    setVoltage(*(uint16_t*)(response + 3) );
    setErrorState(response[7]);
}

// Function to parse the response of the "Clear motor error state" command
void parseClearErrorStateResponse(const uint8_t* response) {
    // Check if the response has the correct command byte (0x9A)
    // motor.temperature = static_cast<int8_t>(response[1]);
    // motor.voltage = *(uint16_t*)(response + 3);  // Assuming little-endian architecture
    // motor.errorState = response[7];

    // motor.isVoltageNormal_ = (motor.errorState & 0x01) != 0;
    // motor.isUnderVoltageProtect_ = (motor.errorState & 0x02) != 0;
    // motor.isTemperatureNormal_ = (motor.errorState & 0x04) != 0;
    // motor.isOverTemperatureProtect_ = (motor.errorState & 0x08) != 0;
    setTemperature(static_cast<int8_t>(response[1]));
    setVoltage(*(uint16_t*)(response + 3) );
    setErrorState(response[7]);

}

// Function to parse the response of the "Read motor state 2" command
void parseReadMotorState2Response(const uint8_t* response) {
    // Check if the response has the correct command byte (0x9C)
    if (response[0] == 0x9C) {
        // Parse the data fields
        //motor.temperature_ = static_cast<int8_t>(response[1]);
        
        setTemperature(static_cast<int8_t>(response[1]));
        //motor.torque_ = static_cast<int16_t>(response[2] | (response[3] << 8));
        setTorque(static_cast<int16_t>(response[2] | (response[3] << 8)));
        //speed_ = static_cast<int16_t>(response[4] | (response[5] << 8));
        setSpeed(static_cast<int16_t>(response[4] | (response[5] << 8)));
        //motor.encoder_ = static_cast<uint16_t>(response[6] | (response[7] << 8));
        setEncoder(static_cast<uint16_t>(response[6] | (response[7] << 8)));
        // You can handle the data as needed here
    }
}


// Function to parse the response of the "Read motor state 3" command
void parseReadMotorState3Response(const uint8_t* response) {
    // Check if the response has the correct command byte (0x9D)
    if (response[0] == 0x9D) {
        // Parse the data fields
        
        iA_ = static_cast<int16_t>(response[2] | (response[3] << 8));
        iB_ = static_cast<int16_t>(response[4] | (response[5] << 8));
        iC_ = static_cast<int16_t>(response[6] | (response[7] << 8));
        this->temperature_ = static_cast<int8_t>(response[1]);
        // You can handle the data as needed here
    }
}

// Function to parse the response based on message type
//MessageType parseResponse(const uint8_t* response) { //, MessageType messageType, BLDCMotor& motor
void parseResponse(const uint8_t* response) { //, MessageType messageType, BLDCMotor& motor
    switch (response[0]) {
        case 0xA0:
            this->parseGeneralStatusResponse(response);
            return;
            //return MessageType::GeneralStatus;
        case 0xA1:
            this->parseTorqueClosedLoopControlResponse(response);
            return;
            //            return MessageType::TorqueClosedLoopControl;            
        case 0xA2:
            this->parseTorqueClosedLoopControlResponse(response);
            return;
            //            return MessageType::SpeedClosedLoopControl;
        case 0xA3:
            this->parseMultiLoopAngleControl1Response(response);return;
            //            return MessageType::MultiLoopAngleControl1;
        case 0xA4:
            this->parseMultiLoopAngleControl2Response(response);
            return;
            //            return MessageType::MultiLoopAngleControl2;
        case 0xA5:
            this->parseSingleLoopAngleControl1Response(response);
            return;
            //            return MessageType::SingleLoopAngleControl1;
        case 0xA6:
            this->parseSingleLoopAngleControl2Response(response);
            return;
            //            return MessageType::SingleLoopAngleControl2;
        case 0xA7:
            this->parseIncrementAngleControlResponse(response);
            return;
            //            return MessageType::IncrementAngleControl1;
        case 0xA8:
            this->parseIncrementAngleControlResponse(response); // there should be two of these but im not sure what happened ??
            return;
            //            return MessageType::IncrementAngleControl2;
        case 0x30:
            this->parseReadPIDParametersResponse(response);
            return;
            //            return MessageType::ReadPIDParameters;
        case 0x31:
            // not implemented
            return;
            //            return MessageType::WritePIDParametersToRAM;
        case 0x32:
            this->parseWritePIDParametersToROMResponse(response);
            return;
            //            return MessageType::WritePIDParametersToROM;
        case 0x33:
            this->parseReadAccelerationResponse(response);
            return;
            //            return MessageType::ReadAcceleration;
        case 0x34:
            this->parseWriteAccelerationToRAMResponse(response);
            return;
            //            return MessageType::WriteAccelerationToRAM; // New message type
        case 0x90:
            this->parseReadEncoderResponse(response);
            //parseReadEncoderResponse(response);
            return;
            //            return MessageType::ReadEncoder;
        case 0x92:
            this->parseReadMultiAngleLoopResponse(response);
            return;
            //            return MessageType::ReadMultiAngleLoopCommand;
        case 0x94:
            this->parseReadMultiAngleLoopResponse(response);
            return;
            //            return MessageType::ReadSingleAngleLoopCommand;
        case 0x95:
            // not implemented
            return;
            //            return MessageType::ClearMotorAngleLoopCommand;
        case 0x9A:
            this->parseReadMotorState1AndErrorStateResponse(response);
            return;
            //            return MessageType::ReadMotorState1AndErrorCommand;
        case 0x9C:
            this->parseReadMotorState2Response(response);
            return;
            //            return MessageType::ReadMotorState2Command;
        case 0x9D:
            this->parseReadMotorState3Response(response);
            return;
            //            return MessageType::ReadMotorState3Command;
                
        // Add cases for other message types if needed
        default:
            // Handle unknown or unsupported message types
            break;
    }
}


    // Constructor to initialize the motor with a unique ID
    BLDCMotor(uint8_t motorId) : motorId_(motorId) {}

    // Function to create a Motor Off command for this motor
    MotorOffCommand createMotorOffCommand() {
        // Create a Motor Off command with the motor's ID
        return MotorOffCommand(motorId_);
    }

        // Function to create a Motor On command for this motor
    MotorOnCommand createMotorOnCommand() {
        // Create a Motor On command with the motor's ID
        return MotorOnCommand(motorId_);
    }

    // Function to create a Motor Stop command for this motor
    MotorStopCommand createMotorStopCommand() {
    // Create a Motor Stop command with the motor's ID
    return MotorStopCommand(motorId_);
    }
    // Function to create an Open Loop Control command for this motor
 
    OpenLoopControlCommand createOpenLoopControlCommand(int16_t powerControl) {
    // Create an Open Loop Control command with the motor's ID and powerControl value
    return OpenLoopControlCommand(powerControl);
    }

        // Function to create a Speed Closed Loop Control command for this motor //  shifted this function to close to the source
    // SpeedClosedLoopControlCommand createSpeedClosedLoopControlCommand(int32_t speedControl) {
    //     // Create a Speed Closed Loop Control command with the motor's ID and speed control value
    //     return SpeedClosedLoopControlCommand(speedControl);
    // }
    
        // Function to create Multi Loop Angle Control Command 1 for this motor
    MultiLoopAngleControl1Command createMultiLoopAngleControl1Command(int32_t angleControl, int16_t maxSpeed) {
        // Create a Multi Loop Angle Control Command 1 with the motor's ID, angleControl, and maxSpeed
        return MultiLoopAngleControl1Command(motorId_, angleControl, maxSpeed);
    }

    // Function to create Multi Loop Angle Control Command 2 for this motor
    MultiLoopAngleControl2Command createMultiLoopAngleControl2Command(int32_t angleControl, uint16_t maxSpeed) {
        // Create a Multi Loop Angle Control Command 2 with the motor's ID, angleControl, and maxSpeed
        return MultiLoopAngleControl2Command(motorId_, angleControl, maxSpeed);
    }
    
    // Function to create Single Loop Angle Control Command 1 for this motor
    SingleLoopAngleControl1Command createSingleLoopAngleControl1Command(uint8_t spinDirection, uint32_t angleControl) {
        // Create a Single Loop Angle Control Command 1 with the motor's ID, spinDirection, and angleControl
        return SingleLoopAngleControl1Command(motorId_, spinDirection, angleControl);
    }

    // Function to create Single Loop Angle Control Command 2 for this motor
    SingleLoopAngleControl2Command createSingleLoopAngleControl2Command(uint8_t spinDirection, uint32_t angleControl, uint16_t maxSpeed) {
        // Create a Single Loop Angle Control Command 2 with the motor's ID, spinDirection, angleControl, and maxSpeed
        return SingleLoopAngleControl2Command(motorId_, spinDirection, angleControl, maxSpeed);
    }

    ReadEncoderCommand readEncoderValuesCommand()
    {

        return ReadEncoderCommand();
    }
    
  // Getters - these are for the current values not to communicate with the motor
    MotorType getMotorType() const { return motorType_; }
    uint8_t getMotorId() const { return motorId_; }
    uint32_t getMotorCANId() { return 0x140 + motorId_ ; }
    int8_t getTemperature() const { return temperature_; }
    int16_t getMotorPower() const { return motorPower_; }
    int16_t getMotorSpeed() const { return motorSpeed_; }
    uint16_t getEncoderPosition() const { return encoderPosition_; }
    int16_t getTorqueCurrent() const { return torqueCurrent_; }
    double getAnglePidKp() const { return anglePidKp_; }
    double getAnglePidKi() const { return anglePidKi_; }
    double getSpeedPidKp() const { return speedPidKp_; }
    double getSpeedPidKi() const { return speedPidKi_; }
    double getIqPidKp() const { return iqPidKp_; }
    double getIqPidKi() const { return iqPidKi_; }
    int32_t getAcceleration() const { return acceleration_; }
    uint16_t getEncoderRaw() const { return encoderRaw_; }
    uint16_t getEncoderOffset() const { return encoderOffset_; }
    int64_t getMotorAngle() const { return motorAngle_; }
    uint32_t getCicleAngle() const { return cicleAngle_; }
    uint16_t getVoltage() const { return voltage_; }
    int8_t getErrorState() const { return errorState_; }
    bool isVoltageNormal() const { return isVoltageNormal_; }
    bool isUnderVoltageProtect() const { return isUnderVoltageProtect_; }
    bool isTemperatureNormal() const { return isTemperatureNormal_; }
    bool isOverTemperatureProtect() const { return isOverTemperatureProtect_; }
    int16_t getTorque() const { return torque_; }
    int16_t getSpeed() const { return speed_; }
    uint16_t getEncoder() const { return encoder_; }

    // Setters - these are for the current values not to communicate with the motor
    void setMotorType(MotorType val) { motorType_ = val; }
    void setMotorId(uint8_t val) { motorId_ = val; }
    void setTemperature(int8_t val) { temperature_ = val; }
    void setMotorPower(int16_t val) { motorPower_ = val; }
    void setMotorSpeed(int16_t val) { motorSpeed_ = val; }
    void setEncoderPosition(uint16_t val) { encoderPosition_ = val; }
    void setTorqueCurrent(int16_t val) { torqueCurrent_ = val; }
    void setAnglePidKp(double val) { anglePidKp_ = val; }
    void setAnglePidKi(double val) { anglePidKi_ = val; }
    void setSpeedPidKp(double val) { speedPidKp_ = val; }
    void setSpeedPidKi(double val) { speedPidKi_ = val; }
    void setIqPidKp(double val) { iqPidKp_ = val; }
    void setIqPidKi(double val) { iqPidKi_ = val; }
    void setAcceleration(int32_t val) { acceleration_ = val; }
    void setEncoderRaw(uint16_t val) { encoderRaw_ = val; }
    void setEncoderOffset(uint16_t val) { encoderOffset_ = val; }
    void setMotorAngle(int64_t val) { motorAngle_ = val; }
    void setCicleAngle(uint32_t val) { cicleAngle_ = val; }
    void setVoltage(uint16_t val) { voltage_ = val; }
    void setErrorState(int8_t val) { 
        errorState_ = val; 
        // Update boolean flags based on errorState_
        isVoltageNormal_ = (errorState_ & 0x01) != 0;
        isUnderVoltageProtect_ = (errorState_ & 0x02) != 0;
        isTemperatureNormal_ = (errorState_ & 0x04) != 0;
        isOverTemperatureProtect_ = (errorState_ & 0x08) != 0;
    }
    void setTorque(int16_t val) { torque_ = val; }
    void setSpeed(int16_t val) { speed_ = val; }
    void setEncoder(uint16_t val) { encoder_ = val; }



};