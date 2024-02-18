#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

class DifferentialDriveNode : public rclcpp::Node {
public:
    DifferentialDriveNode() : Node("differential_drive_node") {
        // Create a publisher to send CAN messages
        can_pub_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);

        // Create a subscriber to receive CAN messages
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>("can_rx", 10, std::bind(&DifferentialDriveNode::receiveCallback, this, std::placeholders::_1));

        // Define the command message (e.g., Read motor state 2)
        can_msgs::msg::Frame command_msg;
        command_msg.id = 0x9C; // Set the CAN ID for the command
        command_msg.dlc = 8;   // Data Length Code (number of data bytes)
        // Set data bytes according to the command format

        // Send the command message
        can_pub_->publish(command_msg);
    }

private:
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

    void receiveCallback(const can_msgs::msg::Frame::SharedPtr msg) {
        // Process the received CAN message (response)
        // Extract and parse data from msg->data

        // Example: Parse and print the received data
        int8_t temperature = static_cast<int8_t>(msg->data[1]);
        int16_t iq = static_cast<int16_t>((msg->data[2] << 8) | msg->data[3]);
        int16_t speed = static_cast<int16_t>((msg->data[4] << 8) | msg->data[5]);
        uint16_t encoder = static_cast<uint16_t>((msg->data[6] << 8) | msg->data[7]);

        RCLCPP_INFO(get_logger(), "Received: Temperature=%d, iq=%d, speed=%d, encoder=%u", temperature, iq, speed, encoder);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveNode>());
    rclcpp::shutdown();
    return 0;
}
