#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <random>

class StringGeneratorNode : public rclcpp::Node
{
public:
    StringGeneratorNode() : Node("string_generator_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("string_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&StringGeneratorNode::publish_strings, this));
    }

private:
    void publish_strings()
    {
        std::string str1 = generate_random_string(10);
        std::string str2 = generate_random_string(10);

        auto message1 = std_msgs::msg::String();
        message1.data = str1;
        RCLCPP_INFO(this->get_logger(), "1. Generált sztring: '%s'", str1.c_str());
        publisher_->publish(message1);

        auto message2 = std_msgs::msg::String();
        message2.data = str2;
        RCLCPP_INFO(this->get_logger(), "2. Generált sztring: '%s'", str2.c_str());
        publisher_->publish(message2);
    }

    std::string generate_random_string(size_t length)
    {
        const std::string chars = "abcdefghijklmnopqrstuvwxyz";
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_int_distribution<> dist(0, chars.size() - 1);
        std::string result;
        for (size_t i = 0; i < length; ++i)
        {
            result += chars[dist(generator)];
        }
        return result;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StringGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
