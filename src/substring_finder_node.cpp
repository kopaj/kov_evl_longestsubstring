#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubstringFinderNode : public rclcpp::Node
{
public:
    SubstringFinderNode() : Node("substring_finder_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "string_topic", 10,
            std::bind(&SubstringFinderNode::find_longest_common_substring, this, std::placeholders::_1));
    }

private:
    std::string longest_common_substring(const std::string &str1, const std::string &str2)
    {
        int m = str1.size();
        int n = str2.size();
        int max_len = 0;
        int ending_index = m;
        std::vector<std::vector<int>> len_table(m + 1, std::vector<int>(n + 1, 0));

        for (int i = 1; i <= m; i++)
        {
            for (int j = 1; j <= n; j++)
            {
                if (str1[i - 1] == str2[j - 1])
                {
                    len_table[i][j] = len_table[i - 1][j - 1] + 1;
                    if (len_table[i][j] > max_len)
                    {
                        max_len = len_table[i][j];
                        ending_index = i;
                    }
                }
            }
        }
        return str1.substr(ending_index - max_len, max_len);
    }

    void find_longest_common_substring(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string received_str = msg->data;
        RCLCPP_INFO(this->get_logger(), "Kapott sztring: '%s'", received_str.c_str());

        // For simplicity, we'll just compare two hardcoded strings here
        std::string str1 = "ros2example";
        std::string str2 = received_str;

        std::string lcs = longest_common_substring(str1, str2);
        RCLCPP_INFO(this->get_logger(), "Leghosszabb közös részsztring: '%s'", lcs.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubstringFinderNode>());
    rclcpp::shutdown();
    return 0;
}
