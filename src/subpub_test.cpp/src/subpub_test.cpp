// basic ROS package of pub/sub that are given with examp[les files, just combined here]
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      publisher2_ = this->create_publisher<std_msgs::msg::String>("my_loc", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! I am a publisher to the topic 'topic' " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);

      auto message2 = std_msgs::msg::String();
      messsage2.data = "I am trying to find my location" + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message2.data.c_str());
      publisher2_->publish(message);

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
    size_t count_;
};

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this,std::placeholders::_1));

      subscription2_ = this->create_subscription<std_msgs::msg::String>(
      "my_loc", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1)); 
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I am subscribed to topic 'topic' and I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  rclcpp::shutdown();
  return 0;
}
