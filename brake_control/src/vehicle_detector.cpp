#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>

class VehicleDetector : public rclcpp::Node
{
public:
    VehicleDetector() : Node("vehicle_detector"), consecutive_detections(0), status_broadcasted(false)
    {
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/bbox", 10, std::bind(&VehicleDetector::bbox_callback, this, std::placeholders::_1));

        status_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/vehicle_status", 10);
    }

private:
    void bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        bool condition_met = false;

        RCLCPP_INFO(this->get_logger(), "Detected %zu vehicles", msg->markers.size());
        for (const auto& marker : msg->markers)
        {
            double x = marker.pose.position.x;
            double y = marker.pose.position.y;

            RCLCPP_INFO(this->get_logger(), "Vehicle Position - X: %f, Y: %f, Z: %f",
                        x, y, marker.pose.position.z);

            if (abs(y) <= 3 && x <= 7 && x>=0.0)
            {
                condition_met = true;
                break; // Condition met, no need to check further
            }
        }

        if (condition_met)
        {
            consecutive_detections++;
        }
        else
        {
            consecutive_detections = 0;
        }

        if (consecutive_detections >= 1 || status_broadcasted)
        {
            status_broadcasted = true; // Once status 1 is broadcasted, always keep it at 1
        }

        int status = status_broadcasted ? 1 : 0;
        RCLCPP_INFO(this->get_logger(), "Status: %d", status);

        std_msgs::msg::Int32 status_msg;
        status_msg.data = status;
        status_publisher_->publish(status_msg);
    }

    int consecutive_detections;
    bool status_broadcasted;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleDetector>());
    rclcpp::shutdown();
    return 0;
}
