#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/qos.hpp>

class PointCloudRelay : public rclcpp::Node
{
public:
    PointCloudRelay() : Node("pointcloud_relay")
    {
        // Reliable QoS for critical usage
        rclcpp::QoS reliable_qos(10);
        reliable_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        // Publisher to reliable topic
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/rgbd_camera/depth_camera/points_reliable", reliable_qos);

        // Subscriber to existing best-effort topic
        rclcpp::QoS best_effort_qos(10);
        best_effort_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rgbd_camera/depth_camera/points",
            best_effort_qos,
            std::bind(&PointCloudRelay::pointcloud_callback, this, std::placeholders::_1)
        );
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Republish to reliable topic
        pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudRelay>());
    rclcpp::shutdown();
    return 0;
}
