#include "rclcpp/rclcpp.hpp"

struct Location {
    double x, y, yaw;
};

class CourierServer : public rclcpp::Node {
    public:
        CourierServer() : Node("courier_server") {
            frame_id_ = declare_parameter<std::string>("frame_id", "map");
            retry_count_ = declare_parameter<int>("retry_count", 2);
            retry_delay_sec_ = declare_parameter<double>("retry_delay_sec", 3.0);
            feedback_period_sec_ = declare_parameter<double>("feedback_period_sec", 1.0);
            leg_timeout_sec_ = declare_parameter<double>("leg_timeout_sec", 0.0);
            busy_reject_ = declare_parameter<bool>("busy_reject", true);

            auto names = declare_parameter<std::vector<std::string>>("location_names", std::vector<std::string>{});
            auto poses = declare_parameter<std::vector<double>>("location_poses", std::vector<double>{});
            for (size_t i = 0; i < names.size() && 3  * (i + 1) <= poses.size(); ++i) {
                locations_[names[i]] = {poses[3 * i], poses[3 * i + 1], poses[3 * i + 2]};
            }
            RCLCPP_INFO(get_logger(), "Courier server initialized with %zu locations, %d retries", locations_.size(), retry_count_);
            for (const auto& [name, loc] : locations_) {
                RCLCPP_INFO(get_logger(), "Location %s: x=%.2f, y=%.2f, yaw=%.2f", name.c_str(), loc.x, loc.y, loc.yaw);
            }
        }
    
    private:
        std::string frame_id_;
        int retry_count_;
        double retry_delay_sec_;
        double feedback_period_sec_;
        double leg_timeout_sec_;
        bool busy_reject_;
        std::unordered_map<std::string, Location> locations_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CourierServer>());
    rclcpp::shutdown();
    return 0;
}