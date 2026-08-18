#include "rclcpp/rclcpp.hpp"
#include "acadbot_courier_interfaces/srv/accept_job.hpp"

using AcceptJob = acadbot_courier_interfaces::srv::AcceptJob;

struct Location {
    double x, y, yaw;
};

struct Job {
    std::string pickup, dropoff;
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

            accept_service_ = create_service<AcceptJob>("courier/accept_job", std::bind(&CourierServer::handle_accept_job, this, std::placeholders::_1, std::placeholders::_2));

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

        rclcpp::Service<AcceptJob>::SharedPtr accept_service_;
        std::unordered_map<uint32_t, Job> jobs_;
        uint32_t next_job_id_{1};
        bool job_active_{false};
        uint32_t active_job_id_{0};

        void handle_accept_job(const std::shared_ptr<AcceptJob::Request> request, std::shared_ptr<AcceptJob::Response> response) {
            auto pickup = locations_.find(request->pickup_name);
            auto dropoff = locations_.find(request->dropoff_name);

            if (pickup == locations_.end()) {
                response->accepted = false;
                response->reason = "unknown location '" + request->pickup_name + "'";
                return;
            }

            if (dropoff == locations_.end()) {
                response->accepted = false;
                response->reason = "unknown location '" + request->dropoff_name + "'";
                return;
            }

            if (request->pickup_name == request->dropoff_name) {
                response->accepted = false;
                response->reason = "pickup and dropoff locations are the same";
                return;
            }

            if (busy_reject_ && job_active_) {
                response->accepted = false;
                response->reason = "robot is busy with job " + std::to_string(active_job_id_);
                return;
            }

            const uint32_t id = next_job_id_++;
            jobs_[id] = {request->pickup_name, request->dropoff_name};
            response->accepted = true;
            response->job_id = id;

            RCLCPP_INFO(get_logger(), "Accepted job %u: %s -> %s", id, request->pickup_name.c_str(), request->dropoff_name.c_str());
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CourierServer>());
    rclcpp::shutdown();
    return 0;
}