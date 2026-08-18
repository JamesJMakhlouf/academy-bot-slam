#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "acadbot_courier_interfaces/srv/accept_job.hpp"

using namespace std::chrono_literals;
using AcceptJob = acadbot_courier_interfaces::srv::AcceptJob;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

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

            nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
            startup_timer_ = create_wall_timer(1s, std::bind(&CourierServer::wait_for_nav2, this));

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

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
        rclcpp::TimerBase::SharedPtr startup_timer_;

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

        void wait_for_nav2() {
            if (!nav_client_->action_server_is_ready()) {
                RCLCPP_INFO(get_logger(), "Waiting for Nav2 'navigate_to_pose' server...");
                return;
            }
            startup_timer_->cancel();
            std::thread(&CourierServer::drive_to, this, "lab_bench").detach();
        }

        const char* result_code_name(rclcpp_action::ResultCode code) {
            switch (code) {
                case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
                case rclcpp_action::ResultCode::CANCELED: return "CANCELED";
                case rclcpp_action::ResultCode::ABORTED: return "ABORTED";
                case rclcpp_action::ResultCode::UNKNOWN: return "UNKNOWN";
                default: return "???";
            }
        }

        
        void drive_to(const std::string& name) {
            auto it = locations_.find(name);
            if (it == locations_.end()) {
                RCLCPP_ERROR(get_logger(), "Unknown location '%s'", name.c_str());
                return;
            }
            const Location& loc = it->second;

            NavigateToPose::Goal goal;
            goal.pose.header.frame_id = frame_id_;
            goal.pose.header.stamp = now();
            goal.pose.pose.position.x = loc.x;
            goal.pose.pose.position.y = loc.y;
            tf2::Quaternion q;
            q.setRPY(0, 0, loc.yaw);
            goal.pose.pose.orientation.x = q.x();
            goal.pose.pose.orientation.y = q.y();
            goal.pose.pose.orientation.z = q.z();
            goal.pose.pose.orientation.w = q.w();

            RCLCPP_INFO(get_logger(), "Driving to %s (%.2f,  %.2f, %.2f)", name.c_str(), loc.x, loc.y, loc.yaw);

            auto promise = std::make_shared<std::promise<rclcpp_action::ResultCode>>();
            auto future = promise->get_future();

            rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
            options.goal_response_callback = [promise](GoalHandleNav::SharedPtr gh) {
                if (!gh) {
                    promise->set_value(rclcpp_action::ResultCode::ABORTED);
                }
            };
            options.feedback_callback = [](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
                RCLCPP_INFO(rclcpp::get_logger("courier_server"), " distance remaining %.2f m", fb->distance_remaining);
            };
            options.result_callback = [promise](const GoalHandleNav::WrappedResult& result) {
                promise->set_value(result.code);
            };

            nav_client_->async_send_goal(goal, options);

            future.wait();
            auto code = future.get();
            RCLCPP_INFO(get_logger(), "drive_to(%s) finished with code %d (%s)", name.c_str(), static_cast<int>(code), result_code_name(code));
        }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<CourierServer>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}