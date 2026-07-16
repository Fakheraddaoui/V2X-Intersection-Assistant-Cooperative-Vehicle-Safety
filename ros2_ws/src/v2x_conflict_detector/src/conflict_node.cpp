#include <rclcpp/rclcpp.hpp>
#include <v2x_msgs/msg/vehicle_state.hpp>
#include <v2x_msgs/msg/brake_command.hpp>
#include "v2x_conflict_detector/ttc.hpp"
#include <unordered_map>

using namespace std::chrono_literals;

class ConflictNode : public rclcpp::Node {
public:
    ConflictNode() : Node("v2x_conflict_detector")
    {
        ttc_threshold_ = declare_parameter("ttc_threshold", 2.0);
        stale_after_   = declare_parameter("stale_after", 0.5);

        sub_ = create_subscription<v2x_msgs::msg::VehicleState>(
            "/v2x/vehicle_states", rclcpp::SensorDataQoS(),
            [this](v2x_msgs::msg::VehicleState::SharedPtr m) {
                agents_[m->vehicle_id] = {*m, now()};
            });

        pub_ = create_publisher<v2x_msgs::msg::BrakeCommand>("/v2x/brake_commands", 10);
        timer_ = create_wall_timer(50ms, [this] { evaluate(); });
    }

private:
    void evaluate()
    {
        std::vector<v2x::Agent> agents;
        const auto t_now = now();
        for (auto it = agents_.begin(); it != agents_.end();) {
            if ((t_now - it->second.second).seconds() > stale_after_) {
                it = agents_.erase(it);       // drop stale vehicles
                continue;
            }
            const auto& m = it->second.first;
            agents.push_back({m.vehicle_id, m.px, m.py, m.vx, m.vy,
                              0.5 * std::hypot(m.length, m.width)});
            ++it;
        }

        for (const auto& c : v2x::detectConflicts(agents, ttc_threshold_)) {
            for (uint32_t id : {c.id_a, c.id_b}) {
                v2x_msgs::msg::BrakeCommand cmd;
                cmd.target_vehicle_id = id;
                cmd.ttc_seconds = static_cast<float>(c.ttc);
                cmd.emergency = true;
                pub_->publish(cmd);
            }
            RCLCPP_WARN(get_logger(), "Conflict %u<->%u TTC=%.2fs — braking",
                        c.id_a, c.id_b, c.ttc);
        }
    }

    double ttc_threshold_, stale_after_;
    std::unordered_map<uint32_t, std::pair<v2x_msgs::msg::VehicleState, rclcpp::Time>> agents_;
    rclcpp::Subscription<v2x_msgs::msg::VehicleState>::SharedPtr sub_;
    rclcpp::Publisher<v2x_msgs::msg::BrakeCommand>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConflictNode>());
    rclcpp::shutdown();
    return 0;
}
