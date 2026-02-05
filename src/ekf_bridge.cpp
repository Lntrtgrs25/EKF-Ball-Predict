#include "rclcpp/rclcpp.hpp"
#include "gyakuenki_interfaces/msg/projected_objects.hpp"
#include "aruku_interfaces/msg/point2.hpp"
#include "bridge_ekfsim/ekf_ball.hpp"

class EKFBridge : public rclcpp::Node {
public:
    EKFBridge() : Node("ekf_bridge") {
        ekf_ = std::make_shared<keisan::ekf_ball>();
        ekf_->init(450.0, 300.0, 150.0, 0.0);
        sub_ = create_subscription<gyakuenki_interfaces::msg::ProjectedObjects>(
            "/gyakuenki_cpp/projected_objects", 10, std::bind(&EKFBridge::callback, this, std::placeholders::_1));
        pub_ = create_publisher<aruku_interfaces::msg::Point2>("/ball/filtered_pos", 10);
    }
private:
    void callback(const gyakuenki_interfaces::msg::ProjectedObjects::SharedPtr msg) {
        for (auto &po : msg->projected_objects) {
            if (po.label == "ball") {
                double rel_x = po.position.x * 100.0;
                double rel_y = po.position.y * -100.0;
                keisan::Matrix<2, 1> z;
                z[0][0] = rel_x; z[1][0] = rel_y;
                ekf_->predict(0.1);
                ekf_->update(z);
                auto pos = ekf_->getPosition();
                auto out = aruku_interfaces::msg::Point2();
                out.x = pos[0][0]; out.y = pos[1][0];
                pub_->publish(out);
                RCLCPP_INFO(this->get_logger(), "Filtered pos: x=%.2f, y=%.2f", out.x, out.y);
            }
        }
    }
    std::shared_ptr<keisan::ekf_ball> ekf_;
    rclcpp::Subscription<gyakuenki_interfaces::msg::ProjectedObjects>::SharedPtr sub_;
    rclcpp::Publisher<aruku_interfaces::msg::Point2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFBridge>());
    rclcpp::shutdown();
    return 0;
}