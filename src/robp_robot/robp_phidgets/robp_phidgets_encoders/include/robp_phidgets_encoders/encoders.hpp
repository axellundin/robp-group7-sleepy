#ifndef ROBP_PHIDGETS_ENCODERS_ENCODERS_HPP
#define ROBP_PHIDGETS_ENCODERS_ENCODERS_HPP

// robp_phidgets
#include <robp_phidgets_encoders/encoder.hpp>

// robp_interfaces
#include <robp_interfaces/msg/encoders.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// STL
#include <memory>

namespace robp::phidgets
{
class Encoders : public rclcpp::Node
{
 public:
	explicit Encoders(rclcpp::NodeOptions const& options);

 private:
	void publish();

 private:
	std::unique_ptr<Encoder> left_;
	std::unique_ptr<Encoder> right_;

	rclcpp::Publisher<robp_interfaces::msg::Encoders>::SharedPtr pub_;
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_ENCODERS_ENCODERS_HPP