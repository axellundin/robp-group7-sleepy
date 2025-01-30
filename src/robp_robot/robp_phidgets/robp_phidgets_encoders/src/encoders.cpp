// robp_phidgets
#include <robp_phidgets_encoders/encoders.hpp>

namespace robp::phidgets
{
Encoders::Encoders(rclcpp::NodeOptions const& options) : Node("encoders", options)
{
	// default open any device
	int serial_num_left  = this->declare_parameter("left_serial", -1);
	int serial_num_right = this->declare_parameter("right_serial", -1);
	// only used if the device is on a VINT hub_port
	int    hub_port_left           = this->declare_parameter("left_hub_port", 4);
	int    hub_port_right          = this->declare_parameter("right_hub_port", 2);
	double data_rate               = this->declare_parameter("data_rate", 20.0);
	int    position_change_trigger = this->declare_parameter("position_change_trigger", 0);

	if (hub_port_left == hub_port_right) {
		RCLCPP_FATAL(this->get_logger(), "Left and right port cannot be the same");
		exit(1);
	}

	pub_ = create_publisher<robp_interfaces::msg::Encoders>("/motor/encoders", 1);

	left_  = std::make_unique<Encoder>(serial_num_left, hub_port_left, false, 0,
                                    std::bind(&Encoders::publish, this));
	right_ = std::make_unique<Encoder>(serial_num_right, hub_port_right, false, 0,
	                                   std::bind(&Encoders::publish, this));

	left_->setDataRate(data_rate);
	right_->setDataRate(data_rate);
	left_->setPositionChangeTrigger(position_change_trigger);
	right_->setPositionChangeTrigger(position_change_trigger);
}

void Encoders::publish()
{
	if (!left_ || !right_ || !left_->changed() || !right_->changed()) {
		return;
	}

	auto msg                 = std::make_unique<robp_interfaces::msg::Encoders>();
	msg->header.stamp        = this->now();
	msg->delta_encoder_left  = left_->change();
	msg->encoder_left        = left_->position();
	msg->delta_encoder_right = -right_->change();
	msg->encoder_right       = -right_->position();

	pub_->publish(std::move(msg));
}
}  // namespace robp::phidgets

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robp::phidgets::Encoders)