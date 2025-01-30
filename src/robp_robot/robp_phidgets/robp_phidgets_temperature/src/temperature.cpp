// robp_phidgets_temperature
#include <robp_phidgets_temperature/temperature.hpp>

// phidgets api
#include <phidgets_api/phidget22.hpp>

namespace robp::phidgets
{
Temperature::Temperature(rclcpp::NodeOptions const &options)
    : Node("temperature", options)
{
	int32_t serial_number = this->declare_parameter("serial_num", -1);
	hub_port_             = this->declare_parameter("hub_port", 0);
	frame_id_             = this->declare_parameter("frame_id", "imu_link");
	double data_rate      = this->declare_parameter("data_rate", 8.0);
	double temperature_change_trigger =
	    this->declare_parameter("temperature_change_trigger", 0.0);

	pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 1);

	create();
	assignEventHandlers();
	::phidgets::helpers::openWaitForAttachment(
	    reinterpret_cast<PhidgetHandle>(temperature_), serial_number, hub_port_, false, 0);

	setDataRate(data_rate);
	setTemperatureChangeTrigger(temperature_change_trigger);
}

Temperature::~Temperature()
{
	PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(temperature_);
	::phidgets::helpers::closeAndDelete(&handle);
}

void Temperature::create()
{
	PhidgetReturnCode ret = PhidgetTemperatureSensor_create(&temperature_);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to create temperature sensor on port " + std::to_string(hub_port_), ret);
	}
}

void Temperature::assignEventHandlers()
{
	PhidgetReturnCode ret;

	ret = PhidgetTemperatureSensor_setOnTemperatureChangeHandler(
	    temperature_, temperatureChangeCallback, this);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to set on temperature change handler on port " +
		        std::to_string(hub_port_),
		    ret);
	}

	ret = Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(temperature_),
	                                 attachCallback, this);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to set attach handler on port " + std::to_string(hub_port_), ret);
	}

	ret = Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(temperature_),
	                                 detachCallback, this);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to set detach handler on port " + std::to_string(hub_port_), ret);
	}

	ret = Phidget_setOnErrorHandler(reinterpret_cast<PhidgetHandle>(temperature_),
	                                errorCallback, this);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to set error handler on port " + std::to_string(hub_port_), ret);
	}
}

double Temperature::dataRate() const
{
	double            rate;
	PhidgetReturnCode ret = PhidgetTemperatureSensor_getDataRate(temperature_, &rate);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to get data rate for temperature sensor on port " +
		        std::to_string(hub_port_),
		    ret);
	}
	return rate;
}

void Temperature::setDataRate(double rate)
{
	PhidgetReturnCode ret = PhidgetTemperatureSensor_setDataRate(temperature_, rate);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to set data rate for temperature sensor on port " +
		        std::to_string(hub_port_),
		    ret);
	}
	data_rate_ = rate;
}

double Temperature::temperature() const
{
	double            temp;
	PhidgetReturnCode ret = PhidgetTemperatureSensor_getTemperature(temperature_, &temp);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to get temperature for temperature sensor on port " +
		        std::to_string(hub_port_),
		    ret);
	}
	return temp;
}

double Temperature::temperatureChangeTrigger() const
{
	double            change;
	PhidgetReturnCode ret =
	    PhidgetTemperatureSensor_getTemperatureChangeTrigger(temperature_, &change);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to get temperature change trigger for temperature sensor on "
		    "port " +
		        std::to_string(hub_port_),
		    ret);
	}
	return change;
}

void Temperature::setTemperatureChangeTrigger(double change)
{
	PhidgetReturnCode ret =
	    PhidgetTemperatureSensor_setTemperatureChangeTrigger(temperature_, change);
	if (EPHIDGET_OK != ret) {
		throw ::phidgets::Phidget22Error(
		    "Failed to set temperature change trigger for temperature sensor on "
		    "port " +
		        std::to_string(hub_port_),
		    ret);
	}
	temperature_change_trigger_ = change;
}

int Temperature::port() const { return hub_port_; }

void Temperature::init()
{
	if (0 <= data_rate_) {
		setDataRate(data_rate_);
	}
	if (0 < temperature_change_trigger_) {
		setTemperatureChangeTrigger(temperature_change_trigger_);
	}
}

void Temperature::publish(double temperature)
{
	auto msg             = std::make_unique<sensor_msgs::msg::Temperature>();
	msg->header.frame_id = frame_id_;
	msg->header.stamp    = this->now();
	msg->temperature     = temperature;
	msg->variance        = 0.0;
	pub_->publish(std::move(msg));
}

void Temperature::temperatureChangeCallback(PhidgetTemperatureSensorHandle /* ch */,
                                            void *ctx, double temperature)
{
	static_cast<Temperature *>(ctx)->publish(temperature);
}

void Temperature::attachCallback(PhidgetHandle /* ch */, void *ctx)
{
	printf("Attach temperature sensor on port %d\n",
	       static_cast<Temperature *>(ctx)->port());
	static_cast<Temperature *>(ctx)->init();
}

void Temperature::detachCallback(PhidgetHandle /* ch */, void *ctx)
{
	printf("Detach temperature sensor on port %d\n",
	       static_cast<Temperature *>(ctx)->port());
}

void Temperature::errorCallback(PhidgetHandle /* ch */, void *ctx,
                                Phidget_ErrorEventCode code, char const *description)
{
	fprintf(stderr, "\x1B[31mError temperature sensor on port %d: %s\033[0m\n",
	        static_cast<Temperature *>(ctx)->port(), description);
	fprintf(stderr, "----------\n");
	PhidgetLog_log(PHIDGET_LOG_ERROR, "Error %d: %s", code, description);
}
}  // namespace robp::phidgets

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robp::phidgets::Temperature)