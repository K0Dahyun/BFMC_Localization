#include "localization.h"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Localization>());
	rclcpp::shutdown();
	return 0;
}