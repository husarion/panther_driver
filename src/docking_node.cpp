#include <DockingController.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "docking_node");
    auto node = std::make_shared<ros::NodeHandle>("~");
    auto docking_controller = std::make_unique<DockingController>(node);

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
