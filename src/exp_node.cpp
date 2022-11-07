#include "aer_auto_exposure_gradient/auto_exp.h"
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting AE node" << std::endl;
  exit(1);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "exp_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    exp_node::ExpNode exp_node(nh, pnh);

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(3); // Use multi-threads
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}