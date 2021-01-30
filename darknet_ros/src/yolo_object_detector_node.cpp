/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "darknet_ros/YoloObjectDetector.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor executor;

  auto yoloObjectDetector1 = std::make_shared<darknet_ros::YoloObjectDetector>();
  
  auto yoloObjectDetector2 = std::make_shared<darknet_ros::YoloObjectDetector>();
  
  auto yoloObjectDetector3 = std::make_shared<darknet_ros::YoloObjectDetector>();
  
  auto yoloObjectDetector4 = std::make_shared<darknet_ros::YoloObjectDetector>();
  
  //auto yoloObjectDetector5 = std::make_shared<darknet_ros::YoloObjectDetector>();
  
  //auto yoloObjectDetector6 = std::make_shared<darknet_ros::YoloObjectDetector>();
  
  //auto yoloObjectDetector7 = std::make_shared<darknet_ros::YoloObjectDetector>();
  
 //auto yoloObjectDetector8 = std::make_shared<darknet_ros::YoloObjectDetector>();


  yoloObjectDetector1->init();
  
  yoloObjectDetector2->init();
  
  yoloObjectDetector3->init();
  
  yoloObjectDetector4->init();
  
  //yoloObjectDetector5->init();
  
  //yoloObjectDetector6->init();
  
  //yoloObjectDetector7->init();
  
  //yoloObjectDetector8->init();
  
  //rclcpp::spin(yoloObjectDetector->get_node_base_interface());
  
  executor.add_node(yoloObjectDetector1->get_node_base_interface());
  
  executor.add_node(yoloObjectDetector2->get_node_base_interface());
  
  executor.add_node(yoloObjectDetector3->get_node_base_interface());
  
  executor.add_node(yoloObjectDetector4->get_node_base_interface());
  
  //executor.add_node(yoloObjectDetector5->get_node_base_interface());
  
 // executor.add_node(yoloObjectDetector6->get_node_base_interface());
  
  //executor.add_node(yoloObjectDetector7->get_node_base_interface());
  
  //executor.add_node(yoloObjectDetector8->get_node_base_interface());
  
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
