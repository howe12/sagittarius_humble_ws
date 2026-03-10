/*
 *  Copyright (c) 2023, NXROBO Ltd.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  Authors: Litian Zhuang <litian.zhuang@nxrobo.com>
 */

#include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool success = true;
  auto node = std::make_shared<sdk_sagittarius_arm::SagittariusArmReal>(success);
  if (success) 
  {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } 
  else 
  {
    RCLCPP_FATAL(LOGGER, "start the sagittarius arm failed!");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}






