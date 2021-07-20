/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CONVERT_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CONVERT_H_

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Time.h"

#include "modules/drivers/lidar_velodyne/pointcloud/velodyne_parser.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class Converter {
 public:
  Converter();
  ~Converter();

  bool init(const VelodyneConf& conf);
  bool convert_packets_to_pointcloud(
      const velodyne_msgs::VelodyneScanUnifiedPtr scan_msg,
      sensor_msgs::PointCloud2Ptr pointcloud);

  bool append(const velodyne_msgs::VelodyneScanUnifiedPtr scan_msg);
  bool pack(sensor_msgs::PointCloud2Ptr pointcloud);
  bool ready();

 private:
  VelodyneConf config_;
  VelodyneParser* parser_ = nullptr;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CONVERT_H_
