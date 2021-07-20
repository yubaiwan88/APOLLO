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

/**
 * @file
 */

#ifndef MODULES_COMMON_APOLLO_APP_H_
#define MODULES_COMMON_APOLLO_APP_H_

#include <csignal>
#include <string>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/status/status.h"

#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class ApolloApp
 *
 * @brief The base module class to define the interface of an Apollo app.
 * An Apollo app runs infinitely until being shutdown by SIGINT or ROS. Many
 * essential components in Apollo, such as localization and control are examples
 * of Apollo apps. The APOLLO_MAIN macro helps developer to setup glog, gflag
 * and ROS in one line.
 * 
 * 用于定义Apollo应用程序接口的基本模块类。
    阿波罗应用程序无限期运行，直到被SIGINT或ROS关闭。
    很多阿波罗的基本组成部分，如定位和控制就是阿波罗的应用程序的例子。
    APOLLO_MAIN宏帮助开发人员在一行中设置glog、gflag和ROS。
 */
class ApolloApp {
 public:
  /**
   * @brief module name. It is used to uniquely identify the app.
   */
  virtual std::string Name() const = 0;

  /**
   * @brief this is the entry point of an Apollo App. It initializes the app,
   * starts the app, and stop the app when the ros has shutdown.
   * 这是阿波罗应用程序的入口点。它会初始化应用程序，启动应用程序，并在ros关闭时停止应用程序。
   */
  virtual int Spin();

  /**
   * The default destructor.
   */
  virtual ~ApolloApp() = default;

  /**
   * @brief set the number of threads to handle ros message callbacks.
   * 设置处理ros消息回调的线程数。
   * The default thread number is 1
   */
  void SetCallbackThreadNumber(uint32_t callback_thread_num);

 protected:
  /**
   * @brief The module initialization function. This is the first function being
   * called when the App starts. Usually this function loads the configurations,
   * subscribe the data from sensors or other modules.
   * 
   * 模块初始化功能。当应用程序启动时，这是第一个调用的函数
   * 通常这个函数加载配置，订阅来自传感器或其他模块的数据。
   * @return Status initialization status
   */
  // 模块的初始化
  virtual apollo::common::Status Init() = 0;

  /**
   * @brief The module start function. Apollo app usually triggered to execute
   * in two ways: 1. Triggered by upstream messages, or 2. Triggered by timer.
   * If an app is triggered by upstream messages, the Start() function usually
   * register a call back function that will be called when an upstream message
   * is received. If an app is triggered by timer, the Start() function usually
   * register a timer callback function.
   * 
   * 模块启动功能。阿波罗应用程序通常触发执行有两种方式：1.由上游消息触发或2.由定时器触发。
    如果应用程序是由上游消息触发的，Start（）函数通常会注册一个回调函数，当收到上游消息时将调用该函数。
    如果应用程序是由计时器触发的， Start（）函数通常会注册一个计时器回调函数。
   * @return Status start status
   */
  // 模块启动
  virtual apollo::common::Status Start() = 0;

  /**
   * @brief The module stop function. This function will be called when
   * after ros::shutdown() has finished. In the default APOLLO_MAIN macro,
   * ros::shutdown() is called when SIGINT is received.
   * 
   * 模块停止功能。在ros:：shutdown（）完成后将调用此函数。
   * 在默认阿波罗主宏中，接收到SIGINT时调用ros:：shutdown（）。
   */
  // 模块停止
  virtual void Stop() = 0;

  /** The callback thread number
   * 回调函数线程的数量
   */
  uint32_t callback_thread_num_ = 1;

 private:
  /**
   * @brief Export flag values to <FLAGS_log_dir>/<name>.flags.
   * 导出标志值
   */
  void ExportFlags() const;
};

void apollo_app_sigint_handler(int signal_num);

}  // namespace common
}  // namespace apollo
/*
始化Google日志工具，
使用Google命令行解析工具解析相关参数，
注册接收中止信号“SIGINT”的处理函数：apollo::common::apollo_app_sigint_handler
                   （该函数的功能十分简单，就是收到中止信号“SIGINT”后，调用ros::shutdown()关闭ROS），
创建apollo::planning::Planning对象：apollo_app_，
初始化ROS环境，调用apollo_app_.Spin()函数开始消息处理循环
*/
#define APOLLO_MAIN(APP)                                       \
  int main(int argc, char **argv) {                            \
    google::InitGoogleLogging(argv[0]);                        \
    google::ParseCommandLineFlags(&argc, &argv, true);         \
    signal(SIGINT, apollo::common::apollo_app_sigint_handler); \
    APP apollo_app_;                                           \
    ros::init(argc, argv, apollo_app_.Name());                 \
    apollo_app_.Spin();                                        \
    return 0;                                                  \
  }

#endif  // MODULES_COMMON_APOLLO_APP_H_
