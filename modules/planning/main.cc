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
// 用于定义Apollo应用程序接口的基本模块类。
#include "modules/common/apollo_app.h"
// 规划模块的主类
#include "modules/planning/planning.h"
// APOLLO_MAIN宏帮助开发人员在一行中设置glog、gflag和ROS。
/*
始化Google日志工具，
使用Google命令行解析工具解析相关参数，
注册接收中止信号“SIGINT”的处理函数：apollo::common::apollo_app_sigint_handler
                   （该函数的功能十分简单，就是收到中止信号“SIGINT”后，调用ros::shutdown()关闭ROS），
创建apollo::planning::Planning对象：apollo_app_，
初始化ROS环境，调用apollo_app_.Spin()函数开始消息处理循环
*/
APOLLO_MAIN(apollo::planning::Planning)
