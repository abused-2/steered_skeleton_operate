/**
 *******************************************************************************
 * @file      :ins_comm.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef CHASSIS_INSTANCE_INS_COMM_HPP_
#define CHASSIS_INSTANCE_INS_COMM_HPP_

#include "DT7.hpp"

// 获取遥控器解码对象（DT7）单例
hello_world::remote_control::DT7 *GetRemoteControl();

#endif /* CHASSIS_INSTANCE_INS_COMM_HPP_ */
