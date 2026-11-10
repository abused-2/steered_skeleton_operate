/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.hpp
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

#ifndef CHASSIS_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_
#define CHASSIS_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_

namespace robot {
class GimbalChassisComm;
}  // namespace robot


// 获取本板双板通讯对象（robot::GimbalChassisComm）单例
robot::GimbalChassisComm *GetGimbalChassisComm();

#endif /* CHASSIS_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_ */
