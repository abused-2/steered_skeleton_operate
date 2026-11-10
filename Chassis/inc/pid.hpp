#ifndef HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_
#define HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_

namespace hello_world {
namespace pid {

class Pid {
 public:
    Pid() = default;
    Pid(float kp, float ki, float kd, float out_max, float integ_max);

    // 参数设置
    void setParams(float kp, float ki, float kd);
    void setLimit(float out_max, float integ_max);

    // 清空积分(状态/模式切换时)
    void reset();

    // 计算输出
    float calc(float ref, float fdb, float dt = 0.001f);

    //数据访问
    float output() { return out_;}

 private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float out_max_ = 0.0f;
    float integ_max_ = 0.0f;

    float integ_ = 0.0f;     // 积分项
    float last_fdb_ = 0.0f;  // 上一拍反馈
    float err_ = 0.0f;       // 本拍误差
    float out_ = 0.0f;       // 本拍输出
    bool is_first_ = true;   // 首拍不计算微分 
};

}}  // namespace

#endif  // HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_