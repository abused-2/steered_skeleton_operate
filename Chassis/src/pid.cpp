#include "pid.hpp"
 
namespace hello_world{
namespace pid{


//限幅
namespace{
float Clamp(float x, float lim){
    if(x > lim)  x = lim;
    else if(x < -lim)  x = -lim;
    return x;
}}

Pid::Pid(float kp, float ki, float kd, float out_max, float integ_max)
        : kp_(kp), ki_(ki), kd_(kd), out_max_(out_max), integ_max_(integ_max) {}

void Pid::setParams(float kp, float ki, float kd){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pid::setLimit(float out_max, float integ_max){
    out_max_ = out_max;
    integ_max_ = integ_max;
}

void Pid::reset(){
    integ_ = 0.0f;
    last_fdb_ = 0.0f;
    err_ = 0.0f;
    out_ = 0.0f;
    is_first_ = true;
}

float Pid::calc(float ref, float fdb, float dt){
    err_ = ref - fdb;

    integ_ += ki_ * err_ * dt;
    integ_ = Clamp(integ_, integ_max_);

    // 微分先行
    float d = 0.0f;
    if (is_first_) {
        last_fdb_ = fdb;
        is_first_ = false;
    } else {
        d = -kd_ * (fdb - last_fdb_) / dt;
    }

    out_ = Clamp(kp_ * err_ + integ_ + d, out_max_);

    last_fdb_ = fdb;
    return out_;
}

}}  //namespace 