#pragma once

struct BangBangDest {
  BangBangDest()
    : x(0.0f), v(0.0f), a(0.0f), is_end(false)
  {
  }

  BangBangDest(float x_, float v_, float a_, bool is_end_)
    : x(x_), v(v_), a(a_), is_end(is_end_)
  {
  }

  float x;
  float v;
  float a;
  bool is_end;
};

struct BangBangPlanner {
  BangBangPlanner(){}
  BangBangPlanner(float x_dest_, float v_lim_, float a_lim_)
  {
    x_dest = x_dest_;
    v_lim = v_lim_;
    a_lim = a_lim_;

    x_acc = v_lim * v_lim / a_lim;
    if(x_acc > x_dest){
      x_acc = x_dest;
      v_max = std::sqrt(a_lim * x_dest);
      t_vel = 0.0f;
    }else{
      v_max = v_lim;
      t_vel = (x_dest - x_acc) / v_max; 
    }

    t_acc = v_max / a_lim;
    t_total = t_acc + t_vel + t_acc;
  }

  BangBangDest operator()(float t, bool reverse=false){
    if(reverse){
      auto dest = calc(t_total - t);
      dest.v = -dest.v;
      dest.is_end = (t >= t_total);
      return dest;
    }else{
      return calc(t);
    }
  }

  BangBangDest calc(float t)
  {
    auto dest = BangBangDest();
    if(t < 0.0f){
      // nothing
    }else if(t < t_acc){
      dest.x = 0.5f * a_lim * t * t;
      dest.v = a_lim * t;
      dest.a = a_lim;
    }else if(t < t_acc + t_vel){
      float dt = t - t_acc;
      dest.x = 0.5f * x_acc + v_max * dt;
      dest.v = v_max;
    }else if(t < t_total){
      float t_ = t_total - t;
      dest.x = x_dest - 0.5f * a_lim * t_ * t_;
      dest.v = a_lim * t_;
      dest.a = -a_lim;
    }else{
      dest.x = x_dest;
      dest.is_end = true;
    }
    return dest;
  }

  float getTime() { return t_total; }

private:
  float x_dest, v_lim, a_lim;
  float x_acc, v_max, t_acc, t_vel, t_total;
};
