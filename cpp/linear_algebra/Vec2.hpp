#pragma once
#include <cmath>
#include <ostream>

namespace linear {
template <class T>
struct Vec2 {
  T x;
  T y;

  Vec2() : x(0), y(0) {}
  Vec2(T x, T y) : x(x), y(y) {}

  T norm() const { return std::sqrt(x * x + y * y); }

  Vec2<T> limit(T max_norm) const {
    if(max_norm <= 0){
      return Vec2(0, 0);
    }
    T ratio = this->norm() / max_norm;
    if(ratio > 1){
      return Vec2<T>(this->x / ratio, this->y / ratio);
    }
    return Vec2<T>(this->x, this->y);
  }

  Vec2<T> operator+(const Vec2<T>& other) const {
    return Vec2<T>(this->x + other.x, this->y + other.y);
  }

  Vec2<T> operator-(const Vec2<T>& other) const {
    return Vec2<T>(this->x - other.x, this->y - other.y);
  }

  T operator*(const Vec2<T>& other) const {
    return this->x * other.x + this->y * other.y;
  }

  Vec2<T> operator*(T ratio) const {
    return Vec2<T>(this->x * ratio, this->y * ratio);
  }

  Vec2<T> operator/(T ratio) const {
    return Vec2<T>(this->x / ratio, this->y / ratio);
  }

  Vec2<T> operator-() const {
    return Vec2<T>(-this->x, -this->y);
  }

  Vec2<T> normalized() const {
    T ratio = norm();
    if(ratio==0){
      return Vec2<T>(0, 0);
    }else{
      return Vec2<T>(this->x / ratio, this->y / ratio);
    }
  }
};

template <class T>
Vec2<T> operator*(const double k, const Vec2<T>& v) {
  return v * k;
}

template <class T>
std::ostream& operator<<(std::ostream& os, const Vec2<T>& v) {
  os << v.x << "," << v.y;
  return os;
}

template <class T>
T d(const Vec2<T>& v1, const Vec2<T>& v2) {
  return (v1 - v2).norm();
}

template <class T>
Vec2<T> e(T theta) {
  return Vec2<T>(std::cos(theta), std::sin(theta));
}

typedef Vec2<double> Vec2d;
typedef Vec2<float> Vec2f;
typedef Vec2<int> Vec2i;
}  // namespace linear
