#ifndef INCLUDE_TIMER_HH
#define INCLUDE_TIMER_HH

#include <chrono>

class Timer {
public:
  Timer(bool flag = true) {
    this->autostart = flag;
    if (autostart)
      this->start_point = std::chrono::steady_clock::now();
  }
  ~Timer() = default;

  void Begin() { this->start_point = std::chrono::steady_clock::now(); }

  void End() {
    this->autostart = false;
    this->end_point = std::chrono::steady_clock::now(); }

  double Get() {
    if(this->autostart)
      this->end_point = std::chrono::steady_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        this->end_point - this->start_point);
    return time.count();
  }

private:
  std::chrono::steady_clock::time_point start_point;
  std::chrono::steady_clock::time_point end_point;
  bool autostart;
};

#endif // INCLUDE_TIMER_HH