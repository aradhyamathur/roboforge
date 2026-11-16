// A joint PID loop in C++: joint targets in, motor torques out.
//
// It does what the SDK's Python `joint_pd` controller does, line for line,
// so compare.py can check that the two give the same result. Use it as the
// starting point for a C++ controller of your own.
#include <roboforge/controller.hpp>

class ArmPID : public rf::Controller {
 public:
  // Gains may be one number for all joints or one per joint: [60, 80, 40].
  explicit ArmPID(const rf::Params& p)
      : kp_(p.numbers("kp", {60.0})), ki_(p.numbers("ki", {20.0})), kd_(p.numbers("kd", {6.0})),
        integral_limit_(p.number("integral_limit", 2.0)) {}

  void setup(const rf::Info& info) override {
    if (info.actuator_names.size() != info.joint_names.size())
      throw std::invalid_argument("ArmPID needs one motor per joint");
    n_ = static_cast<int>(info.joint_names.size());
    low_ = info.ctrl_low;
    high_ = info.ctrl_high;
    reset();
  }

  void reset() override {
    integral_.assign(n_, 0.0);
    target_.clear();
    have_t_ = false;
  }

  rf::Action step(const rf::Obs& obs) override {
    if (obs.joints() == 0) return rf::Action::none();
    // The target: the latest cmd/joint (listed under `inputs` in app.yaml), else hold where we started.
    if (obs.has("cmd/joint")) {
      rf::Vec t = obs.signal("cmd/joint");
      if (static_cast<int>(t.size()) != n_)
        throw std::invalid_argument("cmd/joint has " + std::to_string(t.size()) + " targets for " + std::to_string(n_) + " joints");
      target_ = t;
    }
    if (target_.empty()) target_ = obs.q();

    const double dt = have_t_ ? obs.t() - last_t_ : 0.0;
    last_t_ = obs.t();
    have_t_ = true;

    rf::Vec u(n_);
    for (int i = 0; i < n_; ++i) {
      const double error = target_[i] - obs.q(i);
      integral_[i] = rf::clamp(integral_[i] + error * dt, -integral_limit_, integral_limit_);
      u[i] = gain(kp_, i) * error + gain(ki_, i) * integral_[i] + gain(kd_, i) * -obs.qd(i);
      u[i] = rf::clamp(u[i], low_[i], high_[i]);
    }
    return rf::Action::actuators(u);
  }

 private:
  static double gain(const rf::Vec& g, int i) { return g.size() == 1 ? g[0] : g.at(i); }

  rf::Vec kp_, ki_, kd_;
  double integral_limit_;
  int n_ = 0;
  rf::Vec low_, high_, integral_, target_;
  double last_t_ = 0.0;
  bool have_t_ = false;
};

RF_EXPORT_CONTROLLER(ArmPID)
