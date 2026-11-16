// RoboForge SDK: controllers in C++.
//
// Write a class, export it with RF_EXPORT_CONTROLLER, compile it to a shared
// library, and name it in app.yaml:
//
//     #include <roboforge/controller.hpp>
//
//     class Hold : public rf::Controller {
//      public:
//       explicit Hold(const rf::Params& p) : kp_(p.number("kp", 40.0)) {}
//       rf::Action step(const rf::Obs& obs) override {
//         std::vector<double> u(obs.joints());
//         for (int i = 0; i < obs.joints(); ++i) u[i] = -kp_ * obs.q(i);
//         return rf::Action::actuators(u);
//       }
//      private:
//       double kp_;
//     };
//     RF_EXPORT_CONTROLLER(Hold)
//
//     controllers:
//       - {type: cpp, source: hold.cpp, rate: 500, kp: 60}     # built for you on first run
//
// The controller sees what a Python controller sees (joints, base pose and
// velocity, IMU, vehicle status, and any signal listed in `inputs`) and
// returns one command. It runs in the app's own process at its own rate; the
// SDK does the bus, the links, the recording and everything else.
//
// Header only, C++17, no dependencies. Exceptions thrown from the
// constructor, setup() or step() are caught at the boundary and reported in
// Python with the controller's name and the exception's message.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#define RF_ABI_VERSION 1

// ---------------------------------------------------------------------------
// The C interface between Python and the library. Plain structs, so it does not
// depend on the compiler or standard library the controller was built with.
extern "C" {
typedef struct {
  const char* name;
  int n;                 // number of values; 0 for a text parameter
  const double* values;
  const char* text;      // the value of a text parameter, else NULL
} rf_param;

typedef struct {
  const char* name;
  int n;
  const double* values;
} rf_signal;

typedef struct {
  double t;                                  // app time, s
  int n_joints;
  const double* q;                           // joint positions (rad or m)
  const double* qd;                          // joint velocities
  const double* effort;                      // joint efforts
  int has_pose;
  double position[3];                        // base position, world ENU, m
  double orientation[4];                     // base orientation, quaternion w x y z
  double linear[3];                          // base velocity, world frame, m/s
  double angular[3];                         // base angular velocity, body frame, rad/s
  int has_imu;
  double accel[3];                           // specific force, body FLU, m/s^2
  double gyro[3];                            // rad/s
  int has_status;
  int armed;
  int n_signals;
  const rf_signal* signals;                  // the signals named in `inputs`, those published so far
} rf_obs;

enum { RF_NONE = 0, RF_ACTUATORS = 1, RF_JOINTS = 2, RF_VELOCITY = 3, RF_SETPOINT = 4 };
#define RF_MAX_ACTION 256

typedef struct {
  int kind;
  int n;
  double values[RF_MAX_ACTION];
} rf_action;

typedef struct {
  double dt;                                 // physics step, s
  int n_joints;
  const char* const* joint_names;
  int n_actuators;
  const char* const* actuator_names;
  const double* ctrl_low;                    // actuator limits (+-inf when unlimited)
  const double* ctrl_high;
} rf_info;
}

namespace rf {

using Vec = std::vector<double>;

// ---------------------------------------------------------------------------
// Parameters: every key of the controller's line in app.yaml other than
// type, source, library, rate, name and inputs. Reading one marks it as
// used; a key the constructor never reads is reported as a likely typo.
class Params {
 public:
  Params(int n, const rf_param* p) : p_(p, p + n) {}

  bool has(const std::string& name) const { return find(name) != nullptr; }

  double number(const std::string& name, double fallback) const {
    const rf_param* p = find(name);
    if (!p) return fallback;
    if (p->n != 1) throw std::invalid_argument("parameter '" + name + "' must be one number");
    return p->values[0];
  }
  double number(const std::string& name) const {
    require(name);
    return number(name, 0.0);
  }
  // A list of numbers. A single number is a list of one.
  Vec numbers(const std::string& name, const Vec& fallback = {}) const {
    const rf_param* p = find(name);
    if (!p) return fallback;
    if (p->text) throw std::invalid_argument("parameter '" + name + "' must be numbers, got text");
    return Vec(p->values, p->values + p->n);
  }
  std::string text(const std::string& name, const std::string& fallback = "") const {
    const rf_param* p = find(name);
    if (!p) return fallback;
    if (!p->text) throw std::invalid_argument("parameter '" + name + "' must be text");
    return p->text;
  }

  std::vector<std::string> unused() const {
    std::vector<std::string> out;
    for (const auto& p : p_)
      if (!read_.count(p.name)) out.push_back(p.name);
    return out;
  }
  std::vector<std::string> read() const { return {read_.begin(), read_.end()}; }

 private:
  const rf_param* find(const std::string& name) const {
    read_.insert(name);
    for (const auto& p : p_)
      if (name == p.name) return &p;
    return nullptr;
  }
  void require(const std::string& name) const {
    if (!find(name)) throw std::invalid_argument("parameter '" + name + "' is required");
  }
  std::vector<rf_param> p_;
  mutable std::set<std::string> read_;
};

// ---------------------------------------------------------------------------
// What the controller sees. Cheap to pass: it only points at the data.
class Obs {
 public:
  explicit Obs(const rf_obs* o) : o_(o) {}

  double t() const { return o_->t; }

  int joints() const { return o_->n_joints; }
  double q(int i) const { return o_->q[check(i)]; }
  double qd(int i) const { return o_->qd[check(i)]; }
  double effort(int i) const { return o_->effort[check(i)]; }
  Vec q() const { return Vec(o_->q, o_->q + o_->n_joints); }
  Vec qd() const { return Vec(o_->qd, o_->qd + o_->n_joints); }

  bool has_pose() const { return o_->has_pose != 0; }
  const double* position() const { return o_->position; }          // [3]
  const double* orientation() const { return o_->orientation; }    // [4] w x y z
  const double* linear() const { return o_->linear; }              // [3]
  const double* angular() const { return o_->angular; }            // [3]
  double yaw() const {
    const double* q = o_->orientation;
    return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
  }

  bool has_imu() const { return o_->has_imu != 0; }
  const double* accel() const { return o_->accel; }
  const double* gyro() const { return o_->gyro; }

  bool has_status() const { return o_->has_status != 0; }
  bool armed() const { return o_->armed != 0; }

  // A signal named in `inputs` (cmd/joint gives the target positions, task/goal
  // whatever was published there). has() is false until it is first published.
  bool has(const std::string& name) const { return find(name) != nullptr; }
  Vec signal(const std::string& name) const {
    const rf_signal* s = find(name);
    if (!s)
      throw std::out_of_range("no signal '" + name + "' yet (is it listed in `inputs`, and has anything published it?)");
    return Vec(s->values, s->values + s->n);
  }
  Vec signal(const std::string& name, const Vec& fallback) const { return has(name) ? signal(name) : fallback; }

 private:
  int check(int i) const {
    if (i < 0 || i >= o_->n_joints)
      throw std::out_of_range("joint index " + std::to_string(i) + " out of range (the robot has " + std::to_string(o_->n_joints) + ")");
    return i;
  }
  const rf_signal* find(const std::string& name) const {
    for (int i = 0; i < o_->n_signals; ++i)
      if (name == o_->signals[i].name) return &o_->signals[i];
    return nullptr;
  }
  const rf_obs* o_;
};

// ---------------------------------------------------------------------------
// What the controller returns: one command, or none.
struct Action {
  int kind = RF_NONE;
  Vec values;

  static Action none() { return {}; }
  // One value per motor, in the order of Info::actuator_names (cmd/actuators).
  static Action actuators(const Vec& u) { return {RF_ACTUATORS, u}; }
  // Joint position targets for a joint controller after this one (cmd/joint).
  static Action joints(const Vec& q) { return {RF_JOINTS, q}; }
  // Body-frame velocity (cmd/velocity): forward, sideways, up, then roll, pitch, yaw rate.
  static Action velocity(double forward, double yaw_rate) { return {RF_VELOCITY, {forward, 0, 0, 0, 0, yaw_rate}}; }
  static Action velocity(const Vec& linear3, const Vec& angular3) {
    Vec v(linear3);
    v.insert(v.end(), angular3.begin(), angular3.end());
    return {RF_VELOCITY, v};
  }
  // A world position and heading for a multirotor controller after this one (cmd/setpoint).
  static Action setpoint(double x, double y, double z, double yaw = 0.0) { return {RF_SETPOINT, {x, y, z, yaw}}; }
};

// What the controller is controlling, given once before the first step.
struct Info {
  double dt = 0;
  std::vector<std::string> joint_names, actuator_names;
  Vec ctrl_low, ctrl_high;
};

class Controller {
 public:
  virtual ~Controller() = default;
  virtual void setup(const Info&) {}
  virtual Action step(const Obs& obs) = 0;
  virtual void reset() {}
};

// A PID loop on one value, the same as the SDK's rf.PID.
class PID {
 public:
  PID(double kp = 1, double ki = 0, double kd = 0, double integral_limit = INFINITY)
      : kp(kp), ki(ki), kd(kd), integral_limit(integral_limit) {}
  // `rate` is the measured derivative of the error, which avoids differentiating it.
  double operator()(double error, double dt, double rate) {
    integral += error * dt;
    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;
    return kp * error + ki * integral + kd * rate;
  }
  void reset() { integral = 0; }
  double kp, ki, kd, integral_limit, integral = 0;
};

inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

namespace detail {
struct Holder {
  Controller* c = nullptr;
  std::string unused, read;
};
inline void copy_error(char* err, int len, const std::string& what) {
  if (!err || len <= 0) return;
  std::strncpy(err, what.c_str(), static_cast<std::size_t>(len - 1));
  err[len - 1] = '\0';
}
inline std::string join(const std::vector<std::string>& v) {
  std::string s;
  for (std::size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + v[i];
  return s;
}
}  // namespace detail
}  // namespace rf

// Export one controller class from this library. `Class` needs a constructor
// taking `const rf::Params&` and a `step(const rf::Obs&)` returning rf::Action.
#define RF_EXPORT_CONTROLLER(Class)                                                                  \
  extern "C" {                                                                                       \
  int rf_abi_version() { return RF_ABI_VERSION; }                                                    \
  const char* rf_class_name() { return #Class; }                                                     \
  void* rf_create(int n, const rf_param* p, char* err, int len) {                                    \
    try {                                                                                            \
      rf::Params params(n, p);                                                                       \
      auto* h = new rf::detail::Holder();                                                            \
      h->c = new Class(params);                                                                      \
      h->unused = rf::detail::join(params.unused());                                                 \
      h->read = rf::detail::join(params.read());                                                     \
      return h;                                                                                      \
    } catch (const std::exception& e) {                                                              \
      rf::detail::copy_error(err, len, e.what());                                                    \
    } catch (...) {                                                                                  \
      rf::detail::copy_error(err, len, "unknown exception");                                         \
    }                                                                                                \
    return nullptr;                                                                                  \
  }                                                                                                  \
  const char* rf_unused_params(void* h) { return static_cast<rf::detail::Holder*>(h)->unused.c_str(); } \
  const char* rf_read_params(void* h) { return static_cast<rf::detail::Holder*>(h)->read.c_str(); }  \
  int rf_setup(void* h, const rf_info* in, char* err, int len) {                                     \
    try {                                                                                            \
      rf::Info info;                                                                                 \
      info.dt = in->dt;                                                                              \
      for (int i = 0; i < in->n_joints; ++i) info.joint_names.push_back(in->joint_names[i]);         \
      for (int i = 0; i < in->n_actuators; ++i) {                                                    \
        info.actuator_names.push_back(in->actuator_names[i]);                                        \
        info.ctrl_low.push_back(in->ctrl_low[i]);                                                    \
        info.ctrl_high.push_back(in->ctrl_high[i]);                                                  \
      }                                                                                              \
      static_cast<rf::detail::Holder*>(h)->c->setup(info);                                           \
      return 0;                                                                                      \
    } catch (const std::exception& e) {                                                              \
      rf::detail::copy_error(err, len, e.what());                                                    \
    } catch (...) {                                                                                  \
      rf::detail::copy_error(err, len, "unknown exception");                                         \
    }                                                                                                \
    return -1;                                                                                       \
  }                                                                                                  \
  int rf_step(void* h, const rf_obs* o, rf_action* out, char* err, int len) {                        \
    try {                                                                                            \
      rf::Action a = static_cast<rf::detail::Holder*>(h)->c->step(rf::Obs(o));                       \
      if (a.values.size() > RF_MAX_ACTION) {                                                         \
        rf::detail::copy_error(err, len, "the action has more than 256 values");                     \
        return -1;                                                                                   \
      }                                                                                              \
      out->kind = a.kind;                                                                            \
      out->n = static_cast<int>(a.values.size());                                                    \
      for (std::size_t i = 0; i < a.values.size(); ++i) out->values[i] = a.values[i];                \
      return 0;                                                                                      \
    } catch (const std::exception& e) {                                                              \
      rf::detail::copy_error(err, len, e.what());                                                    \
    } catch (...) {                                                                                  \
      rf::detail::copy_error(err, len, "unknown exception");                                         \
    }                                                                                                \
    return -1;                                                                                       \
  }                                                                                                  \
  void rf_reset(void* h) { static_cast<rf::detail::Holder*>(h)->c->reset(); }                        \
  void rf_destroy(void* h) {                                                                         \
    auto* holder = static_cast<rf::detail::Holder*>(h);                                              \
    delete holder->c;                                                                                \
    delete holder;                                                                                   \
  }                                                                                                  \
  }
