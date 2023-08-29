#define SUBARU_STEERING_LIMITS_GENERATOR(steer_max, rate_up, rate_down,    \
                                         torque_factor, torque_allowance)  \
  {                                                                        \
    .max_steer = (steer_max),                                              \
    .max_rt_delta = 940,                                                   \
    .max_rt_interval = 250000,                                             \
    .max_rate_up = (rate_up),                                              \
    .max_rate_down = (rate_down),                                          \
    .driver_torque_factor = (torque_factor),                               \
    .driver_torque_allowance = (torque_allowance),                         \
    .type = TorqueDriverLimited,                                           \
  }

const LongitudinalLimits SUBARU_LONG_LIMITS = {
  .min_gas = 808,       // appears to be engine braking
  .max_gas = 3400,      // approx  2 m/s^2 when maxing cruise_rpm and cruise_throttle
  .inactive_gas = 1818, // this is zero acceleration
  .max_brake = 600,     // approx -3.5 m/s^2

  .min_transmission_rpm = 0,
  .max_transmission_rpm = 2400,
};

#define SUBARU_MAIN_BUS 0
#define SUBARU_ALT_BUS  1
#define SUBARU_CAM_BUS  2