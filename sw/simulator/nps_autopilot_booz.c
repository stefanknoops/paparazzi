#include "nps_autopilot.h"

#include "booz2_main.h"
#include "nps_sensors.h"
#include "nps_radio_control.h"
#include "booz_radio_control.h"
#include "booz_imu.h"
#include "booz2_analog_baro.h"

#include "actuators.h"

#define BYPASS_AHRS

struct NpsAutopilot autopilot;

#ifdef BYPASS_AHRS
static void sim_overwrite_ahrs(void);
#endif


void nps_autopilot_init(void) {

  nps_radio_control_init();

  booz2_main_init();

}

#include <stdio.h>

void nps_autopilot_run_step(double time __attribute__ ((unused))) {
  
  if (nps_radio_control_available(time)) {
    booz_radio_control_feed();
    booz2_main_event();
  }

  if (nps_sensors_gyro_available()) {
    booz_imu_feed_gyro_accel();
    booz2_main_event();
  }

  if (nps_sensors_mag_available()) {
    booz_imu_feed_mag();
    booz2_main_event();
 }

  if (nps_sensors_baro_available()) {
    Booz2BaroISRHandler(sensors.baro.value);
    booz2_main_event();
  }
#ifdef BYPASS_AHRS
  sim_overwrite_ahrs();
#endif /* BYPASS_AHRS */

  booz2_main_periodic();

  /* 25 */
  if (time < 8) {
    //    double hover = 0.25;
    double hover = 0.2493;
    //   double hover = 0.23;
    //  double hover = 0.;
    //  if (time > 20) hover = 0.25;
    double yaw = 0.000000;
    double pitch = 0.000;
    double roll  = 0.0000;

    autopilot.commands[SERVO_FRONT] = hover + yaw + pitch;
    autopilot.commands[SERVO_BACK]  = hover + yaw - pitch;
    autopilot.commands[SERVO_RIGHT] = hover - yaw - roll ;
    autopilot.commands[SERVO_LEFT]  = hover - yaw + roll;
  }
  else {
    int32_t ut_front = Actuator(SERVO_FRONT) - TRIM_FRONT;
    int32_t ut_back  = Actuator(SERVO_BACK)  - TRIM_BACK;
    int32_t ut_right = Actuator(SERVO_RIGHT) - TRIM_RIGHT;
    int32_t ut_left  = Actuator(SERVO_LEFT)  - TRIM_LEFT;
    autopilot.commands[SERVO_FRONT] = (double)ut_front / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_BACK]  = (double)ut_back  / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_RIGHT] = (double)ut_right / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_LEFT]  = (double)ut_left  / SUPERVISION_MAX_MOTOR;
  }
  //  printf("%f %f %f %f\n", autopilot.commands[SERVO_FRONT], autopilot.commands[SERVO_BACK],
  //	 autopilot.commands[SERVO_RIGHT], autopilot.commands[SERVO_LEFT]);	 

}

#ifdef BYPASS_AHRS
#include "nps_fdm.h"
#include "math/pprz_algebra_int.h"
#include "booz_ahrs.h"
static void sim_overwrite_ahrs(void) {

  //  printf("%f\n", fdm.ltpprz_to_body_eulers.phi);

  booz_ahrs.ltp_to_body_euler.phi   = ANGLE_BFP_OF_REAL(fdm.ltpprz_to_body_eulers.phi);
  booz_ahrs.ltp_to_body_euler.theta = ANGLE_BFP_OF_REAL(fdm.ltpprz_to_body_eulers.theta);
  booz_ahrs.ltp_to_body_euler.psi   = ANGLE_BFP_OF_REAL(fdm.ltpprz_to_body_eulers.psi);

  booz_ahrs.ltp_to_body_quat.qi = QUAT1_BFP_OF_REAL(fdm.ltpprz_to_body_quat.qi);
  booz_ahrs.ltp_to_body_quat.qx = QUAT1_BFP_OF_REAL(fdm.ltpprz_to_body_quat.qx);
  booz_ahrs.ltp_to_body_quat.qy = QUAT1_BFP_OF_REAL(fdm.ltpprz_to_body_quat.qy);
  booz_ahrs.ltp_to_body_quat.qz = QUAT1_BFP_OF_REAL(fdm.ltpprz_to_body_quat.qz);

  booz_ahrs.body_rate.p = RATE_BFP_OF_REAL(fdm.body_ecef_rotvel.p);
  booz_ahrs.body_rate.q = RATE_BFP_OF_REAL(fdm.body_ecef_rotvel.q);
  booz_ahrs.body_rate.r = RATE_BFP_OF_REAL(fdm.body_ecef_rotvel.r);

}
#endif /* BYPASS_AHRS */

