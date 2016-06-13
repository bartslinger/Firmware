#define Bound(_v, _min, _max) {        \
    (_v).p = (_v).p < (_min) ? (_min) : (_v).p > (_max) ? (_max) : (_v).p;  \
    (_v).q = (_v).q < (_min) ? (_min) : (_v).q > (_max) ? (_max) : (_v).q;  \
    (_v).r = (_v).r < (_min) ? (_min) : (_v).r > (_max) ? (_max) : (_v).r;  \
  }

/* ra =  {p, q, r} */
#define RATES_ASSIGN(_ra, _p, _q, _r) {   \
    (_ra).p = (_p);       \
    (_ra).q = (_q);       \
    (_ra).r = (_r);       \
  }

#define FLOAT_RATES_ZERO(_r) {          \
    RATES_ASSIGN(_r, 0., 0., 0.);       \
  }

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC

struct FloatRates {
    float p;
    float q;
    float r;
};

struct ReferenceSystem {
  float err_p;
  float err_q;
  float err_r;
  float rate_p;
  float rate_q;
  float rate_r;
};

struct IndiFilter {
  struct FloatRates ddx;
  struct FloatRates dx;
  struct FloatRates x;

  float zeta;
  float omega;
  float omega_r;
  float omega2;
  float omega2_r;
};

struct IndiEstimation {
  struct IndiFilter u;
  struct IndiFilter rate;
  struct FloatRates g1;
  float g2;
  float mu;
};

struct IndiVariables {
  struct FloatRates angular_accel_ref;
  struct FloatRates du;
  struct FloatRates u_in;
  struct FloatRates u_act_dyn;

  struct IndiFilter u;
  struct IndiFilter rate;
  struct FloatRates g1;
  float g2;

  struct ReferenceSystem reference_acceleration;

  bool adaptive;             ///< Enable adataptive estimation
  float max_rate;            ///< Maximum rate in rate control in rad/s
  float attitude_max_yaw_rate; ///< Maximum yaw rate in atttiude control in rad/s
  struct IndiEstimation est; ///< Estimation parameters for adaptive INDI
};

/*
 * Originally 0.04 for 512 Hz
 * More general:
 * tau = 0.0469
 *
 * alpha = dt / (tau + dt)
 *
 * (which gives 0.04 for dt=1/512)
 *
 */
#define STABILIZATION_INDI_ACT_DYN_P 0.0469f
#define STABILIZATION_INDI_ACT_DYN_Q 0.0469f
#define STABILIZATION_INDI_ACT_DYN_R 0.0469f

#define STABILIZATION_ATTITUDE_SP_MAX_R 350

#if !defined(STABILIZATION_INDI_ACT_DYN_P) && !defined(STABILIZATION_INDI_ACT_DYN_Q) && !defined(STABILIZATION_INDI_ACT_DYN_R)
#error You have to define the first order time constant of the actuator dynamics!
#endif

// these parameters are used in the filtering of the angular acceleration
// define them in the airframe file if different values are required
#ifndef STABILIZATION_INDI_FILT_OMEGA
#define STABILIZATION_INDI_FILT_OMEGA 20.0
#endif

#ifndef STABILIZATION_INDI_FILT_ZETA
#define STABILIZATION_INDI_FILT_ZETA 0.55
#endif

// the yaw sometimes requires more filtering
#ifndef STABILIZATION_INDI_FILT_OMEGA_R
#define STABILIZATION_INDI_FILT_OMEGA_R STABILIZATION_INDI_FILT_OMEGA
#endif

#ifndef STABILIZATION_INDI_FILT_ZETA_R
#define STABILIZATION_INDI_FILT_ZETA_R STABILIZATION_INDI_FILT_ZETA
#endif

#ifndef STABILIZATION_INDI_MAX_RATE
#define STABILIZATION_INDI_MAX_RATE 6.0
#endif

#if STABILIZATION_INDI_USE_ADAPTIVE
#warning "Use caution with adaptive indi. See the wiki for more info"
#endif

#ifndef STABILIZATION_INDI_MAX_R
#define STABILIZATION_INDI_MAX_R STABILIZATION_ATTITUDE_SP_MAX_R
#endif

#define STABILIZATION_INDI_REF_ERR_P 600
#define STABILIZATION_INDI_REF_ERR_Q 600
#define STABILIZATION_INDI_REF_ERR_R 600
#define STABILIZATION_INDI_REF_RATE_P 28
#define STABILIZATION_INDI_REF_RATE_Q 28
#define STABILIZATION_INDI_REF_RATE_R 28
#define STABILIZATION_INDI_ADAPTIVE_MU 0.0001


#define INDI_EST_SCALE 0.001 //The G values are scaled to avoid numerical problems during the estimation

/** in place first order integration of angular rates */
static inline
void float_rates_integrate_fi(struct FloatRates *r, struct FloatRates *dr, float dt)
{
  r->p += dr->p * dt;
  r->q += dr->q * dt;
  r->r += dr->r * dt;
}
