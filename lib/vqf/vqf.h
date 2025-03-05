#ifndef VQF_H
#define VQF_H

#include <cmath>
#include <algorithm>

#define VQF_SINGLE_PRECISION
#define VQF_NO_MOTION_BIAS_ESTIMATION
#define VQF_VR_MODE

constexpr float VQF_PI = 3.14159265358979323846f;
constexpr float VQF_SQRT2 = 1.41421356237309504880f;
constexpr float VQF_EPS = 1.1920929e-07f;

typedef float vqf_real_t;

struct VQFParams {
	/**
	 * @brief Time constant \f$\tau_\mathrm{acc}\f$ for accelerometer low-pass filtering
	 * in seconds.
	 *
	 * Small values for \f$\tau_\mathrm{acc}\f$ imply trust on the accelerometer
	 * measurements and while large values of \f$\tau_\mathrm{acc}\f$ imply trust on the
	 * gyroscope measurements.
	 *
	 * The time constant \f$\tau_\mathrm{acc}\f$ corresponds to the cutoff frequency
	 * \f$f_\mathrm{c}\f$ of the second-order Butterworth low-pass filter as follows:
	 * \f$f_\mathrm{c} = \frac{\sqrt{2}}{2\pi\tau_\mathrm{acc}}\f$.
	 *
	 * Default value: 3.0 s
	 */
	vqf_real_t tauAcc = 3.0;
	/**
	 * @brief Time constant \f$\tau_\mathrm{mag}\f$ for magnetometer update in seconds.
	 *
	 * Small values for \f$\tau_\mathrm{mag}\f$ imply trust on the magnetometer
	 * measurements and while large values of \f$\tau_\mathrm{mag}\f$ imply trust on the
	 * gyroscope measurements.
	 *
	 * The time constant \f$\tau_\mathrm{mag}\f$ corresponds to the cutoff frequency
	 * \f$f_\mathrm{c}\f$ of the first-order low-pass filter for the heading correction
	 * as follows: \f$f_\mathrm{c} = \frac{1}{2\pi\tau_\mathrm{mag}}\f$.
	 *
	 * Default value: 9.0 s
	 */
	vqf_real_t tauMag = 9.0;

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
	/**
	 * @brief Enables gyroscope bias estimation during motion phases.
	 *
	 * If set to true (default), gyroscope bias is estimated based on the inclination
	 * correction only, i.e. without using magnetometer measurements.
	 */
	bool motionBiasEstEnabled = true;
#endif
	/**
	 * @brief Enables rest detection and gyroscope bias estimation during rest phases.
	 *
	 * If set to true (default), phases in which the IMU is at rest are detected. During
	 * rest, the gyroscope bias is estimated from the low-pass filtered gyroscope
	 * readings.
	 */
	bool restBiasEstEnabled = true;
	/**
	 * @brief Enables magnetic disturbance detection and magnetic disturbance rejection.
	 *
	 * If set to true (default), the magnetic field is analyzed. For short disturbed
	 * phases, the magnetometer-based correction is disabled totally. If the magnetic
	 * field is always regarded as disturbed or if the duration of the disturbances
	 * exceeds #magMaxRejectionTime, magnetometer-based updates are performed, but with
	 * an increased time constant.
	 */
	bool magDistRejectionEnabled = true;

	/**
	 * @brief Standard deviation of the initial bias estimation uncertainty (in degrees
	 * per second).
	 *
	 * Default value: 0.5 °/s
	 */
	vqf_real_t biasSigmaInit = 0.5;
	/**
	 * @brief Time in which the bias estimation uncertainty increases from 0 °/s to 0.1
	 * °/s (in seconds).
	 *
	 * This value determines the system noise assumed by the Kalman filter.
	 *
	 * Default value: 100.0 s
	 */
	vqf_real_t biasForgettingTime = 100.0;
	/**
	 * @brief Maximum expected gyroscope bias (in degrees per second).
	 *
	 * This value is used to clip the bias estimate and the measurement error in the
	 * bias estimation update step. It is further used by the rest detection algorithm
	 * in order to not regard measurements with a large but constant angular rate as
	 * rest.
	 *
	 * Default value: 2.0 °/s
	 */
	vqf_real_t biasClip = 2.0;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
	/**
	 * @brief Standard deviation of the converged bias estimation uncertainty during
	 * motion (in degrees per second).
	 *
	 * This value determines the trust on motion bias estimation updates. A small value
	 * leads to fast convergence.
	 *
	 * Default value: 0.1 °/s
	 */
	vqf_real_t biasSigmaMotion = 0.1;
	/**
	 * @brief Forgetting factor for unobservable bias in vertical direction during
	 * motion.
	 *
	 * As magnetometer measurements are deliberately not used during motion bias
	 * estimation, gyroscope bias is not observable in vertical direction. This value is
	 * the relative weight of an artificial zero measurement that ensures that the bias
	 * estimate in the unobservable direction will eventually decay to zero.
	 *
	 * Default value: 0.0001
	 */
	vqf_real_t biasVerticalForgettingFactor = 0.0001;
#endif
	/**
	 * @brief Standard deviation of the converged bias estimation uncertainty during
	 * rest (in degrees per second).
	 *
	 * This value determines the trust on rest bias estimation updates. A small value
	 * leads to fast convergence.
	 *
	 * Default value: 0.03 °
	 */
	vqf_real_t biasSigmaRest = 0.03;

	/**
	 * @brief Time threshold for rest detection (in seconds).
	 *
	 * Rest is detected when the measurements have been close to the low-pass filtered
	 * reference for the given time.
	 *
	 * Default value: 1.5 s
	 */
	vqf_real_t restMinT = 1.5;
	/**
	 * @brief Time constant for the low-pass filter used in rest detection (in seconds).
	 *
	 * This time constant characterizes a second-order Butterworth low-pass filter used
	 * to obtain the reference for rest detection.
	 *
	 * Default value: 0.5 s
	 */
	vqf_real_t restFilterTau = 0.5;
	/**
	 * @brief Angular velocity threshold for rest detection (in °/s).
	 *
	 * For rest to be detected, the norm of the deviation between measurement and
	 * reference must be below the given threshold. (Furthermore, the absolute value of
	 * each component must be below #biasClip).
	 *
	 * Default value: 2.0 °/s
	 */
	vqf_real_t restThGyr = 2.0;
	/**
	 * @brief Acceleration threshold for rest detection (in m/s²).
	 *
	 * For rest to be detected, the norm of the deviation between measurement and
	 * reference must be below the given threshold.
	 *
	 * Default value: 0.5 m/s²
	 */
	vqf_real_t restThAcc = 0.5;

	/**
	 * @brief Time constant for current norm/dip value in magnetic disturbance detection
	 * (in seconds).
	 *
	 * This (very fast) low-pass filter is intended to provide additional robustness
	 * when the magnetometer measurements are noisy or not sampled perfectly in sync
	 * with the gyroscope measurements. Set to -1 to disable the low-pass filter and
	 * directly use the magnetometer measurements.
	 *
	 * Default value: 0.05 s
	 */
	vqf_real_t magCurrentTau = 0.05;
	/**
	 * @brief Time constant for the adjustment of the magnetic field reference (in
	 * seconds).
	 *
	 * This adjustment allows the reference estimate to converge to the observed
	 * undisturbed field.
	 *
	 * Default value: 20.0 s
	 */
	vqf_real_t magRefTau = 20.0;
	/**
	 * @brief Relative threshold for the magnetic field strength for magnetic
	 * disturbance detection.
	 *
	 * This value is relative to the reference norm.
	 *
	 * Default value: 0.1 (10%)
	 */
	vqf_real_t magNormTh = 0.1;
	/**
	 * @brief Threshold for the magnetic field dip angle for magnetic disturbance
	 * detection (in degrees).
	 *
	 * Default vaule: 10 °
	 */
	vqf_real_t magDipTh = 10.0;
	/**
	 * @brief Duration after which to accept a different homogeneous magnetic field (in
	 * seconds).
	 *
	 * A different magnetic field reference is accepted as the new field when the
	 * measurements are within the thresholds #magNormTh and #magDipTh for the given
	 * time. Additionally, only phases with sufficient movement, specified by
	 * #magNewMinGyr, count.
	 *
	 * Default value: 20.0
	 */
	vqf_real_t magNewTime = 20.0;
	/**
	 * @brief Duration after which to accept a homogeneous magnetic field for the first
	 * time (in seconds).
	 *
	 * This value is used instead of #magNewTime when there is no current estimate in
	 * order to allow for the initial magnetic field reference to be obtained faster.
	 *
	 * Default value: 5.0
	 */
	vqf_real_t magNewFirstTime = 5.0;
	/**
	 * @brief Minimum angular velocity needed in order to count time for new magnetic
	 * field acceptance (in °/s).
	 *
	 * Durations for which the angular velocity norm is below this threshold do not
	 * count towards reaching #magNewTime.
	 *
	 * Default value: 20.0 °/s
	 */
	vqf_real_t magNewMinGyr = 20.0;
	/**
	 * @brief Minimum duration within thresholds after which to regard the field as
	 * undisturbed again (in seconds).
	 *
	 * Default value: 0.5 s
	 */
	vqf_real_t magMinUndisturbedTime = 0.5;
	/**
	 * @brief Maximum duration of full magnetic disturbance rejection (in seconds).
	 *
	 * For magnetic disturbances up to this duration, heading correction is fully
	 * disabled and heading changes are tracked by gyroscope only. After this duration
	 * (or for many small disturbed phases without sufficient time in the undisturbed
	 * field in between), the heading correction is performed with an increased time
	 * constant (see #magRejectionFactor).
	 *
	 * Default value: 60.0 s
	 */
	vqf_real_t magMaxRejectionTime = 60.0;
	/**
	 * @brief Factor by which to slow the heading correction during long disturbed
	 * phases.
	 *
	 * After #magMaxRejectionTime of full magnetic disturbance rejection, heading
	 * correction is performed with an increased time constant. This parameter
	 * (approximately) specifies the factor of the increase.
	 *
	 * Furthermore, after spending #magMaxRejectionTime/#magRejectionFactor seconds in
	 * an undisturbed magnetic field, the time is reset and full magnetic disturbance
	 * rejection will be performed for up to #magMaxRejectionTime again.
	 *
	 * Default value: 2.0
	 */
	vqf_real_t magRejectionFactor = 2.0;

    // VR parameters
    vqf_real_t maxAngularRate;
    vqf_real_t vrStabilityThreshold;

    // Filter parameters
    vqf_real_t accLpB[3]{0.0f, 0.0f, 0.0f};  // Accelerometer low-pass filter coefficients
    vqf_real_t magLpB[3]{0.0f, 0.0f, 0.0f};  // Magnetometer low-pass filter coefficients
};

//struct VQFParams {
//    VQFParams();
//
//    // Core parameters
//    vqf_real_t tauAcc;
//    vqf_real_t tauMag;
//    bool restBiasEstEnabled;
//    bool magDistRejectionEnabled;
//    vqf_real_t biasSigmaInit;
//    vqf_real_t biasForgettingTime;
//    vqf_real_t biasClip;
//    vqf_real_t biasSigmaRest;
//    vqf_real_t restMinT;
//    vqf_real_t restFilterTau;
//    vqf_real_t restThGyr;
//    vqf_real_t restThAcc;
//    vqf_real_t magCurrentTau;
//    vqf_real_t magRefTau;
//    vqf_real_t magNormTh;
//    vqf_real_t magDipTh;
//    vqf_real_t magNewTime;
//    vqf_real_t magNewFirstTime;
//    vqf_real_t magNewMinGyr;
//    vqf_real_t magMinUndisturbedTime;
//    vqf_real_t magMaxRejectionTime;
//    vqf_real_t magRejectionFactor;
//
//    // VR parameters
//    vqf_real_t maxAngularRate;
//    vqf_real_t vrStabilityThreshold;
//
//    // Filter parameters
//    vqf_real_t accLpB[3]{0.0f, 0.0f, 0.0f};  // Accelerometer low-pass filter coefficients
//    vqf_real_t magLpB[3]{0.0f, 0.0f, 0.0f};  // Magnetometer low-pass filter coefficients
//};

struct VQFState {
    // Orientation state
    vqf_real_t gyrQuat[4];
    vqf_real_t accQuat[4];
    vqf_real_t delta;

    // System state
    bool restDetected;
    bool magDistDetected;
    bool poseStable;
    vqf_real_t motionEnergy;

    // Filter states
    vqf_real_t lastAccLp[3];
    vqf_real_t accLpState[3*2];
    vqf_real_t lastAccCorrAngularRate;
    vqf_real_t kMagInit;
    vqf_real_t lastMagDisAngle;
    vqf_real_t lastMagCorrAngularRate;
    vqf_real_t bias[3];
    vqf_real_t biasP;

    // Rest detection
    vqf_real_t restLastSquaredDeviations[2];
    vqf_real_t restT;
    vqf_real_t restLastGyrLp[3];
    vqf_real_t restGyrLpState[3*2];
    vqf_real_t restLastAccLp[3];
    vqf_real_t restAccLpState[3*2];

    // Magnetometer state
    vqf_real_t magRefNorm;
    vqf_real_t magRefDip;
    vqf_real_t magUndisturbedT;
    vqf_real_t magRejectT;
    vqf_real_t magCandidateNorm;
    vqf_real_t magCandidateDip;
    vqf_real_t magCandidateT;
    vqf_real_t magNormDip[2];
    vqf_real_t magNormDipLpState[2*2];

    // VR tracking
    vqf_real_t vrCalibrationTimer;
};

class VQF {
public:
    // Default sampling times in seconds
    static constexpr float DEFAULT_GYRO_SAMPLING_TIME = 1.0f/400.0f;  // 400 Hz
    static constexpr float DEFAULT_ACC_SAMPLING_TIME = 1.0f/100.0f;   // 100 Hz
    static constexpr float DEFAULT_MAG_SAMPLING_TIME = 1.0f/100.0f;   // 100 Hz

    VQF(vqf_real_t gyrTs = DEFAULT_GYRO_SAMPLING_TIME, vqf_real_t accTs = -1.0f, vqf_real_t magTs = -1.0f);
    VQF(const VQFParams& params, vqf_real_t gyrTs = DEFAULT_GYRO_SAMPLING_TIME, vqf_real_t accTs = -1.0f, vqf_real_t magTs = -1.0f);

    void updateGyr(const vqf_real_t gyr[3], vqf_real_t gyrTs);
    void updateAcc(const vqf_real_t acc[3]);
    void updateMag(const vqf_real_t mag[3]);

    // VR extensions
    void vrInitSequence(float duration = 5.0f);
    bool getPoseStability() const;
    void getPredictedPose(vqf_real_t out[4], float predictionTime = 0.03f) const;

    // Configuration
    void setTauAcc(vqf_real_t tauAcc);
    void setTauMag(vqf_real_t tauMag);
    void resetState();

    // State access
    void getQuat6D(vqf_real_t out[4]) const;
    void getQuat9D(vqf_real_t out[4]) const;
    vqf_real_t getDelta() const;
    vqf_real_t getBiasEstimate(vqf_real_t out[3]) const;
    bool getRestDetected() const;
    bool getMagDistDetected() const;

private:
    VQFParams params;
    VQFState state;

    struct Coefficients {
        vqf_real_t gyrTs, accTs, magTs;
        vqf_real_t accLpB[3], accLpA[2];
        vqf_real_t kMag;
        vqf_real_t biasP0, biasV, biasRestW;
        vqf_real_t restGyrLpB[3], restGyrLpA[2];
        vqf_real_t restAccLpB[3], restAccLpA[2];
        vqf_real_t kMagRef;
        vqf_real_t magNormDipLpB[3], magNormDipLpA[2];
    } coeffs;

    void setup();
    void handleVrMotionAnomaly();
    void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts,
                   const vqf_real_t b[3], const vqf_real_t a[2], vqf_real_t state[], vqf_real_t out[]);
    static void quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]);
    static void quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3]);
    static void quatApplyDelta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4]);
    static vqf_real_t norm(const vqf_real_t vec[], size_t N);
    static void normalize(vqf_real_t vec[], size_t N);
    static vqf_real_t gainFromTau(vqf_real_t tau, vqf_real_t Ts);
    static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_real_t outB[3], vqf_real_t outA[2]);
};

#endif // VQF_H
