/**
 * @file demo_api.h
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief C API exposing the processing pipeline stages for Python ctypes.
 *
 * Each function corresponds to one pipeline stage. Python pre-allocates output
 * arrays (numpy) and passes raw pointers; no heap allocation crosses the
 * boundary.
 *
 * All array arguments are row-major (C order). Sizes:
 *   accel / gyro / mag   n x 3
 *   quaternion           n x 4  (w, x, y, z)
 *   accel_global         n x 3  (x, y, z)
 *   fft output           nperseg x 2  (re, im interleaved, or separate arrays)
 *   mag_sq output        nperseg/2 + 1
 *
 * @date 2026-05-15
 */

#ifndef DEMO_API_H
#define DEMO_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return the FFT segment length this library was compiled with.
 *
 * Equals SMARTFIN_WELCH_NPERSEG (default 256). Python uses this to allocate
 * the correct output buffer sizes for sf_demo_fft and sf_demo_real_dft_mag_sq.
 *
 * @return CFG::welch_nperseg.
 */
int sf_demo_nperseg(void);

/**
 * @brief Orient raw IMU samples into the Earth frame via Madgwick AHRS.
 *
 * Mirrors Processor::orient_from_imu. Some samples may be dropped by the
 * filter (e.g. during the startup ramp or when dt is too large).
 *
 * @param elapsed_ms      Input elapsed timestamps in ms, length n_in.
 * @param accel_ms2       Body-frame acceleration in m/s^2, shape (n_in, 3).
 * @param gyro_dps        Body-frame rotation rate in deg/s, shape (n_in, 3).
 * @param mag_uT          Body-frame magnetic field in uT, shape (n_in, 3).
 * @param n_in            Number of input samples.
 * @param out_elapsed_ms  Output timestamps in ms, pre-allocated length n_in.
 * @param out_q           Output quaternions (w,x,y,z), pre-allocated (n_in,4).
 * @param out_accel       Output Earth-frame acceleration in m/s^2, (n_in,3).
 * @param out_accel_used  Number of output samples that used accel feedback.
 * @param out_mag_used    Number of output samples that used mag feedback.
 * @param out_bias_used   Number of output samples that updated gyro bias.
 * @return                Number of output samples written (<= n_in).
 */
int sf_demo_orient(
    const uint32_t *elapsed_ms,
    const float    *accel_ms2,
    const float    *gyro_dps,
    const float    *mag_uT,
    int             n_in,
    uint32_t       *out_elapsed_ms,
    double         *out_q,
    double         *out_accel,
    int            *out_accel_used,
    int            *out_mag_used,
    int            *out_bias_used
);

/**
 * @brief Decimate oriented samples by the fixed Decimator factor.
 *
 * Mirrors Processor::decimate. Uses three independent Decimator instances,
 * one per acceleration axis.
 *
 * @param elapsed_ms      Input timestamps in ms, length n_in.
 * @param accel           Earth-frame acceleration in m/s^2, shape (n_in, 3).
 * @param n_in            Number of input samples.
 * @param out_elapsed_ms  Output timestamps in ms, pre-allocated length n_in.
 * @param out_accel       Output acceleration in m/s^2, pre-allocated (n_in,3).
 * @return                Number of output samples written.
 */
int sf_demo_decimate(
    const uint32_t *elapsed_ms,
    const double   *accel,
    int             n_in,
    uint32_t       *out_elapsed_ms,
    double         *out_accel
);

/**
 * @brief Apply zero-phase bandpass filter in place.
 *
 * Mirrors Processor::bandpass: fourth-order Butterworth highpass at 0.05 Hz
 * then lowpass at 0.50 Hz, applied with filtfilt on each axis.
 *
 * @param elapsed_ms  Timestamps in ms, length n (used to estimate sample rate).
 * @param accel       Earth-frame acceleration in m/s^2, shape (n, 3). Modified
 *                    in place.
 * @param n           Number of samples.
 * @return            0 on success, -1 if the ride is too short or has
 *                    non-positive duration.
 */
int sf_demo_bandpass(
    const uint32_t *elapsed_ms,
    double         *accel,
    int             n
);

/**
 * @brief In-place Cooley-Tukey FFT on a complex signal of length nperseg.
 *
 * Mirrors sf::welch::fft. Input is provided as separate real and imaginary
 * arrays; both are modified in place.
 *
 * @param re  Real parts, length nperseg. Modified in place.
 * @param im  Imaginary parts, length nperseg. Modified in place.
 */
void sf_demo_fft(double *re, double *im);

/**
 * @brief One-sided squared-magnitude DFT of a real signal.
 *
 * Mirrors sf::welch::real_dft_mag_sq. Input length must equal nperseg.
 *
 * @param signal  Real input, length nperseg.
 * @param mag_sq  Output |X[k]|^2 for k=0..nperseg/2, length nperseg/2+1.
 */
void sf_demo_real_dft_mag_sq(const double *signal, double *mag_sq);

/**
 * @brief Welch power spectral density estimate for a real signal.
 *
 * Wraps sf::welch::welch using CFG::welch_nperseg and CFG::welch_noverlap.
 * The output has length CFG::welch_nperseg / 2 + 1 and is in units of
 * (input unit)^2 / Hz.
 *
 * @param signal   Real input signal, length n.
 * @param n        Number of input samples.
 * @param fs       Sample rate in Hz.
 * @param out_psd  Output one-sided PSD, pre-allocated length nperseg/2+1.
 */
void sf_demo_welch(
    const double *signal,
    int           n,
    double        fs,
    double       *out_psd
);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_API_H */
