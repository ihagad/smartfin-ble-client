/**
 * @file demo_api.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief C API implementation wrapping the processing pipeline for Python ctypes.
 * @date 2026-05-15
 */

#include "demo_api.h"

#include "proccessing/AHRS/madgwick_ahrs.hpp"
#include "proccessing/config.hpp"
#include "proccessing/welch/welch.hpp"
#include "filter/butterworth.hpp"
#include "filter/decimator.hpp"
#include "filter/filtfilt.hpp"
#include "protocol/ensemble_types.hpp"

#include <complex>

extern "C" {

int sf_demo_nperseg(void)
{
    return CFG::welch_nperseg;
}

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
    int            *out_bias_used)
{
    sf::ahrs::AHRS filter;
    int n_out = 0;
    int accel_used = 0;
    int mag_used = 0;
    int bias_used = 0;

    for (int i = 0; i < n_in; ++i)
    {
        sf::protocol::DecodedImu imu;
        imu.elapsed_time_ms = elapsed_ms[i];
        imu.accel_ms2[0] = accel_ms2[i * 3 + 0];
        imu.accel_ms2[1] = accel_ms2[i * 3 + 1];
        imu.accel_ms2[2] = accel_ms2[i * 3 + 2];
        imu.gyro_dps[0]  = gyro_dps[i * 3 + 0];
        imu.gyro_dps[1]  = gyro_dps[i * 3 + 1];
        imu.gyro_dps[2]  = gyro_dps[i * 3 + 2];
        imu.mag_uT[0]    = mag_uT[i * 3 + 0];
        imu.mag_uT[1]    = mag_uT[i * 3 + 1];
        imu.mag_uT[2]    = mag_uT[i * 3 + 2];

        const auto result = filter.update(imu);
        if (!result.updated)
            continue;

        out_elapsed_ms[n_out]  = imu.elapsed_time_ms;
        out_q[n_out * 4 + 0]  = result.q.w;
        out_q[n_out * 4 + 1]  = result.q.x;
        out_q[n_out * 4 + 2]  = result.q.y;
        out_q[n_out * 4 + 3]  = result.q.z;
        out_accel[n_out * 3 + 0] = result.accel_global.x;
        out_accel[n_out * 3 + 1] = result.accel_global.y;
        out_accel[n_out * 3 + 2] = result.accel_global.z;

        accel_used += result.accel_used ? 1 : 0;
        mag_used += result.mag_used ? 1 : 0;
        bias_used += result.gyro_bias_updated ? 1 : 0;
        ++n_out;
    }

    if (out_accel_used != nullptr)
        *out_accel_used = accel_used;
    if (out_mag_used != nullptr)
        *out_mag_used = mag_used;
    if (out_bias_used != nullptr)
        *out_bias_used = bias_used;

    return n_out;
}

int sf_demo_decimate(
    const uint32_t *elapsed_ms,
    const double   *accel,
    int             n_in,
    uint32_t       *out_elapsed_ms,
    double         *out_accel)
{
    sf::filter::Decimator dx, dy, dz;
    int n_out = 0;

    for (int i = 0; i < n_in; ++i)
    {
        double x = 0.0, y = 0.0, z = 0.0;
        const bool ok = dx.push(accel[i * 3 + 0], x)
                      & dy.push(accel[i * 3 + 1], y)
                      & dz.push(accel[i * 3 + 2], z);
        if (ok)
        {
            out_elapsed_ms[n_out]       = elapsed_ms[i];
            out_accel[n_out * 3 + 0]   = x;
            out_accel[n_out * 3 + 1]   = y;
            out_accel[n_out * 3 + 2]   = z;
            ++n_out;
        }
    }
    return n_out;
}

int sf_demo_bandpass(
    const uint32_t *elapsed_ms,
    double         *accel,
    int             n)
{
    if (n <= 9)
        return -1;

    const double duration_s =
        (elapsed_ms[n - 1] - elapsed_ms[0]) / 1000.0;
    if (duration_s <= 0.0)
        return -1;

    const double fs = (n - 1) / duration_s;

    auto hp = sf::filter::butterworth(
        4, CFG::bandpass_lo_hz, fs, sf::filter::FilterType::Highpass);
    auto lp = sf::filter::butterworth(
        4, CFG::bandpass_hi_hz, fs, sf::filter::FilterType::Lowpass);

    std::vector<double> ax(n), ay(n), az(n);
    for (int i = 0; i < n; ++i)
    {
        ax[i] = accel[i * 3 + 0];
        ay[i] = accel[i * 3 + 1];
        az[i] = accel[i * 3 + 2];
    }

    ax = sf::filter::filtfilt(hp, ax);
    ay = sf::filter::filtfilt(hp, ay);
    az = sf::filter::filtfilt(hp, az);
    ax = sf::filter::filtfilt(lp, ax);
    ay = sf::filter::filtfilt(lp, ay);
    az = sf::filter::filtfilt(lp, az);

    for (int i = 0; i < n; ++i)
    {
        accel[i * 3 + 0] = ax[i];
        accel[i * 3 + 1] = ay[i];
        accel[i * 3 + 2] = az[i];
    }
    return 0;
}

void sf_demo_fft(double *re, double *im)
{
    const int n = CFG::welch_nperseg;
    std::vector<std::complex<double>> x(n);
    for (int i = 0; i < n; ++i)
        x[i] = {re[i], im[i]};

    sf::welch::fft(x);

    for (int i = 0; i < n; ++i)
    {
        re[i] = x[i].real();
        im[i] = x[i].imag();
    }
}

void sf_demo_real_dft_mag_sq(const double *signal, double *mag_sq)
{
    const int n = CFG::welch_nperseg;
    const std::vector<double> sig(signal, signal + n);
    std::vector<double> out;
    sf::welch::real_dft_mag_sq(sig, out);
    for (int i = 0; i < static_cast<int>(out.size()); ++i)
        mag_sq[i] = out[i];
}

void sf_demo_welch(const double *signal, int n, double fs, double *out_psd)
{
    const std::vector<double> sig(signal, signal + n);
    const auto result = sf::welch::welch(
        sig, fs, CFG::welch_nperseg, CFG::welch_noverlap);
    const int bins = CFG::welch_nperseg / 2 + 1;
    for (int k = 0; k < bins; ++k)
        out_psd[k] = result.psd[k];
}

} // extern "C"
