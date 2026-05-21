/**
 * @file metrics.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Wave metric computation from a Welch power spectral density estimate.
 * @date 2026-05-21
 */

#include "metrics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>
#include <stdexcept>

namespace sf::metrics
{

DisplacementPSD integrate_to_displacement(const welch::WelchResult &accel_psd,
                                          double f_low, double f_high)
{
    if (f_low < 0.0)
        throw std::invalid_argument("f_low must be non-negative");
    if (f_low >= f_high)
        throw std::invalid_argument("f_low must be less than f_high");

    DisplacementPSD displacement;
    displacement.freqs = accel_psd.freqs;
    displacement.psd.resize(accel_psd.psd.size(), 0.0);

    for (std::size_t k = 0; k < accel_psd.freqs.size(); ++k)
    {
        double f = accel_psd.freqs[k];
        if (f < f_low || f > f_high)
            continue;
        displacement.psd[k] =
            accel_psd.psd[k] / std::pow(2.0 * std::numbers::pi * f, 4);
    }
    return displacement;
}

VelocityPSD integrate_to_velocity(const welch::WelchResult &accel_psd,
                                  double f_low, double f_high)
{
    if (f_low < 0.0)
        throw std::invalid_argument("f_low must be non-negative");
    if (f_low >= f_high)
        throw std::invalid_argument("f_low must be less than f_high");

    VelocityPSD vel;
    vel.freqs = accel_psd.freqs;
    vel.psd.resize(accel_psd.psd.size(), 0.0);

    for (std::size_t k = 0; k < accel_psd.freqs.size(); ++k)
    {
        double f = accel_psd.freqs[k];
        if (f < f_low || f > f_high)
            continue;
        vel.psd[k] = accel_psd.psd[k] / std::pow(2.0 * std::numbers::pi * f, 2);
    }
    return vel;
}

/**
 * @brief Compute spectral moments from a displacement PSD.
 *
 * Bins are uniformly spaced by df, so each moment is a weighted sum scaled
 * by df: mn = df * sum(f^n * S(f)).
 *
 * @param dpsd  Displacement PSD from integrate_to_displacement().
 * @param df    Frequency bin spacing in Hz (= fs / nperseg).
 * @return      SpectralMoments with m0, m1, m2.
 */
static SpectralMoments compute_moments(const DisplacementPSD &dpsd, double df)
{
    SpectralMoments moments;
    double sum0 = 0, sum1 = 0, sum2 = 0;

    for (std::size_t k = 0; k < dpsd.freqs.size(); ++k)
    {
        sum0 += dpsd.psd[k];
        sum1 += dpsd.psd[k] * dpsd.freqs[k];
        sum2 += dpsd.psd[k] * dpsd.freqs[k] * dpsd.freqs[k];
    }
    moments.m0 = df * sum0;
    moments.m1 = df * sum1;
    moments.m2 = df * sum2;
    return moments;
}

WaveMetrics compute_metrics(const DisplacementPSD &dpsd,
                            const VelocityPSD &vpsd, double df)
{

    WaveMetrics metrics;
    SpectralMoments moments = compute_moments(dpsd, df);
    auto it = std::max_element(dpsd.psd.begin(), dpsd.psd.end());
    double f_peak = dpsd.freqs[std::distance(dpsd.psd.begin(), it)];

    metrics.Hs = 4 * std::sqrt(moments.m0);
    metrics.Tp = 1 / f_peak;
    metrics.Tm01 = moments.m0 / moments.m1;
    metrics.Tm02 = std::sqrt(moments.m0 / moments.m2);
    metrics.v_rms =
        std::sqrt(df * std::accumulate(vpsd.psd.begin(), vpsd.psd.end(), 0.0));
    return metrics;
}

WaveMetrics compute_metrics(const welch::WelchResult &accel_psd, double f_low,
                            double f_high)
{
    return compute_metrics(integrate_to_displacement(accel_psd, f_low, f_high),
                           integrate_to_velocity(accel_psd, f_low, f_high),
                           accel_psd.df);
}

} // namespace sf::metrics
