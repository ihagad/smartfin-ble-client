/**
 * @file metrics.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Wave metric computation from a Welch power spectral density estimate.
 * @date 2026-05-21
 */

#include "metrics.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace sf::metrics
{

DisplacementPSD integrate_to_displacement(const welch::WelchResult &accel_psd,
                                          double f_low, double f_high)
{
}

/**
 * @brief Compute spectral moments from a displacement PSD.
 *
 * Uses the trapezoidal rule over the PSD bins to approximate m0, m1, m2.
 *
 * @param dpsd  Displacement PSD from integrate_to_displacement().
 * @return      SpectralMoments with m0, m1, m2.
 */
static SpectralMoments compute_moments(const DisplacementPSD &dpsd)
{
}

VelocityPSD integrate_to_velocity(const welch::WelchResult &accel_psd,
                                   double f_low, double f_high)
{
}

WaveMetrics compute_metrics(const DisplacementPSD &dpsd, const VelocityPSD &vpsd)
{
}

WaveMetrics compute_metrics(const welch::WelchResult &accel_psd,
                             double f_low, double f_high)
{
    return compute_metrics(
        integrate_to_displacement(accel_psd, f_low, f_high),
        integrate_to_velocity(accel_psd, f_low, f_high)
    );
}

} // namespace sf::metrics
