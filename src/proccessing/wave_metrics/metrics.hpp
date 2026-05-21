/**
 * @file metrics.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Wave metric computation from a Welch power spectral density estimate.
 * @date 2026-05-21
 */

#ifndef METRICS_HPP
#define METRICS_HPP

#include "proccessing/welch/welch.hpp"

namespace sf::metrics
{

/**
 * @brief Frequency-domain integration result.
 *
 * Holds the displacement PSD after converting from acceleration PSD and
 * zeroing bins outside the wave band [0.05, 0.5] Hz.
 * freqs and psd are parallel arrays of the same length as the input
 * WelchResult.
 */
struct DisplacementPSD
{
    std::vector<double> freqs; ///< Frequency bins in Hz, same as WelchResult.
    std::vector<double> psd;   ///< Displacement PSD in m^2/Hz.
};

/**
 * @brief Frequency-domain integration result.
 *
 * Holds the velocity PSD after converting from acceleration PSD and
 * zeroing bins outside the wave band [0.05, 0.5] Hz.
 * freqs and psd are parallel arrays of the same length as the input
 * WelchResult.
 */
struct VelocityPSD
{
    std::vector<double> freqs;
    std::vector<double> psd;
};

/**
 * @brief container for spectral moments
 */
struct SpectralMoments
{
    double m0; ///< Zeroth moment (total variance), m^2.
    double m1; ///< First moment, m^2/s.
    double m2; ///< Second moment, m^2/s^2.
};
/**
 * @brief Derived wave metrics from spectral moment integration.
 */
struct WaveMetrics
{
    double Hs;    ///< Significant wave height = 4*sqrt(m0), m.
    double Tp;    ///< Peak period = 1/f_peak, s.
    double Tm01;  ///< Mean period = m0/m1, s.
    double Tm02;  ///< Mean zero-crossing period = sqrt(m0/m2), s.
    double v_rms; ///< RMS orbital speed = sqrt(m0 of velocity PSD), m/s.
};

/**
 * @brief Convert acceleration PSD to displacement PSD via frequency-domain
 * integration.
 *
 * Divides each bin by (2*pi*f)^4 and zeros bins outside [f_low, f_high] Hz.
 * The DC bin is always zeroed.
 *
 * @param accel_psd  One-sided acceleration PSD from a Welch estimate.
 * @param f_low      Lower bound of the wave band in Hz.
 * @param f_high     Upper bound of the wave band in Hz.
 * @return           DisplacementPSD with the same frequency axis.
 */
DisplacementPSD integrate_to_displacement(const welch::WelchResult &accel_psd,
                                          double f_low, double f_high);

/**
 * @brief Convert acceleration PSD to velocity PSD via frequency-domain
 * integration.
 *
 * Divides each bin by (2*pi*f)^2 and zeros bins outside [f_low, f_high] Hz.
 * The DC bin is always zeroed.
 *
 * @param accel_psd  One-sided acceleration PSD from a Welch estimate.
 * @param f_low      Lower bound of the wave band in Hz.
 * @param f_high     Upper bound of the wave band in Hz.
 * @return           VelocityPSD with the same frequency axis.
 */
VelocityPSD integrate_to_velocity(const welch::WelchResult &accel_psd,
                                   double f_low, double f_high);

/**
 * @brief Compute spectral moments and wave metrics from displacement and
 * velocity PSDs.
 *
 * @param dpsd  Displacement PSD from integrate_to_displacement().
 * @param vpsd  Velocity PSD from integrate_to_velocity().
 * @return      WaveMetrics with Hs, Tp, Tm01, Tm02, v_rms.
 */
WaveMetrics compute_metrics(const DisplacementPSD &dpsd,
                             const VelocityPSD &vpsd);

/**
 * @brief Compute wave metrics directly from an acceleration PSD.
 *
 * Convenience overload. Calls integrate_to_displacement(), integrate_to_velocity(),
 * then compute_metrics(dpsd, vpsd) internally.
 *
 * @param accel_psd  One-sided acceleration PSD from a Welch estimate.
 * @param f_low      Lower bound of the wave band in Hz.
 * @param f_high     Upper bound of the wave band in Hz.
 * @return           WaveMetrics with Hs, Tp, Tm01, Tm02, v_rms.
 */
WaveMetrics compute_metrics(const welch::WelchResult &accel_psd,
                             double f_low, double f_high);

} // namespace sf::metrics

#endif // METRICS_HPP
