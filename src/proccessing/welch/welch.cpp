/**
 * @file welch.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief implementation of welch algorithm
 * @date 2026-05-15
 *
 * @cite Reference:
 * Welch, P. D. (1967), "The use of Fast Fourier Transform for the estimation
 * of power spectra: A method based on time averaging over short, modified
 * periodograms", IEEE Transactions on Audio and Electroacoustics, AU-15 (2):
 * 70-73, Bibcode:1967ITAE...15...70W, doi:10.1109/TAU.1967.1161901.
 */

#include "welch.hpp"
#include "config.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

sf::welch::WelchResult sf::welch::welch(const std::vector<double> &signal, double fs,
                  int nperseg, int noverlap)
{
    if (fs <= 0.0)
        throw std::invalid_argument("fs must be positive");
    if (nperseg != CFG::welch_nperseg)
        throw std::invalid_argument("nperseg must match CFG::welch_nperseg");
    if (nperseg < 4 || (nperseg & (nperseg - 1)) != 0)
        throw std::invalid_argument("nperseg must be a power of 2 and >= 4");
    if (noverlap < 0 || noverlap >= nperseg)
        throw std::invalid_argument(
            "noverlap must satisfy 0 <= noverlap < nperseg");

    const std::size_t n = static_cast<std::size_t>(nperseg);
    const std::size_t bins = n / 2 + 1;
    const int step = nperseg - noverlap;

    WelchResult result;
    result.df = fs / static_cast<double>(nperseg);
    result.freqs.resize(bins);
    result.psd.assign(bins, 0.0);

    for (std::size_t k = 0; k < bins; ++k)
        result.freqs[k] = static_cast<double>(k) * result.df;

    if (signal.size() < n)
        return result;

    std::vector<double> hann_window(n);
    double window_power_sum = 0.0;
    for (std::size_t idx = 0; idx < n; ++idx)
    {
        hann_window[idx] =
            0.5 * (1.0 - std::cos((2.0 * std::numbers::pi * idx) /
                                  static_cast<double>(nperseg)));

        window_power_sum += hann_window[idx] * hann_window[idx];
    }

    std::vector<double> segment(n);
    std::vector<double> mag_sq;
    std::size_t segment_count = 0;

    for (std::size_t start = 0; start + n <= signal.size();
         start += static_cast<std::size_t>(step))
    {
        for (std::size_t idx = 0; idx < n; ++idx)
            segment[idx] = signal[start + idx] * hann_window[idx];

        real_dft_mag_sq(segment, mag_sq);

        for (std::size_t k = 0; k < bins; ++k)
            result.psd[k] += mag_sq[k];
        ++segment_count;
    }

    const double scale =
        1.0 / (static_cast<double>(segment_count) * fs * window_power_sum);
    for (std::size_t k = 0; k < bins; ++k)
    {
        result.psd[k] *= scale;
        if (k != 0 && k != bins - 1)
            result.psd[k] *= 2.0;
    }

    return result;
}
