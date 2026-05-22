/**
 * @file welch_test.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Unit tests for the Welch PSD estimator.
 * @date 2026-05-18
 */

#include "config.hpp"
#include "welch/welch.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

using sf::welch::welch;

static constexpr int N = CFG::welch_nperseg;

TEST(Welch, ReturnsOneSidedFrequencyGrid)
{
    const double fs = 16.0;
    std::vector<double> signal(2 * N, 0.0);

    const auto result = welch(signal, fs, N, N / 2);

    ASSERT_EQ(result.freqs.size(), static_cast<std::size_t>(N / 2 + 1));
    ASSERT_EQ(result.psd.size(), result.freqs.size());
    EXPECT_DOUBLE_EQ(result.df, fs / static_cast<double>(N));
    EXPECT_DOUBLE_EQ(result.freqs.front(), 0.0);
    EXPECT_DOUBLE_EQ(result.freqs.back(), fs / 2.0);
}

TEST(Welch, FindsIntegerBinCosinePeak)
{
    const double fs = 64.0;
    const int peak_bin = 5;
    const std::size_t sample_count = 4 * N;
    std::vector<double> signal(sample_count);

    for (std::size_t i = 0; i < sample_count; ++i)
    {
        signal[i] = std::cos(2.0 * std::numbers::pi * peak_bin *
                             static_cast<double>(i) / static_cast<double>(N));
    }

    const auto result = welch(signal, fs, N, N / 2);

    std::size_t max_bin = 0;
    for (std::size_t k = 1; k < result.psd.size(); ++k)
    {
        if (result.psd[k] > result.psd[max_bin])
            max_bin = k;
    }

    EXPECT_EQ(max_bin, static_cast<std::size_t>(peak_bin));
    EXPECT_DOUBLE_EQ(result.freqs[max_bin],
                     peak_bin * fs / static_cast<double>(N));
}

TEST(Welch, RejectsRuntimeSegmentLengthMismatch)
{
    std::vector<double> signal(N, 0.0);

    EXPECT_THROW(welch(signal, 16.0, N / 2, N / 4), std::invalid_argument);
}

/**
 * @brief All-zero signal produces an all-zero PSD.
 */
TEST(Welch, ZeroSignal)
{
    const double fs = 16.0;
    std::vector<double> signal(4 * N, 0.0);

    const auto result = welch(signal, fs, N, N / 2);

    for (std::size_t k = 0; k < result.psd.size(); ++k)
        EXPECT_DOUBLE_EQ(result.psd[k], 0.0) << "at bin " << k;
}

/**
 * @brief A signal shorter than nperseg returns a zero PSD without crashing.
 */
TEST(Welch, ShortSignalReturnsZeroPSD)
{
    const double fs = 16.0;
    std::vector<double> signal(static_cast<std::size_t>(N) - 1, 1.0);

    const auto result = welch(signal, fs, N, N / 2);

    for (std::size_t k = 0; k < result.psd.size(); ++k)
        EXPECT_DOUBLE_EQ(result.psd[k], 0.0) << "at bin " << k;
}

/**
 * @brief A DC signal has its PSD peak at bin 0.
 */
TEST(Welch, DCInputMaxAtBinZero)
{
    const double fs = 16.0;
    const double A = 2.0;
    std::vector<double> signal(32 * N, A);

    const auto result = welch(signal, fs, N, N / 2);

    std::size_t max_bin = 0;
    for (std::size_t k = 1; k < result.psd.size(); ++k)
    {
        if (result.psd[k] > result.psd[max_bin])
            max_bin = k;
    }

    EXPECT_EQ(max_bin, 0u);
}

/**
 * @brief Parseval check: integrated PSD of a unit cosine approximates 0.5.
 *
 * The mean power of cos is 0.5; the one-sided Welch PSD integral should
 * match within 5% for a long signal.
 */
TEST(Welch, ParsevalUnitCosine)
{
    const double fs = 64.0;
    const int peak_bin = 5;
    const std::size_t sample_count = 32 * static_cast<std::size_t>(N);
    std::vector<double> signal(sample_count);

    for (std::size_t i = 0; i < sample_count; ++i)
    {
        signal[i] = std::cos(2.0 * std::numbers::pi * peak_bin *
                             static_cast<double>(i) / static_cast<double>(N));
    }

    const auto result = welch(signal, fs, N, N / 2);

    double power = 0.0;
    for (std::size_t k = 0; k < result.psd.size(); ++k)
        power += result.psd[k] * result.df;

    EXPECT_NEAR(power, 0.5, 0.025);
}
