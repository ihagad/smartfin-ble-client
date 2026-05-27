/**
 * @file csv_sink.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief ISampleSink that writes decoded samples to per-stream CSV files.
 * @date 2026-05-26
 */

#ifndef CSV_SINK_HPP
#define CSV_SINK_HPP

#include "sample_sink.hpp"
#include "protocol/ensemble_types.hpp"

#include <filesystem>
#include <fstream>

namespace sf::pipeline
{

/**
 * @brief ISampleSink that writes decoded ensembles to per-stream CSV files.
 *
 * Produces accel.csv, gyro.csv, mag.csv, temp.csv, and quat.csv in the
 * given output directory. Values are the decoded physical units as produced
 * by the ensemble decoder (m/s^2, deg/s, uT, floats in [-1,1] for quaternions).
 *
 * Both on_imu and on_quat_imu write to accel/gyro/mag; only on_quat_imu
 * writes to quat.
 */
class CsvSink final : public ISampleSink
{
public:
    /**
     * @brief Open output CSVs in @p dir and write column headers.
     * @param dir Output directory; must already exist.
     * @throws std::runtime_error if any file cannot be opened.
     */
    explicit CsvSink(const std::filesystem::path &dir);

    /**
     * @brief Flush and close all output files.
     */
    ~CsvSink() override;

    CsvSink(const CsvSink &) = delete;
    CsvSink &operator=(const CsvSink &) = delete;

    /**
     * @brief Write accel, gyro, and mag rows from a raw IMU ensemble.
     * @param s Decoded IMU ensemble.
     */
    void on_imu(const sf::protocol::DecodedImu &s) override;

    /**
     * @brief Write accel, gyro, mag, and quat rows from a quaternion IMU ensemble.
     * @param s Decoded quaternion IMU ensemble.
     */
    void on_quat_imu(const sf::protocol::DecodedQuatImu &s) override;

    /**
     * @brief Write a temperature row.
     * @param s Decoded temperature ensemble.
     */
    void on_temperature(const sf::protocol::DecodedTemp &s) override;

    /**
     * @brief No-op; firmware version is not written to CSV.
     * @param s Decoded firmware version ensemble.
     */
    void on_fw_version(const sf::protocol::DecodedFwVersion &s) override;

private:
    std::ofstream accel_; ///< decoded_accel.csv: elapsed_ms, accel_x, accel_y, accel_z
    std::ofstream gyro_;  ///< decoded_gyro.csv:  elapsed_ms, gyro_x, gyro_y, gyro_z
    std::ofstream mag_;   ///< decoded_mag.csv:   elapsed_ms, mag_x, mag_y, mag_z
    std::ofstream temp_;  ///< decoded_temp.csv:  elapsed_ms, temperature, in_water
    std::ofstream quat_;  ///< decoded_quat.csv:  elapsed_ms, q0, q1, q2, q3, heading_accuracy_deg, quat_valid

    /**
     * @brief Write a three-element float array as comma-separated values followed by newline.
     * @param f Output stream.
     * @param v Three-element array.
     */
    static void write_vec3(std::ofstream &f, const float v[3]);
};

} // namespace sf::pipeline

#endif // CSV_SINK_HPP
