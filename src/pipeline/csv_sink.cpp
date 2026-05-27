/**
 * @file csv_sink.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief CsvSink implementation.
 * @date 2026-05-26
 */

#include "csv_sink.hpp"

#include <iomanip>
#include <stdexcept>

namespace sf::pipeline
{

namespace
{

/**
 * @brief Open a CSV file for writing in @p dir, set float precision, throw on failure.
 * @param dir  Output directory.
 * @param name Bare filename (e.g. "accel.csv").
 * @return Open output stream.
 */
std::ofstream open_csv(const std::filesystem::path &dir, const char *name)
{
    std::ofstream f(dir / name);
    if (!f.is_open())
        throw std::runtime_error(std::string("CsvSink: cannot open ") + name);
    // 9 significant digits matches Python's float32 repr used by gen_samples.py
    f << std::setprecision(9);
    return f;
}

} // namespace

CsvSink::CsvSink(const std::filesystem::path &dir)
    : accel_(open_csv(dir, "decoded_accel.csv")), gyro_(open_csv(dir, "decoded_gyro.csv")),
      mag_(open_csv(dir, "decoded_mag.csv")), temp_(open_csv(dir, "decoded_temp.csv")),
      quat_(open_csv(dir, "decoded_quat.csv"))
{
    accel_ << "elapsed_ms,accel_x,accel_y,accel_z\n";
    gyro_ << "elapsed_ms,gyro_x,gyro_y,gyro_z\n";
    mag_ << "elapsed_ms,mag_x,mag_y,mag_z\n";
    temp_ << "elapsed_ms,temperature,in_water\n";
    quat_ << "elapsed_ms,q0,q1,q2,q3,heading_accuracy_deg,quat_valid\n";
}

CsvSink::~CsvSink() = default;

void CsvSink::write_vec3(std::ofstream &f, const float v[3])
{
    f << v[0] << ',' << v[1] << ',' << v[2] << '\n';
}

void CsvSink::on_imu(const sf::protocol::DecodedImu &s)
{
    accel_ << s.elapsed_time_ms << ',';
    write_vec3(accel_, s.accel_ms2);
    gyro_ << s.elapsed_time_ms << ',';
    write_vec3(gyro_, s.gyro_dps);
    mag_ << s.elapsed_time_ms << ',';
    write_vec3(mag_, s.mag_uT);
}

void CsvSink::on_quat_imu(const sf::protocol::DecodedQuatImu &s)
{
    accel_ << s.elapsed_time_ms << ',';
    write_vec3(accel_, s.accel_ms2);
    gyro_ << s.elapsed_time_ms << ',';
    write_vec3(gyro_, s.gyro_dps);
    mag_ << s.elapsed_time_ms << ',';
    write_vec3(mag_, s.mag_uT);
    quat_ << s.elapsed_time_ms << ',' << s.q[0] << ',' << s.q[1] << ',' << s.q[2] << ','
          << s.q[3] << ',' << s.heading_accuracy_deg << ',' << (s.quat_valid ? 1 : 0) << '\n';
}

void CsvSink::on_temperature(const sf::protocol::DecodedTemp &s)
{
    temp_ << s.elapsed_time_ms << ',' << s.temp_c << ',' << (s.in_water ? 1 : 0) << '\n';
}

void CsvSink::on_fw_version(const sf::protocol::DecodedFwVersion &) {}

} // namespace sf::pipeline
