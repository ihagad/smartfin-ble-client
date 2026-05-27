/**
 * @file file_sink.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief ISampleSink that writes decoded ensembles to a binary ride file,
 *        and load_ride() to unpack that file back into decoded structs.
 * @date 2026-05-08
 */

#ifndef FILE_SINK_HPP
#define FILE_SINK_HPP

#include "sample_sink.hpp"
#include "protocol/ensemble_types.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace sf::pipeline {

    /// @brief Magic number at the start of every .sfdat file ("SFDAT").
    static constexpr uint64_t RIDE_FILE_MAGIC = 0x5346444154u;
    /// @brief Current file format version; bumped on breaking layout changes.
    static constexpr uint16_t RIDE_FILE_VERSION = 2;

    /**
     * @brief Tag byte prepended to each record to identify its wire type.
     */
    enum class RecordTag : uint8_t
    {
        Imu = 0x01,     ///< Raw IMU record (WireImu).
        QuatImu = 0x02, ///< Quaternion IMU record (WireQuatImu).
        Temp = 0x03,    ///< Temperature record (WireTemp).
        FwVer = 0x04,   ///< Firmware version record (WireFwVer).
    };

    /**
     * @brief Packed on-disk layout for a raw IMU ensemble.
     */
    struct __attribute__((packed)) WireImu
    {
        uint32_t elapsed_time_ms; ///< Milliseconds since session start.
        float accel_ms2[3];       ///< Linear acceleration in m/s^2 [x, y, z].
        float gyro_dps[3];        ///< Angular rate in deg/s [x, y, z].
        float mag_uT[3];          ///< Magnetic field in uT [x, y, z].
    };

    /**
     * @brief Packed on-disk layout for a quaternion IMU ensemble.
     */
    struct __attribute__((packed)) WireQuatImu
    {
        uint32_t elapsed_time_ms;   ///< Milliseconds since session start.
        float accel_ms2[3];         ///< Linear acceleration in m/s^2 [x, y, z].
        float gyro_dps[3];          ///< Angular rate in deg/s [x, y, z].
        float mag_uT[3];            ///< Magnetic field in uT [x, y, z].
        float q[4];                 ///< Quaternion [q0, q1, q2, q3].
        float heading_accuracy_deg; ///< Estimated heading accuracy in degrees.
        uint8_t quat_valid;         ///< Non-zero when accuracy is within threshold.
    };

    /**
     * @brief Packed on-disk layout for a temperature ensemble.
     */
    struct __attribute__((packed)) WireTemp
    {
        uint32_t elapsed_time_ms; ///< Milliseconds since session start.
        float temp_c;             ///< Water temperature in degrees Celsius.
        uint8_t in_water;         ///< Non-zero when the fin detects water immersion.
    };

    /**
     * @brief Packed on-disk layout for a firmware version ensemble.
     */
    struct __attribute__((packed)) WireFwVer
    {
        uint32_t elapsed_time_ms; ///< Milliseconds since session start.
        char version[33];         ///< Null-terminated version string, max 32 chars.
    };

    /**
     * @brief File header written at offset 0; checked on load to detect
     *        struct layout changes across build versions.
     */
    struct __attribute__((packed)) FileHeader
    {
        uint64_t magic = RIDE_FILE_MAGIC;        ///< Must equal RIDE_FILE_MAGIC.
        uint16_t version = RIDE_FILE_VERSION;    ///< Must equal RIDE_FILE_VERSION.
        uint8_t imu_size = sizeof(WireImu);      ///< sizeof(WireImu) at write time.
        uint8_t quat_size = sizeof(WireQuatImu); ///< sizeof(WireQuatImu) at write time.
        uint8_t temp_size = sizeof(WireTemp);    ///< sizeof(WireTemp) at write time.
        uint8_t fwver_size = sizeof(WireFwVer);  ///< sizeof(WireFwVer) at write time.
    };

    /**
     * @brief ISampleSink that streams decoded ensembles to a binary ride file.
     *
     * Each record is a 1-byte RecordTag followed by the matching packed wire
     * struct. The file opens with a FileHeader for version detection.
     *
     * Writes are synchronous but negligibly cheap at 55 Hz (~2 KB/s).
     * The file is flushed and closed when the sink is destroyed.
     *
     * @par Usage
     * @code
     *   FileSink sink("ride_20260508.sfdat");
     *   Session session(adapter, sink, cfg);
     *   session.run();
     *   // file is flushed and closed when sink goes out of scope
     *
     *   auto data = load_ride("ride_20260508.sfdat");
     * @endcode
     */
    class FileSink final : public ISampleSink
    {
    public:
        /**
         * @brief Open a ride file for writing and write the file header.
         * @param path  Destination file path.
         * @throws std::runtime_error if the file cannot be opened.
         */
        explicit FileSink(const std::string &path);

        /**
         * @brief Flush and close the ride file.
         */
        ~FileSink() override;

        FileSink(const FileSink &) = delete;
        FileSink &operator=(const FileSink &) = delete;

        /**
         * @brief Write a temperature ensemble to the file.
         * @param s  Decoded temperature ensemble.
         */
        void on_temperature(const sf::protocol::DecodedTemp &s) override;

        /**
         * @brief Write a raw IMU ensemble to the file.
         * @param s  Decoded IMU ensemble.
         */
        void on_imu(const sf::protocol::DecodedImu &s) override;

        /**
         * @brief Write a quaternion IMU ensemble to the file.
         * @param s  Decoded quaternion IMU ensemble.
         */
        void on_quat_imu(const sf::protocol::DecodedQuatImu &s) override;

        /**
         * @brief Write a firmware version ensemble to the file.
         * @param s  Decoded firmware version ensemble.
         */
        void on_fw_version(const sf::protocol::DecodedFwVersion &s) override;

    private:
        std::FILE *file_ = nullptr; ///< Handle to the open output file.

        /**
         * @brief Write a single tagged record to the file.
         * @param tag   Record type identifier.
         * @param data  Pointer to the packed wire struct.
         * @param size  Size of the wire struct in bytes.
         */
        void write_record(RecordTag tag, const void *data, std::size_t size);
    };

    /**
     * @brief All decoded records from a single ride file.
     */
    struct RideData
    {
        /// Raw IMU samples
        std::vector<sf::protocol::DecodedImu> imu;
        /// Quaternion IMU samples
        std::vector<sf::protocol::DecodedQuatImu> quat_imu;
        /// Temperature samples
        std::vector<sf::protocol::DecodedTemp> temps;
        /// Firmware version, if present
        std::optional<sf::protocol::DecodedFwVersion> fw_version;
    };

    /**
     * @brief Load a binary ride file produced by FileSink.
     * @param path  Path to the .sfdat ride file.
     * @return RideData populated with every decoded record.
     * @throws std::runtime_error on file open failure, bad magic, or
     *         unsupported version.
     */
    RideData load_ride(const std::string &path);

    /**
     * @brief Replay a RideData into any ISampleSink in timestamp order.
     *
     * Merges the imu, quat_imu, and temps vectors by elapsed_time_ms and
     * calls the appropriate sink method for each record, allowing any
     * downstream sink (AHRS, FFT, CSV, etc.) to be exercised against
     * previously recorded data without touching BLE.
     *
     * @param data  Ride data returned by load_ride().
     * @param sink  Sink to receive the replayed samples.
     */
    void replay_ride(const RideData &data, ISampleSink &sink);

    /**
     * @brief Replay an ordered list of raw BLE notification files into a sink.
     *
     * Decodes each file as a single BLE notification payload in the order given
     * and dispatches every decoded ensemble to @p sink.
     *
     * @param paths Ordered list of .bin notification file paths.
     * @param sink  Sink to receive the decoded samples.
     * @throws std::runtime_error if any file cannot be opened.
     */
    void replay_packets(const std::vector<std::filesystem::path> &paths, ISampleSink &sink);

    /**
     * @brief Replay every @c .bin file in @p dir into a sink in filename order.
     *
     * Convenience wrapper around the vector overload; sorts files alphabetically
     * so a zero-padded sequence prefix controls playback order.
     *
     * @param dir   Directory containing @c .bin notification files.
     * @param sink  Sink to receive the decoded samples.
     * @throws std::runtime_error if any file cannot be opened.
     */
    void replay_packets(const std::filesystem::path &dir, ISampleSink &sink);

} // namespace sf::pipeline

#endif // FILE_SINK_HPP
