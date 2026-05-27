/**
 * @file ensemble_decoder.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief BLE packet decoder implementation.
 * @date 2026-04-30
 *
 */

#include "ensemble_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <span>

namespace sf::protocol
{

/**
 * @brief Size in bytes of the Smartfin transport header.
 */
static constexpr size_t TRANSPORT_HEADER_SIZE = 6;

/**
 * @brief Size in bytes of the packed ensemble header.
 */
static constexpr size_t ENSEMBLE_HEADER_SIZE = 4;

static constexpr size_t TEMP_PAYLOAD_SIZE =
    3; ///< ENS_TEMP (0x01): int16 + uint8
static constexpr size_t IMU_PAYLOAD_SIZE =
    18; ///< ENS_TEMP_HIGH_DATA_RATE_IMU (0x0C): 3×int16 accel + 3×int16 gyro +
        ///< 3×int16 mag
static constexpr size_t QUAT_PAYLOAD_SIZE = 32;
static constexpr size_t TEXT_NCHARS_SIZE =
    1; ///< ENS_TEXT (0x0F): nChars prefix byte
static constexpr size_t TEXT_MAX_CHARS =
    32; ///< ENS_TEXT (0x0F): firmware-enforced max string length

/**
 * @brief Read an unsigned 16-bit little-endian integer from a byte pointer.
 * @param p  Pointer to the first (least significant) byte.
 * @return   Decoded uint16_t value.
 */
static inline uint16_t read_u16_le(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

/**
 * @brief Read a signed 16-bit little-endian integer from a byte pointer.
 * @param p  Pointer to the first (least significant) byte.
 * @return   Decoded int16_t value.
 */
static inline int16_t read_i16_le(const uint8_t *p)
{
    return static_cast<int16_t>(read_u16_le(p));
}

/**
 * @brief Read an unsigned 32-bit little-endian integer from a byte pointer.
 * @param p  Pointer to the first (least significant) byte.
 * @return   Decoded uint32_t value.
 */
static inline uint32_t read_u32_le(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

/**
 * @brief Read a signed 32-bit little-endian integer from a byte pointer.
 * @param p  Pointer to the first (least significant) byte.
 * @return   Decoded int32_t value.
 */
static inline int32_t read_i32_le(const uint8_t *p)
{
    return static_cast<int32_t>(read_u32_le(p));
}

/**
 * @brief Parse the packed ensemble header fields.
 * @param p Pointer to the first header byte.
 * @param ens_type Output ensemble type identifier.
 * @param elapsed_ms Output elapsed time in milliseconds.
 */
static void parse_ensemble_header(const uint8_t *p, uint8_t &ens_type,
                                  uint32_t &elapsed_ms)
{
    uint32_t word = read_u32_le(p);
    ens_type = static_cast<uint8_t>(word & 0x0Fu);
    elapsed_ms = (word >> 4) & 0x0FFFFFFFu;
}

bool decode_packet(std::span<const uint8_t> packet,
                   std::vector<DecodedEnsemble> &out)
{
    if (packet.size() < TRANSPORT_HEADER_SIZE)
    {
        return false;
    }

    /// Transport header layout is version, packet type, sequence, then payload
    /// length.
    const uint16_t payload_len = read_u16_le(packet.data() + 4);
    const size_t payload_end =
        TRANSPORT_HEADER_SIZE + static_cast<size_t>(payload_len);

    /// Clamp to the actual buffer size so malformed lengths cannot read out of
    /// bounds.
    const size_t end = std::min(payload_end, packet.size());

    size_t offset = TRANSPORT_HEADER_SIZE;

    while (offset + ENSEMBLE_HEADER_SIZE <= end)
    {
        uint8_t ens_type;
        uint32_t elapsed_ms;
        parse_ensemble_header(packet.data() + offset, ens_type, elapsed_ms);

        if (ens_type == static_cast<uint8_t>(EnsembleId::Temp))
        {
            const size_t record_size = ENSEMBLE_HEADER_SIZE + TEMP_PAYLOAD_SIZE;
            if (offset + record_size > end)
                break;

            const uint8_t *payload =
                packet.data() + offset + ENSEMBLE_HEADER_SIZE;
            const int16_t scaled_temp = read_i16_le(payload);
            const bool in_water = payload[2] != 0;

            out.push_back(DecodedTemp{
                .elapsed_time_ms = elapsed_ms,
                .temp_c = scaled_temp / 128.0f,
                .in_water = in_water,
            });

            offset += record_size;
            continue;
        }

        if (ens_type == static_cast<uint8_t>(EnsembleId::RawHighRateImu))
        {
            /// Ensemble12_data_t stores 3 accel, 3 gyro, and 3 magnetometer
            /// samples.
            const size_t record_size = ENSEMBLE_HEADER_SIZE + IMU_PAYLOAD_SIZE;
            if (offset + record_size > end)
                break;

            const uint8_t *payload =
                packet.data() + offset + ENSEMBLE_HEADER_SIZE;

            DecodedImu imu{};
            imu.elapsed_time_ms = elapsed_ms;

            for (int i = 0; i < 3; ++i)
            {
                imu.accel_ms2[i] = read_i16_le(payload + i * 2) / 16384.0f;
                imu.gyro_dps[i] = read_i16_le(payload + 6 + i * 2) / 128.0f;
                imu.mag_uT[i] = read_i16_le(payload + 12 + i * 2) / 8.0f;
            }

            out.push_back(imu);
            offset += record_size;
            continue;
        }
        if (ens_type == static_cast<uint8_t>(EnsembleId::QuatHighRateImu))
        {
            const size_t record_size = ENSEMBLE_HEADER_SIZE + QUAT_PAYLOAD_SIZE;
            if (offset + record_size > end)
                break;

            const uint8_t *payload =
                packet.data() + offset + ENSEMBLE_HEADER_SIZE;

            DecodedQuatImu imu{};
            imu.elapsed_time_ms = elapsed_ms;

            static constexpr float Q9  = 512.0f;
            static constexpr float Q7  = 128.0f;
            static constexpr float Q3  = 8.0f;
            static constexpr float Q30 = 1073741824.0f;
            static constexpr float Q12 = 4096.0f;
            static constexpr float kRadToDeg = 57.29577951308232f;

            for (int i = 0; i < 3; ++i)
            {
                imu.accel_ms2[i] = read_i16_le(payload + i * 2) / Q9;
                imu.gyro_dps[i]  = read_i16_le(payload + 6 + i * 2) / Q7;
                imu.mag_uT[i]    = read_i16_le(payload + 12 + i * 2) / Q3;
            }

            const double q1 = read_i32_le(payload + 18) / Q30;
            const double q2 = read_i32_le(payload + 22) / Q30;
            const double q3 = read_i32_le(payload + 26) / Q30;
            const double sq = 1.0 - q1 * q1 - q2 * q2 - q3 * q3;
            const double q0 = std::sqrt(sq > 0.0 ? sq : 0.0);

            imu.q[0] = static_cast<float>(q0);
            imu.q[1] = static_cast<float>(q1);
            imu.q[2] = static_cast<float>(q2);
            imu.q[3] = static_cast<float>(q3);

            // accuracy_q12 is int16, Q12 radians; convert to degrees for the field.
            imu.heading_accuracy_deg =
                (read_i16_le(payload + 30) / Q12) * kRadToDeg;
            imu.quat_valid = imu.heading_accuracy_deg < 10.0f;

            out.push_back(imu);
            offset += record_size;
            continue;
        }
        if (ens_type == static_cast<uint8_t>(EnsembleId::Text))
        {
            const size_t min_record = ENSEMBLE_HEADER_SIZE + TEXT_NCHARS_SIZE;
            if (offset + min_record > end)
                break;

            const uint8_t *payload =
                packet.data() + offset + ENSEMBLE_HEADER_SIZE;
            const size_t nchars = std::min<size_t>(payload[0], TEXT_MAX_CHARS);

            if (offset + min_record + nchars > end)
                break;

            DecodedFwVersion fwv{};
            fwv.elapsed_time_ms = elapsed_ms;
            std::memcpy(fwv.version, payload + TEXT_NCHARS_SIZE, nchars);
            fwv.version[nchars] = '\0';

            out.push_back(fwv);
            offset += min_record + nchars;
            continue;
        }

        /// Stop parsing when an unknown ensemble type is encountered.
        break;
    }

    return true;
}

} // namespace sf::protocol
