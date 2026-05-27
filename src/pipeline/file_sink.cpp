/**
 * @file file_sink.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief FileSink and load_ride() implementations.
 * @date 2026-05-08
 */

#include "file_sink.hpp"

#include "protocol/ensemble_decoder.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace sf::pipeline {

FileSink::FileSink(const std::string &path)
{
    file_ = std::fopen(path.c_str(), "wb");
    if (!file_)
        throw std::runtime_error("FileSink: cannot open " + path);
    const FileHeader hdr{};
    std::fwrite(&hdr, sizeof(hdr), 1, file_);
}

FileSink::~FileSink()
{
    if (file_)
    {
        std::fflush(file_);
        std::fclose(file_);
    }
}

void FileSink::on_temperature(const sf::protocol::DecodedTemp &s)
{
    const WireTemp w{
        s.elapsed_time_ms,
        s.temp_c,
        s.in_water ? uint8_t(1) : uint8_t(0)};
    write_record(RecordTag::Temp, &w, sizeof(w));
}

void FileSink::on_imu(const sf::protocol::DecodedImu &s)
{
    WireImu w{};
    w.elapsed_time_ms = s.elapsed_time_ms;
    for (int i = 0; i < 3; ++i)
    {
        w.accel_ms2[i] = s.accel_ms2[i];
        w.gyro_dps[i] = s.gyro_dps[i];
        w.mag_uT[i] = s.mag_uT[i];
    }
    write_record(RecordTag::Imu, &w, sizeof(w));
}

void FileSink::on_quat_imu(const sf::protocol::DecodedQuatImu &s)
{
    WireQuatImu w{};
    w.elapsed_time_ms = s.elapsed_time_ms;
    for (int i = 0; i < 3; ++i)
    {
        w.accel_ms2[i] = s.accel_ms2[i];
        w.gyro_dps[i] = s.gyro_dps[i];
        w.mag_uT[i] = s.mag_uT[i];
    }
    for (int i = 0; i < 4; ++i)
        w.q[i] = s.q[i];
    w.heading_accuracy_deg = s.heading_accuracy_deg;
    w.quat_valid = s.quat_valid ? uint8_t(1) : uint8_t(0);
    write_record(RecordTag::QuatImu, &w, sizeof(w));
}

void FileSink::on_fw_version(const sf::protocol::DecodedFwVersion &s)
{
    WireFwVer w{};
    w.elapsed_time_ms = s.elapsed_time_ms;
    std::memcpy(w.version, s.version, sizeof(w.version));
    write_record(RecordTag::FwVer, &w, sizeof(w));
}

void FileSink::write_record(RecordTag tag, const void *data, std::size_t size)
{
    const auto t = static_cast<uint8_t>(tag);
    std::fwrite(&t, 1, 1, file_);
    std::fwrite(data, size, 1, file_);
}

RideData load_ride(const std::string &path)
{
    std::FILE *f = std::fopen(path.c_str(), "rb");
    if (!f)
        throw std::runtime_error("load_ride: cannot open " + path);

    FileHeader hdr{};
    if (std::fread(&hdr, sizeof(hdr), 1, f) != 1 ||
        hdr.magic != RIDE_FILE_MAGIC)
    {
        std::fclose(f);
        throw std::runtime_error("load_ride: invalid file header");
    }
    if (hdr.version != RIDE_FILE_VERSION)
    {
        std::fclose(f);
        throw std::runtime_error(
            "load_ride: unsupported version " +
            std::to_string(hdr.version));
    }

    RideData data;
    uint8_t tag;

    while (std::fread(&tag, 1, 1, f) == 1)
    {
        if (tag == static_cast<uint8_t>(RecordTag::Imu))
        {
            WireImu w{};
            if (std::fread(&w, sizeof(w), 1, f) != 1)
                break;
            sf::protocol::DecodedImu s{};
            s.elapsed_time_ms = w.elapsed_time_ms;
            for (int i = 0; i < 3; ++i)
            {
                s.accel_ms2[i] = w.accel_ms2[i];
                s.gyro_dps[i] = w.gyro_dps[i];
                s.mag_uT[i] = w.mag_uT[i];
            }
            data.imu.push_back(s);
        }
        else if (tag == static_cast<uint8_t>(RecordTag::QuatImu))
        {
            WireQuatImu w{};
            if (std::fread(&w, sizeof(w), 1, f) != 1)
                break;
            sf::protocol::DecodedQuatImu s{};
            s.elapsed_time_ms = w.elapsed_time_ms;
            for (int i = 0; i < 3; ++i)
            {
                s.accel_ms2[i] = w.accel_ms2[i];
                s.gyro_dps[i] = w.gyro_dps[i];
                s.mag_uT[i] = w.mag_uT[i];
            }
            for (int i = 0; i < 4; ++i)
                s.q[i] = w.q[i];
            s.heading_accuracy_deg = w.heading_accuracy_deg;
            s.quat_valid = w.quat_valid != 0;
            data.quat_imu.push_back(s);
        }
        else if (tag == static_cast<uint8_t>(RecordTag::Temp))
        {
            WireTemp w{};
            if (std::fread(&w, sizeof(w), 1, f) != 1)
                break;
            data.temps.push_back({w.elapsed_time_ms,
                                  w.temp_c,
                                  w.in_water != 0});
        }
        else if (tag == static_cast<uint8_t>(RecordTag::FwVer))
        {
            WireFwVer w{};
            if (std::fread(&w, sizeof(w), 1, f) != 1)
                break;
            sf::protocol::DecodedFwVersion s{};
            s.elapsed_time_ms = w.elapsed_time_ms;
            std::memcpy(s.version, w.version, sizeof(s.version));
            data.fw_version = s;
        }
        else
        {
            break; // unknown tag, stop parsing
        }
    }

    std::fclose(f);
    return data;
}

void replay_ride(const RideData &data, ISampleSink &sink)
{
    size_t ii = 0, qi = 0, ti = 0;

    const uint32_t INF = UINT32_MAX;

    // Emit fw_version at its timestamp, or at the very end if missing.
    bool     fw_emitted = false;
    uint32_t fw_time    = data.fw_version
                              ? data.fw_version->elapsed_time_ms
                              : INF;

    while (ii < data.imu.size() ||
           qi < data.quat_imu.size() ||
           ti < data.temps.size())
    {
        uint32_t t_imu  = ii < data.imu.size()
                              ? data.imu[ii].elapsed_time_ms : INF;
        uint32_t t_quat = qi < data.quat_imu.size()
                              ? data.quat_imu[qi].elapsed_time_ms : INF;
        uint32_t t_temp = ti < data.temps.size()
                              ? data.temps[ti].elapsed_time_ms : INF;

        uint32_t next = std::min({t_imu, t_quat, t_temp});

        if (!fw_emitted && fw_time <= next)
        {
            sink.on_fw_version(*data.fw_version);
            fw_emitted = true;
        }

        if (t_imu <= t_quat && t_imu <= t_temp)
            sink.on_imu(data.imu[ii++]);
        else if (t_quat <= t_temp)
            sink.on_quat_imu(data.quat_imu[qi++]);
        else
            sink.on_temperature(data.temps[ti++]);
    }

    if (!fw_emitted && data.fw_version)
        sink.on_fw_version(*data.fw_version);
}

namespace
{

/**
 * @brief Dispatch a single decoded ensemble to the appropriate sink method.
 * @param e     Decoded ensemble variant.
 * @param sink  Destination sink.
 */
void dispatch(const sf::protocol::DecodedEnsemble &e, sf::pipeline::ISampleSink &sink)
{
    std::visit(
        [&](auto &&s)
        {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, sf::protocol::DecodedImu>)
                sink.on_imu(s);
            else if constexpr (std::is_same_v<T, sf::protocol::DecodedQuatImu>)
                sink.on_quat_imu(s);
            else if constexpr (std::is_same_v<T, sf::protocol::DecodedTemp>)
                sink.on_temperature(s);
            else if constexpr (std::is_same_v<T, sf::protocol::DecodedFwVersion>)
                sink.on_fw_version(s);
        },
        e);
}

/**
 * @brief Read an entire file into a byte vector.
 * @param p  Path to the file.
 * @return File contents.
 * @throws std::runtime_error if the file cannot be opened.
 */
std::vector<uint8_t> read_bin(const std::filesystem::path &p)
{
    std::FILE *f = std::fopen(p.string().c_str(), "rb");
    if (!f)
        throw std::runtime_error("replay_packets: cannot open " + p.string());
    std::fseek(f, 0, SEEK_END);
    const auto sz = static_cast<std::size_t>(std::ftell(f));
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz);
    std::fread(buf.data(), 1, sz, f);
    std::fclose(f);
    return buf;
}

} // namespace

void replay_packets(const std::vector<std::filesystem::path> &paths, ISampleSink &sink)
{
    std::vector<sf::protocol::DecodedEnsemble> decoded;
    for (const auto &p : paths)
    {
        const auto bytes = read_bin(p);
        sf::protocol::decode_packet(bytes, decoded);
    }
    for (const auto &e : decoded)
        dispatch(e, sink);
}

void replay_packets(const std::filesystem::path &dir, ISampleSink &sink)
{
    std::vector<std::filesystem::path> bins;
    for (const auto &entry : std::filesystem::directory_iterator(dir))
    {
        if (entry.path().extension() == ".bin")
            bins.push_back(entry.path());
    }
    std::sort(bins.begin(), bins.end());
    replay_packets(bins, sink);
}

} // namespace sf::pipeline
