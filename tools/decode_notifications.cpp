/**
 * @file decode_notifications.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief CLI tool: decode a manifest of BLE notification .bin files into CSVs.
 * @date 2026-05-27
 *
 * Usage:
 *   smartfin_decode_notifications --manifest <path> --out-dir <path>
 *
 * The manifest is a plain text file with one .bin path per line, in the order
 * the packets should be decoded. Writes decoded_accel.csv, decoded_gyro.csv,
 * decoded_mag.csv, decoded_temp.csv, and decoded_quat.csv to --out-dir.
 */

#include "pipeline/csv_sink.hpp"
#include "pipeline/file_sink.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

/**
 * @brief Print usage to stderr and return 1.
 * @param prog argv[0].
 * @return 1.
 */
int usage(const char *prog)
{
    std::cerr << "Usage: " << prog
              << " --manifest <path> --out-dir <path>\n";
    return 1;
}

/**
 * @brief Parse --manifest and --out-dir from argv.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @param manifest Output: path to the manifest file.
 * @param out_dir  Output: path to the output directory.
 * @return true on success, false if required arguments are missing or unknown.
 */
bool parse_args(int argc, char **argv, fs::path &manifest, fs::path &out_dir)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--manifest" && i + 1 < argc)
            manifest = argv[++i];
        else if (arg == "--out-dir" && i + 1 < argc)
            out_dir = argv[++i];
        else
        {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }
    return !manifest.empty() && !out_dir.empty();
}

/**
 * @brief Read an ordered list of .bin paths from a manifest file.
 * @param manifest Path to the manifest file.
 * @return Ordered list of paths, one per non-empty line.
 */
std::vector<fs::path> read_manifest(const fs::path &manifest)
{
    std::vector<fs::path> paths;
    std::ifstream f(manifest);
    std::string line;
    while (std::getline(f, line))
    {
        if (!line.empty())
            paths.emplace_back(line);
    }
    return paths;
}

} // namespace

/**
 * @brief Entry point.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on success, nonzero on failure.
 */
int main(int argc, char **argv)
{
    fs::path manifest, out_dir;
    if (!parse_args(argc, argv, manifest, out_dir))
        return usage(argv[0]);

    if (!fs::exists(manifest))
    {
        std::cerr << "Manifest not found: " << manifest << "\n";
        return 1;
    }

    const auto paths = read_manifest(manifest);
    if (paths.empty())
    {
        std::cerr << "Manifest is empty: " << manifest << "\n";
        return 1;
    }

    for (const auto &p : paths)
    {
        if (!fs::exists(p))
        {
            std::cerr << "Packet file not found: " << p << "\n";
            return 1;
        }
    }

    std::error_code ec;
    fs::create_directories(out_dir, ec);
    if (ec)
    {
        std::cerr << "Cannot create output directory " << out_dir
                  << ": " << ec.message() << "\n";
        return 1;
    }

    try
    {
        sf::pipeline::CsvSink sink(out_dir);
        sf::pipeline::replay_packets(paths, sink);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Decode failed: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Decoded " << paths.size() << " packets -> " << out_dir << "\n"
              << "  decoded_accel.csv\n"
              << "  decoded_gyro.csv\n"
              << "  decoded_mag.csv\n"
              << "  decoded_temp.csv\n"
              << "  decoded_quat.csv\n";
    return 0;
}
