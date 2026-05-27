/**
 * @file contract_test.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Contract test: decodes firmware .bin notification fixtures and writes
 *        output CSVs for comparison against fixture inputs.
 * @date 2026-05-27
 */

#include "pipeline/csv_sink.hpp"
#include "pipeline/file_sink.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

/**
 * @brief Read an ordered list of .bin paths from a manifest file.
 *
 * The manifest is a plain text file with one absolute or relative path per
 * line, in the order the Python script wants them decoded.
 *
 * @param manifest Path to the manifest file.
 * @return Ordered list of paths.
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

/**
 * @brief Resolve the CSV output directory, creating it if needed.
 * @return Path to the output directory.
 */
fs::path contract_out_dir()
{
    const char *p = std::getenv("SF_CONTRACT_OUT");
    fs::path out = p ? fs::path(p) : fs::temp_directory_path() / "contract_out";
    fs::create_directories(out);
    return out;
}

} // namespace

TEST(ContractTest, DecodesBinFiles)
{
    const char *manifest_env = std::getenv("SF_CONTRACT_MANIFEST");
    ASSERT_NE(manifest_env, nullptr) << "SF_CONTRACT_MANIFEST is not set";

    const fs::path manifest(manifest_env);
    ASSERT_TRUE(fs::exists(manifest)) << "SF_CONTRACT_MANIFEST does not exist: " << manifest;

    const auto paths = read_manifest(manifest);
    ASSERT_FALSE(paths.empty()) << "Manifest is empty: " << manifest;

    const fs::path out_dir = contract_out_dir();
    sf::pipeline::CsvSink sink(out_dir);
    ASSERT_NO_THROW(sf::pipeline::replay_packets(paths, sink));
}

TEST(ContractTest, OutputCsvFilesAreNonEmpty)
{
    const fs::path out_dir = contract_out_dir();

    for (const char *name : {"accel.csv", "gyro.csv", "mag.csv"})
    {
        const fs::path p = out_dir / name;
        EXPECT_TRUE(fs::exists(p)) << name << " was not written";
        EXPECT_GT(fs::file_size(p), 0u) << name << " is empty";
    }
}
