#pragma once

#include <filesystem>
#include <string>

namespace tools::time_sync
{
// Stable byte identity for calibration snapshots. Not a cryptographic signature.
std::string file_fingerprint(const std::filesystem::path & path);
}  // namespace tools::time_sync
