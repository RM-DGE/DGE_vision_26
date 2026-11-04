#include "tools/time_sync_session.hpp"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace tools::time_sync
{
std::string file_fingerprint(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("Cannot read calibration snapshot: " + path.string());
  std::uint64_t hash = 14695981039346656037ULL;
  char data[8192];
  while (input.read(data, sizeof(data)) || input.gcount()) {
    for (std::streamsize i = 0; i < input.gcount(); ++i) {
      hash ^= static_cast<unsigned char>(data[i]);
      hash *= 1099511628211ULL;
    }
  }
  if (!input.eof())
    throw std::runtime_error("Failed reading calibration snapshot: " + path.string());
  std::ostringstream out;
  out << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0') << hash;
  return out.str();
}
}  // namespace tools::time_sync
