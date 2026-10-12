#ifndef TOOLS__PATH_HPP
#define TOOLS__PATH_HPP

#include <filesystem>
#include <string>

namespace tools
{
inline std::string resolve_config_path(
  const std::string & config_path, const std::string & value)
{
  const std::filesystem::path requested(value);
  if (requested.is_absolute()) return requested.string();

  const auto config = std::filesystem::absolute(config_path);
  const auto candidates = {
    requested,
    config.parent_path() / requested,
    config.parent_path().parent_path() / requested};
  for (const auto & candidate : candidates) {
    if (std::filesystem::exists(candidate)) return candidate.string();
  }
  return (config.parent_path().parent_path() / requested).string();
}
}

#endif
