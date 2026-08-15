// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/path_utils.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace moveit2_extended
{
namespace fs = std::filesystem;

namespace
{
constexpr const char* kPackagePrefix = "package://";

void setError(std::string* error, std::string message)
{
  if (error)
  {
    *error = std::move(message);
  }
}
}  // namespace

std::string resolvePackageUri(const std::string& uri, std::string* error)
{
  setError(error, {});
  if (uri.rfind(kPackagePrefix, 0) != 0)
  {
    return uri;
  }

  const std::string rest = uri.substr(std::string(kPackagePrefix).size());
  const size_t slash = rest.find('/');
  const std::string package = slash == std::string::npos ? rest : rest.substr(0, slash);
  const std::string relative = slash == std::string::npos ? std::string{} : rest.substr(slash + 1);
  if (package.empty())
  {
    setError(error, "malformed package URI (no package name): " + uri);
    return {};
  }

  try
  {
    const std::string share = ament_index_cpp::get_package_share_directory(package);
    return relative.empty() ? share : (fs::path(share) / relative).string();
  }
  catch (const std::exception& exc)
  {
    setError(error, "cannot resolve package '" + package + "' in '" + uri + "': " + exc.what());
    return {};
  }
}

std::vector<std::string> listFiles(const std::string& directory, const std::string& extension, bool recursive,
                                   std::string* error)
{
  setError(error, {});
  std::vector<std::string> out;

  std::string resolve_error;
  const std::string root = resolvePackageUri(directory, &resolve_error);
  if (root.empty())
  {
    setError(error, resolve_error);
    return out;
  }

  std::error_code ec;
  if (!fs::exists(root, ec) || !fs::is_directory(root, ec))
  {
    setError(error, "not a directory: " + root);
    return out;
  }

  const auto keep = [&extension](const fs::path& p) {
    return extension.empty() || p.extension().string() == extension;
  };

  if (recursive)
  {
    for (fs::recursive_directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec))
    {
      if (it->is_regular_file(ec) && keep(it->path()))
      {
        out.push_back(it->path().string());
      }
    }
  }
  else
  {
    for (fs::directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec))
    {
      if (it->is_regular_file(ec) && keep(it->path()))
      {
        out.push_back(it->path().string());
      }
    }
  }

  // Directory iteration order is filesystem-dependent. Sorting keeps the registration order (and
  // therefore which of two same-named trees wins) reproducible across machines.
  std::sort(out.begin(), out.end());
  return out;
}

std::string readFile(const std::string& path, std::string* error)
{
  setError(error, {});
  std::string resolve_error;
  const std::string resolved = resolvePackageUri(path, &resolve_error);
  if (resolved.empty())
  {
    setError(error, resolve_error);
    return {};
  }

  std::ifstream in(resolved);
  if (!in)
  {
    setError(error, "cannot open for reading: " + resolved);
    return {};
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

bool writeFile(const std::string& path, const std::string& content, std::string* error)
{
  setError(error, {});
  std::string resolve_error;
  const std::string resolved = resolvePackageUri(path, &resolve_error);
  if (resolved.empty())
  {
    setError(error, resolve_error);
    return false;
  }

  std::error_code ec;
  const fs::path parent = fs::path(resolved).parent_path();
  if (!parent.empty())
  {
    fs::create_directories(parent, ec);
    if (ec)
    {
      setError(error, "cannot create directory " + parent.string() + ": " + ec.message());
      return false;
    }
  }

  std::ofstream out(resolved, std::ios::binary | std::ios::trunc);
  if (!out)
  {
    setError(error, "cannot open for writing: " + resolved);
    return false;
  }
  out << content;
  if (!out)
  {
    setError(error, "write failed: " + resolved);
    return false;
  }
  return true;
}

}  // namespace moveit2_extended
