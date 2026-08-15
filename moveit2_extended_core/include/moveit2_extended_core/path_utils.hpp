// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <string>
#include <vector>

namespace moveit2_extended
{

/** Resolve "package://pkg/relative/path" against the ament index. Returns the input unchanged when
 *  it is not a package URI, so callers can accept either form without branching.
 *
 *  On failure returns an empty string and, when `error` is non-null, why. Package URIs are how
 *  Objectives and config files refer to each other, so a silent failure here surfaces much later
 *  as a confusing "objective not found". */
std::string resolvePackageUri(const std::string& uri, std::string* error = nullptr);

/** Every file under `directory` (package:// accepted) whose name ends in `extension`, sorted so a
 *  directory listing produces a stable order across machines. Returns an empty list when the
 *  directory does not exist -- a configured-but-absent objective directory is a warning for the
 *  caller to report, not a hard error. */
std::vector<std::string> listFiles(const std::string& directory, const std::string& extension, bool recursive,
                                   std::string* error = nullptr);

/** Read a whole file. Empty on failure, with the reason in `error`. */
std::string readFile(const std::string& path, std::string* error = nullptr);

/** Write `content` to `path`, creating parent directories. Returns false and sets `error` on
 *  failure. */
bool writeFile(const std::string& path, const std::string& content, std::string* error = nullptr);

}  // namespace moveit2_extended
