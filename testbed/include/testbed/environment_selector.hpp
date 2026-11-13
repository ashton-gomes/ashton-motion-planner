#pragma once

#include <iosfwd>
#include <string>

// List the default scene once, followed by other XMLs in the same directory.
std::string selectEnvironment(const std::string& default_scene,
                              std::istream& input,
                              std::ostream& output);
