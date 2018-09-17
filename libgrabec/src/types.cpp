#include "types.h"

namespace grabec
{

void DispError(const int err, const std::string& msg)
{
  std::string description;
  switch (err)
  {
  case OK:
    description = "SUCCESS";
    break;
  case ECONFIG:
    description = "Configuration FAILED";
    break;
  case EREG:
    description = "Registration FAILED";
    break;
  case EACTIVE:
    description = "Activation FAILED";
    break;
    case EINIT:
      description = "Initialization FAILED";
      break;
    case EINV:
      description = "Invalid value";
      break;
  }
  std::cerr << ANSI_COLOR_RED << msg << description << ANSI_COLOR_RESET
            << std::endl;
}

} // end namespace grabec
