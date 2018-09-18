/**
 * @file types.cpp
 * @author Simone Comari
 * @date 18 Sep 2018
 * @brief File containing definitions of functions declared in types.h.
 */

#include "types.h"

namespace grabec
{

void DispRetVal(const int err, const char * msg, ...)
{
  va_list args;
  va_start(args, msg);
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
  case FAIL:
    description = "FAILED";
    break;
  }
  std::string full_msg = ANSI_COLOR_RED + (msg + description + ANSI_COLOR_RESET);
  vprintf(full_msg.c_str(), args);
  va_end(args);
}

} // end namespace grabec
