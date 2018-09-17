#ifndef GRABCOMMON_LIBGRABEC_TYPES_H
#define GRABCOMMON_LIBGRABEC_TYPES_H

#include <iostream>
#include <string>
#include "grabcommon.h"

namespace grabec {

  enum Errors {
    OK,
    ECONFIG,
    EREG,
    EACTIVE,
    EINIT,
    EINV
  };

  void DispError(const int err, const std::string& msg);

}  // end namespace grabec

#endif // GRABCOMMON_LIBGRABEC_TYPES_H
