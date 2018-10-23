#ifndef GRABCOMMON_LIBCDPR_ROBOTCONFIGJSONPARSER_H
#define GRABCOMMON_LIBCDPR_ROBOTCONFIGJSONPARSER_H

#include <fstream>

#include <QString>

#include "types.h"
#include "json.hpp"
#include "grabcommon.h"

// Aliases -----------------------------------------------------------
using json = nlohmann::json; // JSON library support

class RobotConfigJsonParser
{
public:
  RobotConfigJsonParser();

  bool ParseFile(const std::string& filename, const bool verbose = false);
  bool ParseFile(const char* filename, const bool verbose = false);
  bool ParseFile(const QString& filename, const bool verbose = false);
  bool ParseFile(const std::string& filename, grabcdpr::Params* const params,
                 const bool verbose = false);
  bool ParseFile(const char* filename, grabcdpr::Params* const params,
                 const bool verbose = false);
  bool ParseFile(const QString& filename, grabcdpr::Params* const params,
                 const bool verbose = false);

  grabcdpr::Params GetConfigStruct() const { return config_params_; }
  void GetConfigStruct(grabcdpr::Params* const params) const { *params = config_params_; }

  void PrintConfig() const;

private:
  grabcdpr::Params config_params_;
  bool file_parsed_ = false;

  bool ExtractConfig(const json& raw_data);
  bool ExtractPlatform(const json& raw_data);
  bool ExtractCables(const json& raw_data);

  bool ArePlatformParamsValid() const;
  bool AreCablesParamsValid() const;
};

#endif // GRABCOMMON_LIBCDPR_ROBOTCONFIGJSONPARSER_H
