/**
 * @file robotconfigjsonparser.h
 * @author Simone Comari
 * @date Jan 2022
 * @brief This file include a parser for JSON configuration file for GRAB CDPR.
 */

#ifndef GRABCOMMON_LIBCDPR_ROBOTCONFIGJSONPARSER_H
#define GRABCOMMON_LIBCDPR_ROBOTCONFIGJSONPARSER_H

#include <fstream>

#include <QString>

#include "json.hpp"

#include "cdpr_types.h"

#define GRAVITY -9.80665 /**< Earth gravity approximate value. */

using json = nlohmann::json; /**< JSON library support alias */

/**
 * @brief A parser for JSON configuration file for GRAB CDPR.
 */
class RobotConfigJsonParser
{
 public:
  /**
   * @brief RobotConfigJsonParser default constructor.
   */
  RobotConfigJsonParser() {}
  virtual ~RobotConfigJsonParser() {}

  /**
   * @brief Parse a JSON configuration file.
   * @param[in] filename Configuration filepath.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool parseFile(const std::string& filename, const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file.
   * @param[in] filename Configuration filepath.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool parseFile(const char* filename, const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file.
   * @param[in] filename Configuration filepath.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool parseFile(const QString& filename, const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file and fills a parameters structure.
   * @param[in] filename Configuration filepath.
   * @param[out] params Parameters structure to be filled with parsed data.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool parseFile(const std::string& filename, grabcdpr::RobotParams* const params,
                 const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file and fills a parameters structure.
   * @param[in] filename Configuration filepath.
   * @param[out] params Parameters structure to be filled with parsed data.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool parseFile(const char* filename, grabcdpr::RobotParams* const params,
                 const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file and fills a parameters structure.
   * @param[in] filename Configuration filepath.
   * @param[out] params Parameters structure to be filled with parsed data.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool parseFile(const QString& filename, grabcdpr::RobotParams* const params,
                 const bool verbose = false);

  /**
   * @brief Get parsed configuration structure.
   * @return A configuration structure.
   * @warning If file was not correctly parsed yet, it returns an empty structure without
   * errors or warnings.
   */
  grabcdpr::RobotParams getConfigStruct() const { return config_params_; }
  /**
   * @brief Get parsed configuration structure.
   * @param[out] params The configuration structure to be filled with parsed data.
   * @warning If file was not correctly parsed yet, it returns an empty structure without
   * errors or warnings.
   */
  void getConfigStruct(grabcdpr::RobotParams* const params) const;

  /**
   * @brief Print parsed configuration parameters set, if present.
   */
  virtual void printConfig() const;

 protected:
  grabcdpr::RobotParams config_params_;
  bool file_parsed_ = false;

  virtual bool extractConfig(const json& raw_data);
  bool extractPlatform(const json& raw_data);
  virtual bool extractActuators(const json& raw_data);

  grabcdpr::RotParametrization str2RotParametrization(const std::string& str);

  bool arePlatformParamsValid() const;
  bool areWinchParamsValid(const grabcdpr::WinchParams& params) const;
  bool areActuatorsParamsValid(const grabcdpr::ActuatorParams& params) const;
};

#endif // GRABCOMMON_LIBCDPR_ROBOTCONFIGJSONPARSER_H
