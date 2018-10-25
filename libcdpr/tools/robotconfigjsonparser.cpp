#include "robotconfigjsonparser.h"

///////////////////////////////////////////////
//// Public methods
///////////////////////////////////////////////
RobotConfigJsonParser::RobotConfigJsonParser()
{
  config_params_.platform = new grabcdpr::PlatformParams;
}

bool RobotConfigJsonParser::ParseFile(const std::string& filename,
                                      const bool verbose /* = false*/)
{
  std::cout << "Parsing file '" << filename << "'...\n";

  // Check file extension
  size_t found = filename.rfind(std::string(".json"));
  if (found == std::string::npos || found != filename.length() - 5)
  {
    std::cerr << "[ERROR] Invaild file type. Only '.json' files can be parsed."
              << std::endl;
    return false;
  }

  // Open file
  std::ifstream ifile(filename);
  if (!ifile.is_open())
  {
    std::cerr << "[ERROR] Could not open file " << filename << std::endl;
    return false;
  }

  // Parse JSON (generic) data
  json raw_data;
  ifile >> raw_data;
  ifile.close();

  // Extract information and arrange them properly
  file_parsed_ = ExtractConfig(raw_data);

  // Display data
  if (file_parsed_ && verbose)
    PrintConfig();

  return file_parsed_;
}

bool RobotConfigJsonParser::ParseFile(const char* filename,
                                      const bool verbose /*= false*/)
{
  return ParseFile(std::string(filename), verbose);
}

bool RobotConfigJsonParser::ParseFile(const QString& filename,
                                      const bool verbose /*= false*/)
{
  return ParseFile(filename.toStdString(), verbose);
}

bool RobotConfigJsonParser::ParseFile(const std::string& filename,
                                      grabcdpr::Params* params,
                                      const bool verbose /*= false*/)
{
  if (ParseFile(filename, verbose))
  {
    GetConfigStruct(params);
    return true;
  }
  return false;
}

bool RobotConfigJsonParser::ParseFile(const char* filename, grabcdpr::Params* params,
                                      const bool verbose /*= false*/)
{
  return ParseFile(std::string(filename), params, verbose);
}

bool RobotConfigJsonParser::ParseFile(const QString& filename, grabcdpr::Params* params,
                                      const bool verbose /*= false*/)
{
  return ParseFile(filename.toStdString(), params, verbose);
}

void RobotConfigJsonParser::PrintConfig() const
{
  if (!file_parsed_)
  {
    std::cerr << "[ERROR] No file was parsed yet!" << std::endl;
    return;
  }

  std::cout << "PLATFORM PARAMETERS\n============================="
            << "\n mass\t\t\t" << config_params_.platform->mass << "\n ext_force_loc\n"
            << config_params_.platform->ext_force_loc << " ext_torque_loc\n"
            << config_params_.platform->ext_torque_loc << " pos_PG_loc\n"
            << config_params_.platform->pos_PG_loc << " inertia_mat_G_loc\n"
            << config_params_.platform->inertia_mat_G_loc << std::endl;
  for (size_t i = 0; i < config_params_.cables.size(); i++)
  {
    std::cout << "CABLE PARAMETERS #" << i << "\n============================="
              << "\n l0\t\t\t" << config_params_.cables[i].l0 << "\n motor_cable_tau\t\t"
              << config_params_.cables[i].motor_cable_tau << "\n motor_encoder_res\t\t"
              << config_params_.cables[i].motor_encoder_res
              << "\n swivel_pulley_encoder_res\t"
              << config_params_.cables[i].swivel_pulley_encoder_res
              << "\n swivel_pulley_r\t\t" << config_params_.cables[i].swivel_pulley_r
              << "\n pos_OD_glob\n" << config_params_.cables[i].pos_OD_glob
              << " pos_PA_loc\n" << config_params_.cables[i].pos_PA_loc << " vers_i\n"
              << config_params_.cables[i].vers_i << " vers_j\n"
              << config_params_.cables[i].vers_j << " vers_k\n"
              << config_params_.cables[i].vers_k << std::endl;
  }
}

///////////////////////////////////////////////
//// Private methods
///////////////////////////////////////////////

bool RobotConfigJsonParser::ExtractConfig(const json& raw_data)
{
  if (!ExtractPlatform(raw_data))
    return false;

  config_params_.cables.clear();
  return ExtractCables(raw_data);
}

bool RobotConfigJsonParser::ExtractPlatform(const json& raw_data)
{
  if (raw_data.count("platform") != 1)
  {
    std::cerr << "[ERROR] Missing or invalid platform structure!" << std::endl;
    return false;
  }
  json platform = raw_data["platform"];
  std::string field;
  try
  {
    field = "mass";
    config_params_.platform->mass = platform[field];
    for (uint8_t i = 0; i < 3; i++)
    {
      field = "ext_force_loc";
      config_params_.platform->ext_force_loc(i + 1) = platform[field].at(i).at(0);
      field = "ext_torque_loc";
      config_params_.platform->ext_torque_loc(i + 1) = platform[field].at(i).at(0);
      field = "pos_PG_loc";
      config_params_.platform->pos_PG_loc(i + 1) = platform[field].at(i).at(0);
      field = "inertia_mat_G_loc";
      config_params_.platform->inertia_mat_G_loc.SetRow(
        i + 1, platform[field].at(i).get<std::vector<double>>());
    }
  }
  catch (json::type_error)
  {
    std::cerr << "[ERROR] Missing or invalid platform parameter field: " << field
              << std::endl;
    return false;
  }
  return ArePlatformParamsValid();
}

bool RobotConfigJsonParser::ExtractCables(const json& raw_data)
{
  if (raw_data.count("cable") != 1)
  {
    std::cerr << "[ERROR] Missing or invalid cable structure!" << std::endl;
    return false;
  }
  json cables = raw_data["cable"];
  std::string field;
  for (auto& cable : cables)
  {
    grabcdpr::CableParams temp;
    try
    {
      field = "l0";
      temp.l0 = cable[field];
      field = "motor_cable_tau";
      temp.motor_cable_tau = cable[field];
      field = "motor_encoder_res";
      temp.motor_encoder_res = cable[field];
      field = "swivel_pulley_encoder_res";
      temp.swivel_pulley_encoder_res = cable[field];
      field = "swivel_pulley_r";
      temp.swivel_pulley_r = cable[field];
      for (uint8_t i = 0; i < 3; i++)
      {
        field = "pos_OD_glob";
        temp.pos_OD_glob(i + 1) = cable[field].at(i).at(0);
        field = "pos_PA_loc";
        temp.pos_PA_loc(i + 1) = cable[field].at(i).at(0);
        field = "vers_i";
        temp.vers_i(i + 1) = cable[field].at(i).at(0);
        field = "vers_j";
        temp.vers_j(i + 1) = cable[field].at(i).at(0);
        field = "vers_k";
        temp.vers_k(i + 1) = cable[field].at(i).at(0);
      }
    }
    catch (json::type_error)
    {
      std::cerr << "[ERROR] Missing or invalid cable parameter field: " << field
                << std::endl;
      return false;
    }

    if (!AreCableParamsValid(temp))
      return false;
    config_params_.cables.push_back(temp);
  }
  return true;
}

bool RobotConfigJsonParser::ArePlatformParamsValid() const
{
  bool ret = true;

  if (config_params_.platform->mass <= 0.0)
  {
    std::cerr << "[ERROR] platform mass must be strictly positive!" << std::endl;
    ret = false;
  }

  if (!config_params_.platform->inertia_mat_G_loc.IsPositiveDefinite())
  {
    std::cerr << "[ERROR] platform inertia matrix must be positive definite!"
              << std::endl;
    ret = false;
  }

  return ret;
}

bool RobotConfigJsonParser::AreCableParamsValid(const grabcdpr::CableParams& params) const
{
  bool ret = true;

  if (params.l0 < 0.0)
  {
    std::cerr << "[ERROR] cable length must be non negative!" << std::endl;
    ret = false;
  }

  if (params.motor_cable_tau <= 0.0)
  {
    std::cerr
      << "[ERROR] cable length-to-motor revolution ratio must be strictly positive!"
      << std::endl;
    ret = false;
  }

  if (params.motor_encoder_res <= 0.0)
  {
    std::cerr << "[ERROR] motor encoder resolution must be strictly positive!"
              << std::endl;
    ret = false;
  }

  if (params.swivel_pulley_encoder_res <= 0.0)
  {
    std::cerr << "[ERROR] swivel pulley encoder resolution must be strictly positive!"
              << std::endl;
    ret = false;
  }

  if (params.swivel_pulley_r < 0.0)
  {
    std::cerr << "[ERROR] swivel pulley radius must be non negative!" << std::endl;
    ret = false;
  }

  return ret;
}
