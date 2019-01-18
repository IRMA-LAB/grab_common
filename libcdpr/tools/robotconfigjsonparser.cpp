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
            << "\n mass\t\t" << config_params_.platform->mass << "\n ext_force_loc\n"
            << config_params_.platform->ext_force_loc << " ext_torque_loc\n"
            << config_params_.platform->ext_torque_loc << " pos_PG_loc\n"
            << config_params_.platform->pos_PG_loc << " inertia_mat_G_loc\n"
            << config_params_.platform->inertia_mat_G_loc << std::endl;
  for (size_t i = 0; i < config_params_.actuators.size(); i++)
  {
    std::cout << "ACTUATOR PARAMETERS #" << i << "\n============================="
              << "\n " << (config_params_.actuators[i].active
                             ? "ACTIVE\n-----------"
                             : "INACTIVE\n--------------")
              << "\n Winch:\n-----------"
              << "\n   l0\t\t" << config_params_.actuators[i].winch.l0
              << "\n   drum_pitch\t" << config_params_.actuators[i].winch.drum_pitch
              << "\n   drum_diameter\t" << config_params_.actuators[i].winch.drum_diameter
              << "\n   gear_ratio\t\t" << config_params_.actuators[i].winch.gear_ratio
              << "\n   motor_encoder_res\t"
              << config_params_.actuators[i].winch.motor_encoder_res
              << "\n   pos_PA_loc\n" << config_params_.actuators[i].winch.pos_PA_loc
              << "\n Swivel Pulley:\n--------------------"
              << "\n   encoder_res\t" << config_params_.actuators[i].pulley.encoder_res
              << "\n   radius\t\t" << config_params_.actuators[i].pulley.radius
              << "\n   pos_OD_glob\n" << config_params_.actuators[i].pulley.pos_OD_glob
              << "   vers_i\n" << config_params_.actuators[i].pulley.vers_i
              << "   vers_j\n" << config_params_.actuators[i].pulley.vers_j
              << "   vers_k\n" << config_params_.actuators[i].pulley.vers_k << std::endl;
  }
}

///////////////////////////////////////////////
//// Private methods
///////////////////////////////////////////////

bool RobotConfigJsonParser::ExtractConfig(const json& raw_data)
{
  if (!ExtractPlatform(raw_data))
    return false;

  config_params_.actuators.clear();
  return ExtractActuators(raw_data);
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

bool RobotConfigJsonParser::ExtractActuators(const json& raw_data)
{
  if (raw_data.count("actuator") != 1)
  {
    std::cerr << "[ERROR] Missing or invalid actuator structure!" << std::endl;
    return false;
  }
  json actuators = raw_data["actuator"];
  std::string field, subfield;
  for (auto& actuator : actuators)
  {
    grabcdpr::ActuatorParams temp;
    try
    {
      field = "active";
      subfield = "";
      temp.active = actuator[field];
      field = "winch";
      subfield = "drum_pitch";
      temp.winch.drum_pitch = actuator[field][subfield];
      subfield = "drum_diameter";
      temp.winch.drum_diameter = actuator[field][subfield];
      subfield = "gear_ratio";
      temp.winch.gear_ratio = actuator[field][subfield];
      subfield = "l0";
      temp.winch.l0 = actuator[field][subfield];
      subfield = "motor_encoder_res";
      temp.winch.motor_encoder_res = actuator[field][subfield];

      for (uint8_t i = 0; i < 3; i++)
      {
        field = "winch";
        subfield = "pos_PA_loc";
        temp.winch.pos_PA_loc(i + 1) = actuator[field][subfield].at(i).at(0);

        field = "pulley";
        subfield = "pos_OD_glob";
        temp.pulley.pos_OD_glob(i + 1) = actuator[field][subfield].at(i).at(0);
        subfield = "vers_i";
        temp.pulley.vers_i(i + 1) = actuator[field][subfield].at(i).at(0);
        subfield = "vers_j";
        temp.pulley.vers_j(i + 1) = actuator[field][subfield].at(i).at(0);
        subfield = "vers_k";
        temp.pulley.vers_k(i + 1) = actuator[field][subfield].at(i).at(0);
      }

      subfield = "encoder_res";
      temp.pulley.encoder_res = actuator[field][subfield];
      subfield = "radius";
      temp.pulley.radius = actuator[field][subfield];
    }
    catch (json::type_error)
    {
      std::cerr << "[ERROR] Missing or invalid actuator parameter: " << field
                << (subfield == "" ? "" : "-") << subfield << std::endl;
      return false;
    }

    if (!AreCableParamsValid(temp))
      return false;
    config_params_.actuators.push_back(temp);
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

bool RobotConfigJsonParser::AreCableParamsValid(
  const grabcdpr::ActuatorParams& params) const
{
  bool ret = true;

  if (params.winch.l0 < 0.0)
  {
    std::cerr << "[ERROR] cable length must be non negative!" << std::endl;
    ret = false;
  }

  if (params.winch.drum_diameter <= 0.0)
  {
    std::cerr << "[ERROR] motor drum diameter must be strictly positive!" << std::endl;
    ret = false;
  }

  if (params.winch.drum_pitch <= 0.0)
  {
    std::cerr << "[ERROR] motor drum pitch must be strictly positive!" << std::endl;
    ret = false;
  }

  if (params.winch.gear_ratio <= 0.0)
  {
    std::cerr << "[ERROR] motor gear ration must be strictly positive!" << std::endl;
    ret = false;
  }

  if (params.winch.motor_encoder_res <= 0.0)
  {
    std::cerr << "[ERROR] motor encoder resolution must be strictly positive!"
              << std::endl;
    ret = false;
  }

  if (params.pulley.encoder_res <= 0.0)
  {
    std::cerr << "[ERROR] swivel pulley encoder resolution must be strictly positive!"
              << std::endl;
    ret = false;
  }

  if (params.pulley.radius < 0.0)
  {
    std::cerr << "[ERROR] swivel pulley radius must be non negative!" << std::endl;
    ret = false;
  }

  return ret;
}
