#include <QString>
#include <QtTest>

#include <fstream>
#include <iostream>
#include <string>

#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"

#include "cdpr_types.h"
#include "diffkinematics.h"
#include "kinematics.h"
#include "robotconfigjsonparser.h"

using namespace matlab::engine;

class LibcdprTest: public QObject
{
  Q_OBJECT

 private:
  std::unique_ptr<MATLABEngine> matlab_ptr_;
  // Create MATLAB data array factory
  matlab::data::ArrayFactory factory_;
  grabcdpr::RobotParams params_;

  void addCable2WS(const grabcdpr::CableVars& cable, const std::string& var_name);
  void addPlatform2WS(const grabcdpr::PlatformVars& platform,
                      const std::string& var_name);
  void addRobot2WS(const grabcdpr::RobotVars& robot, const std::string& var_name);

  grabcdpr::CableVars getCableFromWS(const std::string& var_name);
  grabcdpr::PlatformVars getPlatformFromWS(const std::string& var_name);
  grabcdpr::RobotVars getRobotFromWS(const std::string& var_name);

 private Q_SLOTS:
  void initTestCase();

  // Tools
  void testRobotConfigJsonParser();

  // Kinematics
  void testUpdatePlatformPose();
  void testUpdatePosA();
  void testCalcPulleyVersors();
  void testCalcSwivelAngle();
  void testCalcTangentAngle();
  void testCalcCableVectors();
  void testUpdateJacobiansRow();
  void testUpdateCableZeroOrd();
  void testUpdateUpdateIKZeroOrd();
};

void LibcdprTest::addCable2WS(const grabcdpr::CableVars& cable,
                              const std::string& var_name)
{
  // Create variables
  auto length         = factory_.createScalar<double>(cable.length);
  auto swivel_ang     = factory_.createScalar<double>(cable.swivel_ang);
  auto tan_ang        = factory_.createScalar<double>(cable.tan_ang);
  auto speed          = factory_.createScalar<double>(cable.speed);
  auto swivel_ang_vel = factory_.createScalar<double>(cable.swivel_ang_vel);
  auto tan_ang_vel    = factory_.createScalar<double>(cable.tan_ang_vel);
  auto acceleration   = factory_.createScalar<double>(cable.acceleration);
  auto swivel_ang_acc = factory_.createScalar<double>(cable.swivel_ang_acc);
  auto tan_ang_acc    = factory_.createScalar<double>(cable.tan_ang_acc);
  auto pos_PA_glob    = factory_.createArray<double>({3, 1}, cable.pos_PA_glob.Begin(),
                                                  cable.pos_PA_glob.End());
  auto pos_OA_glob    = factory_.createArray<double>({3, 1}, cable.pos_OA_glob.Begin(),
                                                  cable.pos_OA_glob.End());
  auto pos_DA_glob    = factory_.createArray<double>({3, 1}, cable.pos_DA_glob.Begin(),
                                                  cable.pos_DA_glob.End());
  auto pos_BA_glob    = factory_.createArray<double>({3, 1}, cable.pos_BA_glob.Begin(),
                                                  cable.pos_BA_glob.End());
  auto vers_u =
    factory_.createArray<double>({3, 1}, cable.vers_u.Begin(), cable.vers_u.End());
  auto vers_w =
    factory_.createArray<double>({3, 1}, cable.vers_w.Begin(), cable.vers_w.End());
  auto vers_n =
    factory_.createArray<double>({3, 1}, cable.vers_n.Begin(), cable.vers_n.End());
  auto vers_t =
    factory_.createArray<double>({3, 1}, cable.vers_t.Begin(), cable.vers_t.End());
  auto vel_OA_glob    = factory_.createArray<double>({3, 1}, cable.vel_OA_glob.Begin(),
                                                  cable.vel_OA_glob.End());
  auto vel_BA_glob    = factory_.createArray<double>({3, 1}, cable.vel_BA_glob.Begin(),
                                                  cable.vel_BA_glob.End());
  auto vers_u_dot     = factory_.createArray<double>({3, 1}, cable.vers_u_dot.Begin(),
                                                 cable.vers_u_dot.End());
  auto vers_w_dot     = factory_.createArray<double>({3, 1}, cable.vers_w_dot.Begin(),
                                                 cable.vers_w_dot.End());
  auto vers_n_dot     = factory_.createArray<double>({3, 1}, cable.vers_n_dot.Begin(),
                                                 cable.vers_n_dot.End());
  auto vers_t_dot     = factory_.createArray<double>({3, 1}, cable.vers_t_dot.Begin(),
                                                 cable.vers_t_dot.End());
  auto acc_OA_glob    = factory_.createArray<double>({3, 1}, cable.acc_OA_glob.Begin(),
                                                  cable.acc_OA_glob.End());
  auto geom_jacob_row = factory_.createArray<double>(
    {1, POSE_DIM}, cable.geom_jacob_row.Begin(), cable.geom_jacob_row.End());
  auto anal_jacob_row = factory_.createArray<double>(
    {1, POSE_DIM}, cable.anal_jacob_row.Begin(), cable.anal_jacob_row.End());

  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"length", std::move(length));
  matlab_ptr_->setVariable(u"swivel_ang", std::move(swivel_ang));
  matlab_ptr_->setVariable(u"tan_ang", std::move(tan_ang));
  matlab_ptr_->setVariable(u"speed", std::move(speed));
  matlab_ptr_->setVariable(u"swivel_ang_vel", std::move(swivel_ang_vel));
  matlab_ptr_->setVariable(u"tan_ang_vel", std::move(tan_ang_vel));
  matlab_ptr_->setVariable(u"acceleration", std::move(acceleration));
  matlab_ptr_->setVariable(u"swivel_ang_acc", std::move(swivel_ang_acc));
  matlab_ptr_->setVariable(u"tan_ang_acc", std::move(tan_ang_acc));
  matlab_ptr_->setVariable(u"pos_PA_glob", std::move(pos_PA_glob));
  matlab_ptr_->setVariable(u"pos_OA_glob", std::move(pos_OA_glob));
  matlab_ptr_->setVariable(u"pos_DA_glob", std::move(pos_DA_glob));
  matlab_ptr_->setVariable(u"pos_BA_glob", std::move(pos_BA_glob));
  matlab_ptr_->setVariable(u"vers_u", std::move(vers_u));
  matlab_ptr_->setVariable(u"vers_w", std::move(vers_w));
  matlab_ptr_->setVariable(u"vers_n", std::move(vers_n));
  matlab_ptr_->setVariable(u"vers_t", std::move(vers_t));
  matlab_ptr_->setVariable(u"geom_jacob_row", std::move(geom_jacob_row));
  matlab_ptr_->setVariable(u"anal_jacob_row", std::move(anal_jacob_row));
  matlab_ptr_->setVariable(u"vel_OA_glob", std::move(vel_OA_glob));
  matlab_ptr_->setVariable(u"vel_BA_glob", std::move(vel_BA_glob));
  matlab_ptr_->setVariable(u"vers_u_dot", std::move(vers_u_dot));
  matlab_ptr_->setVariable(u"vers_w_dot", std::move(vers_w_dot));
  matlab_ptr_->setVariable(u"vers_n_dot", std::move(vers_n_dot));
  matlab_ptr_->setVariable(u"vers_t_dot", std::move(vers_t_dot));
  matlab_ptr_->setVariable(u"acc_OA_glob", std::move(acc_OA_glob));

  // Create cable structure in matlab workspace
  auto var_name_u = convertUTF8StringToUTF16String(var_name);
  matlab_ptr_->eval(var_name_u + u" = CableVar();");
  // Fill fields one by one
  matlab_ptr_->eval(var_name_u + u".length = length;");
  matlab_ptr_->eval(var_name_u + u".swivel_ang = swivel_ang;");
  matlab_ptr_->eval(var_name_u + u".tan_ang = tan_ang;");
  matlab_ptr_->eval(var_name_u + u".pos_PA_glob = pos_PA_glob;");
  matlab_ptr_->eval(var_name_u + u".pos_OA_glob = pos_OA_glob;");
  matlab_ptr_->eval(var_name_u + u".pos_DA_glob = pos_DA_glob;");
  matlab_ptr_->eval(var_name_u + u".pos_BA_glob = pos_BA_glob;");
  matlab_ptr_->eval(var_name_u + u".vers_u = vers_u;");
  matlab_ptr_->eval(var_name_u + u".vers_w = vers_w;");
  matlab_ptr_->eval(var_name_u + u".vers_n = vers_n;");
  matlab_ptr_->eval(var_name_u + u".vers_rho = vers_t;");
  matlab_ptr_->eval(var_name_u + u".geometric_jacobian_row = geom_jacob_row;");
  matlab_ptr_->eval(var_name_u + u".speed = speed;");
  matlab_ptr_->eval(var_name_u + u".swivel_ang_vel = swivel_ang_vel;");
  matlab_ptr_->eval(var_name_u + u".tan_ang_vel = tan_ang_vel;");
  matlab_ptr_->eval(var_name_u + u".vel_OA_glob = vel_OA_glob;");
  matlab_ptr_->eval(var_name_u + u".vel_BA_glob = vel_BA_glob;");
  matlab_ptr_->eval(var_name_u + u".vers_u_deriv = vers_u_dot;");
  matlab_ptr_->eval(var_name_u + u".vers_w_deriv = vers_w_dot;");
  matlab_ptr_->eval(var_name_u + u".vers_n_deriv = vers_n_dot;");
  matlab_ptr_->eval(var_name_u + u".vers_rho_deriv = vers_t_dot;");
  matlab_ptr_->eval(var_name_u + u".acceleration = acceleration;");
  matlab_ptr_->eval(var_name_u + u".swivel_ang_acc = swivel_ang_acc;");
  matlab_ptr_->eval(var_name_u + u".tan_ang_acc = tan_ang_acc;");
  matlab_ptr_->eval(var_name_u + u".acc_OA_glob = acc_OA_glob;");
  matlab_ptr_->eval(var_name_u + u".analitic_jacobian_row = anal_jacob_row;");
}

void LibcdprTest::addPlatform2WS(const grabcdpr::PlatformVars& platform,
                                 const std::string& var_name)
{
  // Create variables
  auto position     = factory_.createArray<double>({3, 1}, platform.position.Begin(),
                                               platform.position.End());
  auto pos_PG_glob  = factory_.createArray<double>({3, 1}, platform.pos_PG_glob.Begin(),
                                                  platform.pos_PG_glob.End());
  auto pos_OG_glob  = factory_.createArray<double>({3, 1}, platform.pos_OG_glob.Begin(),
                                                  platform.pos_OG_glob.End());
  auto velocity     = factory_.createArray<double>({3, 1}, platform.velocity.Begin(),
                                               platform.velocity.End());
  auto angular_vel  = factory_.createArray<double>({3, 1}, platform.angular_vel.Begin(),
                                                  platform.angular_vel.End());
  auto vel_OG_glob  = factory_.createArray<double>({3, 1}, platform.vel_OG_glob.Begin(),
                                                  platform.vel_OG_glob.End());
  auto acceleration = factory_.createArray<double>({3, 1}, platform.acceleration.Begin(),
                                                   platform.acceleration.End());
  auto angular_acc  = factory_.createArray<double>({3, 1}, platform.angular_acc.Begin(),
                                                  platform.angular_acc.End());
  auto acc_OG_glob  = factory_.createArray<double>({3, 1}, platform.acc_OG_glob.Begin(),
                                                  platform.acc_OG_glob.End());
  auto orientation  = factory_.createArray<double>({3, 1}, platform.orientation.Begin(),
                                                  platform.orientation.End());
  auto orientation_dot = factory_.createArray<double>(
    {3, 1}, platform.orientation_dot.Begin(), platform.orientation_dot.End());
  auto orientation_ddot = factory_.createArray<double>(
    {3, 1}, platform.orientation_ddot.Begin(), platform.orientation_ddot.End());
  auto rot_mat = factory_.createArray<double>({3, 3}, platform.rot_mat.Begin(),
                                              platform.rot_mat.End());
  auto h_mat =
    factory_.createArray<double>({3, 3}, platform.h_mat.Begin(), platform.h_mat.End());
  auto dh_mat =
    factory_.createArray<double>({3, 3}, platform.dh_mat.Begin(), platform.dh_mat.End());
  auto pose = factory_.createArray<double>({POSE_DIM, 1}, platform.pose.Begin(),
                                           platform.pose.End());

  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"position", std::move(position));
  matlab_ptr_->setVariable(u"pos_PG_glob", std::move(pos_PG_glob));
  matlab_ptr_->setVariable(u"pos_OG_glob", std::move(pos_OG_glob));
  matlab_ptr_->setVariable(u"velocity", std::move(velocity));
  matlab_ptr_->setVariable(u"angular_vel", std::move(angular_vel));
  matlab_ptr_->setVariable(u"vel_OG_glob", std::move(vel_OG_glob));
  matlab_ptr_->setVariable(u"acceleration", std::move(acceleration));
  matlab_ptr_->setVariable(u"angular_acc", std::move(angular_acc));
  matlab_ptr_->setVariable(u"acc_OG_glob", std::move(acc_OG_glob));
  matlab_ptr_->setVariable(u"orientation", std::move(orientation));
  matlab_ptr_->setVariable(u"orientation_dot", std::move(orientation_dot));
  matlab_ptr_->setVariable(u"orientation_ddot", std::move(orientation_ddot));
  matlab_ptr_->setVariable(u"rot_mat", std::move(rot_mat));
  matlab_ptr_->setVariable(u"h_mat", std::move(h_mat));
  matlab_ptr_->setVariable(u"dh_mat", std::move(dh_mat));
  matlab_ptr_->setVariable(u"pose", std::move(pose));

  // Create cable structure in matlab workspace
  auto var_name_u = convertUTF8StringToUTF16String(var_name);
  matlab_ptr_->eval(var_name_u + u" = PlatformVar();");
  // Fill fields one by one
  matlab_ptr_->eval(var_name_u + u".position = position;");
  matlab_ptr_->eval(var_name_u + u".pos_PG_glob = pos_PG_glob;");
  matlab_ptr_->eval(var_name_u + u".pos_OG_glob = pos_OG_glob;");
  matlab_ptr_->eval(var_name_u + u".velocity = velocity;");
  matlab_ptr_->eval(var_name_u + u".angular_vel = angular_vel;");
  matlab_ptr_->eval(var_name_u + u".vel_OG_glob = vel_OG_glob;");
  matlab_ptr_->eval(var_name_u + u".acceleration = acceleration;");
  matlab_ptr_->eval(var_name_u + u".angular_acc = angular_acc;");
  matlab_ptr_->eval(var_name_u + u".acc_OG_glob = acc_OG_glob;");
  matlab_ptr_->eval(var_name_u + u".orientation = orientation;");
  matlab_ptr_->eval(var_name_u + u".orientation_deriv = orientation_dot;");
  matlab_ptr_->eval(var_name_u + u".orientation_deriv_2 = orientation_ddot;");
  matlab_ptr_->eval(var_name_u + u".rot_mat = rot_mat;");
  matlab_ptr_->eval(var_name_u + u".H_mat = h_mat;");
  matlab_ptr_->eval(var_name_u + u".H_mat_deriv = dh_mat;");
  matlab_ptr_->eval(var_name_u + u".pose = pose;");
}


void LibcdprTest::addRobot2WS(const grabcdpr::RobotVars& robot,
                              const std::string& var_name)
{
  auto var_name_u = convertUTF8StringToUTF16String(var_name);
  auto num_cables = convertUTF8StringToUTF16String(std::to_string(robot.cables.size()));
  matlab_ptr_->eval(var_name_u + u" = CdprVar(" + num_cables + u");");
  addPlatform2WS(robot.platform, var_name + ".platform");
  for (uint i = 0; i < robot.cables.size(); i++)
    addCable2WS(robot.cables[i], var_name + ".cable(" + std::to_string(i + 1) + ", 1)");
}

grabcdpr::CableVars LibcdprTest::getCableFromWS(const std::string& var_name)
{
  // Get cable object
  matlab::data::Array cable_matlab =
    matlab_ptr_->getVariable(convertUTF8StringToUTF16String(var_name));

  // Get cable properties (only array ones)
  matlab::data::TypedArray<double> pos_PA_glob =
    matlab_ptr_->getProperty(cable_matlab, u"pos_PA_glob");
  matlab::data::TypedArray<double> pos_OA_glob =
    matlab_ptr_->getProperty(cable_matlab, u"pos_OA_glob");
  matlab::data::TypedArray<double> pos_DA_glob =
    matlab_ptr_->getProperty(cable_matlab, u"pos_DA_glob");
  matlab::data::TypedArray<double> pos_BA_glob =
    matlab_ptr_->getProperty(cable_matlab, u"pos_BA_glob");
  matlab::data::TypedArray<double> vers_u =
    matlab_ptr_->getProperty(cable_matlab, u"vers_u");
  matlab::data::TypedArray<double> vers_w =
    matlab_ptr_->getProperty(cable_matlab, u"vers_w");
  matlab::data::TypedArray<double> vers_n =
    matlab_ptr_->getProperty(cable_matlab, u"vers_n");
  matlab::data::TypedArray<double> vers_t =
    matlab_ptr_->getProperty(cable_matlab, u"vers_rho");
  matlab::data::TypedArray<double> vel_OA_glob =
    matlab_ptr_->getProperty(cable_matlab, u"vel_OA_glob");
  matlab::data::TypedArray<double> vel_BA_glob =
    matlab_ptr_->getProperty(cable_matlab, u"vel_BA_glob");
  matlab::data::TypedArray<double> vers_u_dot =
    matlab_ptr_->getProperty(cable_matlab, u"vers_u_deriv");
  matlab::data::TypedArray<double> vers_w_dot =
    matlab_ptr_->getProperty(cable_matlab, u"vers_w_deriv");
  matlab::data::TypedArray<double> vers_n_dot =
    matlab_ptr_->getProperty(cable_matlab, u"vers_n_deriv");
  matlab::data::TypedArray<double> vers_t_dot =
    matlab_ptr_->getProperty(cable_matlab, u"vers_rho_deriv");
  matlab::data::TypedArray<double> acc_OA_glob =
    matlab_ptr_->getProperty(cable_matlab, u"acc_OA_glob");
  matlab::data::TypedArray<double> geom_jacob_row =
    matlab_ptr_->getProperty(cable_matlab, u"geometric_jacobian_row");
  matlab::data::TypedArray<double> anal_jacob_row =
    matlab_ptr_->getProperty(cable_matlab, u"analitic_jacobian_row");

  // Build corresponding cable structure
  grabcdpr::CableVars cable;
  cable.length         = matlab_ptr_->getProperty(cable_matlab, u"length")[0];
  cable.swivel_ang     = matlab_ptr_->getProperty(cable_matlab, u"swivel_ang")[0];
  cable.tan_ang        = matlab_ptr_->getProperty(cable_matlab, u"tan_ang")[0];
  cable.speed          = matlab_ptr_->getProperty(cable_matlab, u"speed")[0];
  cable.swivel_ang_vel = matlab_ptr_->getProperty(cable_matlab, u"swivel_ang_vel")[0];
  cable.tan_ang_vel    = matlab_ptr_->getProperty(cable_matlab, u"tan_ang_vel")[0];
  cable.acceleration   = matlab_ptr_->getProperty(cable_matlab, u"acceleration")[0];
  cable.swivel_ang_acc = matlab_ptr_->getProperty(cable_matlab, u"swivel_ang_acc")[0];
  cable.tan_ang_acc    = matlab_ptr_->getProperty(cable_matlab, u"tan_ang_acc")[0];
  cable.pos_PA_glob.Fill(pos_PA_glob.begin(), pos_PA_glob.end());
  cable.pos_OA_glob.Fill(pos_OA_glob.begin(), pos_OA_glob.end());
  cable.pos_DA_glob.Fill(pos_DA_glob.begin(), pos_DA_glob.end());
  cable.pos_BA_glob.Fill(pos_BA_glob.begin(), pos_BA_glob.end());
  cable.vers_u.Fill(vers_u.begin(), vers_u.end());
  cable.vers_w.Fill(vers_w.begin(), vers_w.end());
  cable.vers_n.Fill(vers_n.begin(), vers_n.end());
  cable.vers_t.Fill(vers_t.begin(), vers_t.end());
  cable.vel_OA_glob.Fill(vel_OA_glob.begin(), vel_OA_glob.end());
  cable.vel_BA_glob.Fill(vel_BA_glob.begin(), vel_BA_glob.end());
  cable.vers_u_dot.Fill(vers_u_dot.begin(), vers_u_dot.end());
  cable.vers_w_dot.Fill(vers_w_dot.begin(), vers_w_dot.end());
  cable.vers_n_dot.Fill(vers_n_dot.begin(), vers_n_dot.end());
  cable.vers_t_dot.Fill(vers_t_dot.begin(), vers_t_dot.end());
  cable.acc_OA_glob.Fill(acc_OA_glob.begin(), acc_OA_glob.end());
  cable.geom_jacob_row.Fill(geom_jacob_row.begin(), geom_jacob_row.end());
  cable.anal_jacob_row.Fill(anal_jacob_row.begin(), anal_jacob_row.end());

  return cable;
}

grabcdpr::PlatformVars LibcdprTest::getPlatformFromWS(const std::string& var_name)
{
  // Get platform object
  matlab::data::Array platform_matlab =
    matlab_ptr_->getVariable(convertUTF8StringToUTF16String(var_name));

  // Get platform properties (only array ones)
  matlab::data::TypedArray<double> position =
    matlab_ptr_->getProperty(platform_matlab, u"position");
  matlab::data::TypedArray<double> pos_PG_glob =
    matlab_ptr_->getProperty(platform_matlab, u"pos_PG_glob");
  matlab::data::TypedArray<double> pos_OG_glob =
    matlab_ptr_->getProperty(platform_matlab, u"pos_OG_glob");
  matlab::data::TypedArray<double> velocity =
    matlab_ptr_->getProperty(platform_matlab, u"velocity");
  matlab::data::TypedArray<double> angular_vel =
    matlab_ptr_->getProperty(platform_matlab, u"angular_vel");
  matlab::data::TypedArray<double> vel_OG_glob =
    matlab_ptr_->getProperty(platform_matlab, u"vel_OG_glob");
  matlab::data::TypedArray<double> acceleration =
    matlab_ptr_->getProperty(platform_matlab, u"acceleration");
  matlab::data::TypedArray<double> angular_acc =
    matlab_ptr_->getProperty(platform_matlab, u"angular_acc");
  matlab::data::TypedArray<double> acc_OG_glob =
    matlab_ptr_->getProperty(platform_matlab, u"acc_OG_glob");
  matlab::data::TypedArray<double> orientation =
    matlab_ptr_->getProperty(platform_matlab, u"orientation");
  matlab::data::TypedArray<double> orientation_dot =
    matlab_ptr_->getProperty(platform_matlab, u"orientation_deriv");
  matlab::data::TypedArray<double> orientation_ddot =
    matlab_ptr_->getProperty(platform_matlab, u"orientation_deriv_2");
  matlab::data::TypedArray<double> pose =
    matlab_ptr_->getProperty(platform_matlab, u"pose");
  matlab::data::TypedArray<double> rot_mat =
    matlab_ptr_->getProperty(platform_matlab, u"rot_mat");
  matlab::data::TypedArray<double> h_mat =
    matlab_ptr_->getProperty(platform_matlab, u"H_mat");
  matlab::data::TypedArray<double> dh_mat =
    matlab_ptr_->getProperty(platform_matlab, u"H_mat_deriv");

  // Build corresponding platform structure
  grabcdpr::PlatformVars platform;
  platform.position.Fill(position.begin(), position.end());
  platform.pos_PG_glob.Fill(pos_PG_glob.begin(), pos_PG_glob.end());
  platform.pos_OG_glob.Fill(pos_OG_glob.begin(), pos_OG_glob.end());
  platform.velocity.Fill(velocity.begin(), velocity.end());
  platform.angular_vel.Fill(angular_vel.begin(), angular_vel.end());
  platform.vel_OG_glob.Fill(vel_OG_glob.begin(), vel_OG_glob.end());
  platform.acceleration.Fill(acceleration.begin(), acceleration.end());
  platform.angular_acc.Fill(angular_acc.begin(), angular_acc.end());
  platform.acc_OG_glob.Fill(acc_OG_glob.begin(), acc_OG_glob.end());
  platform.orientation.Fill(orientation.begin(), orientation.end());
  platform.orientation_dot.Fill(orientation_dot.begin(), orientation_dot.end());
  platform.orientation_ddot.Fill(orientation_ddot.begin(), orientation_ddot.end());
  platform.pose.Fill(pose.begin(), pose.end());
  // Matlab iterator works column-by-column, so we need to transpose all matrices
  platform.rot_mat = grabnum::Matrix3d(rot_mat.begin(), rot_mat.end()).Transpose();
  platform.h_mat   = grabnum::Matrix3d(h_mat.begin(), h_mat.end()).Transpose();
  platform.dh_mat  = grabnum::Matrix3d(dh_mat.begin(), dh_mat.end()).Transpose();

  return platform;
}

grabcdpr::RobotVars LibcdprTest::getRobotFromWS(const std::string& var_name)
{
  auto var_name_u = convertUTF8StringToUTF16String(var_name);
  grabcdpr::RobotVars robot(params_.platform.rot_parametrization);
  // Get platform
  robot.platform = getPlatformFromWS(var_name + ".platform");
  // Get cables
  for (uint i = 1; i <= params_.actuators.size(); i++)
    robot.cables.push_back(
      getCableFromWS(var_name + ".cable(" + std::to_string(i) + ", 1)"));
  return robot;
}

//--------- Init ---------------//

void LibcdprTest::initTestCase()
{
  // Start matlab engine synchronously
  matlab_ptr_ = startMATLAB();

  // Load same robot parameters in matlab workspace
  matlab_ptr_->eval(u"cdpr_p = "
                    u"CdprParameter('/home/simo/repos/cable_robot/libs/grab_common/"
                    u"libcdpr/tests', 'pass.json');");
}

//--------- Tools ---------------//

void LibcdprTest::testRobotConfigJsonParser()
{
  // test parsing with different inputs
  RobotConfigJsonParser parser;
  parser.ParseFile("../tests/fail1.json");
  parser.ParseFile(QString("../tests/fail2.json"));
  parser.ParseFile(std::string("../tests/pass.json"));

  // test getters
  params_ = parser.GetConfigStruct();
  parser.GetConfigStruct(&params_);

  // test display
  //  parser.PrintConfig();
}

//--------- Kinematics ---------------//

void LibcdprTest::testUpdatePlatformPose()
{
  // Setup dummy input
  grabcdpr::PlatformVars platform;
  grabnum::Vector3d position({1, 2, 3});
  grabnum::Vector3d orientation({0.1, 0.2, 0.3});
  grabnum::Vector3d pos_PG_loc({0, -1, 0.5});

  // Load dummy input to Matlab workspace
  addPlatform2WS(platform, "platform_v");
  auto position_matlab =
    factory_.createArray<double>({3, 1}, position.Begin(), position.End());
  auto orientation_matlab =
    factory_.createArray<double>({3, 1}, orientation.Begin(), orientation.End());
  auto pos_PG_loc_matlab =
    factory_.createArray<double>({3, 1}, pos_PG_loc.Begin(), pos_PG_loc.End());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"position", std::move(position_matlab));
  matlab_ptr_->setVariable(u"orientation", std::move(orientation_matlab));
  matlab_ptr_->setVariable(u"pos_PG_loc", std::move(pos_PG_loc_matlab));

  // Call C++ function implementation to be tested
  grabcdpr::UpdatePlatformPose(position, orientation, pos_PG_loc, platform);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"platform_v = UpdatePlatformPose(position, orientation, "
                    u"RotationParametrizations.TILT_TORSION, pos_PG_loc, platform_v);");

  // Get matlab results
  grabcdpr::PlatformVars matlab_platform = getPlatformFromWS("platform_v");

  // Check they are the same
  QVERIFY(platform.pos_PG_glob.IsApprox(matlab_platform.pos_PG_glob));
  QVERIFY(platform.pos_OG_glob.IsApprox(matlab_platform.pos_OG_glob));
  QVERIFY(platform.position.IsApprox(matlab_platform.position));
  QVERIFY(platform.orientation.IsApprox(matlab_platform.orientation));
  QVERIFY(platform.rot_mat.IsApprox(matlab_platform.rot_mat));
}

void LibcdprTest::testUpdatePosA()
{
  // Setup dummy input
  grabcdpr::CableVars cable;
  grabcdpr::PlatformVars platform;
  platform.rot_mat = grabnum::Matrix3d(1.);
  platform.position.Fill({1, 2., 0.5});
  // Load dummy input to Matlab workspace
  addCable2WS(cable, "cable_v");
  addPlatform2WS(platform, "platform_v");

  // Call C++ function implementation to be tested
  grabcdpr::UpdatePosA(params_.actuators[0], platform, cable);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cable_v = "
                    u"UpdatePosA(cdpr_p.cable(1).pos_A_loc, cdpr_p.cable(1).pos_D_glob, "
                    u"platform_v.position, platform_v.rot_mat, cable_v);");

  // Get matlab results
  grabcdpr::CableVars matlab_cable = getCableFromWS("cable_v");

  // Check they are the same
  QCOMPARE(cable.pos_PA_glob, matlab_cable.pos_PA_glob);
  QCOMPARE(cable.pos_OA_glob, matlab_cable.pos_OA_glob);
  QCOMPARE(cable.pos_DA_glob, matlab_cable.pos_DA_glob);
}

void LibcdprTest::testCalcPulleyVersors()
{
  // Setup dummy input
  grabcdpr::CableVars cable;
  cable.swivel_ang = 0.5;
  // Load dummy input to Matlab workspace
  addCable2WS(cable, "cable_v");

  // Call C++ function implementation to be tested
  grabcdpr::CalcPulleyVersors(params_.actuators[0].pulley, cable);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cable_v = CalcPulleyVersors(cdpr_p.cable(1).vers_i, "
                    u"cdpr_p.cable(1).vers_j, cable_v);");

  // Get matlab results
  grabcdpr::CableVars matlab_cable = getCableFromWS("cable_v");

  // Check they are the same
  QCOMPARE(cable.vers_u, matlab_cable.vers_u);
  QCOMPARE(cable.vers_w, matlab_cable.vers_w);
}

void LibcdprTest::testCalcSwivelAngle()
{
  // Setup dummy input
  grabnum::Vector3d pos_DA_glob({1, 2, 3});
  // Load dummy input to Matlab workspace
  auto pos_DA_glob_matlab =
    factory_.createArray<double>({3, 1}, pos_DA_glob.Begin(), pos_DA_glob.End());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"pos_DA_glob", std::move(pos_DA_glob_matlab));

  // Call C++ function implementation to be tested
  double swivel_angle =
    grabcdpr::CalcSwivelAngle(params_.actuators[0].pulley, pos_DA_glob);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"swivel_angle = CalcSwivelAngle(cdpr_p.cable(1).vers_i, "
                    u"cdpr_p.cable(1).vers_j, pos_DA_glob);");

  // Get matlab results
  matlab::data::TypedArray<double> matlab_swivel_angle =
    matlab_ptr_->getVariable(u"swivel_angle");

  // Check they are the same
  QCOMPARE(swivel_angle, matlab_swivel_angle[0]);
}

void LibcdprTest::testCalcTangentAngle()
{
  // Setup dummy input
  grabnum::Vector3d vers_u({0, 1, 0});
  grabnum::Vector3d pos_DA_glob({1, 2, 3});
  // Load dummy input to Matlab workspace
  auto pos_DA_glob_matlab =
    factory_.createArray<double>({3, 1}, pos_DA_glob.Begin(), pos_DA_glob.End());
  auto vers_u_matlab = factory_.createArray<double>({3, 1}, vers_u.Begin(), vers_u.End());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"pos_DA_glob", std::move(pos_DA_glob_matlab));
  matlab_ptr_->setVariable(u"vers_u", std::move(vers_u_matlab));

  // Call C++ function implementation to be tested
  double tan_angle =
    grabcdpr::CalcTangentAngle(params_.actuators[0].pulley, vers_u, pos_DA_glob);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"tan_angle = CalcTangentAngle(cdpr_p.cable(1).vers_k, "
                    u"cdpr_p.cable(1).swivel_pulley_r, vers_u, pos_DA_glob);");

  // Get matlab results
  matlab::data::TypedArray<double> matlab_tan_angle =
    matlab_ptr_->getVariable(u"tan_angle");

  // Check they are the same
  QCOMPARE(tan_angle, matlab_tan_angle[0]);
}

void LibcdprTest::testCalcCableVectors()
{
  // Setup dummy input
  grabcdpr::CableVars cable;
  cable.tan_ang = 0.5;
  cable.vers_u.Fill({0, 0, 1});
  cable.pos_DA_glob.Fill({1, 2, 3});
  // Load dummy input to Matlab workspace
  addCable2WS(cable, "cable_v");

  // Call C++ function implementation to be tested
  grabcdpr::CalcCableVectors(params_.actuators[0].pulley, cable);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cable_v = CalcCableVectors(cdpr_p.cable(1).swivel_pulley_r, "
                    u"cdpr_p.cable(1).vers_k, cable_v);");

  // Get matlab results
  grabcdpr::CableVars matlab_cable = getCableFromWS("cable_v");

  // Check they are the same
  QCOMPARE(cable.vers_n, matlab_cable.vers_n);
  QCOMPARE(cable.vers_t, matlab_cable.vers_t);
  QCOMPARE(cable.pos_BA_glob, matlab_cable.pos_BA_glob);
}

void LibcdprTest::testUpdateJacobiansRow()
{
  // Setup dummy input
  grabcdpr::CableVars cable;
  cable.vers_u.Fill({0, 0, 1});
  grabnum::Matrix3d h_mat({1, 2, 3, 4, 5, 6, 7, 8, 9});
  // Load dummy input to Matlab workspace
  addCable2WS(cable, "cable_v");
  auto h_mat_matlab = factory_.createArray<double>({3, 3}, h_mat.Begin(), h_mat.End());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"h_mat", std::move(h_mat_matlab));

  // Call C++ function implementation to be tested
  grabcdpr::UpdateJacobiansRow(h_mat, cable);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"[geometric, analitic] = CalcPlatformJacobianRow(cable_v.vers_rho,"
                    u"cable_v.pos_PA_glob, h_mat);");

  // Get matlab results
  matlab::data::TypedArray<double> matlab_geom_jacob_row =
    matlab_ptr_->getVariable(u"geometric");
  matlab::data::TypedArray<double> matlab_anal_jacob_row =
    matlab_ptr_->getVariable(u"analitic");
  grabnum::MatrixXd<1, POSE_DIM> geom_jacob_row(matlab_geom_jacob_row.begin(),
                                                matlab_geom_jacob_row.end());
  grabnum::MatrixXd<1, POSE_DIM> anal_jacob_row(matlab_anal_jacob_row.begin(),
                                                matlab_anal_jacob_row.end());

  // Check they are the same
  QCOMPARE(cable.anal_jacob_row, geom_jacob_row);
  QCOMPARE(cable.geom_jacob_row, anal_jacob_row);
}

void LibcdprTest::testUpdateCableZeroOrd()
{
  // Setup dummy input
  grabcdpr::PlatformVars platform;
  grabnum::Vector3d position({1, 2, 3});
  grabnum::Matrix3d rot_mat({0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9});
  grabcdpr::CableVars cable;
  // Load dummy input to Matlab workspace
  addPlatform2WS(platform, "platform_v");
  addCable2WS(cable, "cable_v");

  // Call C++ function implementation to be tested
  grabcdpr::UpdateCableZeroOrd(params_.actuators[0], platform, cable);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cable_v = UpdateCableZeroOrd(cdpr_p.cable(1), "
                    u"platform_v, cable_v);");

  // Get matlab results
  grabcdpr::CableVars matlab_cable = getCableFromWS("cable_v");

  // Check they are the same
  QCOMPARE(cable.swivel_ang, matlab_cable.swivel_ang);
  QCOMPARE(cable.tan_ang, matlab_cable.tan_ang);
  //  QCOMPARE(cable.length, matlab_cable.length); // different implementation!!
  QVERIFY(cable.pos_DA_glob.IsApprox(matlab_cable.pos_DA_glob));
  QVERIFY(cable.vers_u.IsApprox(matlab_cable.vers_u));
  QVERIFY(cable.pos_BA_glob.IsApprox(matlab_cable.pos_BA_glob));
}

void LibcdprTest::testUpdateUpdateIKZeroOrd()
{
  // Setup dummy input
  grabcdpr::RobotVars robot(params_.actuators.size(),
                            params_.platform.rot_parametrization);
  grabnum::Vector3d position({1, 2, 3});
  grabnum::Vector3d orientation({0.1, 0.2, 0.3});
  // Load dummy input to Matlab workspace
  addRobot2WS(robot, "cdpr_v");
  auto position_matlab =
    factory_.createArray<double>({3, 1}, position.Begin(), position.End());
  auto orientation_matlab =
    factory_.createArray<double>({3, 1}, orientation.Begin(), orientation.End());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"position", std::move(position_matlab));
  matlab_ptr_->setVariable(u"orientation", std::move(orientation_matlab));

  // Call C++ function implementation to be tested
  grabcdpr::UpdateIK0(position, orientation, params_, robot);
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cdpr_v = UpdateIKZeroOrd(position, orientation, cdpr_p, cdpr_v);");

  // Get matlab results
  grabcdpr::RobotVars matlab_robot = getRobotFromWS("cdpr_v");

  // Check they are the same
  QCOMPARE(robot.platform.pose, matlab_robot.platform.pose);
  QCOMPARE(robot.platform.rot_mat, matlab_robot.platform.rot_mat);
  for (uint i = 0; i < robot.cables.size(); i++)
  {
    QCOMPARE(robot.cables[i].swivel_ang, matlab_robot.cables[i].swivel_ang);
    QCOMPARE(robot.cables[i].tan_ang, matlab_robot.cables[i].tan_ang);
    QCOMPARE(robot.cables[i].length, matlab_robot.cables[i].length);
  }
}

QTEST_APPLESS_MAIN(LibcdprTest)

#include "libcdpr_test.moc"
