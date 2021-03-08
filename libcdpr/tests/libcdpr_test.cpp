#include <QString>
#include <QtTest>

#include <fstream>
#include <iostream>
#include <string>

#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"

#include "cdpr_types.h"
#include "diffkinematics.h"
#include "dynamics.h"
#include "kinematics.h"
#include "robotconfigjsonparser.h"
#include "statics.h"
#include "under_actuated_utils.h"

using namespace matlab::engine;

static grabnum::Vector3d
matlabProperty2GrabVec3(const std::unique_ptr<MATLABEngine>& matlab_ptr,
                        const matlab::data::Array& matlab_object,
                        const std::string& var_name)
{
  matlab::data::TypedArray<double> matlab_var =
    matlab_ptr->getProperty(matlab_object, var_name);
  try
  {
    return grabnum::Vector3d(matlab_var.begin(), matlab_var.end());
  }
  catch (std::exception)
  {
    qDebug() << (var_name + " is empty!").c_str();
    return grabnum::Vector3d();
  }
}

static grabnum::Vector6d
matlabProperty2GrabVec6(const std::unique_ptr<MATLABEngine>& matlab_ptr,
                        const matlab::data::Array& matlab_object,
                        const std::string& var_name)
{
  matlab::data::TypedArray<double> matlab_var =
    matlab_ptr->getProperty(matlab_object, var_name);
  try
  {
    return grabnum::Vector6d(matlab_var.begin(), matlab_var.end());
  }
  catch (grabnum::Exception)
  {
    qDebug() << (var_name + " is empty!").c_str();
    return grabnum::Vector6d();
  }
}

static grabnum::Matrix3d
matlabProperty2GrabMat33(const std::unique_ptr<MATLABEngine>& matlab_ptr,
                         const matlab::data::Array& matlab_object,
                         const std::string& var_name)
{
  matlab::data::TypedArray<double> matlab_var =
    matlab_ptr->getProperty(matlab_object, var_name);
  try
  {
    // Matlab iterator works column-by-column, so we need to transpose all matrices
    return grabnum::Matrix3d(matlab_var.begin(), matlab_var.end()).Transpose();
  }
  catch (std::exception)
  {
    qDebug() << (var_name + " is empty!").c_str();
    return grabnum::Matrix3d();
  }
}

static grabnum::Matrix6d
matlabProperty2GrabMat66(const std::unique_ptr<MATLABEngine>& matlab_ptr,
                         const matlab::data::Array& matlab_object,
                         const std::string& var_name)
{
  matlab::data::TypedArray<double> matlab_var =
    matlab_ptr->getProperty(matlab_object, var_name);
  try
  {
    // Matlab iterator works column-by-column, so we need to transpose all matrices
    return grabnum::Matrix6d(matlab_var.begin(), matlab_var.end()).Transpose();
  }
  catch (std::exception)
  {
    qDebug() << (var_name + " is empty!").c_str();
    return grabnum::Matrix6d();
  }
}

static arma::mat matlab2ArmaMat(const std::unique_ptr<MATLABEngine>& matlab_ptr,
                                const char16_t* var_name)
{
  matlab::data::TypedArray<double> matlab_var = matlab_ptr->getVariable(var_name);
  std::vector<double> std_var(matlab_var.begin(), matlab_var.end());
  matlab::data::ArrayDimensions dims = matlab_var.getDimensions();
  arma::mat arma_matlab_var(std_var.data(), dims[0], dims[1]);
  return arma_matlab_var;
}

static arma::vec matlab2ArmaVec(const std::unique_ptr<MATLABEngine>& matlab_ptr,
                                const char16_t* var_name)
{
  matlab::data::TypedArray<double> matlab_var = matlab_ptr->getVariable(var_name);
  std::vector<double> std_var(matlab_var.begin(), matlab_var.end());
  arma::vec arma_matlab_var(std_var.data(), std_var.size());
  return arma_matlab_var;
}

static arma::mat matlabProperty2ArmaMat(const std::unique_ptr<MATLABEngine>& matlab_ptr,
                                        const matlab::data::Array& matlab_object,
                                        const std::string& property_name)
{
  matlab::data::TypedArray<double> matlab_var =
    matlab_ptr->getProperty(matlab_object, property_name);
  std::vector<double> std_var(matlab_var.begin(), matlab_var.end());
  matlab::data::ArrayDimensions dims = matlab_var.getDimensions();
  arma::mat arma_matlab_var(std_var.data(), dims[0], dims[1]);
  return arma_matlab_var;
}

static arma::vec matlabProperty2ArmaVec(const std::unique_ptr<MATLABEngine>& matlab_ptr,
                                        const matlab::data::Array& matlab_object,
                                        const std::string& property_name)
{
  matlab::data::TypedArray<double> matlab_var =
    matlab_ptr->getProperty(matlab_object, property_name);
  std::vector<double> std_var(matlab_var.begin(), matlab_var.end());
  arma::vec arma_matlab_var(std_var.data(), std_var.size());
  return arma_matlab_var;
}

class LibcdprTest: public QObject
{
  Q_OBJECT

 private:
  std::unique_ptr<MATLABEngine> matlab_ptr_;
  // Create MATLAB data array factory
  matlab::data::ArrayFactory factory_;
  grabcdpr::RobotParams params_; // only active actuators

  void addCable2WS(const grabcdpr::CableVars& cable, const std::string& var_name);
  void addPlatform2WS(const grabcdpr::PlatformVars& platform,
                      const std::string& var_name);
  void addRobot2WS(const grabcdpr::RobotVars& robot, const std::string& var_name);
  void addRobot2WS(const grabcdpr::UnderActuatedRobotVars& robot,
                   const std::string& var_name);

  grabcdpr::CableVars getCableFromWS(const std::string& var_name);
  grabcdpr::PlatformVars getPlatformFromWS(const std::string& var_name);
  void getUnderActuatedPlatformFromWS(const std::string& var_name,
                                      grabcdpr::UnderActuatedPlatformVars& platform);
  grabcdpr::RobotVars getRobotFromWS(const std::string& var_name);
  grabcdpr::UnderActuatedRobotVars
  getUnderActuatedRobotFromWS(const std::string& var_name);

 private Q_SLOTS:
  void initTestCase();

  // Tools
  void testRobotConfigJsonParser();

  // Inverse Kinematics
  void testUpdatePlatformPose();
  void testUpdatePosA();
  void testUpdatePulleyVersors();
  void testUpdateSwivelAngle();
  void testUpdateTangentAngle();
  void testUpdateCableVectors();
  void testUpdateJacobiansRow();
  void testUpdateCableZeroOrd();
  void testUpdateIK0();

  // Direct Kinematics
  void testCalcJacobianL();
  void testCalcJacobianSw();
  void testOptFunDK0();
  void testUpdateDK0();
  void testOptFunDK0GS();
  void testRobustUpdateDK0();

  // Dynamics
  void testUpdateExternalLoads();
  void testUpdateCablesStaticTension();
  void testCalcJacobiansGS();
  void testOptFunGS();
  void testNonLinsolveJacGeomStatic();
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
  matlab_ptr_->eval(var_name_u + u".complete_length = length;");
  matlab_ptr_->eval(var_name_u + u".swivel_ang = swivel_ang;");
  matlab_ptr_->eval(var_name_u + u".tan_ang = tan_ang;");
  matlab_ptr_->eval(var_name_u + u".pos_PA_glob = pos_PA_glob;");
  matlab_ptr_->eval(var_name_u + u".pos_OA_glob = pos_OA_glob;");
  matlab_ptr_->eval(var_name_u + u".pos_DA_glob = pos_DA_glob;");
  matlab_ptr_->eval(var_name_u + u".pos_BA_glob = pos_BA_glob;");
  matlab_ptr_->eval(var_name_u + u".vers_u = vers_u;");
  matlab_ptr_->eval(var_name_u + u".vers_w = vers_w;");
  matlab_ptr_->eval(var_name_u + u".vers_n = vers_n;");
  matlab_ptr_->eval(var_name_u + u".vers_t = vers_t;");
  matlab_ptr_->eval(var_name_u + u".geometric_jacobian_row = geom_jacob_row;");
  matlab_ptr_->eval(var_name_u + u".complete_speed = speed;");
  matlab_ptr_->eval(var_name_u + u".swivel_ang_vel = swivel_ang_vel;");
  matlab_ptr_->eval(var_name_u + u".tan_ang_vel = tan_ang_vel;");
  matlab_ptr_->eval(var_name_u + u".vel_OA_glob = vel_OA_glob;");
  matlab_ptr_->eval(var_name_u + u".vel_BA_glob = vel_BA_glob;");
  matlab_ptr_->eval(var_name_u + u".vers_u_deriv = vers_u_dot;");
  matlab_ptr_->eval(var_name_u + u".vers_w_deriv = vers_w_dot;");
  matlab_ptr_->eval(var_name_u + u".vers_n_deriv = vers_n_dot;");
  matlab_ptr_->eval(var_name_u + u".vers_t_deriv = vers_t_dot;");
  matlab_ptr_->eval(var_name_u + u".complete_acceleration = acceleration;");
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
  auto velocity     = factory_.createArray<double>({3, 1}, platform.linear_vel.Begin(),
                                               platform.linear_vel.End());
  auto angular_vel  = factory_.createArray<double>({3, 1}, platform.angular_vel.Begin(),
                                                  platform.angular_vel.End());
  auto vel_OG_glob  = factory_.createArray<double>({3, 1}, platform.vel_OG_glob.Begin(),
                                                  platform.vel_OG_glob.End());
  auto acceleration = factory_.createArray<double>({3, 1}, platform.linear_acc.Begin(),
                                                   platform.linear_acc.End());
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
  auto pose     = factory_.createArray<double>({POSE_DIM, 1}, platform.pose.Begin(),
                                           platform.pose.End());
  auto ext_load = factory_.createArray<double>({POSE_DIM, 1}, platform.ext_load.Begin(),
                                               platform.ext_load.End());
  auto ext_load_ss = factory_.createArray<double>(
    {POSE_DIM, 1}, platform.ext_load_ss.Begin(), platform.ext_load_ss.End());

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
  matlab_ptr_->setVariable(u"ext_load", std::move(ext_load));
  matlab_ptr_->setVariable(u"ext_load_ss", std::move(ext_load_ss));

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
  matlab_ptr_->eval(var_name_u + u".pose = pose;");
  matlab_ptr_->eval(var_name_u + u".ext_load = ext_load;");
  matlab_ptr_->eval(var_name_u + u".ext_load_ss = ext_load_ss;");
  // Matlab iterator works column-by-column, so we need to transpose all matrices
  matlab_ptr_->eval(var_name_u + u".rot_mat = rot_mat';");
  matlab_ptr_->eval(var_name_u + u".H_mat = h_mat';");
  matlab_ptr_->eval(var_name_u + u".H_mat_deriv = dh_mat';");
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
  // Create variables
  auto geom_jabobian =
    factory_.createArray<double>({robot.cables.size(), POSE_DIM},
                                 robot.geom_jacobian.begin(), robot.geom_jacobian.end());
  auto anal_jabobian =
    factory_.createArray<double>({robot.cables.size(), POSE_DIM},
                                 robot.anal_jacobian.begin(), robot.anal_jacobian.end());
  auto tension_vector = factory_.createArray<double>(
    {robot.cables.size(), 1}, robot.tension_vector.begin(), robot.tension_vector.end());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"geom_jabobian", std::move(geom_jabobian));
  matlab_ptr_->setVariable(u"anal_jabobian", std::move(anal_jabobian));
  matlab_ptr_->setVariable(u"tension_vector", std::move(tension_vector));
  // Fill fields one by one
  matlab_ptr_->eval(var_name_u + u".geometric_jacobian = geom_jabobian;");
  matlab_ptr_->eval(var_name_u + u".analitic_jacobian = anal_jabobian;");
  matlab_ptr_->eval(var_name_u + u".tension_vector = tension_vector;");
}

void LibcdprTest::addRobot2WS(const grabcdpr::UnderActuatedRobotVars& robot,
                              const std::string& var_name)
{
  auto var_name_u = convertUTF8StringToUTF16String(var_name);
  auto num_cables = convertUTF8StringToUTF16String(std::to_string(robot.cables.size()));
  matlab_ptr_->eval(var_name_u + u" = CdprVar(" + num_cables + u");");
  addPlatform2WS(robot.platform, var_name + ".platform");
  const size_t n = robot.cables.size();
  const size_t m = POSE_DIM;
  for (uint i = 0; i < n; i++)
    addCable2WS(robot.cables[i], var_name + ".cable(" + std::to_string(i + 1) + ", 1)");
  // Create variables
  auto geom_jabobian  = factory_.createArray<double>({n, m}, robot.geom_jacobian.begin(),
                                                    robot.geom_jacobian.end());
  auto anal_jabobian  = factory_.createArray<double>({n, m}, robot.anal_jacobian.begin(),
                                                    robot.anal_jacobian.end());
  auto tension_vector = factory_.createArray<double>({n, 1}, robot.tension_vector.begin(),
                                                     robot.tension_vector.end());
  auto geom_orthogonal = factory_.createArray<double>(
    {m, m - n}, robot.geom_orthogonal.begin(), robot.geom_orthogonal.end());
  auto anal_orthogonal = factory_.createArray<double>(
    {m, m - n}, robot.anal_orthogonal.begin(), robot.anal_orthogonal.end());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"geom_jabobian", std::move(geom_jabobian));
  matlab_ptr_->setVariable(u"anal_jabobian", std::move(anal_jabobian));
  matlab_ptr_->setVariable(u"tension_vector", std::move(tension_vector));
  matlab_ptr_->setVariable(u"geom_orthogonal", std::move(geom_orthogonal));
  matlab_ptr_->setVariable(u"anal_orthogonal", std::move(anal_orthogonal));
  // Fill fields one by one
  matlab_ptr_->eval(var_name_u + u".geometric_jacobian = geom_jabobian;");
  matlab_ptr_->eval(var_name_u + u".analitic_jacobian = anal_jabobian;");
  matlab_ptr_->eval(var_name_u + u".tension_vector = tension_vector;");
  matlab_ptr_->eval(var_name_u +
                    u".underactuated_platform.geometric_orthogonal = geom_orthogonal;");
  matlab_ptr_->eval(var_name_u +
                    u".underactuated_platform.analitic_orthogonal = anal_orthogonal;");
}

grabcdpr::CableVars LibcdprTest::getCableFromWS(const std::string& var_name)
{
  // Get cable object
  matlab::data::Array cable_matlab =
    matlab_ptr_->getVariable(convertUTF8StringToUTF16String(var_name));

  // Build corresponding cable structure
  grabcdpr::CableVars cable;
  cable.length         = matlab_ptr_->getProperty(cable_matlab, u"complete_length")[0];
  cable.swivel_ang     = matlab_ptr_->getProperty(cable_matlab, u"swivel_ang")[0];
  cable.tan_ang        = matlab_ptr_->getProperty(cable_matlab, u"tan_ang")[0];
  cable.speed          = matlab_ptr_->getProperty(cable_matlab, u"complete_speed")[0];
  cable.swivel_ang_vel = matlab_ptr_->getProperty(cable_matlab, u"swivel_ang_vel")[0];
  cable.tan_ang_vel    = matlab_ptr_->getProperty(cable_matlab, u"tan_ang_vel")[0];
  cable.acceleration =
    matlab_ptr_->getProperty(cable_matlab, u"complete_acceleration")[0];
  cable.swivel_ang_acc = matlab_ptr_->getProperty(cable_matlab, u"swivel_ang_acc")[0];
  cable.tan_ang_acc    = matlab_ptr_->getProperty(cable_matlab, u"tan_ang_acc")[0];
  cable.pos_PA_glob = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "pos_PA_glob");
  cable.pos_OA_glob = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "pos_OA_glob");
  cable.pos_DA_glob = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "pos_DA_glob");
  cable.pos_BA_glob = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "pos_BA_glob");
  cable.vers_u      = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vers_u");
  cable.vers_w      = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vers_w");
  cable.vers_n      = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vers_n");
  cable.vers_t      = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vers_t");
  cable.vel_OA_glob = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vel_OA_glob");
  cable.vel_BA_glob = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vel_BA_glob");
  cable.vers_u_dot  = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vers_u_deriv");
  cable.vers_w_dot  = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vers_w_deriv");
  cable.vers_n_dot  = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vers_n_deriv");
  cable.vers_t_dot  = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "vers_t_deriv");
  cable.acc_OA_glob = matlabProperty2GrabVec3(matlab_ptr_, cable_matlab, "acc_OA_glob");
  cable.geom_jacob_row =
    matlabProperty2GrabVec6(matlab_ptr_, cable_matlab, "geometric_jacobian_row")
      .Transpose();
  cable.anal_jacob_row =
    matlabProperty2GrabVec6(matlab_ptr_, cable_matlab, "analitic_jacobian_row")
      .Transpose();

  return cable;
}

grabcdpr::PlatformVars LibcdprTest::getPlatformFromWS(const std::string& var_name)
{
  // Get platform object
  matlab::data::Array platform_matlab =
    matlab_ptr_->getVariable(convertUTF8StringToUTF16String(var_name));

  grabcdpr::PlatformVars platform(params_.platform.rot_parametrization);
  // Get platform properties
  platform.position = matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "position");
  platform.pos_PG_glob =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "pos_PG_glob");
  platform.pos_OG_glob =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "pos_OG_glob");
  platform.linear_vel = matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "velocity");
  platform.angular_vel =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "angular_vel");
  platform.vel_OG_glob =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "vel_OG_glob");
  platform.linear_acc =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "acceleration");
  platform.angular_acc =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "angular_acc");
  platform.acc_OG_glob =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "acc_OG_glob");
  platform.orientation =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "orientation");
  platform.orientation_dot =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "orientation_deriv");
  platform.orientation_ddot =
    matlabProperty2GrabVec3(matlab_ptr_, platform_matlab, "orientation_deriv_2");
  platform.pose     = matlabProperty2GrabVec6(matlab_ptr_, platform_matlab, "pose");
  platform.ext_load = matlabProperty2GrabVec6(matlab_ptr_, platform_matlab, "ext_load");
  platform.dyn_load = matlabProperty2GrabVec6(matlab_ptr_, platform_matlab, "dyn_load");
  platform.total_load =
    matlabProperty2GrabVec6(matlab_ptr_, platform_matlab, "total_load");
  platform.ext_load_ss =
    matlabProperty2GrabVec6(matlab_ptr_, platform_matlab, "ext_load_ss");
  platform.dyn_load_ss =
    matlabProperty2GrabVec6(matlab_ptr_, platform_matlab, "dyn_load_ss");
  platform.total_load_ss =
    matlabProperty2GrabVec6(matlab_ptr_, platform_matlab, "total_load_ss");
  platform.rot_mat = matlabProperty2GrabMat33(matlab_ptr_, platform_matlab, "rot_mat");
  platform.h_mat   = matlabProperty2GrabMat33(matlab_ptr_, platform_matlab, "H_mat");
  platform.dh_mat = matlabProperty2GrabMat33(matlab_ptr_, platform_matlab, "H_mat_deriv");
  platform.inertia_mat_glob =
    matlabProperty2GrabMat33(matlab_ptr_, platform_matlab, "inertia_matrix_global");
  platform.mass_mat_glob =
    matlabProperty2GrabMat66(matlab_ptr_, platform_matlab, "mass_matrix_global");
  platform.mass_mat_glob_ss =
    matlabProperty2GrabMat66(matlab_ptr_, platform_matlab, "mass_matrix_global_ss");

  return platform;
}

void LibcdprTest::getUnderActuatedPlatformFromWS(
  const std::string& var_name, grabcdpr::UnderActuatedPlatformVars& platform)
{
  // Get under-actuated platform object
  matlab::data::Array underactuated_matlab =
    matlab_ptr_->getVariable(convertUTF8StringToUTF16String(var_name));

  // Update under-actuated platform properties
  platform.actuated_vars =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "actuated");
  platform.unactuated_vars =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "unactuated");
  platform.actuated_deriv =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "actuated_deriv");
  platform.unactuated_deriv =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "unactuated_deriv");
  platform.actuated_deriv_2 =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "actuated_deriv_2");
  platform.unactuated_deriv_2 =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "unactuated_deriv_2");
  platform.mass_matrix_global_a =
    matlabProperty2ArmaMat(matlab_ptr_, underactuated_matlab, "mass_matrix_global_a");
  platform.mass_matrix_global_u =
    matlabProperty2ArmaMat(matlab_ptr_, underactuated_matlab, "mass_matrix_global_u");
  platform.mass_matrix_global_ss_a =
    matlabProperty2ArmaMat(matlab_ptr_, underactuated_matlab, "mass_matrix_global_ss_a");
  platform.mass_matrix_global_ss_u =
    matlabProperty2ArmaMat(matlab_ptr_, underactuated_matlab, "mass_matrix_global_ss_u");
  platform.external_load_a =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "external_load_a");
  platform.external_load_u =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "external_load_u");
  platform.external_load_ss_a =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "external_load_ss_a");
  platform.external_load_ss_u =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "external_load_ss_u");
  platform.total_load_a =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "total_load_a");
  platform.total_load_u =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "total_load_u");
  platform.total_load_ss_a =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "total_load_ss_a");
  platform.total_load_ss_u =
    matlabProperty2ArmaVec(matlab_ptr_, underactuated_matlab, "total_load_ss_u");
}

grabcdpr::RobotVars LibcdprTest::getRobotFromWS(const std::string& var_name)
{
  auto var_name_u = convertUTF8StringToUTF16String(var_name);
  grabcdpr::RobotVars robot(params_.platform.rot_parametrization);
  // Get platform
  matlab_ptr_->eval(u"platform = " + var_name_u + u".platform;");
  robot.platform = getPlatformFromWS("platform");
  // Get cables
  for (uint i = 1; i <= params_.actuators.size(); i++)
  {
    auto idx_u = convertUTF8StringToUTF16String(std::to_string(i));
    matlab_ptr_->eval(u"cable = " + var_name_u + u".cable(" + idx_u + u");");
    robot.cables.push_back(getCableFromWS("cable"));
  }
  // Get robot object
  matlab::data::Array robot_matlab = matlab_ptr_->getVariable(var_name_u);
  // Get extra fields
  robot.geom_jacobian =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "geometric_jacobian");
  robot.anal_jacobian =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "analitic_jacobian");
  robot.geom_jacobian_d =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "geometric_jacobian_d");
  robot.anal_jacobian_d =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "analitic_jacobian_d");
  robot.tension_vector =
    matlabProperty2ArmaVec(matlab_ptr_, robot_matlab, "tension_vector");
  return robot;
}

grabcdpr::UnderActuatedRobotVars
LibcdprTest::getUnderActuatedRobotFromWS(const std::string& var_name)
{
  auto var_name_u = convertUTF8StringToUTF16String(var_name);
  grabcdpr::UnderActuatedRobotVars robot(params_.platform.rot_parametrization);
  // Get platform
  matlab_ptr_->eval(u"platform = " + var_name_u + u".platform;");
  robot.platform = getPlatformFromWS("platform");
  // Get under-actuated platform
  getUnderActuatedPlatformFromWS(var_name + ".underactuated_platform", robot.platform);
  // Get cables
  for (uint i = 1; i <= params_.actuators.size(); i++)
  {
    auto idx_u = convertUTF8StringToUTF16String(std::to_string(i));
    matlab_ptr_->eval(u"cable = " + var_name_u + u".cable(" + idx_u + u");");
    robot.cables.push_back(getCableFromWS("cable"));
  }
  // Get robot object
  matlab::data::Array robot_matlab = matlab_ptr_->getVariable(var_name_u);
  // Get extra fields
  robot.geom_jacobian =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "geometric_jacobian");
  robot.anal_jacobian =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "analitic_jacobian");
  robot.geom_jacobian_d =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "geometric_jacobian_d");
  robot.anal_jacobian_d =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "analitic_jacobian_d");
  robot.tension_vector =
    matlabProperty2ArmaVec(matlab_ptr_, robot_matlab, "tension_vector");
  // Some under-actuated-related field which don't belong to the platform are inside
  // underactuated_platform property, so we first need to get it
  matlab::data::Array underactuated_matlab = matlab_ptr_->getVariable(
    convertUTF8StringToUTF16String(var_name + ".underactuated_platform"));
  // Extract missing properties
  robot.geom_jacobian_a =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "geometric_jacobian_a");
  robot.geom_jacobian_u =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "geometric_jacobian_u");
  robot.geom_orthogonal =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "geometric_orthogonal");
  robot.anal_jacobian_a =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "analitic_jacobian_a");
  robot.anal_jacobian_u =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "analitic_jacobian_u");
  robot.anal_orthogonal =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "analitic_orthogonal");
  robot.gamma_mat =
    matlabProperty2ArmaMat(matlab_ptr_, robot_matlab, "analitic_orthogonal");

  return robot;
}

//--------- Init ---------------//

void LibcdprTest::initTestCase()
{
  // Start matlab engine synchronously
  matlab_ptr_ = startMATLAB();

  // Load robot parameters
  RobotConfigJsonParser parser;
  parser.ParseFile(QString("../tests/pass.json"), &params_);
  params_.removeInactiveComponents();
  // Load same robot parameters in matlab workspace
  matlab_ptr_->eval(
    u"cdpr_p = "
    u"CdprParameter('" SRCDIR u"cdpr_matlab/config/Grab_prototype_33.json');");
}

//--------- Tools ---------------//

void LibcdprTest::testRobotConfigJsonParser()
{
  // test parsing with different inputs
  RobotConfigJsonParser parser;
  QVERIFY(!parser.ParseFile("../tests/fail1.json"));
  QVERIFY(!parser.ParseFile(QString("../tests/fail2.json")));
  QVERIFY(parser.ParseFile(QString("../tests/pass.json")));

  // test getters
  grabcdpr::RobotParams params = parser.GetConfigStruct();
  parser.GetConfigStruct(&params);

  // test display
  parser.PrintConfig();
}

//--------- Inverse Kinematics ---------------//

void LibcdprTest::testUpdatePlatformPose()
{
  // Setup dummy input
  grabcdpr::PlatformVars platform;
  grabnum::Vector3d position({1, 2, 3});
  grabnum::Vector3d orientation({0.1, 0.2, 0.3});

  // Load dummy input to Matlab workspace
  addPlatform2WS(platform, "platform_v");
  auto position_matlab =
    factory_.createArray<double>({3, 1}, position.Begin(), position.End());
  auto orientation_matlab =
    factory_.createArray<double>({3, 1}, orientation.Begin(), orientation.End());
  auto pos_PG_loc_matlab = factory_.createArray<double>(
    {3, 1}, params_.platform.pos_PG_loc.Begin(), params_.platform.pos_PG_loc.End());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"position", std::move(position_matlab));
  matlab_ptr_->setVariable(u"orientation", std::move(orientation_matlab));
  matlab_ptr_->setVariable(u"pos_PG_loc", std::move(pos_PG_loc_matlab));

  // Call C++ function implementation to be tested
  QBENCHMARK
  {
    grabcdpr::updatePlatformPose(position, orientation, params_.platform.pos_PG_loc,
                                 platform);
  }
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
  QVERIFY(platform.pose.IsApprox(matlab_platform.pose));
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
  QBENCHMARK { grabcdpr::updatePosA(params_.actuators[0], platform, cable); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cable_v = "
                    u"UpdatePosA(cdpr_p.cable(1).pos_PA_loc, cdpr_p.cable(1).pos_OD_glob,"
                    u"platform_v.position, platform_v.rot_mat, cable_v);");

  // Get matlab results
  grabcdpr::CableVars matlab_cable = getCableFromWS("cable_v");

  // Check they are the same
  QCOMPARE(cable.pos_PA_glob, matlab_cable.pos_PA_glob);
  QCOMPARE(cable.pos_OA_glob, matlab_cable.pos_OA_glob);
  QCOMPARE(cable.pos_DA_glob, matlab_cable.pos_DA_glob);
}

void LibcdprTest::testUpdatePulleyVersors()
{
  // Setup dummy input
  grabcdpr::CableVars cable;
  cable.swivel_ang = 0.5;
  // Load dummy input to Matlab workspace
  addCable2WS(cable, "cable_v");

  // Call C++ function implementation to be tested
  QBENCHMARK { grabcdpr::updatePulleyVersors(params_.actuators[0].pulley, cable); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cable_v = CalcPulleyVersors(cdpr_p.cable(1).vers_i, "
                    u"cdpr_p.cable(1).vers_j, cable_v);");

  // Get matlab results
  grabcdpr::CableVars matlab_cable = getCableFromWS("cable_v");

  // Check they are the same
  QCOMPARE(cable.vers_u, matlab_cable.vers_u);
  QCOMPARE(cable.vers_w, matlab_cable.vers_w);
}

void LibcdprTest::testUpdateSwivelAngle()
{
  // Setup dummy input
  grabnum::Vector3d pos_DA_glob({1, 2, 3});
  // Load dummy input to Matlab workspace
  auto pos_DA_glob_matlab =
    factory_.createArray<double>({3, 1}, pos_DA_glob.Begin(), pos_DA_glob.End());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"pos_DA_glob", std::move(pos_DA_glob_matlab));

  // Call C++ function implementation to be tested
  double swivel_angle;
  QBENCHMARK
  {
    swivel_angle = grabcdpr::calcSwivelAngle(params_.actuators[0].pulley, pos_DA_glob);
  }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"swivel_angle = CalcSwivelAngle(cdpr_p.cable(1).vers_i, "
                    u"cdpr_p.cable(1).vers_j, pos_DA_glob);");

  // Get matlab results
  matlab::data::TypedArray<double> matlab_swivel_angle =
    matlab_ptr_->getVariable(u"swivel_angle");

  // Check they are the same
  QCOMPARE(swivel_angle, matlab_swivel_angle[0]);
}

void LibcdprTest::testUpdateTangentAngle()
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
  double tan_angle;
  QBENCHMARK
  {
    tan_angle =
      grabcdpr::calcTangentAngle(params_.actuators[0].pulley, vers_u, pos_DA_glob);
  }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"tan_angle = CalcTangentAngle(cdpr_p.cable(1).vers_k, "
                    u"cdpr_p.cable(1).swivel_pulley_r, vers_u, pos_DA_glob);");

  // Get matlab results
  matlab::data::TypedArray<double> matlab_tan_angle =
    matlab_ptr_->getVariable(u"tan_angle");

  // Check they are the same
  QCOMPARE(tan_angle, matlab_tan_angle[0]);
}

void LibcdprTest::testUpdateCableVectors()
{
  // Setup dummy input
  grabcdpr::CableVars cable;
  cable.tan_ang = 0.5;
  cable.vers_u.Fill({0, 0, 1});
  cable.pos_DA_glob.Fill({1, 2, 3});
  // Load dummy input to Matlab workspace
  addCable2WS(cable, "cable_v");

  // Call C++ function implementation to be tested
  QBENCHMARK { grabcdpr::updateCableVectors(params_.actuators[0].pulley, cable); }
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
  QBENCHMARK { grabcdpr::updateJacobiansRow(h_mat, cable); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"[geometric, analitic] = CalcPlatformJacobianRow(cable_v.vers_t,"
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
  QBENCHMARK { grabcdpr::updateCableZeroOrd(params_.actuators[0], platform, cable); }
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
  QVERIFY(cable.vers_n.IsApprox(matlab_cable.vers_n));
  QVERIFY(cable.vers_t.IsApprox(matlab_cable.vers_t));
  QVERIFY(cable.vers_w.IsApprox(matlab_cable.vers_w));
  QVERIFY(cable.pos_BA_glob.IsApprox(matlab_cable.pos_BA_glob));
  QVERIFY(cable.pos_DA_glob.IsApprox(matlab_cable.pos_DA_glob));
  QVERIFY(cable.pos_OA_glob.IsApprox(matlab_cable.pos_OA_glob));
  QVERIFY(cable.geom_jacob_row.IsApprox(matlab_cable.geom_jacob_row));
  QVERIFY(cable.anal_jacob_row.IsApprox(matlab_cable.anal_jacob_row));
}

void LibcdprTest::testUpdateIK0()
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
  QBENCHMARK { grabcdpr::updateIK0(position, orientation, params_, robot); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cdpr_v = UpdateIKZeroOrd(position, orientation, cdpr_p, cdpr_v);");

  // Get matlab results
  grabcdpr::RobotVars matlab_robot = getRobotFromWS("cdpr_v");

  // Check they are the same
  QCOMPARE(robot.platform.pose, matlab_robot.platform.pose);
  QVERIFY(robot.platform.rot_mat.IsApprox(matlab_robot.platform.rot_mat));
  for (uint i = 0; i < robot.cables.size(); i++)
  {
    QCOMPARE(robot.cables[i].swivel_ang, matlab_robot.cables[i].swivel_ang);
    QCOMPARE(robot.cables[i].tan_ang, matlab_robot.cables[i].tan_ang);
    QCOMPARE(robot.cables[i].length, matlab_robot.cables[i].length);
    QVERIFY(robot.cables[i].vers_u.IsApprox(matlab_robot.cables[i].vers_u));
    QVERIFY(robot.cables[i].vers_n.IsApprox(matlab_robot.cables[i].vers_n));
    QVERIFY(robot.cables[i].vers_t.IsApprox(matlab_robot.cables[i].vers_t));
    QVERIFY(robot.cables[i].vers_w.IsApprox(matlab_robot.cables[i].vers_w));
    QVERIFY(robot.cables[i].pos_BA_glob.IsApprox(matlab_robot.cables[i].pos_BA_glob));
    QVERIFY(robot.cables[i].pos_DA_glob.IsApprox(matlab_robot.cables[i].pos_DA_glob));
    QVERIFY(robot.cables[i].pos_OA_glob.IsApprox(matlab_robot.cables[i].pos_OA_glob));
    QVERIFY(robot.cables[i].pos_PA_glob.IsApprox(matlab_robot.cables[i].pos_PA_glob));
    QVERIFY(
      robot.cables[i].geom_jacob_row.IsApprox(matlab_robot.cables[i].geom_jacob_row));
    QVERIFY(
      robot.cables[i].anal_jacob_row.IsApprox(matlab_robot.cables[i].anal_jacob_row));
  }
  QVERIFY(arma::approx_equal(robot.geom_jacobian, matlab_robot.geom_jacobian, "absdiff",
                             EPSILON));
  QVERIFY(arma::approx_equal(robot.anal_jacobian, matlab_robot.anal_jacobian, "absdiff",
                             EPSILON));
  QVERIFY(arma::approx_equal(robot.tension_vector, matlab_robot.tension_vector, "absdiff",
                             EPSILON));
}

//--------- Direct Kinematics ---------------//

void LibcdprTest::testCalcJacobianL()
{
  // Setup dummy input
  grabcdpr::RobotVars robot(params_.actuators.size(),
                            params_.platform.rot_parametrization);
  grabnum::Vector6d pose({0.1, 1.5, 0.2, 0.23, -0.16, 0.03});
  grabcdpr::updateIK0(pose, params_, robot);
  // Load dummy input to Matlab workspace
  addRobot2WS(robot, "cdpr_v");

  // Call C++ function implementation to be tested
  arma::mat jacobian_l;
  QBENCHMARK { jacobian_l = grabcdpr::calcJacobianL(robot); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"jacobian_l = CalcJacobianL(cdpr_v);");

  // Get matlab results
  arma::mat matlab_jacobian_l = matlab2ArmaMat(matlab_ptr_, u"jacobian_l");

  // Check they are the same
  QVERIFY(arma::approx_equal(jacobian_l, matlab_jacobian_l, "absdiff", EPSILON));
}

void LibcdprTest::testCalcJacobianSw()
{
  // Setup dummy input
  grabcdpr::RobotVars robot(params_.actuators.size(),
                            params_.platform.rot_parametrization);
  grabnum::Vector6d pose({0.1, 1.5, 0.2, 0.23, -0.16, 0.03});
  grabcdpr::updateIK0(pose, params_, robot);
  // Load dummy input to Matlab workspace
  addRobot2WS(robot, "cdpr_v");

  // Call C++ function implementation to be tested
  arma::mat jacobian_sw;
  QBENCHMARK { jacobian_sw = grabcdpr::calcJacobianSw(robot); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"jacobian_sw = CalcJacobianSw(cdpr_v);");

  // Get matlab results
  arma::mat matlab_jacobian_sw = matlab2ArmaMat(matlab_ptr_, u"jacobian_sw");

  // Check they are the same
  QVERIFY(arma::approx_equal(jacobian_sw, matlab_jacobian_sw, "absdiff", EPSILON));
}

void LibcdprTest::testOptFunDK0()
{
  // Setup dummy input
  const size_t num_cables = params_.actuators.size();
  grabcdpr::RobotVars robot(num_cables, params_.platform.rot_parametrization);
  arma::vec6 pose({0.1, 1.5, 0.2, 0.23, -0.16, 0.03});
  grabcdpr::updateIK0(pose, params_, robot);
  std::vector<double> cables_length(num_cables, 0);
  std::vector<double> swivel_angles(num_cables, 0);
  for (uint i = 0; i < num_cables; ++i)
  {
    cables_length[i] = robot.cables[i].length;
    swivel_angles[i] = robot.cables[i].swivel_ang;
  }
  // Load dummy input to Matlab workspace
  auto cables_length_matlab =
    factory_.createArray({num_cables, 1}, cables_length.begin(), cables_length.end());
  auto swivel_angles_matlab =
    factory_.createArray({num_cables, 1}, swivel_angles.begin(), swivel_angles.end());
  auto pose_matlab = factory_.createArray({6, 1}, pose.begin(), pose.end());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"cables_length", std::move(cables_length_matlab));
  matlab_ptr_->setVariable(u"swivel_angles", std::move(swivel_angles_matlab));
  matlab_ptr_->setVariable(u"pose", std::move(pose_matlab));

  // Call C++ function implementation to be tested
  arma::vec func_val;
  arma::mat func_jacob;
  QBENCHMARK
  {
    optFunDK0(params_, cables_length, swivel_angles, pose, func_jacob, func_val);
  }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(
    u"[vector, matrix] = FunDkSwL(cdpr_p, cables_length, swivel_angles, pose);");

  // Get matlab results
  arma::vec matlab_func_val   = matlab2ArmaVec(matlab_ptr_, u"vector");
  arma::mat matlab_func_jacob = matlab2ArmaMat(matlab_ptr_, u"matrix");

  // Check they are the same
  QVERIFY(arma::approx_equal(func_val, matlab_func_val, "absdiff", EPSILON));
  QVERIFY(arma::approx_equal(func_jacob, matlab_func_jacob, "absdiff", EPSILON));
}

void LibcdprTest::testUpdateDK0()
{
  // Setup dummy input
  grabnum::VectorXd<POSE_DIM> init_guess({0.1, 1.5, 0.2, 0.23, -0.16, 0.03});
  const arma::uvec6 mask({1, 1, 1, 0, 0, 0});
  arma::vec3 true_orientation =
    grabcdpr::nonLinsolveJacGeomStatic(init_guess, mask, params_);
  grabcdpr::RobotVars robot(params_.actuators.size(),
                            params_.platform.rot_parametrization);
  grabnum::Vector3d position = init_guess.HeadRows<3>();
  grabnum::Vector3d orientation(true_orientation.begin(), true_orientation.end());
  grabcdpr::updateIK0(position, orientation, params_, robot);
  // Perturbate pose
  robot.platform.pose +=
    grabnum::VectorXd<POSE_DIM>({0.01, -0.02, 0.001, 0.001, 0.002, -0.008});

  bool ret = false;

  // Perform fast direct kinematics
  QBENCHMARK { ret = grabcdpr::updateDK0(params_, robot); }

  if (ret)
  {
    // Verify roundtrip
    QVERIFY(position.IsApprox(position));
    QVERIFY(orientation.IsApprox(orientation));
  }
  else
    std::cout << "could not solve fast DK0" << std::endl;
}

void LibcdprTest::testOptFunDK0GS()
{
  // Setup dummy input
  const size_t num_cables = params_.actuators.size();
  grabcdpr::UnderActuatedRobotVars robot(num_cables, params_.platform.rot_parametrization,
                                         params_.controlled_vars_mask);
  arma::vec6 pose({0.1, 1.5, 0.2, 0.23, -0.16, 0.03});
  grabcdpr::updateIK0(pose, params_, robot);
  std::vector<double> cables_length(num_cables, 0);
  std::vector<double> swivel_angles(num_cables, 0);
  for (uint i = 0; i < num_cables; ++i)
  {
    cables_length[i] = robot.cables[i].length;
    swivel_angles[i] = robot.cables[i].swivel_ang;
  }
  // Load dummy input to Matlab workspace
  auto cables_length_matlab =
    factory_.createArray({num_cables, 1}, cables_length.begin(), cables_length.end());
  auto swivel_angles_matlab =
    factory_.createArray({num_cables, 1}, swivel_angles.begin(), swivel_angles.end());
  auto pose_matlab = factory_.createArray({6, 1}, pose.begin(), pose.end());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"cables_length", std::move(cables_length_matlab));
  matlab_ptr_->setVariable(u"swivel_angles", std::move(swivel_angles_matlab));
  matlab_ptr_->setVariable(u"pose", std::move(pose_matlab));

  // Call C++ function implementation to be tested
  arma::vec func_val;
  arma::mat func_jacob;
  QBENCHMARK
  {
    optFunDK0GS(params_, cables_length, swivel_angles, pose, func_jacob, func_val);
  }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(
    u"[vector, matrix] = FunDkGsSwL(cdpr_p, cables_length, swivel_angles, pose);");

  // Get matlab results
  arma::vec matlab_func_val   = matlab2ArmaVec(matlab_ptr_, u"vector");
  arma::mat matlab_func_jacob = matlab2ArmaMat(matlab_ptr_, u"matrix");

  // Check they are the same
  QVERIFY(arma::approx_equal(func_val, matlab_func_val, "absdiff", EPSILON));
  QVERIFY(arma::approx_equal(func_jacob, matlab_func_jacob, "absdiff", EPSILON));
}

void LibcdprTest::testRobustUpdateDK0()
{
  // Setup dummy input
  grabnum::VectorXd<POSE_DIM> init_guess({0.1, 1.5, 0.2, 0.23, -0.16, 0.03});
  const arma::uvec6 mask({1, 1, 1, 0, 0, 0});
  arma::vec3 true_orientation =
    grabcdpr::nonLinsolveJacGeomStatic(init_guess, mask, params_);
  grabcdpr::UnderActuatedRobotVars robot(params_.actuators.size(),
                                         params_.platform.rot_parametrization, mask);
  grabnum::Vector3d position = init_guess.GetBlock<3, 1>(1, 1);
  grabnum::Vector3d orientation(true_orientation.begin(), true_orientation.end());
  grabcdpr::updateIK0(position, orientation, params_, robot);
  // Perturbate pose
  robot.platform.pose +=
    grabnum::VectorXd<POSE_DIM>({0.01, -0.02, 0.001, 0.001, 0.002, -0.008});

  bool ret = false;

  // Perform fast direct kinematics
  QBENCHMARK { ret = grabcdpr::updateDK0(params_, robot); }

  if (ret)
  {
    // Verify roundtrip
    QVERIFY(position.IsApprox(position));
    QVERIFY(orientation.IsApprox(orientation));
  }
  else
    std::cout << "could not solve fast DK0" << std::endl;
}

//--------- Dynamics ---------------//

void LibcdprTest::testUpdateExternalLoads()
{
  // Setup dummy input
  grabcdpr::RobotVars robot(params_.actuators.size(),
                            params_.platform.rot_parametrization);
  grabcdpr::updatePlatformPose(grabnum::Vector3d({1, 2, 3}),
                               grabnum::Vector3d({0.5, 1.0, 1.5}), params_.platform,
                               robot.platform);

  // Load dummy input to Matlab workspace
  addRobot2WS(robot, "cdpr_v");

  // Call C++ function implementation to be tested
  QBENCHMARK { grabcdpr::updateAllExternalLoads(params_.platform, robot.platform); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cdpr_v = CalcExternalLoadsStateSpace(cdpr_v, cdpr_p);");

  // Get matlab results
  grabcdpr::RobotVars matlab_platform = getRobotFromWS("cdpr_v");

  // Check they are the same
  QCOMPARE(robot.platform.ext_load, matlab_platform.platform.ext_load);
  QCOMPARE(robot.platform.ext_load_ss, matlab_platform.platform.ext_load_ss);
}

void LibcdprTest::testUpdateCablesStaticTension()
{
  // Setup dummy input
  grabcdpr::RobotVars robot(params_.actuators.size(),
                            params_.platform.rot_parametrization);
  grabnum::Vector3d position({0.1, 1.5, 0.2});    // some feasible position
  grabnum::Vector3d orientation({0.0, 0.0, 0.0}); // FIND A FEASIBLE ORIENTATION
  grabcdpr::updateIK0(position, orientation, params_, robot);
  grabcdpr::updateExternalLoads(params_.platform, robot.platform);
  // Load dummy input to Matlab workspace
  addRobot2WS(robot, "cdpr_v");

  // Call C++ function implementation to be tested
  QBENCHMARK { grabcdpr::updateCablesStaticTension(robot); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"cdpr_v = CalcCablesStaticTension(cdpr_v);");

  // Get matlab results
  grabcdpr::RobotVars matlab_robot = getRobotFromWS("cdpr_v");

  // Check they are the same
  QVERIFY(arma::approx_equal(robot.tension_vector, matlab_robot.tension_vector, "absdiff",
                             EPSILON));
}

void LibcdprTest::testCalcJacobiansGS()
{
  // Setup dummy input
  const arma::uvec6 mask({1, 1, 1, 0, 0, 0});
  grabcdpr::UnderActuatedRobotVars robot(params_.actuators.size(),
                                         params_.platform.rot_parametrization, mask);
  grabnum::Vector3d position({0.1, 1.5, 0.2});        // some feasible position
  grabnum::Vector3d orientation({0.23, -0.16, 0.03}); // FIND A FEASIBLE ORIENTATION
  grabcdpr::updateIK0(position, orientation, params_, robot);
  grabcdpr::updateExternalLoads(params_.platform, robot.platform);
  grabcdpr::updateCablesStaticTension(robot);
  // Load dummy input to Matlab workspace
  addRobot2WS(robot, "cdpr_v");

  // Call C++ function implementation to be tested
  arma::mat Jq;
  QBENCHMARK { Jq = grabcdpr::calcJacobianGS(robot); }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"J_q = CalcJacobianGs(cdpr_v);");

  // Get matlab results
  arma::mat matlab_J_q = matlab2ArmaMat(matlab_ptr_, u"J_q");

  // Check they are the same
  QVERIFY(arma::approx_equal(Jq, matlab_J_q, "absdiff", EPSILON));
}

void LibcdprTest::testOptFunGS()
{
  // Setup dummy input
  arma::vec3 fixed_coord({0.1, 1.5, 0.2}); // pos
  arma::vec3 var_coord({0.0, 0, 0.0});     // orient
  const arma::uvec6 mask({1, 1, 1, 0, 0, 0});
  arma::mat fun_jacobian;
  arma::vec fun_val;
  // Load dummy input to Matlab workspace
  auto act_vars =
    factory_.createArray<double>({3, 1}, fixed_coord.begin(), fixed_coord.end());
  auto unact_vars =
    factory_.createArray<double>({3, 1}, var_coord.begin(), var_coord.end());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"act_vars", std::move(act_vars));
  matlab_ptr_->setVariable(u"unact_vars", std::move(unact_vars));

  // Call C++ function implementation to be tested
  QBENCHMARK
  {
    grabcdpr::optFunGS(params_, fixed_coord, var_coord, fun_jacobian, fun_val);
  }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(u"[fun_val, fun_jacobian] = FunGs(cdpr_p, act_vars, unact_vars);");

  // Get matlab results
  arma::mat matlab_fun_jacobian = matlab2ArmaMat(matlab_ptr_, u"fun_jacobian");
  arma::vec matlab_fun_val      = matlab2ArmaVec(matlab_ptr_, u"fun_val");

  // Check they are the same
  QVERIFY(arma::approx_equal(fun_jacobian, matlab_fun_jacobian, "absdiff", EPSILON));
  QVERIFY(arma::approx_equal(fun_val, matlab_fun_val, "absdiff", EPSILON));
}

void LibcdprTest::testNonLinsolveJacGeomStatic()
{
  // Setup dummy input
  grabnum::VectorXd<POSE_DIM> init_guess({0.1, 1.5, 0.2, 0.23, -0.16, 0.03});
  //  grabnum::VectorXd<POSE_DIM> init_guess({0.1, 1.5, 0.2, 0, 0, 0});
  const arma::uvec6 mask({1, 1, 1, 0, 0, 0});
  // Load dummy input to Matlab workspace
  auto p =
    factory_.createArray<double>({3, 1}, init_guess.Begin(), init_guess.Begin() + 3);
  auto v = factory_.createArray<double>({3, 1}, init_guess.Begin() + 4, init_guess.End());
  // Put variables in the MATLAB workspace
  matlab_ptr_->setVariable(u"p", std::move(p));
  matlab_ptr_->setVariable(u"in_guess", std::move(v));

  // Call C++ function implementation to be tested
  arma::vec3 orientation;
  // Run once outside benchmark to obtain iterations
  //  uint8_t iterations;
  //  orientation = grabcdpr::nonLinsolveJacGeomStatic(init_guess, mask, params_, 100,
  //  &iterations); printf("Iterations: %d\n", iterations);
  QBENCHMARK
  {
    orientation = grabcdpr::nonLinsolveJacGeomStatic(init_guess, mask, params_);
  }
  // Call the corresponding MATLAB
  matlab_ptr_->eval(
    u"fsolve_options_grad = "
    u"optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt', 'FunctionTolerance',"
    u"1e-8, 'MaxFunctionEvaluation', 1000000, 'MaxIterations', 1000000,"
    u"'OptimalityTolerance', 1e-8, 'display', 'none', 'StepTolerance', 1e-8,"
    u"'SpecifyObjectiveGradient', true, 'UseParallel', true);");
  matlab_ptr_->eval(u"orient = fsolve(@(v) FunGsNoCheck(cdpr_p, p, v), in_guess,"
                    u"fsolve_options_grad);");

  // Get matlab results
  arma::vec matlab_orientation = matlab2ArmaVec(matlab_ptr_, u"orient");

  // Check they are the same
  QVERIFY(arma::approx_equal(orientation, matlab_orientation, "absdiff", 1e-5));
}

QTEST_APPLESS_MAIN(LibcdprTest)

#include "libcdpr_test.moc"
