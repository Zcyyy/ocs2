/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: FactoryFunctions.cpp
*
*          Created On: 2022年12月14日 星期三 16时34分59秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "ocs2_whole_body_model/FactoryFunctions.h"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>

namespace ocs2 {
namespace wholebody_model {

PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath) {
  // add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

  return getPinocchioInterfaceFromUrdfFile(urdfFilePath, jointComposite);
}

PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath, const std::vector<std::string>& jointNames) {
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFilePath);
  if (urdfTree == nullptr) {
    throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model!");
  }

  // remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for (joint_pair_t& jointPair : newModel->joints_) {
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) == jointNames.end()) {
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }

  // add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

  return getPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
}

WholeBodyModelInfo createWholeBodyModelInfo(const PinocchioInterface& interface, const WholeBodyModelType& type,
                                              const vector_t& nominalJointAngles, const std::vector<std::string>& threeDofContactNames,
                                              const std::vector<std::string>& sixDofContactNames) {
  const auto& model = interface.getModel();
  auto data = interface.getData();

  if (model.nq != nominalJointAngles.size() + 6) {
    const int expaectedNumJoints = model.nq - 6;
    throw std::runtime_error("[WholeBodyModelInfo] nominalJointAngles.size() should be " + std::to_string(expaectedNumJoints));
  }

  WholeBodyModelInfoTpl<scalar_t> info;
  info.wholebodyModelType = type;
  info.numThreeDofContacts = threeDofContactNames.size();
  info.numSixDofContacts = sixDofContactNames.size();
  info.generalizedCoordinatesNum = model.nq;
  info.actuatedDofNum = info.generalizedCoordinatesNum - 6;
  info.stateDim = info.generalizedCoordinatesNum + 6;
  info.inputDim = info.actuatedDofNum + 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  info.totalMass = pinocchio::computeTotalMass(model);

  for (const auto& name : threeDofContactNames) {
    info.endEffectorFrameIndices.push_back(model.getBodyId(name));
  }

  for (const auto& name : sixDofContactNames) {
    info.endEffectorFrameIndices.push_back(model.getBodyId(name));
  }

  // make sure the nominal base frame is aligned with the world frame
  info.qPinocchioNominal.resize(model.nq);
  info.qPinocchioNominal << vector_t::Zero(6), nominalJointAngles;
  for(int i = 0; i < model.names.size(); i++)
  {
    //std::cout << "**********" << std::endl;
    //std::cout << model.names[i] << std::endl;
    //std::cout << "**********" << std::endl;
    //std::cout << model.inertias[i].inertia().matrix().cast<scalar_t>() << std::endl;
    info.WholeBodyInertiaNominal.push_back(std::make_pair(model.names[i], model.inertias[i].inertia().matrix().cast<scalar_t>()));
  }
  info.comToBasePositionNominal.setZero();
  if (info.wholebodyModelType == WholeBodyModelType::WholeBodyDynamics) {
    const vector_t vPinocchioNominal = vector_t::Zero(info.generalizedCoordinatesNum);
    pinocchio::ccrba(model, data, info.qPinocchioNominal, vPinocchioNominal);
    //info.WholeBodyInertiaNominal = data.Ig.inertia().matrix();
    info.comToBasePositionNominal = info.qPinocchioNominal.template head<3>() - data.com[0];
  }

  return info;
}

WholeBodyModelType loadWholeBodyType(const std::string& configFilePath, const std::string& fieldName) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<WholeBodyModelType>(type);
}

vector_t loadDefaultJointState(size_t numJointState, const std::string& configFilePath, const std::string& fieldName) {
  vector_t defaultJoints(numJointState);
  ocs2::loadData::loadEigenMatrix(configFilePath, fieldName, defaultJoints);
  return defaultJoints;
}

}
}