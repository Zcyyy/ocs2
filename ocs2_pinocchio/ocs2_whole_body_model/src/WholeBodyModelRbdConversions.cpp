/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: WholeBodyModelRbdConversions.h
*
*          Created On: 2022年12月14日 星期三 21时49分26秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "ocs2_whole_body_model/WholeBodyModelRbdConversions.h"

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include "ocs2_whole_body_model/AccessHelperFunctions.h"
#include "ocs2_whole_body_model/ModelHelperFunctions.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WholeBodyModelRbdConversions::WholeBodyModelRbdConversions(PinocchioInterface pinocchioInterface, const WholeBodyModelInfo& info)
    : pinocchioInterface_(std::move(pinocchioInterface)), mapping_(info) {
  mapping_.setPinocchioInterface(pinocchioInterface_);
  pGains_ = vector_t::Zero(mapping_.getWholeBodyModelInfo().generalizedCoordinatesNum);
  dGains_ = vector_t::Zero(mapping_.getWholeBodyModelInfo().generalizedCoordinatesNum);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WholeBodyModelRbdConversions::computeBaseKinematicsFromWholeBodyModel(const vector_t& state, const vector_t& input,
                                                                             const vector_t& jointAccelerations, Vector6& basePose,
                                                                             Vector6& baseVelocity, Vector6& baseAcceleration) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_.getWholeBodyModelInfo();
  const auto qPinocchio = mapping_.getPinocchioJointPosition(state);

  updateWholeBodyDynamics(pinocchioInterface_, info, qPinocchio);

  // Base Pose in world frame
  basePose = qPinocchio.head<6>();
  const auto basePosition = basePose.head<3>();
  const auto baseOrientation = basePose.tail<3>();

  // Base Velocity in world frame
  const auto& A = getWholeBodyMomentumMatrix(pinocchioInterface_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto Ab_inv = computeFloatingBaseWholeBodyMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info.actuatedDofNum);

  const vector_t vPinocchio = mapping_.getPinocchioJointVelocity(state, input);
  baseVelocity.head<3>() = vPinocchio.head<3>();
  const Vector3 derivativeEulerAnglesZyx = vPinocchio.segment<3>(3);
  baseVelocity.tail<3>() = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(baseOrientation, derivativeEulerAnglesZyx);

  const auto Adot = pinocchio::dccrba(model, data, qPinocchio, vPinocchio);
  Vector6 wholebodyMomentumRate = info.totalMass * getNormalizedWholeBodyMomentumRate(pinocchioInterface_, info, input);
  wholebodyMomentumRate.noalias() -= Adot * vPinocchio;
  wholebodyMomentumRate.noalias() -= Aj * jointAccelerations.head(info.actuatedDofNum);
  const Vector6 qbaseDdot = Ab_inv * wholebodyMomentumRate;

  // Base Acceleration in world frame
  baseAcceleration.head<3>() = qbaseDdot.head<3>();
  baseAcceleration.tail<3>() =
      getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(baseOrientation, derivativeEulerAnglesZyx, qbaseDdot.tail<3>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t WholeBodyModelRbdConversions::computeWholeBodyStateFromRbdModel(const vector_t& rbdState) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_.getWholeBodyModelInfo();

  vector_t qPinocchio(info.generalizedCoordinatesNum);
  qPinocchio.head<3>() = rbdState.segment<3>(3);
  qPinocchio.segment<3>(3) = rbdState.head<3>();
  qPinocchio.tail(info.actuatedDofNum) = rbdState.segment(6, info.actuatedDofNum);

  vector_t vPinocchio(info.generalizedCoordinatesNum);
  vPinocchio.head<3>() = rbdState.segment<3>(info.generalizedCoordinatesNum + 3);
  vPinocchio.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPinocchio.segment<3>(3), rbdState.segment<3>(info.generalizedCoordinatesNum));
  vPinocchio.tail(info.actuatedDofNum) = rbdState.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

  updateWholeBodyDynamics(pinocchioInterface_, info, qPinocchio);
  const auto& A = getWholeBodyMomentumMatrix(pinocchioInterface_);

  vector_t state(info.stateDim);
  wholebody_model::getNormalizedMomentum(state, info).noalias() = A * vPinocchio / info.totalMass;
  wholebody_model::getGeneralizedCoordinates(state, info) = qPinocchio;

  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t WholeBodyModelRbdConversions::computeRbdStateFromWholeBodyModel(const vector_t& state, const vector_t& input) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_.getWholeBodyModelInfo();
  const vector_t jointAccelerations = vector_t::Zero(info.actuatedDofNum);

  Vector6 basePose, baseVelocity, baseAcceleration;
  computeBaseKinematicsFromWholeBodyModel(state, input, jointAccelerations, basePose, baseVelocity, baseAcceleration);

  vector_t rbdState(2 * info.generalizedCoordinatesNum);
  rbdState.segment<3>(0) = basePose.tail<3>();
  rbdState.segment<3>(3) = basePose.head<3>();
  rbdState.segment(6, info.actuatedDofNum) = wholebody_model::getJointAngles(state, info);
  rbdState.segment<3>(info.generalizedCoordinatesNum) = baseVelocity.tail<3>();
  rbdState.segment<3>(info.generalizedCoordinatesNum + 3) = baseVelocity.head<3>();
  rbdState.tail(info.actuatedDofNum) = wholebody_model::getJointVelocities(input, info);
  return rbdState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t WholeBodyModelRbdConversions::computeRbdTorqueFromWholeBodyModel(const vector_t& state, const vector_t& input,
                                                                            const vector_t& jointAccelerations) {
  const auto& info = mapping_.getWholeBodyModelInfo();
  const vector_t measuredRbdState = vector_t::Zero(2 * info.generalizedCoordinatesNum);
  const vector_t pGains = vector_t::Zero(info.generalizedCoordinatesNum);
  const vector_t dGains = vector_t::Zero(info.generalizedCoordinatesNum);
  return computeRbdTorqueFromWholeBodyModelPD(state, input, jointAccelerations, measuredRbdState, pGains, dGains);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t WholeBodyModelRbdConversions::computeRbdTorqueFromWholeBodyModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                                              const vector_t& desiredJointAccelerations,
                                                                              const vector_t& measuredRbdState) {
  return computeRbdTorqueFromWholeBodyModelPD(desiredState, desiredInput, desiredJointAccelerations, measuredRbdState, pGains_, dGains_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t WholeBodyModelRbdConversions::computeRbdTorqueFromWholeBodyModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                                              const vector_t& desiredJointAccelerations,
                                                                              const vector_t& measuredRbdState, const vector_t& pGains,
                                                                              const vector_t& dGains) {
  // handles
  const auto& info = mapping_.getWholeBodyModelInfo();
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  // desired
  Vector6 desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration;
  computeBaseKinematicsFromWholeBodyModel(desiredState, desiredInput, desiredJointAccelerations, desiredBasePose, desiredBaseVelocity,
                                           desiredBaseAcceleration);
  vector_t qDesired(info.generalizedCoordinatesNum), vDesired(info.generalizedCoordinatesNum), aDesired(info.generalizedCoordinatesNum);
  qDesired << desiredBasePose, wholebody_model::getJointAngles(desiredState, info);
  vDesired << desiredBaseVelocity, wholebody_model::getJointVelocities(desiredInput, info);
  aDesired << desiredBaseAcceleration, desiredJointAccelerations;

  pinocchio::container::aligned_vector<pinocchio::Force> fextDesired(model.njoints, pinocchio::Force::Zero());
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
    const auto jointIndex = model.frames[frameIndex].parent;
    const Vector3 translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const Matrix3 rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const Vector3 contactForce = rotationWorldFrameToJointFrame * wholebody_model::getContactForces(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce);
  }
  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
    const auto jointIndex = model.frames[frameIndex].parent;
    const Vector3 translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const Matrix3 rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const Vector3 contactForce = rotationWorldFrameToJointFrame * wholebody_model::getContactForces(desiredInput, i, info);
    const Vector3 contactTorque = rotationWorldFrameToJointFrame * wholebody_model::getContactTorques(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce) + contactTorque;
  }

  // measured
  vector_t qMeasured(info.generalizedCoordinatesNum), vMeasured(info.generalizedCoordinatesNum);
  qMeasured.head<3>() = measuredRbdState.segment<3>(3);
  qMeasured.segment<3>(3) = measuredRbdState.head<3>();
  qMeasured.tail(info.actuatedDofNum) = measuredRbdState.segment(6, info.actuatedDofNum);
  vMeasured.head<3>() = measuredRbdState.segment<3>(info.generalizedCoordinatesNum + 3);
  vMeasured.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured.segment<3>(3), measuredRbdState.segment<3>(info.generalizedCoordinatesNum));
  vMeasured.tail(info.actuatedDofNum) = measuredRbdState.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

  // PD feedback augmentation
  const vector_t pdFeedback = pGains.cwiseProduct(qDesired - qMeasured) + dGains.cwiseProduct(vDesired - vMeasured);

  // feedforward plus PD on acceleration level
  const vector_t aAugmented = aDesired + pdFeedback;
  return pinocchio::rnea(model, data, qDesired, vDesired, aAugmented, fextDesired);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WholeBodyModelRbdConversions::loadSettings(const std::string& fileName, const std::string& fieldName, bool verbose) {
  if (verbose) {
    std::cerr << "\n#### WholeBodyModelRbdConversionsSettings:\n";
    std::cerr << "#### =============================================================================" << std::endl;
  }

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);
  const std::string WholeBodyModelRbdConversionsFieldName = fieldName + ".wholebody_model_rbd_conversions";

  std::vector<scalar_t> pGainsVec, dGainsVec;
  loadData::loadStdVector(fileName, WholeBodyModelRbdConversionsFieldName + ".pGains", pGainsVec, verbose);
  if (!pGainsVec.empty()) {
    pGains_ = Eigen::Map<vector_t>(pGainsVec.data(), pGainsVec.size());
  }
  loadData::loadStdVector(fileName, WholeBodyModelRbdConversionsFieldName + ".dGains", dGainsVec, verbose);
  if (!dGainsVec.empty()) {
    dGains_ = Eigen::Map<vector_t>(dGainsVec.data(), dGainsVec.size());
  }
}

}  // namespace ocs2