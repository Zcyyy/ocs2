/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: ModelHelperFunctions.cpp
*
*          Created On: 2022年12月14日 星期三 19时36分48秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_whole_body_model/ModelHelperFunctions.h>

#include <ocs2_whole_body_model/AccessHelperFunctions.h>

#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace ocs2 {

template <typename SCALAR_T>
void updateWholeBodyDynamics(PinocchioInterfaceTpl<SCALAR_T>& interface, const WholeBodyModelInfoTpl<SCALAR_T>& info,
                              const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q) {
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR_T, 3, 3>;
  using matrix6_t = Eigen::Matrix<SCALAR_T, 6, 6>;

  const auto& model = interface.getModel();
  auto& data = interface.getData();

  //std::cout << info.wholebodyModelType << std::endl;
  switch (info.wholebodyModelType) {
    case WholeBodyModelType::WholeBodyDynamics: {
      //std::cout << " dooooo something??????? " << std::endl;
      pinocchio::computeCentroidalMap(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      break;
    }
    /*
    case CentroidalModelType::FullCentroidalDynamics: {
        //TODO:~~~~~~~~~~~~~~~
      //pinocchio::computeCentroidalMap(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case WholeBodyModelType::WholeBodyDynamics: {
      const vector3_t eulerAnglesZyx = q.template segment<3>(3);
      const matrix3_t mappingZyx = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAnglesZyx);
      const matrix3_t rotationBaseToWorld = getRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
      const vector3_t comToBasePositionInWorld = rotationBaseToWorld * info.comToBasePositionNominal;
      const matrix3_t skewSymmetricMap = skewSymmetricMatrix(comToBasePositionInWorld);
      const matrix3_t mat1 = rotationBaseToWorld * info.centroidalInertiaNominal;
      const matrix3_t mat2 = rotationBaseToWorld.transpose() * mappingZyx;
      matrix6_t Ab = matrix6_t::Zero();
      Ab.template topLeftCorner<3, 3>().diagonal().array() = info.robotMass;
      Ab.template topRightCorner<3, 3>().noalias() = info.robotMass * skewSymmetricMap * mappingZyx;
      Ab.template bottomRightCorner<3, 3>().noalias() = mat1 * mat2;
      data.Ag = Eigen::Matrix<SCALAR_T, -1, -1>::Zero(6, info.generalizedCoordinatesNum);
      data.Ag.template leftCols<6>() = Ab;
      data.com[0] = q.template head<3>() - comToBasePositionInWorld;
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    */
    default: {
      throw std::runtime_error("The chosen whole body model type is not supported.");
      break;
    }
  }
}

template <typename SCALAR_T>
void updateWholeBodyDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T>& interface, const WholeBodyModelInfoTpl<SCALAR_T>& info,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v) {
  using matrix6x_t = Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  vector_t a = vector_t::Zero(info.generalizedCoordinatesNum);
  matrix6x_t dhdq(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotdq(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotdv(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotda(6, info.generalizedCoordinatesNum);

  switch (info.wholebodyModelType) {
    case WholeBodyModelType::WholeBodyDynamics: {
      //compute something
      std::cout << " Do nothing! " << std::endl;
      break;
    }
    /*
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalDynamicsDerivatives(model, data, q, v, a, dhdq, dhdotdq, dhdotdv, dhdotda);
      data.Ag = dhdotda;
      // Filling in data.dFdq is a hack since data.dhdq is not available
      data.dFdq.setZero(6, info.generalizedCoordinatesNum);
      data.dFdq.template middleCols<3>(3) = getCentroidalMomentumZyxGradient(interface, info, q, v);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      // Filling in data.dFdq is a hack since data.dhdq is not available
      data.dFdq.setZero(6, info.generalizedCoordinatesNum);
      data.dFdq.template middleCols<3>(3) = getCentroidalMomentumZyxGradient(interface, info, q, v);
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    */
    default: {
      throw std::runtime_error("The chosen Whole Body model type is not supported.");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
const Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>& getWholeBodyMomentumMatrix(const PinocchioInterfaceTpl<SCALAR_T>& interface) {
  return interface.getData().Ag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getPositionComToContactPointInWorldFrame(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                       const WholeBodyModelInfoTpl<SCALAR_T>& info, size_t contactIndex) {
  const auto& data = interface.getData();
  return (data.oMf[info.endEffectorFrameIndices[contactIndex]].translation() - data.com[0]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame(
    const PinocchioInterfaceTpl<SCALAR_T>& interface, const WholeBodyModelInfoTpl<SCALAR_T>& info, size_t contactIndex) {
  const auto& model = interface.getModel();
  auto data = interface.getData();
  Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic> jacobianWorldToContactPointInWorldFrame;
  jacobianWorldToContactPointInWorldFrame.setZero(6, info.generalizedCoordinatesNum);
  pinocchio::getFrameJacobian(model, data, info.endEffectorFrameIndices[contactIndex], pinocchio::LOCAL_WORLD_ALIGNED,
                              jacobianWorldToContactPointInWorldFrame);
  Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> J_com = getWholeBodyMomentumMatrix(interface).template topRows<3>() / info.totalMass;
  return (jacobianWorldToContactPointInWorldFrame.template topRows<3>() - J_com);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 1> getNormalizedWholeBodyMomentumRate(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                  const WholeBodyModelInfoTpl<SCALAR_T>& info,
                                                                  const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input) {
  const Eigen::Matrix<SCALAR_T, 3, 1> gravityVector(SCALAR_T(0.0), SCALAR_T(0.0), SCALAR_T(-9.81));
  Eigen::Matrix<SCALAR_T, 6, 1> WholeBodyMomentumRate;
  WholeBodyMomentumRate << info.totalMass * gravityVector, Eigen::Matrix<SCALAR_T, 3, 1>::Zero();

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto contactForceInWorldFrame = wholebody_model::getContactForces(input, i, info);
    const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrame(interface, info, i);
    WholeBodyMomentumRate.template head<3>() += contactForceInWorldFrame;
    WholeBodyMomentumRate.template tail<3>().noalias() += positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame);
  }  // end of i loop

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto contactForceInWorldFrame = wholebody_model::getContactForces(input, i, info);
    const auto contactTorqueInWorldFrame = wholebody_model::getContactTorques(input, i, info);
    const auto positionComToContactPointInWorldFrame = getPositionComToContactPointInWorldFrame(interface, info, i);
    WholeBodyMomentumRate.template head<3>() += contactForceInWorldFrame;
    WholeBodyMomentumRate.template tail<3>().noalias() +=
        positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame) + contactTorqueInWorldFrame;
  }  // end of i loop

  // normalize by the total mass
  WholeBodyMomentumRate /= info.totalMass;

  return WholeBodyMomentumRate;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// Explicit template instantiation
template void updateWholeBodyDynamics<scalar_t>(PinocchioInterface&, const WholeBodyModelInfo&, const vector_t&);
template void updateWholeBodyDynamics<ad_scalar_t>(PinocchioInterfaceCppAd&, const WholeBodyModelInfoCppAd&, const ad_vector_t&);

template void updateWholeBodyDynamicsDerivatives<scalar_t>(PinocchioInterface&, const WholeBodyModelInfo& info, const vector_t&,
                                                            const vector_t&);
template void updateWholeBodyDynamicsDerivatives<ad_scalar_t>(PinocchioInterfaceCppAd&, const WholeBodyModelInfoCppAd& info,
                                                               const ad_vector_t&, const ad_vector_t&);

template const Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>& getWholeBodyMomentumMatrix<scalar_t>(const PinocchioInterface&);
template const Eigen::Matrix<ad_scalar_t, 6, Eigen::Dynamic>& getWholeBodyMomentumMatrix<ad_scalar_t>(const PinocchioInterfaceCppAd&);

template Eigen::Matrix<scalar_t, 3, 1> getPositionComToContactPointInWorldFrame<scalar_t>(const PinocchioInterface&,
                                                                                          const WholeBodyModelInfo&, size_t);
template Eigen::Matrix<ad_scalar_t, 3, 1> getPositionComToContactPointInWorldFrame<ad_scalar_t>(const PinocchioInterfaceCppAd&,
                                                                                                const WholeBodyModelInfoCppAd&, size_t);

template Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame<scalar_t>(
    const PinocchioInterface&, const WholeBodyModelInfo&, size_t);
template Eigen::Matrix<ad_scalar_t, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame<ad_scalar_t>(
    const PinocchioInterfaceCppAd&, const WholeBodyModelInfoCppAd&, size_t);

template Eigen::Matrix<scalar_t, 6, 1> getNormalizedWholeBodyMomentumRate<scalar_t>(const PinocchioInterface&, const WholeBodyModelInfo&, const vector_t& );
template Eigen::Matrix<ad_scalar_t, 6, 1> getNormalizedWholeBodyMomentumRate<ad_scalar_t>(const PinocchioInterfaceCppAd&, const WholeBodyModelInfoCppAd&, const ad_vector_t&);

}
