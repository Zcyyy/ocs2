/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: WholeBodyModelPinocchioMapping.cpp
*
*          Created On: 2022年12月14日 星期三 21时22分02秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "ocs2_whole_body_model/WholeBodyModelPinocchioMapping.h"

//#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "ocs2_whole_body_model/AccessHelperFunctions.h"
#include "ocs2_whole_body_model/ModelHelperFunctions.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
WholeBodyModelPinocchioMappingTpl<SCALAR>::WholeBodyModelPinocchioMappingTpl(WholeBodyModelInfoTpl<SCALAR> WholeBodyModelInfo)
    : pinocchioInterfacePtr_(nullptr), wholebodyModelInfo_(std::move(WholeBodyModelInfo)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
WholeBodyModelPinocchioMappingTpl<SCALAR>::WholeBodyModelPinocchioMappingTpl(const WholeBodyModelPinocchioMappingTpl& rhs)
    : pinocchioInterfacePtr_(nullptr), wholebodyModelInfo_(rhs.wholebodyModelInfo_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
WholeBodyModelPinocchioMappingTpl<SCALAR>* WholeBodyModelPinocchioMappingTpl<SCALAR>::clone() const {
  return new WholeBodyModelPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void WholeBodyModelPinocchioMappingTpl<SCALAR>::setPinocchioInterface(const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface) {
  pinocchioInterfacePtr_ = &pinocchioInterface;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto WholeBodyModelPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return wholebody_model::getGeneralizedCoordinates(state, wholebodyModelInfo_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto WholeBodyModelPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const -> vector_t {
  const auto& model = pinocchioInterfacePtr_->getModel();
  const auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = wholebodyModelInfo_;
  assert(info.stateDim == state.rows());
  assert(info.inputDim == input.rows());

  const auto& A = getWholeBodyMomentumMatrix(*pinocchioInterfacePtr_);
  const Eigen::Matrix<SCALAR, 6, 6> Ab = A.template leftCols<6>();
  const auto Ab_inv = computeFloatingBaseWholeBodyMomentumMatrixInverse(Ab);

  const auto jointVelocities = wholebody_model::getJointVelocities(input, info).head(info.actuatedDofNum);

  Eigen::Matrix<SCALAR, 6, 1> momentum = info.totalMass * wholebody_model::getNormalizedMomentum(state, info);
  if (info.wholebodyModelType == WholeBodyModelType::WholeBodyDynamics) {
    std::cout << "what should i do? " << std::endl;
  }
  /*
  if (info.wholebodyModelType == WholeBodyModelType::FullCentroidalDynamics) {
    momentum.noalias() -= A.rightCols(info.actuatedDofNum) * jointVelocities;
  }
  */

  vector_t vPinocchio(info.generalizedCoordinatesNum);
  vPinocchio.template head<6>().noalias() = Ab_inv * momentum;
  vPinocchio.tail(info.actuatedDofNum) = jointVelocities;

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto WholeBodyModelPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  const auto& model = pinocchioInterfacePtr_->getModel();
  const auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = wholebodyModelInfo_;
  assert(info.stateDim == state.rows());

  // Partial derivatives of joint velocities
  matrix_t jointVelocitiesDerivativeInput = matrix_t::Zero(info.actuatedDofNum, info.inputDim);
  jointVelocitiesDerivativeInput.rightCols(info.actuatedDofNum).setIdentity();

  // Partial derivatives of the floating base variables
  // TODO: move getFloatingBaseCentroidalMomentumMatrixInverse(Ab) to PreComputation
  matrix_t floatingBaseVelocitiesDerivativeState = matrix_t::Zero(6, info.stateDim);
  matrix_t floatingBaseVelocitiesDerivativeInput = matrix_t::Zero(6, info.inputDim);
  const auto& A = getWholeBodyMomentumMatrix(*pinocchioInterfacePtr_);
  const Eigen::Matrix<SCALAR, 6, 6> Ab = A.template leftCols<6>();
  const auto Ab_inv = computeFloatingBaseWholeBodyMomentumMatrixInverse(Ab);
  floatingBaseVelocitiesDerivativeState.leftCols(6) = info.totalMass * Ab_inv;

  using matrix6x_t = Eigen::Matrix<SCALAR, 6, Eigen::Dynamic>;
  matrix6x_t dhdq(6, info.generalizedCoordinatesNum);
  switch (info.wholebodyModelType) {
    case WholeBodyModelType::WholeBodyDynamics: {
        std::cout << " TODO: do something in this " << std::endl;
        break;
    }
    /*
    case WholeBodyModelType::FullCentroidalDynamics: {
      pinocchio::translateForceSet(data.dHdq, data.com[0], dhdq.const_cast_derived());
      for (size_t k = 0; k < model.nv; ++k) {
        dhdq.template block<3, 1>(pinocchio::Force::ANGULAR, k) +=
            data.hg.linear().cross(data.dFda.template block<3, 1>(pinocchio::Force::LINEAR, k)) / data.Ig.mass();
      }
      dhdq.middleCols(3, 3) = data.dFdq.middleCols(3, 3);
      const auto Aj = A.rightCols(info.actuatedDofNum);
      floatingBaseVelocitiesDerivativeState.rightCols(info.generalizedCoordinatesNum).noalias() = -Ab_inv * dhdq;
      floatingBaseVelocitiesDerivativeInput.rightCols(info.actuatedDofNum).noalias() = -Ab_inv * Aj;
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      dhdq = data.dFdq;
      floatingBaseVelocitiesDerivativeState.middleCols(6, 6).noalias() = -Ab_inv * dhdq.leftCols(6);
      break;
    }
    */
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }

  matrix_t dvdx = matrix_t::Zero(info.generalizedCoordinatesNum, info.stateDim);
  dvdx.template topRows<6>() = floatingBaseVelocitiesDerivativeState;
  matrix_t dvdu = matrix_t::Zero(info.generalizedCoordinatesNum, info.inputDim);
  dvdu << floatingBaseVelocitiesDerivativeInput, jointVelocitiesDerivativeInput;
  matrix_t dfdx = matrix_t::Zero(Jq.rows(), wholebodyModelInfo_.stateDim);
  dfdx.middleCols(6, info.generalizedCoordinatesNum) = Jq;
  dfdx.noalias() += Jv * dvdx;
  const matrix_t dfdu = Jv * dvdu;
  return {dfdx, dfdu};
}

// explicit template instantiation
template class ocs2::WholeBodyModelPinocchioMappingTpl<ocs2::scalar_t>;
template class ocs2::WholeBodyModelPinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace ocs2