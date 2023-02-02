/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: WholeBodyModelpinocchioMapping.h
*
*          Created On: 2022年12月14日 星期三 21时01分56秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "ocs2_whole_body_model/WholeBodyModelInfo.h"

namespace ocs2 {

template <typename SCALAR>
class WholeBodyModelPinocchioMappingTpl;

using WholeBodyModelPinocchioMapping = WholeBodyModelPinocchioMappingTpl<scalar_t>;
using WholeBodyModelPinocchioMappingCppAd = WholeBodyModelPinocchioMappingTpl<ad_scalar_t>;

/**
 * Centroidal Dynamics:
 *
 * State: x = [ linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_positions ]'
 * @remark: The linear and angular momenta are expressed with respect to the centroidal frame (a frame centered at
 * the CoM and aligned with the inertial frame).
 *
 * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
 * @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
 *
 *
 * Pinocchio Joint Positions: qPinocchio = [ base_position, base_orientation_zyx, joint_positions ]'
 * @remark: Base position is expressed with respect to the inertial frame
 *
 * Pinocchio Joint Velocities: vPinocchio = [ base_linear_velocity, base_orientation_zyx_derivatives, joint_velocities ]'
 * @remark: Base linear velocity is expressed with respect to the inertial frame
 */
template <typename SCALAR>
class WholeBodyModelPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  /**
   * Constructor
   * @param [in] centroidalModelInfo : centroidal model information.
   */
  explicit WholeBodyModelPinocchioMappingTpl(WholeBodyModelInfoTpl<SCALAR> WholeBodyModelInfo);

  ~WholeBodyModelPinocchioMappingTpl() override = default;
  WholeBodyModelPinocchioMappingTpl* clone() const override;

  /** Sets the pinocchio interface for caching
   * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
   * @note The pinocchio interface must be set before calling the getters.
   */
  void setPinocchioInterface(const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface) override;

  /**
   * Computes the vector of generalized coordinates (qPinocchio) used by pinocchio functions from the robot state variables
   *
   * @param [in] state: system state vector
   * @return pinocchio joint positions, which are also the robot's generalized positions with a ZYX-Euler angle
   * parameterization of the base orientation
   */
  vector_t getPinocchioJointPosition(const vector_t& state) const override;

  /**
   * Computes the vector of generalized velocities (vPinocchio) used by pinocchio functions from the robot state and input variables
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return pinocchio joint velocities, which are also the time derivatives of the pinocchio joint positions
   *
   * @note requires pinocchioInterface to be updated with:
   *       ocs2::updateCentroidalDynamics(interface, info, q)
   */
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

  /**
   * Maps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu.
   * @param [in] state: system state vector
   * @param [in] Jq: jacobian with respect to pinocchio joint positions
   * @param [in] Jv: jacobian with respect to pinocchio joint velocities
   * @return a pair {dfdx, dfdu} containing the jacobians with respect to the system state and input
   *
   * @note requires pinocchioInterface to be updated with:
   *       ocs2::updateCentroidalDynamicsDerivatives(interface, info, q, v)
   *
   * TODO: Add Jacobian w.r.t generalized accelerations as argument to get a full implicit dependence on the inputs
   */
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  /**
   * Returns a structure containing robot-specific information needed for the centroidal dynamics computations.
   */
  const WholeBodyModelInfoTpl<SCALAR>& getWholeBodyModelInfo() const { return wholebodyModelInfo_; }

 private:
  WholeBodyModelPinocchioMappingTpl(const WholeBodyModelPinocchioMappingTpl& rhs);

  const PinocchioInterfaceTpl<SCALAR>* pinocchioInterfacePtr_;
  const WholeBodyModelInfoTpl<SCALAR> wholebodyModelInfo_;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class WholeBodyModelPinocchioMappingTpl<scalar_t>;
extern template class WholeBodyModelPinocchioMappingTpl<ad_scalar_t>;

}  // namespace ocs2
