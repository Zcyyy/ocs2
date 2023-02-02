/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: ModelHelperFunctions.h
*
*          Created On: 2022年12月14日 星期三 19时40分20秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#pragma once

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_whole_body_model/WholeBodyModelInfo.h"

namespace ocs2 {

/**
 * Get the inverse of the sub-block of the centroidal momentum matrix which corresponds to the floating base variables.
 *  Ab_inv = [  1/m I_{3,3},    -1/m*Ab_12*Ab_22^(-1),
 *              O_{3,3},         Ab_22^(-1)          ]
 *
 * @param [in] A(q): centroidal momentum matrix
 * @return Ab_inv(q): inverse of the 6x6 left-block of A(q)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseWholeBodyMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab);

/**
 * Updates the centroidal momentum matrix in data.Ag and the CoM position in data.com[0] for the FullCentroidalDynamics
 * model and the SingleRigidBodyDynamics Model
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 *
 * @remark: This function also internally calls:
 *       pinocchio::forwardKinematics(model, data, q)
 *       pinocchio::computeJointJacobians(model, data, q) (only for the FullCentroidalDynamics case)
 *       pinocchio::updateFramePlacements(model, data)
 */
template <typename SCALAR_T>
void updateWholeBodyDynamics(PinocchioInterfaceTpl<SCALAR_T>& interface, const WholeBodyModelInfoTpl<SCALAR_T>& info,
                              const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q);

/**
 * Updates the centroidal momentum derivatives (such as in data.dHdq) for the FullCentroidalDynamics model
 * and the SingleRigidBodyDynamics Model
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 * @param [in] v: pinocchio joint velocities (derivatives of generalized coordinates)
 *
 * @remark: This function also internally calls:
 *       pinocchio::forwardKinematics(model, data, q)
 *       pinocchio::computeJointJacobians(model, data, q)
 *       pinocchio::updateFramePlacements(model, data)
 */
template <typename SCALAR_T>
void updateWholeBodyDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T>& interface, const WholeBodyModelInfoTpl<SCALAR_T>& info,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v);

/**
 * Computes derivatives of the mapping (ZYX-Euler angles derivatives --> Global angular velocities)
 * with respect to the base orientation (ZYX-Euler angles)
 *
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @return A tensor representing the derivative of the mapping w.r.t the ZYX-Euler angles
 */
template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles);

/**
 * Computes derivatives of the rotation matrix (base frame --> world frame) with respect to
 * the base orientation (in ZYX-Euler angles)
 *
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @return A tensor representing the derivative of the rotation matrix w.r.t the ZYX-Euler angles
 */
template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getRotationMatrixZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles);

/**
 * Computes derivatives of centroidal momentum with respect to the base orientation (in ZYX-Euler angles)
 *
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 * @param [in] v: pinocchio joint velocities (derivatives of generalized coordinates)
 * @return Derivative of centroidal momentum w.r.t the ZYX-Euler Angles
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 3> getWholeBodyMomentumZyxGradient(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                               const WholeBodyModelInfoTpl<SCALAR_T>& info,
                                                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v);

/**
 * Returns the centroidal momentum matrix from the pinocchioInterface data
 * @param [in] interface: pinocchio robot interface containing model + data
 * @return centroidal momentum matrix from data.Ag
 *
 * @note requires pinocchioInterface to be updated with:
 *       ocs2::updateCentroidalDynamics(interface, info, q)
 */
template <typename SCALAR_T>
const Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>& getWholeBodyMomentumMatrix(const PinocchioInterfaceTpl<SCALAR_T>& interface);

/**
 * Computes the CoM to contact point position in world frame
 *
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] contactIndex: index of the contact point
 * @return: position of the contact point w.r.t CoM expressed in world frame
 *
 * @note requires pinocchioInterface to be updated with:
 *       ocs2::updateCentroidalDynamics(interface, info, q)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getPositionComToContactPointInWorldFrame(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                       const WholeBodyModelInfoTpl<SCALAR_T>& info, size_t contactIndex);

/**
 * Computes the CoM to contact point translational Jacobian in world frame
 *
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] contactIndex: index of the contact point
 * @return: CoM to contact point translational Jacobian expressed in world frame
 *
 * @note requires pinocchioInterface to be updated with:
 *       ocs2::updateCentroidalDynamics(interface, info, q) (should be called first)
 *       pinocchio::computeJointJacobians(model, data, q)
 *       pinocchio::updateFramePlacements(model, data)
 */
// TODO: Need to copy data here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame(
    const PinocchioInterfaceTpl<SCALAR_T>& interface, const WholeBodyModelInfoTpl<SCALAR_T>& info, size_t contactIndex);

/**
 * Computes the derivative of the normalized centroidal momentum (linear + angular) expressed in the centroidal frame
 *
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] input: system input vector
 * @return: time derivative of normalized centroidal momentum
 *
 * @note requires pinocchioInterface to be updated with:
 *       ocs2::updateCentroidalDynamics(interface, info, q)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 1> getNormalizedWholeBodyMomentumRate(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                  const WholeBodyModelInfoTpl<SCALAR_T>& info,
                                                                  const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input);

}

#include "implementation/ModelHelperFunctionsImpl.h"
