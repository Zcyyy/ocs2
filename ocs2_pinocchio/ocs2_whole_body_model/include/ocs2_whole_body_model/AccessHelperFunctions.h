/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: AccessHelperFunctions.h
*
*          Created On: 2022年12月14日 星期三 18时50分07秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#pragma once

#include <Eigen/Core>

#include "ocs2_whole_body_model/WholeBodyModelInfo.h"

namespace ocs2 {
namespace wholebody_model {

/**
 * Provides read/write access to the contact forces.
 */
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 3, 1> getContactForces(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                             const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the contact forces.
 */
template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 3, 1> getContactForces(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                         const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the contact torques.
 */
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 3, 1> getContactTorques(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                              const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the contact torques.
 */
template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 3, 1> getContactTorques(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                          const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the joint velocities.
 */
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getJointVelocities(Eigen::MatrixBase<Derived>& input, const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the joint velocities.
 */
template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getJointVelocities(const Eigen::MatrixBase<Derived>& input,
                                                            const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the mass-normalized momentum.
 */
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 6, 1> getNormalizedMomentum(Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the mass-normalized momentum.
 */
template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 6, 1> getNormalizedMomentum(const Eigen::MatrixBase<Derived>& state,
                                                              const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the base pose.
 */
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 6, 1> getBasePose(Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the base pose.
 */
template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 6, 1> getBasePose(const Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the joint angles.
 */
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getJointAngles(Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the joint angles.
 */
template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getJointAngles(const Eigen::MatrixBase<Derived>& state,
                                                        const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the generalized coordinates.
 */
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the generalized coordinates.
 */
template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state,
                                                                   const WholeBodyModelInfoTpl<SCALAR>& info);

}  // namespace centroidal_model
}  // namespace ocs2

#include "implementation/AccessHelperFunctionsImpl.h"