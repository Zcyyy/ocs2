/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: AccessHelperFunctionsImpl.h
*
*          Created On: 2022年12月14日 星期三 18时56分54秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

namespace ocs2 {
namespace wholebody_model {

template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 3, 1> getContactForces(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                             const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  const size_t contactForceIndex = 3 * contactIndex;
  const size_t contactWrenchIndex = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
  const size_t startRow = (contactIndex < info.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
  return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 3, 1> getContactForces(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                         const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  const size_t contactForceIndex = 3 * contactIndex;
  const size_t contactWrenchIndex = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
  const size_t startRow = (contactIndex < info.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
  return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
}

template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 3, 1> getContactTorques(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                              const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  assert(contactIndex >= info.numThreeDofContacts);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts) + 3;
  return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 3, 1> getContactTorques(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                          const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  assert(contactIndex >= info.numThreeDofContacts);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts) + 3;
  return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
}

template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getJointVelocities(Eigen::MatrixBase<Derived>& input, const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.actuatedDofNum, 1);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getJointVelocities(const Eigen::MatrixBase<Derived>& input,
                                                            const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.actuatedDofNum, 1);
}

template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 6, 1> getNormalizedMomentum(Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<Derived, 6, 1>(state.derived(), 0, 0);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 6, 1> getNormalizedMomentum(const Eigen::MatrixBase<Derived>& state,
                                                              const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<const Derived, 6, 1>(state.derived(), 0, 0);
}

template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 6, 1> getBasePose(Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<Derived, 6, 1>(state.derived(), 6, 0);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 6, 1> getBasePose(const Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<const Derived, 6, 1>(state.derived(), 6, 0);
}

template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getJointAngles(Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<Derived, -1, 1>(state.derived(), 12, 0, info.actuatedDofNum, 1);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getJointAngles(const Eigen::MatrixBase<Derived>& state,
                                                        const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<const Derived, -1, 1>(state.derived(), 12, 0, info.actuatedDofNum, 1);
}

template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state, const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<Derived, -1, 1>(state.derived(), 6, 0, info.generalizedCoordinatesNum, 1);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state,
                                                                   const WholeBodyModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<const Derived, -1, 1>(state.derived(), 6, 0, info.generalizedCoordinatesNum, 1);
}
}
}