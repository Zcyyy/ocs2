/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: WholeBodyModelInfo.h
*
*          Created On: 2022年12月13日 星期二 18时04分17秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#pragma once

#include <iostream>
#include <string>
#include <type_traits>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

namespace ocs2{

template <typename SCALAR>
class WholeBodyModelInfoTpl;

using WholeBodyModelInfo = WholeBodyModelInfoTpl<scalar_t>;
using WholeBodyModelInfoCppAd = WholeBodyModelInfoTpl<ad_scalar_t>;

enum class WholeBodyModelType {WholeBodyDynamics};

std::string toString(WholeBodyModelType type);
std::ostream& operator<<(std::ostream& os, WholeBodyModelType type);

template <typename SCALAR>
struct WholeBodyModelInfoTpl{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using scalar_t = SCALAR;
    using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using vector3_t = Eigen::Matrix<SCALAR, 3, 1>;
    using matrix3_t = Eigen::Matrix<SCALAR, 3, 3>;

    using robotMassTpl = std::pair<std::string, scalar_t>;
    using WholeBodyInertiaNominalTpl = std::pair<const std::string, matrix3_t>;

    template <typename T>  // Template for conditional compilation using SFINAE
    using EnableIfScalar_t = typename std::enable_if<std::is_same<T, scalar_t>::value, bool>::type;

    WholeBodyModelType wholebodyModelType;
    size_t numThreeDofContacts;
    size_t numSixDofContacts;
    std::vector<size_t> endEffectorFrameIndices;
    size_t generalizedCoordinatesNum;
    size_t actuatedDofNum;
    size_t stateDim;
    size_t inputDim;
    scalar_t totalMass;
    vector_t qPinocchioNominal;
    vector3_t comToBasePositionNominal;
    std::vector<robotMassTpl> robotMass;
    std::vector<WholeBodyInertiaNominalTpl> WholeBodyInertiaNominal;

    template <typename T = SCALAR, EnableIfScalar_t<T> = true>
    WholeBodyModelInfoCppAd toCppAd() const;
};

extern template struct WholeBodyModelInfoTpl<scalar_t>;
extern template struct WholeBodyModelInfoTpl<ad_scalar_t>;
}