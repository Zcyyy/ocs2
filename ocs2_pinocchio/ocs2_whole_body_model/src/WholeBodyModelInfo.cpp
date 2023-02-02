/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: WholeBodyModelInfo.cpp
*
*          Created On: 2022年12月14日 星期三 15时23分31秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#include "ocs2_whole_body_model/WholeBodyModelInfo.h"

namespace ocs2{
std::string toString(WholeBodyModelType type){
    switch (type) {
        case WholeBodyModelType::WholeBodyDynamics:
            return "WholeBodyDynamics";
        default:
            return "Unkonwn Dynamics Type!";
    }
}

std::ostream& operator<<(std::ostream& os, WholeBodyModelType type) {
    os << toString(type);
    return os;
}

template <>
template <>
WholeBodyModelInfoCppAd WholeBodyModelInfo::toCppAd() const {
  WholeBodyModelInfoCppAd cppAdInfo;

  cppAdInfo.wholebodyModelType = this->wholebodyModelType;
  cppAdInfo.numThreeDofContacts = this->numThreeDofContacts;
  cppAdInfo.numSixDofContacts = this->numSixDofContacts;
  cppAdInfo.endEffectorFrameIndices = this->endEffectorFrameIndices;
  cppAdInfo.generalizedCoordinatesNum = this->generalizedCoordinatesNum;
  cppAdInfo.actuatedDofNum = this->actuatedDofNum;
  cppAdInfo.stateDim = this->stateDim;
  cppAdInfo.inputDim = this->inputDim;
  cppAdInfo.totalMass = ad_scalar_t(this->totalMass);
  cppAdInfo.qPinocchioNominal = this->qPinocchioNominal.cast<ad_scalar_t>();
  cppAdInfo.comToBasePositionNominal = this->comToBasePositionNominal.cast<ad_scalar_t>();
  for(auto j : this->WholeBodyInertiaNominal)
  {
    cppAdInfo.WholeBodyInertiaNominal.push_back(std::make_pair(j.first, j.second.cast<ad_scalar_t>()));
  }
  for(auto i : this->robotMass)
  {
    cppAdInfo.robotMass.push_back(std::make_pair(i.first, (ad_scalar_t)i.second));
  }

  return cppAdInfo;
}

// explicit template instantiation
template struct ocs2::WholeBodyModelInfoTpl<ocs2::scalar_t>;
template struct ocs2::WholeBodyModelInfoTpl<ocs2::ad_scalar_t>;
}