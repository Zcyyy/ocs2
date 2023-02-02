/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_whole_body_model/PinocchioWholeBodyDynamicsAD.h"

#include "ocs2_whole_body_model/AccessHelperFunctions.h"
#include "ocs2_whole_body_model/ModelHelperFunctions.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioWholeBodyDynamicsAD::PinocchioWholeBodyDynamicsAD(const PinocchioInterface& pinocchioInterface, const WholeBodyModelInfo& info,
                                                             const std::string& modelName, const std::string& modelFolder,
                                                             bool recompileLibraries, bool verbose) {
  auto systemFlowMapFunc = [&](const ad_vector_t& x, ad_vector_t& y) {
    // initialize CppAD interface
    auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

    // mapping
    WholeBodyModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
    mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);

    ad_vector_t state = x.head(info.stateDim);
    ad_vector_t input = x.tail(info.inputDim);
    y = getValueCppAd(pinocchioInterfaceCppAd, mappingCppAd, state, input);
  };

  systemFlowMapCppAdInterfacePtr_.reset(
      new CppAdInterface(systemFlowMapFunc, info.stateDim + info.inputDim, modelName + "_systemFlowMap", modelFolder));

  if (recompileLibraries) {
    systemFlowMapCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    systemFlowMapCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioWholeBodyDynamicsAD::PinocchioWholeBodyDynamicsAD(const PinocchioWholeBodyDynamicsAD& rhs)
    : systemFlowMapCppAdInterfacePtr_(new CppAdInterface(*rhs.systemFlowMapCppAdInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioWholeBodyDynamicsAD::getValueCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                         const WholeBodyModelPinocchioMappingCppAd& mappingCppAd, const ad_vector_t& state,
                                                         const ad_vector_t& input) {
  const auto& info = mappingCppAd.getWholeBodyModelInfo();
  assert(info.stateDim == state.rows());

  const ad_vector_t qPinocchio = mappingCppAd.getPinocchioJointPosition(state);
  updateWholeBodyDynamics(pinocchioInterfaceCppAd, info, qPinocchio);

  ad_vector_t stateDerivative(info.stateDim);

  // compute center of mass acceleration and derivative of the normalized angular momentum
  wholebody_model::getNormalizedMomentum(stateDerivative, info) =
      getNormalizedWholeBodyMomentumRate(pinocchioInterfaceCppAd, info, input);

  // derivatives of the floating base variables + joint velocities
  wholebody_model::getGeneralizedCoordinates(stateDerivative, info) = mappingCppAd.getPinocchioJointVelocity(state, input);

  return stateDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioWholeBodyDynamicsAD::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
  return systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PinocchioWholeBodyDynamicsAD::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input) const {
  const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
  VectorFunctionLinearApproximation approx;
  approx.f = systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput);
  const matrix_t dynamicsJacobian = systemFlowMapCppAdInterfacePtr_->getJacobian(stateInput);
  //std::cout << dynamicsJacobian << std::endl;
  approx.dfdx = dynamicsJacobian.leftCols(state.rows());
  //std::cout << approx.dfdu << std::endl;
  approx.dfdu = dynamicsJacobian.rightCols(input.rows());
  return approx;
}

}  // namespace ocs2
