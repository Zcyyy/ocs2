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

#include "ocs2_oc/rollout/InitializerRollout.h"

#include <ocs2_core/NumericTraits.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
InitializerRollout::InitializerRollout(const Initializer& initializer, rollout::Settings rolloutSettings /* = rollout::Settings() */)
    : RolloutBase(std::move(rolloutSettings)), initializerPtr_(initializer.clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
InitializerRollout* InitializerRollout::clone() const {
  return new InitializerRollout(*initializerPtr_, this->settings());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 代码初始化滚动 :: 运行（）函数
vector_t InitializerRollout::run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller, ModeSchedule& modeSchedule, scalar_array_t& timeTrajectory, size_array_t& postEventIndices, vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  // 如果初始时间大于最终时间，抛出异常
  if (initTime > finalTime) {
      throw std::runtime_error("[InitializerRollout::run] The initial time should be less-equal to the final time!");
  } 
  
  // 抽取子系统
  const auto timeIntervalArray = findActiveModesTimeInterval(initTime, finalTime, modeSchedule.eventTimes);
  const auto numSubsystems = timeIntervalArray.size();
  const auto numEvents = numSubsystems - 1;
  const size_t maxNumSteps = (timeIntervalArray.back().second - timeIntervalArray.front().first) / settings().timeStep;

  // 清除输出轨迹
  timeTrajectory.clear();
  timeTrajectory.reserve(maxNumSteps + 2 * numSubsystems);
  stateTrajectory.clear();
  stateTrajectory.reserve(maxNumSteps + 2 * numSubsystems); 
  inputTrajectory.clear();
  inputTrajectory.reserve(maxNumSteps + 2 * numSubsystems);
  postEventIndices.clear();
  postEventIndices.reserve(numEvents); 

  // 初始化输入向量和状态向量
  vector_t input;
  vector_t state = initState;

  // 遍历子系统
  for (size_t i = 0; i < numSubsystems; i++) {
    // 计算每一步所需要的时间
    const size_t numSteps = (timeIntervalArray[i].second - timeIntervalArray[i].first) / settings().timeStep;
    const scalar_t remainderTime = timeIntervalArray[i].second - (timeIntervalArray[i].first + numSteps * settings().timeStep);

    // 根据当前的timeIntervalArray，从起始状态到最终状态执行（numSteps + 1）步操作
    for (size_t k = 0; k < numSteps + 1; k++) {
      // 向timeTrajectory容器中添加相关信息
      timeTrajectory.push_back(timeIntervalArray[i].first + k * settings().timeStep);
      stateTrajectory.push_back(state);
      
      // 重新定义timestep防止舍入误差影响正确
      const scalar_t timeStep = k < numSteps ? settings().timeStep : remainderTime;
      
      // 计算input和state
      initializerPtr_->compute(timeTrajectory.back(), stateTrajectory.back(), timeTrajectory.back() + timeStep, input, state);
      std::cout << "compute input in TODOs: " << input << std::endl;
      inputTrajectory.push_back(input); 
    } // end of k loop

    // if the remainder time is not very small push a new entry otherwise modify the last time
    if (remainderTime > 10.0 * numeric_traits::limitEpsilon<scalar_t>()) {
      timeTrajectory.push_back(timeIntervalArray[i].second);
      stateTrajectory.push_back(state);
      inputTrajectory.push_back(input);
    } else {
      timeTrajectory.back() = timeIntervalArray[i].second;
    }

    if (i < numEvents) {
      postEventIndices.push_back(stateTrajectory.size());
    }
  }
  return stateTrajectory.back();
}

/*
vector_t InitializerRollout::run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller,
                                 ModeSchedule& modeSchedule, scalar_array_t& timeTrajectory, size_array_t& postEventIndices,
                                 vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  std::cout << "TODO: check this function: Why inputTrajectory 2550?" << std::endl;
  if (initTime > finalTime) {
    throw std::runtime_error("[InitializerRollout::run] The initial time should be less-equal to the final time!");
  }

  // extract sub-systems
  const auto timeIntervalArray = findActiveModesTimeInterval(initTime, finalTime, modeSchedule.eventTimes);
  const auto numSubsystems = timeIntervalArray.size();
  const auto numEvents = numSubsystems - 1;
  const size_t maxNumSteps = (timeIntervalArray.back().second - timeIntervalArray.front().first) / settings().timeStep;

  // clearing the output trajectories
  timeTrajectory.clear();
  timeTrajectory.reserve(maxNumSteps + 2 * numSubsystems);
  stateTrajectory.clear();
  stateTrajectory.reserve(maxNumSteps + 2 * numSubsystems);
  inputTrajectory.clear();
  inputTrajectory.reserve(maxNumSteps + 2 * numSubsystems);
  postEventIndices.clear();
  postEventIndices.reserve(numEvents);

  vector_t input;
  vector_t state = initState;
  for (size_t i = 0; i < numSubsystems; i++) {
    const size_t numSteps = (timeIntervalArray[i].second - timeIntervalArray[i].first) / settings().timeStep;
    const scalar_t remainderTime = timeIntervalArray[i].second - (timeIntervalArray[i].first + numSteps * settings().timeStep);

    // take (numSteps + 1) steps from timeIntervalArray[i].first to (timeIntervalArray[i].second - remainderTime)
    for (size_t k = 0; k < numSteps + 1; k++) {
      timeTrajectory.push_back(timeIntervalArray[i].first + k * settings().timeStep);
      stateTrajectory.push_back(state);
      const scalar_t timeStep = k < numSteps ? settings().timeStep : remainderTime;
      initializerPtr_->compute(timeTrajectory.back(), stateTrajectory.back(), timeTrajectory.back() + timeStep, input, state);
      inputTrajectory.push_back(input);
    }  // end of k loop

    // if the remainder time is not very small push a new entry otherwise modify the last time
    if (remainderTime > 10.0 * numeric_traits::limitEpsilon<scalar_t>()) {
      timeTrajectory.push_back(timeIntervalArray[i].second);
      stateTrajectory.push_back(state);
      inputTrajectory.push_back(input);
    } else {
      timeTrajectory.back() = timeIntervalArray[i].second;
    }

    if (i < numEvents) {
      postEventIndices.push_back(stateTrajectory.size());
    }
  }  // end of i loop

  return stateTrajectory.back();
}
*/

}  // namespace ocs2
