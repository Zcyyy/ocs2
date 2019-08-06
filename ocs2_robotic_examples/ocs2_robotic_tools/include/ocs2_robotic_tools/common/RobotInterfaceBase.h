/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#pragma once

// C++
#include <string>
#include <iostream>
#include <cstdlib>

// OCS2
#include <ocs2_core/Dimensions.h>
#include <ocs2_slq/SLQ_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_core/misc/loadEigenMatrix.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_mpc/MPC_BASE.h>

namespace ocs2{

/**
 * This class implements an interface class to all the robotic examples.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
  */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RobotInterfaceBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		state_dim_ = STATE_DIM,
		input_dim_ = INPUT_DIM
	};

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	using size_array_t = typename DIMENSIONS::size_array_t;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t;
	using eigen_scalar_t = typename DIMENSIONS::eigen_scalar_t;
	using eigen_scalar_array_t = typename DIMENSIONS::eigen_scalar_array_t;
	using state_vector_t = typename DIMENSIONS::state_vector_t;
	using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
	using input_vector_t = typename DIMENSIONS::input_vector_t;
	using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
	using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
	using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;

	using dynamics_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;
	using dynamics_derivatives_t =  DerivativesBase<STATE_DIM, INPUT_DIM>;
	using cost_t = CostFunctionBase<STATE_DIM, INPUT_DIM>;
	using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM>;
	using mpc_t = MPC_BASE<STATE_DIM, INPUT_DIM>;

	/**
	 * Constructor
	 */
	RobotInterfaceBase() = default;

	/**
	 * Destructor
	 */
	~RobotInterfaceBase() = default;

	/**
	 * Gets the initial state
	 *
	 * @param initialState: Initial state
	 */
	void getInitialState(state_vector_t& initialState) const;

	/**
	 * Gets MPC settings.
	 *
	 * @return MPC settings
	 */
	MPC_Settings& mpcSettings();

	/**
	 * @brief getLogicRulesPtr
	 * @return Pointer to the internal logic rules
	 */
	virtual std::shared_ptr<HybridLogicRules> getLogicRulesPtr() { return nullptr; }

	/**
	 * @brief getMpcPtr
	 * @return pointer to internal mpc instance
	 */
	virtual mpc_t * getMpcPtr() = 0;

	/**
	 * @brief getDynamicsPtr
	 * @return a pointer to the interal system dynamics
	 */
	virtual dynamics_t * getDynamicsPtr() = 0;

	/**
	 * @brief getDynamicsDerivativesPtr
	 * @return a pointer to the internal system dynamics derivatives
	 */
	virtual dynamics_derivatives_t const * getDynamicsDerivativesPtr() = 0;

	/**
	 * @brief getCostPtr
	 * @return pointer to internal cost function
	 */
	virtual cost_t const * getCostPtr() = 0;

	/**
	 * @brief getConstraintPtr
	 * @return pointer to internal constraint object
	 */
	virtual constraint_t* getConstraintPtr() { return nullptr; }

	/**
	 * Setups all optimizers which you require.
	 *
	 * @param [in] taskFile: Task's file full path.
	 */
	virtual void setupOptimizer(const std::string& taskFile) = 0;

	/**
	 * Loads the settings from the task file.
	 *
	 * @param [in] taskFile: Task's file full path.
	 */
	virtual void loadSettings(const std::string& taskFile) = 0;

	/**
	 * Defines the time partitioning based on the task file values:
	 * "mpcTimeHorizon.timehorizon" and "mpcTimeHorizon.numPartitions".
	 * Time partitioning defines the time horizon and the number of data partitioning.
	 *
	 * @param [in] taskFile: Task's file full path.
	 * @param [out] timeHorizon: MPC time horizon.
	 * @param [out] numPartitions: The number of data partitioning.
	 * @param [out] partitioningTimes: The time partitioning.
	 * @param [in] verbose: Whether to print out the loaded variables.
	 */
	static void definePartitioningTimes(
			const std::string& taskFile,
			scalar_t& timeHorizon,
			size_t& numPartitions,
			scalar_array_t& partitioningTimes,
			bool verbose = false);

	/**
	 * Loads initial state from the task file.
	 *
	 * @param [in] taskFile: Task's file full path.
	 * @param [out] initialState: Initial state.
	 */
	static void loadInitialState(
			const std::string& taskFile,
			state_vector_t& initialState);

	/**
	 * Loads MPC time horizon and the number of data partitioning from the task file.
	 *
	 * @param [in] taskFile: Task's file full path.
	 * @param [out] timeHorizon: MPC time horizon.
	 * @param [out] numPartitions: The number of data partitioning.
	 * @param [in] verbose: Whether to print out the loaded variables.
	 */
	static void loadMpcTimeHorizon(
			const std::string& taskFile,
			scalar_t& timeHorizon,
			size_t& numPartitions,
			bool verbose = false);


protected:
	/**************
	 * Variables
	 **************/
	MPC_Settings mpcSettings_;

	state_vector_t initialState_;
	input_vector_t initialInput_;

};

} // namespace ocs2

#include "implementation/RobotInterfaceBase.h"
