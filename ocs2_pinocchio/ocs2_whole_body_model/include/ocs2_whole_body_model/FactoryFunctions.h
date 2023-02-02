/*************************************************************************
*
*              Author: {Chongyang Zhang}
*                Mail: {zcy@stu.hit.edu.cn}
*            FileName: FactoryFunctions.h
*
*          Created On: 2022年12月14日 星期三 16时10分32秒
*     Licensed under The {GPL} License [see LICENSE for details]
*
************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_whole_body_model/WholeBodyModelInfo.h"

namespace ocs2 {
namespace wholebody_model {

/**
 * Create a WholeBodyModel PinocchioInterface from a URDF.
 * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
 */
PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath);

/**
 * Create a WholeBodyModel PinocchioInterface from a URDF.
 * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
 * @param [in] jointNames: Any joint that is not listed in jointNames (a.k.a the extraneous joints) will be removed from the urdf.
 */
PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath, const std::vector<std::string>& jointNames);

/**
 * Create a scalar-typed WholeBodyModelInfo.
 * @param [in] interface: Pinocchio interface
 * @param [in] type: Type of template model (SRBD or FRBD)
 * @param [in] nominalJointAngles: nominal joint angles used in the SRBD model.
 * @param [in] threeDofContactNames: Names of end-effectors with 3 DoF contacts (force)
 * @param [in] sixDofContactNames: Names of end-effectors with 6 DoF contacts (force + torque)
 * @return CentroidalModelInfo
 */
WholeBodyModelInfo createWholeBodyModelInfo(const PinocchioInterface& interface, const WholeBodyModelType& type,
                                              const vector_t& nominalJointAngles, const std::vector<std::string>& threeDofContactNames,
                                              const std::vector<std::string>& sixDofContactNames);

/** Load WholeBodyModelType for a config file */
WholeBodyModelType loadWholeBodyType(const std::string& configFilePath, const std::string& fieldName = "WholeBodyModelType");

/** Load default joint state for a config file */
vector_t loadDefaultJointState(size_t numJointState, const std::string& configFilePath, const std::string& fieldName = "defaultJointState");

}
}

