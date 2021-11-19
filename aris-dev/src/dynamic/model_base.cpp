﻿#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <limits>
#include <type_traits>
#include <ios>

#include "aris/core/core.hpp"
#include "aris/dynamic/model_base.hpp"

namespace aris::dynamic
{
	ARIS_REGISTRATION
	{
		//typedef Environment&(Model::*EnvironmentFunc)();
		//typedef aris::core::PointerArray<Variable, Element> &(Model::*VarablePoolFunc)();
		//typedef aris::core::PointerArray<Part, Element> &(Model::*PartPoolFunc)();
		//typedef aris::core::PointerArray<Joint, Element> &(Model::*JointPoolFunc)();
		//typedef aris::core::PointerArray<Motion, Element> &(Model::*MotionPoolFunc)();
		//typedef aris::core::PointerArray<GeneralMotion, Element> &(Model::*GeneralMotionPoolFunc)();
		//typedef aris::core::PointerArray<Force, Element> &(Model::*ForcePoolFunc)();
		//typedef aris::core::PointerArray<Solver, Element> &(Model::*SolverPoolFunc)();
		//typedef aris::core::PointerArray<Simulator, Element> &(Model::*SimulatorPoolFunc)();
		//typedef aris::core::PointerArray<SimResult, Element> &(Model::*SimResultPoolFunc)();
		//typedef aris::core::PointerArray<Calibrator, Element> &(Model::*CalibratorPoolFunc)();

		//aris::core::class_<aris::core::PointerArray<Variable, Element>>("VariablePoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<Part, Element>>("PartPoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<Joint, Element>>("JointPoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<Motion, Element>>("MotionPoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<GeneralMotion, Element>>("GeneralMotionPoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<Force, Element>>("ForcePoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<Solver, Element>>("SolverPoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<Simulator, Element>>("SimulatorPoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<SimResult, Element>>("SimResultPoolElement")
		//	.asRefArray()
		//	;
		//aris::core::class_<aris::core::PointerArray<Calibrator, Element>>("CalibratorPoolElement")
		//	.asRefArray()
		//	;

		//aris::core::class_<Model>("Model")
		//	.prop("time", &Model::setTime, &Model::time)
		//	.prop("environment", EnvironmentFunc(&Model::environment))
		//	.prop("variable_pool", &Model::resetVariablePool, VarablePoolFunc(&Model::variablePool))
		//	.prop("part_pool", &Model::resetPartPool,  PartPoolFunc(&Model::partPool))
		//	.prop("motion_pool", &Model::resetMotionPool, MotionPoolFunc(&Model::motionPool))
		//	.prop("joint_pool", &Model::resetJointPool, JointPoolFunc(&Model::jointPool))
		//	.prop("general_motion_pool", &Model::resetGeneralMotionPool, GeneralMotionPoolFunc(&Model::generalMotionPool))
		//	.prop("force_pool", &Model::resetForcePool, ForcePoolFunc(&Model::forcePool))
		//	.prop("solver_pool", &Model::resetSolverPool, SolverPoolFunc(&Model::solverPool))
		//	//.prop<SimulatorPoolFunc>("simulator_pool", &Model::simulatorPool)
		//	//.prop<SimResultPoolFunc>("sim_result_pool", &Model::simResultPool)
		//	//.prop<CalibratorPoolFunc>("calibrator_pool", &Model::calibratorPool)
		//	;
	}
}
