﻿#ifndef ARIS_DYNAMIC_PUMA_H_
#define ARIS_DYNAMIC_PUMA_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	// puma机器人构型： !!!! 注意，末端的位置姿态还不是完全符合，tbd !!!!
	//                                                   y    EE(tool0)
	//                                                   ^  x
	//                         x4           y5    x6     | /
	//                       \ - ********** o *** - ***  *----> z                                
	//                    d3 * | ... d4 ... | ... d5 ... |             
	//                  \  *
	//               --- *                              
	//               a3  *                                
	//                .  *                              
	//               --- o  y3
	//                .  *                               
	//               a2  *                                   
	//                .  *                               
	//  --- | *** a1 *** o                               
	//   .  z1           y2	
	//  d1
	//   .  z
	//   .  ^  y
	//   .  | /
	//  --- *----> x  
	//     O(wobj0)
	//
	struct ARIS_API PumaParam
	{
		// DH PARAM //
		double d1{ 0.0 };
		double a1{ 0.0 };
		double a2{ 0.0 };
		double d3{ 0.0 };
		double a3{ 0.0 };
		double d4{ 0.0 };

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 3> > mot_frc_vec;
	};
	auto ARIS_API createModelPuma(const PumaParam &param)->std::unique_ptr<aris::dynamic::Model>;

	class ARIS_API PumaInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual setPmEE(const double *ee_pm, const double *ext_axes)->void
		{
			dynamic_cast<GeneralMotion&>(model()->generalMotionPool()[0]).setMpm(ee_pm);
			if (ext_axes)
			{
				for (int i = 6; i < model()->motionPool().size(); ++i)
				{
					model()->motionPool()[i].setMp(ext_axes[i - 6]);
				}
			}
		}
		auto setWhichRoot(int root_of_0_to_7)->void;
		auto whichRoot()const->int;

		virtual ~PumaInverseKinematicSolver();
		explicit PumaInverseKinematicSolver();
		ARIS_DECLARE_BIG_FOUR(PumaInverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
