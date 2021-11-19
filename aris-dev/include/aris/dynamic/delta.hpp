﻿#ifndef ARIS_DYNAMIC_DELTA_H_
#define ARIS_DYNAMIC_DELTA_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic {
	//---------------------------------------------------------------------------------------------
	// 包含4个自由度。共4个杆件  link1~4
	// 尺寸示意：
	//                   上平台 
	//                      base(长度a，平台中心到关节的距离)
	//                    *-------o      -> R1
	//                             \     -> link1 (长度b)
	//                              \o   -> S2
	//                              /|   -> link1 (长度c)
	//                             / o   -> S3
	//                            / /
	//               S4 <-       o /     -> link2&3 (长度d)
	//           转轴Re <-  |----|/         Re 连接并联末端 up 与最终的末端 ee，从而变成 4 自由度
	//               S5 <-       o
	//                       ee(长度e)
	//---------------------------------------------------------------------------------------------
	// 零位：
	//                 (0,0,0)           o
	//                    |-------o------|       ->  x方向
	//                                 / o
	//                                / /
	//                               / /
	//                              o /
	//                    |---------|/
	//                              o
	//---------------------------------------------------------------------------------------------
	// 俯视图：
	//                       y
	//                       ^
	//               支联2   |
	//                    o
	//                     \
	//                      \
	//                       *------o      -> x   支联1
	//                      /
	//                     /
	//                    o
	//               支联3
	//


	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	// 简化版的delta参数
	// 适用于：三根支联对称分布
	struct ARIS_API DeltaParam {
		// DH PARAM //
		double a{ 0.0 };
		double b{ 0.0 };
		double c{ 0.0 };
		double d{ 0.0 };
		double e{ 0.0 };

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// inertia vector, size must be 4
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 4
		std::vector<std::array<double, 3> > mot_frc_vec;
	};

	// 详细的delta参数
	// 适用于每根支联尺寸不一样
	struct ARIS_API DeltaFullParam {
		// DH PARAM //
		double a1{ 0.0 }, b1{ 0.0 }, c1{ 0.0 }, d1{ 0.0 }, e1{ 0.0 }, theta1{ 0.0 };
		double a2{ 0.0 }, b2{ 0.0 }, c2{ 0.0 }, d2{ 0.0 }, e2{ 0.0 }, theta2{ aris::PI * 2 / 3 };
		double a3{ 0.0 }, b3{ 0.0 }, c3{ 0.0 }, d3{ 0.0 }, e3{ 0.0 }, theta3{ -aris::PI * 2 / 3 };

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// inertia vector, size must be 4
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 4
		std::vector<std::array<double, 3> > mot_frc_vec;
	};

	auto ARIS_API createModelDelta(const DeltaParam &param)->std::unique_ptr<aris::dynamic::Model>;
	auto ARIS_API createModelDelta(const DeltaFullParam &param)->std::unique_ptr<aris::dynamic::Model>;

	///
	/// @}
}

#endif
