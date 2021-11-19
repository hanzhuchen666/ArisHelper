﻿#include <iostream>
#include <regex>
#include <charconv>

#include <aris.hpp>

#include <aris/dynamic/puma_5axis.hpp>

using namespace aris::dynamic;
using namespace aris::robot;

//系统传递函数H(s)=1/(ms)
void PIDcalOne(double m, double ts, double *KP)
{
	double T = ts / 3.0;
	KP[0] = m / T;
}
//系统传递函数H(s)=1/(ms+h)
void PIDcalTeo(double m, double h, double ts, double overshoot, double *KP, double *KI)
{
	double temp = std::log(overshoot);
	double kesi = 1 / sqrt(1 + aris::PI * aris::PI / temp / temp);
	double omega = 4 / kesi / ts;

	KI[0] = omega * omega * m;
	KP[0] = 2 * kesi * omega * m - h;
}
auto f(aris::dynamic::Model *m, double *A)
{
	auto &s = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool()[1]);
	s.kinPos();
	s.kinVel();
	s.cptGeneralInverseDynamicMatrix();
	s.cptJacobiWrtEE();

	// J_inv
	double U[36], tau[6], J_inv[36], tau2[6];
	aris::Size p[6], rank;
	s_householder_utp(6, 6, s.Jf(), U, tau, p, rank, 1e-7);
	s_householder_utp2pinv(6, 6, rank, U, tau, p, J_inv, tau2, 1e-7);

	// M = (M + I) * J_inv 
	double M[36], tem[36];
	s_mc(6, 6, s.M(), s.nM(), M, 6);
	for (int i = 0; i < 6; ++i)M[at(i, i, 6)] += m->motionPool()[i].frcCoe()[2];
	s_mm(6, 6, 6, M, J_inv, tem);
	s_mm(6, 6, 6, J_inv, T(6), tem, 6, A, 6);
}

auto createModelRokaeXB4_5(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
{
	aris::dynamic::PumaParam param;
	param.d1 = 0.3295;
	param.a1 = 0.04;
	param.a2 = 0.275;
	param.d3 = 0.0;
	param.a3 = 0.025;
	param.d4 = 0.28;

	param.tool0_pe[2] = 0.078;

	param.iv_vec =
	{
		{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
		{ 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
		{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
		{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
		{ 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
		{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
	};

	param.mot_frc_vec =
	{
		{ 9.34994758321915, 7.80825641041495, 0.00000000000000 },
		{ 11.64080253106441, 13.26518528472506, 3.55567932576820 },
		{ 4.77014054273075, 7.85644357492508, 0.34445460269183 },
		{ 3.63141668516122, 3.35461524886318, 0.14824771620542 },
		{ 2.58310846982020, 1.41963212641879, 0.04855267273770 },
		{ 1.78373986219597, 0.31920640440152, 0.03381545544099 },
	};

	return aris::dynamic::createModelPuma5(param);
}


int main(int argc, char *argv[])
{
	auto &cs = aris::server::ControlServer::instance();

	cs.resetMaster(aris::robot::rokae::xb4::createMaster().release());
	cs.resetController(aris::robot::rokae::xb4::createController().release());
	cs.resetModel(aris::robot::rokae::xb4::createModel().release());
	cs.resetPlanRoot(aris::robot::rokae::xb4::createPlanRoot().release());



	try
	{
		cs.init();
		cs.open();
		cs.start();


		// 读取数据 //
		double data[6];
		cs.controller().ftSensorPool()[0].getFtData(data);

		std::cout << aris::core::toXmlString(cs) << std::endl;




		cs.runCmdLine();
		//aris::core::toXmlFile(cs, "C:\\Users\\py033\\Desktop\\test.xml");
		//aris::core::fromXmlFile(cs, "C:\\Users\\py033\\Desktop\\test.xml");
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	

	

	return 0;
}