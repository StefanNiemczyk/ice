/*
 * Base.cpp
 *
 *  Created on: 22.10.2014
 *      Author: endy
 */

#include <iostream>
#include <cstdio>

#include <thread>
#include <chrono>
#include "ros/ros.h"
#include "Base.h"
#include "clock/AlicaROSClock.h"
#include "communication/AlicaRosCommunication.h"
#include "SigFault.h"

using namespace std;

namespace ice
{

	Base::Base(string roleSetName, string masterPlanName, string roleSetDir)
	{
		ae = new alica::AlicaEngine();
		bc = new alica::BehaviourCreator();
		cc = new alica::ConditionCreator();
		uc = new alica::UtilityFunctionCreator();
		crc = new alica::ConstraintCreator();

		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));

		ae->init(bc, cc, uc, crc, roleSetName, masterPlanName, roleSetDir, false);
	}

	void Base::start()
	{
		ae->start();
	}

	Base::~Base()
	{
		ae->shutdown();
		delete ae->getIAlicaClock();
		delete ae->getCommunicator();
		delete ae;
		delete cc;
		delete bc;
		delete uc;
		delete crc;
	}

} /* namespace ice */

void printUsage(const char *str)
{
	printf("Usage: %s -m \"Masterplan\" [-rd \"RoleSetDirectory\"] [-rset \"RoleSet\"]", str);
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		printUsage(argv[0]);
		return 0;
	}

	puts("Initialising ROS...");

	ros::init(argc, argv, supplementary::SystemConfig::getInstance()->getHostname()+"_Base");

	//This makes segfaults to exceptions
	segfaultdebug::init_segfault_exceptions();

	string masterplan = "";
	string rolesetdir = ".";
	string roleset = "";

	for (int i = 1; i < argc; i++)
	{
		if (string(argv[i]) == "-m" || string(argv[i]) == "-masterplan")
		{
			masterplan = argv[i + 1];
			i++;
		}

		if (string(argv[i]) == "-rd" || string(argv[i]) == "-rolesetdir")
		{
			rolesetdir = argv[i + 1];
			i++;
		}
		if (string(argv[i]) == "-r" || string(argv[i]) == "-roleset")
		{
			roleset = argv[i + 1];
			i++;
		}
	}
	if (masterplan.size() == 0 || rolesetdir.size() == 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	cout << "\tMasterplan is:       \"" << masterplan << "\"" << endl;
	cout << "\tRolset Directory is: \"" << rolesetdir << "\"" << endl;
	cout << "\tRolset is:           \"" << (roleset.empty() ? "Default" : roleset) << "\"" << endl;

	cout << "\nConstructing Base ..." << endl;
	ice::Base* base = new ice::Base(roleset, masterplan, rolesetdir);

	cout << "\nStarting Base ..." << endl;
	base->start();

	while (ros::ok())
	{
		std::chrono::milliseconds dura(500);
		std::this_thread::sleep_for(dura);
	}

	return 0;
}