/*
 * Base.h
 *
 *  Created on: 22.10.2014
 *      Author: endy
 */

#ifndef ICE_ICE_BASE_H_
#define ICE_ICE_BASE_H_

#include <iostream>

#include "engine/AlicaEngine.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "UtilityFunctionCreator.h"
#include "ConstraintCreator.h"

using namespace std;

namespace ice
{

	class Base
	{
	public:
		Base(string roleSetName, string masterPlanName, string roleSetDir);
		virtual ~Base();

		void start();

		alica::AlicaEngine* ae;
		alica::BehaviourCreator* bc;
		alica::ConditionCreator* cc;
		alica::UtilityFunctionCreator* uc;
		alica::ConstraintCreator* crc;
	};

} /* namespace ttb */

#endif /* ICE_ICE_BASE_H_ */
