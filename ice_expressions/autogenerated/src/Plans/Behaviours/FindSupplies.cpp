#include "Plans/Behaviours/FindSupplies.h"
#include <container/PositionOrientation3D.h>

/*PROTECTED REGION ID(inccpp1484927578806) ENABLED START*/ //Add additional includes here
#include <iostream>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1484927578806) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    FindSupplies::FindSupplies() :
            DomainBehaviour("FindSupplies")
    {
        /*PROTECTED REGION ID(con1484927578806) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    FindSupplies::~FindSupplies()
    {
        /*PROTECTED REGION ID(dcon1484927578806) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void FindSupplies::run(void* msg)
    {
        /*PROTECTED REGION ID(run1484927578806) ENABLED START*/ //Add additional options here
 		auto pos = this->kb->positionOwn->getLast()->getInformation();
		printf("My position is: \n");
		pos->print(1);

		// TODO: Check if at landmark

       /*PROTECTED REGION END*/
    }
    void FindSupplies::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1484927578806) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1484927578806) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
