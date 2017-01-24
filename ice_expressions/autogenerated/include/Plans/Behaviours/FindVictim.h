#ifndef FindVictim_H_
#define FindVictim_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1484927612604) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class FindVictim : public DomainBehaviour
    {
    public:
        FindVictim();
        virtual ~FindVictim();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1484927612604) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1484927612604) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1484927612604) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* FindVictim_H_ */
