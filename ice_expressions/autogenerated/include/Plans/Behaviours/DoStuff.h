#ifndef DoStuff_H_
#define DoStuff_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1484927143799) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DoStuff : public DomainBehaviour
    {
    public:
        DoStuff();
        virtual ~DoStuff();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1484927143799) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1484927143799) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1484927143799) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DoStuff_H_ */
