#ifndef FindSupplies_H_
#define FindSupplies_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1484927578806) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class FindSupplies : public DomainBehaviour
    {
    public:
        FindSupplies();
        virtual ~FindSupplies();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1484927578806) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1484927578806) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1484927578806) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* FindSupplies_H_ */
