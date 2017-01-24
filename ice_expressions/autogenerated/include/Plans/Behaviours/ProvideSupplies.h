#ifndef ProvideSupplies_H_
#define ProvideSupplies_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1484927620984) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class ProvideSupplies : public DomainBehaviour
    {
    public:
        ProvideSupplies();
        virtual ~ProvideSupplies();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1484927620984) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1484927620984) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1484927620984) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* ProvideSupplies_H_ */
