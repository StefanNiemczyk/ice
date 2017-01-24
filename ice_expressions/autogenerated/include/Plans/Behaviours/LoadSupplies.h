#ifndef LoadSupplies_H_
#define LoadSupplies_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1484927595866) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class LoadSupplies : public DomainBehaviour
    {
    public:
        LoadSupplies();
        virtual ~LoadSupplies();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1484927595866) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1484927595866) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1484927595866) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* LoadSupplies_H_ */
