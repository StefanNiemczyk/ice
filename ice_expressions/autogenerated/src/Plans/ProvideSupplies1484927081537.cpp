#include "Plans/ProvideSupplies1484927081537.h"
using namespace alica;
/*PROTECTED REGION ID(eph1484927081537) ENABLED START*/ //Add additional using directives here
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
    //Plan:ProvideSupplies

    /* generated comment
     
     Task: DefaultTask  -> EntryPoint-ID: 1484927081539

     */
    shared_ptr<UtilityFunction> UtilityFunction1484927081537::getUtilityFunction(Plan* plan)
    {
        /*PROTECTED REGION ID(1484927081537) ENABLED START*/

        shared_ptr < UtilityFunction > defaultFunction = make_shared < DefaultUtilityFunction > (plan);
        return defaultFunction;

        /*PROTECTED REGION END*/
    }

    //State: Success in Plan: ProvideSupplies

    //State: FindSupplies in Plan: ProvideSupplies

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): FindSuppliesDefault, (PlanID): 1484927591005 
     *
     * Tasks: 
     *   - DefaultTask (1414681164704) (Entrypoint: 1484927081539)
     *
     * States:
     *   - Success (1484927117903)
     *   - FindSupplies (1484927333656)
     *   - LoadSupplies (1484927341401)
     *   - FindVictim (1484927352148)
     *   - ProvideSupplies (1484927371709)
     *
     * Vars:
     */
    bool TransitionCondition1484927401848::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1484927398698) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: LoadSupplies in Plan: ProvideSupplies

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): LoadSuppliesDefault, (PlanID): 1484927607502 
     *
     * Tasks: 
     *   - DefaultTask (1414681164704) (Entrypoint: 1484927081539)
     *
     * States:
     *   - Success (1484927117903)
     *   - FindSupplies (1484927333656)
     *   - LoadSupplies (1484927341401)
     *   - FindVictim (1484927352148)
     *   - ProvideSupplies (1484927371709)
     *
     * Vars:
     */
    bool TransitionCondition1484927426260::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1484927424896) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: FindVictim in Plan: ProvideSupplies

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): FindVictimDefault, (PlanID): 1484927617722 
     *
     * Tasks: 
     *   - DefaultTask (1414681164704) (Entrypoint: 1484927081539)
     *
     * States:
     *   - Success (1484927117903)
     *   - FindSupplies (1484927333656)
     *   - LoadSupplies (1484927341401)
     *   - FindVictim (1484927352148)
     *   - ProvideSupplies (1484927371709)
     *
     * Vars:
     */
    bool TransitionCondition1484927433844::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1484927432879) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    //State: ProvideSupplies in Plan: ProvideSupplies

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): ProvideSuppliesDefault, (PlanID): 1484927627341 
     *
     * Tasks: 
     *   - DefaultTask (1414681164704) (Entrypoint: 1484927081539)
     *
     * States:
     *   - Success (1484927117903)
     *   - FindSupplies (1484927333656)
     *   - LoadSupplies (1484927341401)
     *   - FindVictim (1484927352148)
     *   - ProvideSupplies (1484927371709)
     *
     * Vars:
     */
    bool TransitionCondition1484927391389::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1484927389371) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

    /*
     *		
     * Transition:
     *   - Name: MISSING_NAME, ConditionString: , Comment :  
     *
     * Plans in State: 				
     *   - Plan - (Name): ProvideSuppliesDefault, (PlanID): 1484927627341 
     *
     * Tasks: 
     *   - DefaultTask (1414681164704) (Entrypoint: 1484927081539)
     *
     * States:
     *   - Success (1484927117903)
     *   - FindSupplies (1484927333656)
     *   - LoadSupplies (1484927341401)
     *   - FindVictim (1484927352148)
     *   - ProvideSupplies (1484927371709)
     *
     * Vars:
     */
    bool TransitionCondition1484927394940::evaluate(shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(1484927392769) ENABLED START*/
        return false;
        /*PROTECTED REGION END*/

    }

}
