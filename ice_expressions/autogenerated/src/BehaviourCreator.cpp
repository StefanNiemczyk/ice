using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/Behaviours/ProvideSupplies.h"

#include  "Plans/Behaviours/LoadSupplies.h"

#include  "Plans/Behaviours/FindSupplies.h"

#include  "Plans/Behaviours/FindVictim.h"

namespace alica
{

    BehaviourCreator::BehaviourCreator()
    {
    }

    BehaviourCreator::~BehaviourCreator()
    {
    }

    shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId)
    {
        switch (behaviourConfId)
        {

            case 1484927627341:

                return make_shared<ProvideSupplies>();
                break;

            case 1484927607502:

                return make_shared<LoadSupplies>();
                break;

            case 1484927591005:

                return make_shared<FindSupplies>();
                break;

            case 1484927617722:

                return make_shared<FindVictim>();
                break;

            default:
                cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
                throw new exception();
                break;
        }
    }
}
