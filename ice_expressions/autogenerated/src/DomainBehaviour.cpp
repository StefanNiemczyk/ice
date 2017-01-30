#include "DomainBehaviour.h"

namespace alica
{
    DomainBehaviour::DomainBehaviour(string name) :
            BasicBehaviour(name)
    {
			this->kb = ice::getTBKB("leonardo");
    }

    DomainBehaviour::~DomainBehaviour()
    {
    }
} /* namespace alica */
