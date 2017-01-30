#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"
#include <TBKnowledgeBase.h>
#include <GetTBKB.h>

namespace alica
{
    class DomainBehaviour : public BasicBehaviour
    {
    public:
		ice::TBKnowledgeBase *kb;
        DomainBehaviour(string name);
        virtual ~DomainBehaviour();
    };
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

