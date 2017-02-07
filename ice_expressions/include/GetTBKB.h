#ifndef GETTBKB_H_
#define GETTBKB_H_ 

#include <string>
#include <TBKnowledgeBase.h>

namespace ice {

std::shared_ptr<TBKnowledgeBase> getTBKB(std::string robotName);

}

#endif // GETTBKB_H_   

