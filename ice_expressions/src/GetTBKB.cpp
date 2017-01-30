#include <GetTBKB.h>

ice::TBKnowledgeBase *ice::getTBKB(std::string robotName) {
	static ice::TBKnowledgeBase tbkb(robotName);
	return &tbkb;
}

