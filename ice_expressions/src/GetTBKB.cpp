#include <GetTBKB.h>
#include <memory>
#include <thread>

std::shared_ptr<ice::TBKnowledgeBase> ice::getTBKB(std::string robotName) {
	static std::shared_ptr<ice::TBKnowledgeBase> tbkb;
	static bool initialised = false;

	if (!initialised) {
		printf("Initialising TBKnowledgeBase...\n");
		tbkb = std::make_shared<ice::TBKnowledgeBase>(robotName);
		tbkb->init();
		tbkb->start();

		// wait some time to enable the engines to find each other
		std::this_thread::sleep_for(std::chrono::milliseconds {100});
		initialised = true;	
	}

	return tbkb;
}

