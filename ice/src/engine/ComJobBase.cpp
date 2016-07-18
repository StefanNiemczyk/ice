
#include "ice/communication/jobs/ComJobBase.h"

#include "ice/communication/jobs/IdentityRequest.h"

namespace ice {

  std::map<uint8_t, jobCreator> ComJobRegistry::jobs;
}
