#ifndef REPRESENTATION_INSTANCE_H
#define REPRESENTATION_INSTANCE_H

#include <string>

#include "RepresentationType.h"

namespace ice {

struct RepresentationInstance {

public:
  RepresentationType type;
  std::string name;

};

}

#endif // REPRESENTATION_TYPE_H
