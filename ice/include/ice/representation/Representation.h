#ifndef REPRESENTATION_H
#define REPRESENTATION_H

#include <string>


namespace ice {

struct Representation {
  ice::Representation *parent;
  std::string name;
};

}

#endif // REPRESENTATION_H
