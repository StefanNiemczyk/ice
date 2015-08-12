#ifndef REPRESENTATION_H
#define REPRESENTATION_H

#include <string>


namespace ice {

struct Representation {
  std::string name;
  Representation *parent = NULL;
};

}

#endif // REPRESENTATION_H
