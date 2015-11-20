#ifndef REPRESENTATION_H
#define REPRESENTATION_H

#include <map>
#include <string>


namespace ice {

struct Representation {
  std::string name;

  Representation *parent;
  std::vector<Representation*> subclasses;
  std::map<std::string, int> mapping;
};

}

#endif // REPRESENTATION_H
