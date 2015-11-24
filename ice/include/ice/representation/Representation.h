#ifndef REPRESENTATION_H
#define REPRESENTATION_H

#include <map>
#include <string>


namespace ice {

enum BasicRepresentationType {
  UNSET,
  NONE,
  BOOL,
  BYTE,
  UNSIGNED_BYTE,
  SHORT,
  INT,
  LONG,
  UNSIGNED_SHORT,
  UNSIGNED_INT,
  UNSIGNED_LONG,
  FLOAT,
  DOUBLE,
  STRING,
};

struct Representation {
  std::string name;

  std::vector<Representation*> subclasses;
  std::map<std::string, int> mapping;
  BasicRepresentationType type;

  bool isBasic()
  {
    return (this->type != BasicRepresentationType::UNSET && this->type != BasicRepresentationType::NONE);
  }
};

}

#endif // REPRESENTATION_H
