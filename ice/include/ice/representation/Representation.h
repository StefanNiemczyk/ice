#ifndef REPRESENTATION_H
#define REPRESENTATION_H

#include <initializer_list>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ice/representation/split.h"

namespace ice
{

enum BasicRepresentationType
{
  UNSET, NONE, BOOL, BYTE, UNSIGNED_BYTE, SHORT, INT, LONG, UNSIGNED_SHORT, UNSIGNED_INT, UNSIGNED_LONG, FLOAT, DOUBLE,
  STRING,
};

struct Representation
{
  std::string name;

  std::vector<std::shared_ptr<Representation>> dimensions;
  std::vector<std::string> dimensionNames;
  BasicRepresentationType type;

  bool isBasic()
  {
    return (this->type != BasicRepresentationType::UNSET && this->type != BasicRepresentationType::NONE);
  }

  std::vector<int>* accessPath(std::string &dimensions)
  {
    auto d = split(dimensions.c_str(), ';');

    return this->accessPath(d.get());
  }

  std::vector<int>* accessPath(std::initializer_list<std::string> dimensions)
  {
    std::vector<std::string> d = dimensions;

    return this->accessPath(&d);
  }

  std::vector<int>* accessPath(std::vector<std::string> &dimensions)
  {
    return this->accessPath(&dimensions);
  }

  std::vector<int>* accessPath(std::vector<std::string> *dimensions)
  {
    std::vector<int>* path = new std::vector<int>();
    bool result = this->accessPath(dimensions, 0, path);

    if (result == false)
      return nullptr;

    return path;
  }

  std::string pathToString(std::vector<int>* path)
  {
    std::stringstream ss;

    this->pathToString(path, 0, ss);

    return ss.str();
  }

private:
  bool accessPath(std::vector<std::string> *dimensions, int index, std::vector<int> *path)
  {
    for (int i = 0; i < this->dimensionNames.size(); ++i)
    {
      if (this->dimensionNames.at(i) == dimensions->at(index))
      {

        auto dim = this->dimensions.at(i);
        path->push_back(i);

        if (index + 1 < dimensions->size())
          return dim->accessPath(dimensions, index + 1, path);
        else
          return true;
      }
    }

    return false;
  }

  void pathToString(std::vector<int>* path, int index, std::stringstream &ss)
  {
    if (index > 0)
      ss << " -> ";

    ss << this->dimensionNames.at(path->at(index));

    auto rep = this->dimensions.at(path->at(index));

    if (false == rep->isBasic() && index + 1 < path->size())
      rep->pathToString(path, index + 1, ss);
  }
};

}

#endif // REPRESENTATION_H
