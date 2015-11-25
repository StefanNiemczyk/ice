#ifndef REPRESENTATION_H
#define REPRESENTATION_H

#include <map>
#include <memory>
#include <string>
#include <vector>

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

  std::vector<std::shared_ptr<Representation>> dimensions;
  std::vector<std::string> dimensionNames;
  BasicRepresentationType type;

  bool isBasic()
  {
    return (this->type != BasicRepresentationType::UNSET && this->type != BasicRepresentationType::NONE);
  }

  int* accessPath(std::vector<std::string> dimensions)
  {
    std::vector<int> path;
    bool result = this->accessPath(dimensions, 0, path);

    if (result == false)
      return nullptr;

    int* arr = new int[path.size()];
    std::copy(path.begin(), path.end(), arr);

    return arr;
  }

private:
  bool accessPath(std::vector<std::string> dimensions, int index, std::vector<int> &path)
  {
    for (int i=0; i < this->dimensionNames.size(); ++i)
    {

      if (this->dimensionNames.at(i) == dimensions[index])
      {
        auto dim = this->dimensions.at(i);
        path.push_back(i);

        if (dim->isBasic())
          return true;
        else
          return dim->accessPath(dimensions, index+1, path);
      }
    }

    return false;
  }
};

}

#endif // REPRESENTATION_H
