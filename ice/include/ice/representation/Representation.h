/*
 * Representation.h
 *
 *  Created on: 15.07.2015
 *      Author: paspartout
 */

#ifndef ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATION_H_
#define ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATION_H_

#include "RepresentationType.h"
#include <string>
#include <vector>

namespace ice {

class Representation {
public:
  Representation();
  virtual ~Representation();

  RepresentationType type;

  int fromCSV(std::string reprStr, const char delim = ';');

  template<typename T>
  T get() {
    return static_cast<T>(data);
  }

private:
  void *convertDataStr(const char* dataStrs, RepresentationType type);

  void *data;
};

}  // namespace ice

#endif /* ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATION_H_ */
