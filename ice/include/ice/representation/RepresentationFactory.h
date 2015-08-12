/*
 * Representation.h
 *
 *  Created on: 15.07.2015
 *      Author: paspartout
 */

#ifndef ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_
#define ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_

#include <ice/representation/Representation.h>
#include <string>
#include <vector>
#include <memory>

#include "Representation.h"

namespace ice {

class RepresentationFactory {
public:
  RepresentationFactory();
  virtual ~RepresentationFactory();

  Representation fromCSV(std::string reprStr, const char delim = ';');
  std::shared_ptr<std::vector<Representation>> fromCSVStrings(
      std::vector<std::string> lines);

  std::shared_ptr<std::vector<Representation>> reps;
};

}  // namespace ice

#endif /* ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_ */
