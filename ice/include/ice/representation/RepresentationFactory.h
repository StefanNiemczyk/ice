/*
 * Representation.h
 *
 *  Created on: 15.07.2015
 *      Author: paspartout
 */

#ifndef ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_
#define ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_

#include <string>
#include <vector>
#include <memory>
#include <map>

#include "Representation.h"
#include "RepresentationInstance.h"

namespace ice {

class RepresentationFactory {
public:
  RepresentationFactory();
  virtual ~RepresentationFactory();

  int fromCSVStrings(std::vector<std::string> lines);
  std::shared_ptr<Representation> getRepresentation(std::string representation);

  std::shared_ptr<RepresentationInstance> makeInstance(std::string name);
  std::shared_ptr<RepresentationInstance> makeInstance(std::shared_ptr<Representation> representation);

  void printReps();

private:
  void printReps(std::shared_ptr<Representation> representation, int depth);
  std::shared_ptr<Representation> fromCSV(std::string reprStr, std::map<std::string, std::shared_ptr<Representation>> *tmpMap, const char delim = ';');
  BasicRepresentationType getBasicRep(std::string rep);
  std::shared_ptr<Representation> addOrGet(std::string name, std::map<std::string, std::shared_ptr<Representation>> *tmpMap);

  std::map<std::string, std::shared_ptr<Representation>> repMap;
  std::map<std::string, BasicRepresentationType> typeMap;
};

}  // namespace ice

#endif /* ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_ */
