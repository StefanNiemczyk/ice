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

//  std::shared_ptr<std::vector<Representation*>> getRepVec();
//  std::shared_ptr<std::map<std::string, Representation*>> getRepMap();

  Representation* getRepresentation(std::string representation);

  RepresentationInstance *makeInstance(std::string name);
  RepresentationInstance *makeInstance(Representation* representation);

  void printReps();

private:
  void printReps(Representation* representation, int depth);
  Representation* fromCSV(std::string reprStr, std::map<std::string, Representation*> *tmpMap, const char delim = ';');
  BasicRepresentationType getBasicRep(std::string rep);
  Representation* addOrGet(std::string name, std::map<std::string, Representation*> *tmpMap);

  std::map<std::string, Representation*> repMap;
  std::map<std::string, BasicRepresentationType> typeMap;
};

}  // namespace ice

#endif /* ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_ */
