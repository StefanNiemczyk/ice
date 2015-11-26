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

#include "ice/representation/Representation.h"

//Forward declaration
namespace ice
{
class ICEngine;
class OntologyInterface;
class GContainer;
} /* namespace ice */

namespace ice {

class GContainerFactory {
public:
  GContainerFactory();
  GContainerFactory(std::weak_ptr<ICEngine> engine);
  virtual ~GContainerFactory();

  void init();
  void cleanUp();

  void readFromOntology(std::shared_ptr<OntologyInterface> ontologyInterface);

  int fromCSVStrings(std::unique_ptr<std::vector<std::string>> lines);
  std::shared_ptr<Representation> getRepresentation(std::string representation);

  std::shared_ptr<GContainer> makeInstance(std::string name);
  std::shared_ptr<GContainer> makeInstance(std::shared_ptr<Representation> representation);

  void printReps();

private:
  std::weak_ptr<ICEngine> engine;
  std::shared_ptr<OntologyInterface> ontologyInterface;
  void printReps(std::shared_ptr<Representation> representation, int depth);
  std::shared_ptr<Representation> fromCSV(std::string reprStr, std::map<std::string, std::shared_ptr<Representation>> *tmpMap, const char delim = ';');
  BasicRepresentationType getBasicRep(std::string rep);
  std::shared_ptr<Representation> addOrGet(std::string name, std::map<std::string, std::shared_ptr<Representation>> *tmpMap);

  std::map<std::string, std::shared_ptr<Representation>> repMap;
  std::map<std::string, BasicRepresentationType> typeMap;
};

}  // namespace ice

#endif /* ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_ */
