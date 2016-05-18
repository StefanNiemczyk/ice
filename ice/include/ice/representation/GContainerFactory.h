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
struct DimensionDesc;
class GContainer;
class ICEngine;
class OntologyInterface;
struct TransDesc;
class Transformation;
} /* namespace ice */

//Forward declaration
namespace el
{
class Logger;
} /* namespace el */

namespace ice
{

class GContainerFactory : public std::enable_shared_from_this<GContainerFactory>
{
public:
  GContainerFactory();
  GContainerFactory(std::weak_ptr<ICEngine> engine);
  virtual ~GContainerFactory();

  void init();
  void cleanUp();

  void setOntologyInterface(std::shared_ptr<OntologyInterface> ontology);
  void readFromOntology();

  int fromCSVStrings(std::unique_ptr<std::vector<std::string>> lines);
  std::shared_ptr<Representation> getRepresentation(std::string representation);

  std::shared_ptr<GContainer> makeInstance(std::string name);
  std::shared_ptr<GContainer> makeInstance(std::shared_ptr<Representation> representation);

  std::shared_ptr<Transformation> fromXMLDesc(TransDesc* desc);
  void* convertStringToBasic(BasicRepresentationType type, std::string value);

  void printReps();

  bool addTransformation(std::string &name, std::shared_ptr<Transformation> &transformation);
  std::shared_ptr<Transformation> getTransformation(std::string &sourceRep, std::string &targetRep);
  std::shared_ptr<Transformation> getTransformation(std::vector<std::string> &sourceReps, std::string &targetRep);
  std::shared_ptr<Transformation> getTransformationTo(std::string &targetRep);
  std::shared_ptr<Transformation> getTransformationByName(std::string &name);

private:
  GContainer* makeGContainerInstance(std::shared_ptr<Representation> representation);
  bool extractOperations(std::shared_ptr<Transformation> transformation, std::shared_ptr<Representation> representation,
                         std::vector<DimensionDesc> &ops, std::vector<std::string> &path,
                         std::map<int, std::shared_ptr<Representation>> &reps);
  void printReps(std::shared_ptr<Representation> representation, int depth);
  std::shared_ptr<Representation> fromCSV(std::string reprStr,
                                          std::map<std::string, std::shared_ptr<Representation>> *tmpMap,
                                          const char delim = ';');
  BasicRepresentationType getBasicRep(std::string rep);
  std::shared_ptr<Representation> addOrGet(std::string name,
                                           std::map<std::string, std::shared_ptr<Representation>> *tmpMap);

private:
  el::Logger* _log;
  std::weak_ptr<ICEngine> engine;
  std::shared_ptr<OntologyInterface> ontologyInterface;
  std::map<std::string, std::shared_ptr<Representation>> repMap;
  std::map<std::string, BasicRepresentationType> typeMap;
  std::map<std::string, std::shared_ptr<Transformation>> transformations;
};

}  // namespace ice

#endif /* ICE_ICE_INCLUDE_ICE_REPRESENTATION_REPRESENTATIONFACTORY_H_ */
