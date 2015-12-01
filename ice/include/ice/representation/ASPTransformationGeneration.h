/*
 * ASPTransformationGeneration.h
 *
 *  Created on: Dec 1, 2015
 *      Author: sni
 */

#ifndef ASPTRANSFORMATIONGENERATION_H_
#define ASPTRANSFORMATIONGENERATION_H_

#include <memory>
#include <tuple>
#include <vector>
#include <gringo/value.hh>

#include "ClingWrapper.h"

#include "easylogging++.h"

//Forward declarations
namespace ice
{
class GContainerFactory;
class ICEngine;
class OntologyInterface;
class Transformation;
} /* namespace ice */

//Forward declarations
namespace supplementary
{
class ClingWrapper;
} /* namespace ice */

namespace ice
{

class ASPTransformationGeneration
{
public:
  ASPTransformationGeneration();
  ASPTransformationGeneration(std::weak_ptr<ICEngine> engine);
  virtual ~ASPTransformationGeneration();

  void init();
  void cleanUp();
  void readInfoStructureFromOntology();
  void extractTransformations();

  void setOntology(std::shared_ptr<OntologyInterface> ontology);
  void setGContainerFactory(std::shared_ptr<GContainerFactory> factory);

private:
  void extractDeviation(supplementary::ClingWrapper &asp, std::vector<std::tuple<std::string, std::string>> &deviations,
                        Gringo::Value &simRep, std::string type);

private:
  std::weak_ptr<ICEngine> engine;
  std::shared_ptr<OntologyInterface> ontology;
  std::shared_ptr<GContainerFactory> containerFactory;
  std::vector<std::string> entities; /**< The entites as strings */
  bool groundingDirty;
  el::Logger* _log; /**< Logger */
};

} /* namespace ice */

#endif /* ASPTRANSFORMATIONGENERATION_H_ */
