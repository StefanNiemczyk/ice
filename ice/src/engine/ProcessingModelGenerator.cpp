/*
 * ProcessingModel.cpp
 *
 *  Created on: Apr 23, 2015
 *      Author: sni
 */

#include <ice/model/ProcessingModelGenerator.h>

#include "ice/ontology/OntologyInterface.h"
#include "ice/ICEngine.h"

namespace ice
{

ProcessingModelGenerator::ProcessingModelGenerator(std::weak_ptr<ICEngine> engine)
{
  this->engine = engine;
}

ProcessingModelGenerator::~ProcessingModelGenerator()
{
  //
}

void ProcessingModelGenerator::init()
{
  auto en = this->engine.lock();
  this->nodeStore = en->getNodeStore();
  this->coordinator = en->getCoordinator();
  this->ontology = en->getOntologyInterface();
  this->initInternal();
}

void ProcessingModelGenerator::cleanUp()
{
  this->ontology.reset();
  this->coordinator.reset();
  this->nodeStore.reset();
  this->cleanUpInternal();
}

} /* namespace ice */
