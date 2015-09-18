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
//  this->running = false;
}

ProcessingModelGenerator::~ProcessingModelGenerator()
{
  // stop worker thread

}

void ProcessingModelGenerator::init()
{
  // create worker thread
//  this->running = true;
  auto en = this->engine.lock();
  this->nodeStore = en->getNodeStore();
  this->coordinator = en->getCoordinator();
  this->ontology = en->getOntologyInterface();
//  this->worker = std::thread(&ProcessingModelGenerator::workerTask, this);

  this->initInternal();
}

void ProcessingModelGenerator::cleanUp()
{
//  {
//    std::lock_guard<std::mutex> guard(mtx_);
//
//    if (this->running == false)
//      return;
//
//    this->running = false;
//  }
//
//  this->cv.notify_all();
//  this->worker.join();

  this->ontology.reset();

  this->cleanUpInternal();
}

} /* namespace ice */
