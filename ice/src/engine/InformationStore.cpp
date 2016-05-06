/*
 * InformationStore.cpp
 *
 *  Created on: May 6, 2016
 *      Author: sni
 */

#include <ice/information/InformationStore.h>

#include "ice/ontology/OntologyInterface.h"

namespace ice
{

InformationStore::InformationStore(std::shared_ptr<OntologyInterface> ontology)
{
  _log = el::Loggers::getLogger("InformationStore");
  this->ontology = ontology;

}

InformationStore::~InformationStore()
{
  // TODO Auto-generated destructor stub
}

bool InformationStore::init()
{
  return true;
}

bool InformationStore::cleanUp()
{
  return true;
}

} /* namespace ice */
