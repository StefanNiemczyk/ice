/*
 * InformationStore.cpp
 *
 *  Created on: May 6, 2016
 *      Author: sni
 */

#include <ice/information/InformationStore.h>

#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/representation/GContainer.h"
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

void InformationStore::addInformation(std::shared_ptr<InformationSpecification> infoSpec,
                    std::shared_ptr<GElement> info)
{
  this->information[infoSpec] = info;
}

int InformationStore::getInformation(std::shared_ptr<InformationSpecification> request,
                   std::vector<std::shared_ptr<GElement>> &outInfo)
{
  int count = 0;

  for (auto &info : this->information)
  {
    if (info.first->checkRequest(request))
    {
      outInfo.push_back(info.second);
      ++count;
    }
  }

  return count;
}

} /* namespace ice */
