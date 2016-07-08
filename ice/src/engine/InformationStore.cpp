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

void InformationStore::addInformation(std::shared_ptr<InformationSpecification> specification,
                    std::shared_ptr<GContainer> information)
{
  auto info = std::make_shared<InformationElement<GContainer>>(specification, information);
  this->addInformation(info);
}

void InformationStore::addInformation(std::shared_ptr<InformationElement<GContainer>> &info)
{
  auto spec = info->getSpecification();
  this->information[spec] = info;

  for (auto &callback : this->infoCallbacks)
  {
    if (spec->checkRequest(callback.first))
    {
      callback.second(spec, info);
    }
  }
}

int InformationStore::getInformation(std::shared_ptr<InformationSpecification> request,
                   std::vector<std::shared_ptr<InformationElement<GContainer>>> &outInfo)
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

void InformationStore::registerCallback(std::shared_ptr<InformationSpecification> request,
                                        InfoCallback callback)
{
  this->infoCallbacks.push_back(std::make_pair(request, callback));
}

bool InformationStore::unregisterCallback(std::shared_ptr<InformationSpecification> request,
                                          InfoCallback callback)
{
  for (int i=0; i < this->infoCallbacks.size(); ++i)
  {
    auto &cb = this->infoCallbacks[i];

    if (cb.first == request
        && &cb.second == &callback)
    {
      this->infoCallbacks.erase(this->infoCallbacks.begin() + i);
      return true;
    }
  }

  return false;
}

} /* namespace ice */
