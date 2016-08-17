/*
 * InformationStore.cpp
 *
 *  Created on: May 6, 2016
 *      Author: sni
 */

#include <ice/information/InformationStore.h>

#include "ice/ICEngine.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/Transformation.h"
#include "ice/ontology/OntologyInterface.h"


namespace ice
{

InformationStore::InformationStore(std::weak_ptr<ICEngine> engine) : engine(engine)
{
  _log = el::Loggers::getLogger("InformationStore");
}

InformationStore::~InformationStore()
{
  //
}

bool InformationStore::init()
{
  auto e = this->engine.lock();
  this->gcontainerFactory = e->getGContainerFactory();
  this->ontology = e->getOntologyInterface();

  return true;
}

bool InformationStore::cleanUp()
{
  this->gcontainerFactory.reset();
  this->ontology.reset();
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
                   std::vector<std::shared_ptr<InformationElement<GContainer>>> &outInfo,
                   bool useTransfromation)
{
  int count = 0;

  for (auto &info : this->information)
  {
    if (request->getEntity() != "*" && request->getEntity() != info.first->getEntity())
    {
      continue;
    }
    if (request->getEntityType() != info.first->getEntityType())
    {
      continue;
    }
    if (request->getScope() != info.first->getScope())
    {
      continue;
    }
    if (request->getRepresentation() != info.first->getRepresentation())
    {
      if (useTransfromation && request->getRelatedEntity() == ""
          && info.first->getRelatedEntity() == "")
      {
        // check if transformation exists
        auto rep = this->gcontainerFactory->getTransformation(info.first->getRepresentation(),
                                                              request->getRepresentation());

        if (rep == nullptr)
          continue;

        std::shared_ptr<GContainer> input[1];
        input[1] = info.second->getInformation();
        auto transInfo = rep->transform(input);

        if (transInfo == nullptr)
          continue;

        auto spec = std::make_shared<InformationSpecification>(info.first->getEntity(),
                                                               info.first->getEntityType(),
                                                               info.first->getScope(),
                                                               request->getRepresentation(),
                                                               info.first->getRelatedEntity());
        auto element = std::make_shared<InformationElement<GContainer>>(spec, transInfo);
        outInfo.push_back(element);
        ++count;
      }

      continue;
    }
    if (request->getRelatedEntity() != info.first->getRelatedEntity())
    {
      continue;
    }

    outInfo.push_back(info.second);
    ++count;
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
