
#include "ice/model/aspModel/ASPSystem.h"

#include "ice/ICEngine.h"
#include "ClingWrapper.h"
#include "External.h"
#include "easylogging++.h"

namespace ice
{

ASPSystem::ASPSystem(std::string iri, std::weak_ptr<ICEngine> engine, std::shared_ptr<EngineState> state,
                     std::shared_ptr<supplementary::External> external) :
    iri(iri), engine(engine), state(state), systemExternal(external)
{
  this->_log = el::Loggers::getLogger("ASPSystem");
}

ASPSystem::~ASPSystem()
{
  //
}

std::shared_ptr<EngineState> ASPSystem::getEngineState()
{
  return this->state;
}

const std::string ASPSystem::getIri() const
{
  return this->iri;
}

const std::string ASPSystem::getShortIri() const
{
  int index = this->iri.find_last_of("#");
  std::string asp = (index != std::string::npos ? this->iri.substr(index + 1, this->iri.length()) : this->iri);
  std::transform(asp.begin(), asp.begin() + 1, asp.begin(), ::tolower);

  return asp;
}

std::shared_ptr<supplementary::External> ASPSystem::getSystemExternal()
{
  return this->systemExternal;
}

void ASPSystem::setEngineState(std::shared_ptr<EngineState> state)
{
  this->state = state;
}

std::shared_ptr<ASPElement> ASPSystem::getASPElementByName(ASPElementType type, std::string const name)
{
  switch (type)
  {
    case ASP_COMPUTATION_NODE:
      for (auto node : this->aspNodes)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_SOURCE_NODE:
      for (auto node : this->aspSourceNodes)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_IRO_NODE:
      for (auto node : this->aspIro)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_REQUIRED_STREAM:
      for (auto node : this->aspRequiredStreams)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_REQUIRED_MAP:
      for (auto node : this->aspRequiredMaps)
      {
        if (node->name == name)
          return node;
      }
      break;
  }

  return nullptr;
}

std::shared_ptr<ASPElement> ASPSystem::getASPElementByName(std::string const name)
{
  for (auto node : this->aspNodes)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspSourceNodes)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspIro)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspRequiredStreams)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspRequiredMaps)
  {
    if (node->name == name)
      return node;
  }

  return nullptr;
}

void ASPSystem::addASPElement(std::shared_ptr<ASPElement> node)
{
  switch (node->type)
  {
    case ASP_COMPUTATION_NODE:
      this->aspNodes.push_back(node);
      break;
    case ASP_SOURCE_NODE:
      this->aspSourceNodes.push_back(node);
      break;
    case ASP_IRO_NODE:
      this->aspIro.push_back(node);
      break;
    case ASP_REQUIRED_STREAM:
      this->aspRequiredStreams.push_back(node);
      break;
    case ASP_REQUIRED_MAP:
      this->aspRequiredMaps.push_back(node);
      break;
  }
}

void ASPSystem::updateExternals(bool activateRequired)
{
  bool active = false;

  if (this->getEngineState())
  {
    active = this->getEngineState()->isCooperationPossible();
  }
  else
  {
    auto e = this->engine.lock();
    auto coord = e->getCoordinator();
    this->setEngineState(coord->getEngineState(this->getIri()));

    if (this->getEngineState())
      active = this->getEngineState()->isCooperationPossible();
    else
      active = false;
  }

  this->systemExternal->assign(active);

  for (auto element : this->aspIro)
  {
    element->external->assign(active);
  }

  for (auto element : this->aspNodes)
  {
    element->external->assign(active);
  }

  for (auto element : this->aspSourceNodes)
  {
    element->external->assign(active);
  }

  for (auto element : this->aspRequiredStreams)
  {
    element->external->assign(active && activateRequired);
  }
}

} /* namespace ice */
