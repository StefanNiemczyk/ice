
#include <ice/model/aspModel/ASPSystem.h>
#include <ice/ICEngine.h>
#include "easylogging++.h"

namespace ice
{

ASPSystem::ASPSystem(std::weak_ptr<ICEngine> engine, std::shared_ptr<EngineState> state,
                     std::shared_ptr<supplementary::External> external) :
    engine(engine), state(state), systemExternal(external)
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
  }
}

} /* namespace ice */
