
#ifndef ASP_SYSTEM_H_
#define ASP_SYSTEM_H_

#include <memory>
#include <vector>

#include "ClingWrapper.h"

#include "ice/Identifier.h"
#include "ice/Time.h"

// Forward declaration
namespace ice
{
class BaseInformationStream;
class CooperationRequest;
class CooperationResponse;
class ICEngine;
class InformationModel;
class IntersectionInformationModel;
class TimeFactory;
class EngineState;
} /* namespace ice */
namespace el
{
class Logger;
} /* namespace el */

namespace ice
{

//* ASPNodeState
/**
 * Enum of states an ASP node.
 *
 */
enum ASPElementState
{
  NEW_ELEMENT, ADDED_TO_ASP
};

//* ASPNodeType
/**
 * Enum of ASP node types
 *
 */
enum ASPElementType
{
  ASP_COMPUTATION_NODE, ASP_SOURCE_NODE, ASP_IRO_NODE, ASP_MAP_NODE, ASP_REQUIRED_STREAM, ASP_REQUIRED_MAP
};
const std::string ASPElementTypeNames[] {"ASP_COMPUTATION_NODE", "ASP_SOURCE_NODE", "ASP_IRO_NODE", "ASP_MAP_NODE",
                                         "ASP_REQUIRED_STREAM", "ASP_REQUIRED_MAP"};

//* ASPNode
/**
 * This struct contains the asp informations of a node.
 *
 */
struct ASPElement
{
  std::shared_ptr<supplementary::External> external;
  std::string aspString;
  std::string name;
  std::string className;
  std::map<std::string, std::string> config;
  ASPElementState state;
  ASPElementType type;
};

//* EngineState
/**
 * This class stores the meta information from other engines.
 *
 */
class ASPSystem
{
public:
  /*!
   * \brief This constructor initialize the object and sets the unique identifier.
   *
   * This constructor initialize the object and sets the unique identifier.
   *
   * /param engine The main engine.
   * /param state The engine state.
   */
  ASPSystem(std::weak_ptr<ICEngine> engine, std::shared_ptr<EngineState> state);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~ASPSystem();

  std::shared_ptr<EngineState> getEngineState();

  std::shared_ptr<ASPElement> getASPElementByName(ASPElementType type, std::string const name);

  std::shared_ptr<ASPElement> getASPElementByName(std::string const name);

  void addASPElement(std::shared_ptr<ASPElement> node);

private:
  std::weak_ptr<ICEngine> engine; /**< The main engine */
  std::shared_ptr<EngineState> state; /**< The engine state */
  std::vector<std::shared_ptr<ASPElement>> aspNodes; /**< Vector of asp nodes */
  std::vector<std::shared_ptr<ASPElement>> aspSourceNodes; /**< Vector of asp source nodes */
  std::vector<std::shared_ptr<ASPElement>> aspIro; /**< Vector of asp nodes */
  std::vector<std::shared_ptr<ASPElement>> aspRequiredStreams; /**< Vector of asp nodes */
  el::Logger* _log; /**< Logger */
};

} /* namespace ice */

#endif /* ENGINESTATE_H_ */
