/*
 * EngineState.h
 *
 *  Created on: Jun 11, 2014
 *      Author: sni
 */

#ifndef ENGINESTATE_H_
#define ENGINESTATE_H_

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
class Logger;
class TimeFactory;
} /* namespace ice */

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

//* CooperationState
/**
 * Enum of states of cooperation between two engines.
 *
 */
enum CooperationState
{
  UNKNOWN, //< 0 Unknown state
  RETRY_NEGOTIATION, //< 1 retry the negotiation process
  INFORMATION_MODEL_REQUESTED, //< 2 information model was requested
  INFORMATION_MODEL_SEND, //< 3 information model was send
  COOPERATION_REQUEST_SEND, //< 4 cooperation request was send
  COOPERATION_RESPONSE_SEND, //< 5 cooperation response was send
  COOPERATION_REFUSE_SEND, //< 6 cooperation refuse was send
  COOPERATION_ACCEPT_SEND, //< 7 cooperation accept was send
  NEGOTIATION_FINISHED_SEND, //< 8 negotiation finished was send
  STOP_COOPERATION_SEND, //< 9 stop cooperation was send
  COOPERATION, //< 10 active cooperation with this engine
  NO_COOPERATION //< 11 no cooperation with this engine
};

//* EngineState
/**
 * This class stores the meta information from other engines.
 *
 */
class EngineState
{
public:
  /*!
   * \brief This constructor initialize the object and sets the unique identifier.
   *
   * This constructor initialize the object and sets the unique identifier.
   *
   * /param engineId The identifier of the engine.
   * /param systemIri The ontology iri of this system.
   * /param engine The main engine.
   */
  EngineState(const identifier engineId, std::weak_ptr<ICEngine> engine);

  /*!
   * \brief This contructor initialize the object and sets the unique identifier.
   *
   * This contructor initialize the object and sets the unique identifier.
   *
   * /param systemIri The ontology iri of this system.
   * /param engine The main engine.
   */
  EngineState(std::string const systemIri, std::weak_ptr<ICEngine> engine);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~EngineState();

  /*!
   * \brief Returns the id of the engine.
   *
   * Returns the id of the engine.
   */
  const identifier getEngineId() const;

  /*!
   * \brief Sets the id of the engine.
   *
   * Sets the id of the engine.
   *
   * \param id The new identifier.
   */
  void setEngineId(const identifier id);

  /*!
   * \brief Returns the iri of this engine.
   *
   * Returns the iri of this engine.
   */
  const std::string getSystemIri() const;

  /*!
   * \brief Returns the short version of the iri of this engine.
   *
   * Returns the iri short version of the iri of this engine.
   */
  const std::string getSystemIriShort() const;

  /*!
   * \brief Returns the information model of the engine.
   *
   * Returns the information model of the engine.
   */
  const std::shared_ptr<InformationModel> getInformationModel() const;

  /*!
   * \brief Sets the information model of the engine.
   *
   * Sets the information model of the engine.
   *
   * \param informationModel The information model.
   */
  void setInformationModel(const std::shared_ptr<InformationModel> informationModel);

  /*!
   * \brief Returns the timestamp of the last activity of this engine.
   *
   * Returns the timestamp of the last activity of this engine.
   */
  time getTimeLastActivity() const;

  /*!
   * \brief Sets the timestamp of the last activity of this engine.
   *
   * Sets the timestamp of the last activity of this engine.
   *
   * @param timeLastActivity The timestamp of the last activity of this engine.
   */
  void setTimeLastActivity(time timeLastActivity);

  /*!
   * \brief Updates the last activity time.
   *
   * Updates the last activity time.
   */
  void updateTimeLastActivity();

  /*!
   * \brief Returns a pointer to the vector of intersections.
   *
   * Returns a pointer to the vector of intersections.
   */
  const std::vector<std::shared_ptr<IntersectionInformationModel>>* getIntersections() const;

  /*!
   * \brief Returns the cooperation state.
   *
   * Returns the cooperation state.
   */
  const CooperationState getCooperationState() const;

  /*!
   * \brief Sets the cooperation state.
   *
   * Sets the cooperation state.
   *
   * @param cooperationState The new cooperation state.
   */
  void setCooperationState(CooperationState cooperationState);

  /*!
   * \brief Returns the time stamp of the last cooperation state update.
   *
   * Returns the time stamp of the last cooperation state update.
   */
  const time getTimeLastStateUpdate() const;

  /*!
   * \brief Returns the cooperation request.
   *
   * Returns the cooperation request.
   */
  const std::shared_ptr<CooperationRequest> getCooperationRequest() const;

  /*!
   * \brief Sets the cooperation request.
   *
   * Sets the cooperation request.
   *
   * \param cooperationRequest The cooperation request.
   */
  void setCooperationRequest(const std::shared_ptr<CooperationRequest> cooperationRequest);

  /*!
   * \brief Returns the cooperation response.
   *
   * Returns the cooperation response.
   */
  const std::shared_ptr<CooperationResponse> getCooperationResponse() const;

  /*!
   * \brief Sets the cooperation response.
   *
   * Sets the cooperation response.
   *
   * \param cooperationRequest The cooperation response.
   */
  void setCooperationResponse(const std::shared_ptr<CooperationResponse> cooperationResponse);

  /*!
   * \brief Returns a pointer to the list of streams offered.
   *
   * Returns a pointer to the list of streams offered.
   */
  std::vector<std::shared_ptr<BaseInformationStream>>* getStreamsOffered();

  /*!
   * \brief Returns a pointer to the list of streams requested.
   *
   * Returns a pointer to the list of streams requested.
   */
  std::vector<std::shared_ptr<BaseInformationStream>>* getStreamsRequested();

  /*!
   * \brief Returns the retry counter.
   *
   * Returns the retry counter.
   */
  int getRetryCounter() const;

  /*!
   * \brief Sets the retry counter.
   *
   * Sets the retry counter.
   *
   * \param retryCounter The new retry counter value.
   */
  void setRetryCounter(int retryCounter);

  /*!
   * \brief Increase the retry counter by one.
   *
   * Increase the retry counter by one.
   */
  int increaseRetryCounter();

  /*!
   * \brief Resets the retry counter to zero.
   *
   * Resets the retry counter to zero.
   */
  void resetRetryCounter();

  std::shared_ptr<ASPElement> getASPElementByName(ASPElementType type, std::string const name);
  std::shared_ptr<ASPElement> getASPElementByName(std::string const name);

  void addASPElement(std::shared_ptr<ASPElement> node);

private:
  identifier engineId; /**< Unique identifier of the engine */
  const std::string systemIri; /**< The iri of this system */
  CooperationState cooperationState; /**< State of the cooperation between this engine and the main engine */
  std::weak_ptr<ICEngine> engine; /**< The main engine */
  std::vector<std::shared_ptr<ASPElement>> aspNodes; /**< Vector of asp nodes */
  std::vector<std::shared_ptr<ASPElement>> aspSourceNodes; /**< Vector of asp source nodes */
  std::vector<std::shared_ptr<ASPElement>> aspIro; /**< Vector of asp nodes */
  std::vector<std::shared_ptr<ASPElement>> aspRequiredStreams; /**< Vector of asp nodes */
  std::shared_ptr<InformationModel> informationModel; /**< The information model of this engine */
  time timeLastActivity; /**< Time stamp of the last activity of this engine */
  time timeLastStateUpdate; /**< Time stamp of the last cooperation state update */
  std::shared_ptr<TimeFactory> timeFactory; /**< Time factory */
  std::shared_ptr<CooperationRequest> cooperationRequest; /**< Cooperation request */
  std::shared_ptr<CooperationResponse> cooperationResponse; /**< The response to an cooperation request */
  std::vector<std::shared_ptr<IntersectionInformationModel>> intersections; /**< List of intersections in the information model of the main engine and the model from this engine */
  std::vector<std::shared_ptr<BaseInformationStream>> streamsOffered; /**< List of streams offered from the main to this engine */
  std::vector<std::shared_ptr<BaseInformationStream>> streamsRequested; /**< List of streams requested from the main */
  int retryCounter; /**< Counts the retrys of the current activity */
  Logger* _log; /**< Logger */
};

} /* namespace ice */

#endif /* ENGINESTATE_H_ */
