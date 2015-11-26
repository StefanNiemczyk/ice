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

#include "ice/model/ProcessingModel.h"
#include "ice/Identifier.h"
#include "ice/Time.h"

// Forward declaration
namespace ice
{
class BaseInformationStream;
class Configuration;
class Communication;
class ICEngine;
class Node;
class TimeFactory;
class EngineState;
} /* namespace ice */
namespace el
{
class Logger;
} /* namespace el */

namespace ice
{

//* EngineConnection
/**
 * Containter that stores information about a connection to another engine.
 *
 */
struct EngineConnection
{
  std::shared_ptr<EngineState> connectionToEngine;
  int delay;
};

//* CooperationState
/**
 * Enum of states of cooperation between two engines.
 *
 */
enum CooperationState
{
  UNKNOWN,                      //< 0 Unknown state
  SYSTEM_SPEC_REQUESTED,        //< 1 Request the specification of another engine
  SYSTEM_SPEC_SEND,             //< 2 System specification send
  SYSTEM_SPEC_RECEIVCED,        //< 3 Received a system specification
  SUB_MODEL_RECEIVED,           //< 4 Received a sub model description
  SUB_MODEL_RESPONSE_RECEIVED,  //< 5 Received a response for a sub model

  //-----------------------------------------------------------------------------------
  RETRY_NEGOTIATION,            //< 1 retry the negotiation process
  INFORMATION_MODEL_REQUESTED,  //< 2 information model was requested
  INFORMATION_MODEL_SEND,       //< 3 information model was send
  COOPERATION_REQUEST_SEND,     //< 4 cooperation request was send
  COOPERATION_RESPONSE_SEND,    //< 5 cooperation response was send
  COOPERATION_REFUSE_SEND,      //< 6 cooperation refuse was send
  COOPERATION_ACCEPT_SEND,      //< 7 cooperation accept was send
  NEGOTIATION_FINISHED_SEND,    //< 8 negotiation finished was send
  STOP_COOPERATION_SEND,        //< 9 stop cooperation was send
  COOPERATION,                  //< 10 active cooperation with this engine
  NO_COOPERATION                //< 11 no cooperation with this engine
};

//* CooperationContainer
/**
 *
 *
 */
struct CooperationContainer
{
  CooperationState state = CooperationState::UNKNOWN; /**< State of the cooperation */
  std::shared_ptr<SubModelDesc> subModel; /**< Sub model description */
  std::vector<std::shared_ptr<BaseInformationStream>> streamsSend; /**< List of streams send to this engine */
  std::vector<std::shared_ptr<BaseInformationStream>> streamsReceived; /**< List of streams received from this engine */
};

//* EngineState
/**
 * This class stores the meta information from other engines.
 *
 */
class EngineState : public enable_shared_from_this<EngineState>
{
  friend Node;
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
   * \brief Sets the iri of this engine.
   *
   * Sets the iri of this engine.
   *
   * \param iri The iri
   */
  void setSystemIri(const std::string iri);

  /*!
   * \brief Returns the short version of the iri of this engine.
   *
   * Returns the iri short version of the iri of this engine.
   */
  const std::string getSystemIriShort();

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
   * \brief Returns the time stamp of the last cooperation state update.
   *
   * Returns the time stamp of the last cooperation state update.
   */
  const time getTimeLastStateUpdate() const;

  /*!
   * \brief Returns a pointer to the list of streams offered.
   *
   * Returns a pointer to the list of streams offered.
   */
  std::shared_ptr<CooperationContainer> getOffering();

  /*!
   * \brief Returns a pointer to the list of streams requested.
   *
   * Returns a pointer to the list of streams requested.
   */
  std::shared_ptr<CooperationContainer> getRequesting();

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

  bool isCooperationPossible() const;

  bool isNodesKnown() const;

  void setNodesKnown(bool value);

  void updateOffering(std::vector<std::shared_ptr<Node>> *nodes,
                      std::vector<std::shared_ptr<BaseInformationStream>> *streamsSend,
                      std::vector<std::shared_ptr<BaseInformationStream>> *streamsReceived);

  void updateRequesting(std::vector<std::shared_ptr<Node>> *nodes,
                       std::vector<std::shared_ptr<BaseInformationStream>> *streamsSend,
                       std::vector<std::shared_ptr<BaseInformationStream>> *streamsReceived);

  void clearOffering();

  void clearRequesting();

  std::vector<std::shared_ptr<EngineConnection>> getConnections();

private:
  void updateContainer(std::shared_ptr<CooperationContainer> container, std::vector<std::shared_ptr<Node>> *nodes,
                       std::vector<std::shared_ptr<BaseInformationStream>> *streamsSend,
                       std::vector<std::shared_ptr<BaseInformationStream>> *streamsReceived);

  void clearContainer(std::shared_ptr<CooperationContainer> container);

private:
  std::weak_ptr<ICEngine> engine; /**< The main engine */
  std::shared_ptr<TimeFactory> timeFactory; /**< Time factory to create and compare time stamps */
  std::shared_ptr<Configuration> config; /**< Configuration object */
  std::shared_ptr<Communication> communication; /**< Communication interface */
  identifier engineId; /**< Unique identifier of the engine */
  std::string systemIri; /**< The iri of this system */
  std::string systemIriShort; /**< Short iri version */
  std::shared_ptr<CooperationContainer> offering; /**< Container for parts of the information processing offered to this engine */
  std::shared_ptr<CooperationContainer> requesting; /**< Container for parts of the information processing requested from this engine */
  std::set<std::shared_ptr<Node>> nodesActivated; /**< Nodes activated by this engine */
  time timeLastActivity; /**< Time stamp of the last activity of this engine */
  time timeLastStateUpdate; /**< Time stamp of the last cooperation state update */
  int retryCounter; /**< Counts the retrys of the current activity */
  std::vector<std::shared_ptr<EngineConnection>> connections; /**< Connections of the engine to other engines */
  bool nodesKnown; /**< True if the nodes of this engine are known, false otherwise */
  el::Logger* _log; /**< Logger */
};

} /* namespace ice */

#endif /* ENGINESTATE_H_ */
