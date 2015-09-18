/*
 * RosCommunication.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: sni
 */

#include "ice/ros/RosCommunication.h"

#include "ice/container/Position.h"
#include "ice/coordination/Coordinator.h"
#include "ice/coordination/CooperationRequest.h"
#include "ice/coordination/CooperationResponse.h"
#include "ice/processing/LambdaTask.h"
#include "ice/processing/NodeDescription.h"
#include "ice/ros/RosTimeFactory.h"
#include "ice/ICEngine.h"

#include "ice_msgs/Position.h"
#include "easylogging++.h"
#include "serialize.h"

namespace ice
{

RosCommunication::RosCommunication(std::weak_ptr<ICEngine> engine) :
    Communication(engine)
{
  this->_log = el::Loggers::getLogger("RosCommunication");
  this->running = false;
}

RosCommunication::~RosCommunication()
{
}

void RosCommunication::init()
{
  Communication::init();

  // init ros channels
  this->heartbeatPublisher = this->nodeHandel.advertise<ice_msgs::Heartbeat>("ice/heartbeat", 100);
  this->heartbeatSubscriber = this->nodeHandel.subscribe("ice/heartbeat", 100, &RosCommunication::onHeartbeat, this);
  this->coordinationPublisher = this->nodeHandel.advertise<ice_msgs::ICECoordination>("ice/coordination", 100);
  this->coordinationSubscriber = this->nodeHandel.subscribe("ice/coordination", 100, &RosCommunication::onCoordination,
                                                            this);

//  this->informationModelPublisher = this->nodeHandel.advertise<ice_msgs::InformationModel>("ice/information_model",
//                                                                                           100);
//  this->informationModelSubscriber = this->nodeHandel.subscribe("ice/information_model", 100,
//                                                                &RosCommunication::onInformationModel, this);
//  this->cooperationRequestPublisher = this->nodeHandel.advertise<ice_msgs::CooperationRequest>(
//      "ice/cooperation_request", 100);
//  this->cooperationRequestSubscriber = this->nodeHandel.subscribe("ice/cooperation_request", 100,
//                                                                  &RosCommunication::onCooperationRequest, this);
//  this->cooperationResponsePublisher = this->nodeHandel.advertise<ice_msgs::CooperationResponse>(
//      "ice/cooperation_response", 100);
//  this->cooperationResponseSubscriber = this->nodeHandel.subscribe("ice/cooperation_response", 100,
//                                                                   &RosCommunication::onCooperationResponse, this);

  this->running = true;
  this->worker = std::thread(&RosCommunication::workerTask, this);
}

void RosCommunication::cleanUp()
{
  Communication::cleanUp();

  this->heartbeatPublisher.shutdown();
  this->heartbeatSubscriber.shutdown();
  this->coordinationPublisher.shutdown();
  this->coordinationSubscriber.shutdown();

  if (this->running == false)
    return;

  this->running = false;
  this->worker.join();
}

void RosCommunication::sendHeartbeat()
{
  _log->verbose(1, "Sending heartbeat");

  ice_msgs::Heartbeat heartbeat;

//  auto arr = IDGenerator::toByte(this->engineId);
//  for (int i = 0; i < 16; ++i)
//  {
//    std:: cout << ((int) arr[i]) << " ";
//    heartbeat.header.senderId.id.push_back(arr[i]);
//  }
//  std::cout << std::endl;

//  heartbeat.header.senderId.id.resize(16);
//  std::copy(this->engineId.begin(), this->engineId.end(), heartbeat.header.senderId.id.begin());
  heartbeat.header.senderId.value = this->engineId;
  heartbeat.header.timestamp = ros::Time::now();

  this->heartbeatPublisher.publish(heartbeat);
}

void RosCommunication::sendCoordinationMsg(identifier receiverId, RosCoordinationCommand command)
{
  ice_msgs::ICECoordination msg;

  msg.header.senderId.value = this->engineId;
  msg.header.timestamp = ros::Time::now();
  ice_msgs::Identifier receiver;
  receiver.value = receiverId;
  msg.header.receiverIds.push_back(receiver);
  msg.command = command;

  this->coordinationPublisher.publish(msg);
}

void RosCommunication::sendCoordinationMsg(identifier receiverId, RosCoordinationCommand command, std::vector<uint8_t> &bytes)
{
  ice_msgs::ICECoordination msg;

  msg.header.senderId.value = this->engineId;
  msg.header.timestamp = ros::Time::now();
  ice_msgs::Identifier receiver;
  receiver.value = receiverId;
  msg.header.receiverIds.push_back(receiver);
  msg.command = command;
  msg.bytes = bytes;

  this->coordinationPublisher.publish(msg);
}

void RosCommunication::sendSystemSpecRequest(identifier receiverId)
{
  _log->info("Sending system spec request to engine '%v'", receiverId);

  this->sendCoordinationMsg(receiverId, RosCoordinationCommand::SYSTEM_SPEC_REQUEST);
}

void RosCommunication::sendSystemSpecResponse(
    identifier receiverId, std::tuple<std::string, std::vector<std::string>, std::vector<std::string>> &content)
{
  _log->info("Sending system specification to engine '%v'", receiverId);

  StreamType res;
  serialize(content,res);

  this->sendCoordinationMsg(receiverId, RosCoordinationCommand::SYSTEM_SPEC_RESPONSE, res);
}

void RosCommunication::sendSubModelRequest(identifier receiverId, SubModelDesc &modelDesc)
{
  _log->info("Sending sub model request to engine '%v'", receiverId);

  StreamType res;
  serialize(modelDesc,res);

  this->sendCoordinationMsg(receiverId, RosCoordinationCommand::SUB_MODEL_REQUEST, res);
}

void RosCommunication::sendSubModelResponse(identifier receiverId, int index, bool accept)
{
  _log->info("Sending sub model response to engine '%v'", receiverId);

  StreamType res;
  serialize(std::tuple<int, bool>(index, accept),res);

  this->sendCoordinationMsg(receiverId, RosCoordinationCommand::SUB_MODEL_RESPONSE, res);
}

void RosCommunication::sendNegotiationFinished(identifier receiverId)
{
  _log->info("Send negotiation finished from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

  this->sendCommand(receiverId, RosCoordinationCommand::NEGOTIATION_FINISHED);
}

void RosCommunication::sendStopCooperation(identifier receiverId)
{
  _log->info("Send stop cooperation from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

  this->sendCommand(receiverId, RosCoordinationCommand::STOP_COOPERATION);
}


// -#######################################################################

void RosCommunication::sendInformationRequest(identifier receiverId)
{
  _log->info("Send information model request from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

  this->sendCommand(receiverId, RosCoordinationCommand::REQUEST_INFORMATION_MODEL);
}

void RosCommunication::sendInformationModel(identifier receiverId, std::shared_ptr<InformationModel> informationModel)
{
  _log->info("Send information model from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

//  ice_msgs::InformationModel model;
//  model.header.senderId.id.resize(16);
//  std::copy(this->engineId.begin(), this->engineId.end(), model.header.senderId.id.begin());
//
//  ice_msgs::Identifier receiver;
//  receiver.id.resize(16);
//  std::copy(receiverId.begin(), receiverId.end(), receiver.id.begin());
//  model.header.receiverIds.push_back(receiver);
//
//  // add streams
//  for (auto stream : *informationModel->getStreams())
//  {
//    ice_msgs::StreamDescription desc;
//
//    desc.informationIdentifier.id.resize(16);
//    std::copy(stream->getId().begin(), stream->getId().end(), desc.informationIdentifier.id.begin());
//    desc.shared = stream->isShared();
//
//    model.streamDescriptions.push_back(desc);
//  }
//
//  // add stream templates
//  for (auto streamTemplate : *informationModel->getStreamTemplates())
//  {
//    ice_msgs::StreamTemplateDescription desc;
//
//    desc.informationIdentifier.id.resize(16);
//    std::copy(streamTemplate->getId().begin(), streamTemplate->getId().end(), desc.informationIdentifier.id.begin());
//
//    model.streamTemplateDescriptions.push_back(desc);
//  }
//
//  // add nodes
//  for (auto node : *informationModel->getNodeDescriptions())
//  {
//    ice_msgs::NodeDescription desc;
//
//    desc.className = node->getClassName();
//
//    int inputSize, inputTemplateSize, outputSize;
//    const boost::uuids::uuid* inputs = node->getInputUuids(&inputSize);
//    const boost::uuids::uuid* inputTemplates = node->getInputTemplateIds(&inputTemplateSize);
//    const boost::uuids::uuid* outputs = node->getOutputIds(&outputSize);
//
//    for (int i = 0; i < inputSize; ++i)
//    {
//      ice_msgs::Identifier id;
//      id.id.resize(16);
//      std::copy(inputs[i].begin(), inputs[i].end(), id.id.begin());
//      desc.inputInformation.push_back(id);
//    }
//
//    for (int i = 0; i < inputTemplateSize; ++i)
//    {
//      ice_msgs::Identifier id;
//      id.id.resize(16);
//      std::copy(inputTemplates[i].begin(), inputTemplates[i].end(), id.id.begin());
//      desc.inputTemplateInformation.push_back(id);
//    }
//
//    for (int i = 0; i < outputSize; ++i)
//    {
//      ice_msgs::Identifier id;
//      id.id.resize(16);
//      std::copy(outputs[i].begin(), outputs[i].end(), id.id.begin());
//      desc.outputInformation.push_back(id);
//    }
//
//    model.nodeDescription.push_back(desc);
//  }
//
//  this->informationModelPublisher.publish(model);
}

void RosCommunication::sendCooperationRequest(identifier receiverId,
                                              std::shared_ptr<CooperationRequest> cooperationRequest)
{
  _log->info("Send cooperation request from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

//  ice_msgs::CooperationRequest request;
//  request.header.senderId.id.resize(16);
//  std::copy(this->engineId.begin(), this->engineId.end(), request.header.senderId.id.begin());
//
//  ice_msgs::Identifier receiver;
//  receiver.id.resize(16);
//  std::copy(receiverId.begin(), receiverId.end(), receiver.id.begin());
//  request.header.receiverIds.push_back(receiver);
//
//  // add offers
//  for (auto offer : *cooperationRequest->getOffers())
//  {
//    ice_msgs::StreamDescription desc;
//
//    desc.informationIdentifier.id.resize(16);
//    std::copy(offer->getId().begin(), offer->getId().end(), desc.informationIdentifier.id.begin());
//    desc.shared = offer->isShared();
//
//    request.offers.push_back(desc);
//  }
//
//  // add requests
//  for (auto requests : *cooperationRequest->getRequests())
//  {
//    ice_msgs::StreamTemplateDescription desc;
//
//    desc.informationIdentifier.id.resize(16);
//    std::copy(requests->getId().begin(), requests->getId().end(), desc.informationIdentifier.id.begin());
//
//    request.requests.push_back(desc);
//  }
//
//  this->cooperationRequestPublisher.publish(request);
}

void RosCommunication::sendCooperationResponse(identifier receiverId,
                                               std::shared_ptr<CooperationResponse> cooperationResponse)
{
  _log->info("Send cooperation response from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

//  ice_msgs::CooperationRequest response;
//  response.header.senderId.id.resize(16);
//  std::copy(this->engineId.begin(), this->engineId.end(), response.header.senderId.id.begin());
//
//  ice_msgs::Identifier receiver;
//  receiver.id.resize(16);
//  std::copy(receiverId.begin(), receiverId.end(), receiver.id.begin());
//  response.header.receiverIds.push_back(receiver);
//
//  // add offers
//  for (auto offer : *cooperationResponse->getOffersAccepted())
//  {
//    ice_msgs::StreamDescription desc;
//
//    desc.informationIdentifier.id.resize(16);
//    std::copy(offer->getId().begin(), offer->getId().end(), desc.informationIdentifier.id.begin());
//    desc.shared = offer->isShared();
//
//    response.offers.push_back(desc);
//  }
//
//  // add requests
//  for (auto requests : *cooperationResponse->getRequestsAccepted())
//  {
//    ice_msgs::StreamTemplateDescription desc;
//
//    desc.informationIdentifier.id.resize(16);
//    std::copy(requests->getId().begin(), requests->getId().end(), desc.informationIdentifier.id.begin());
//
//    response.requests.push_back(desc);
//  }
//
//  this->cooperationResponsePublisher.publish(response);
}

void RosCommunication::sendCooperationRefuse(identifier receiverId)
{
  _log->info("Send cooperation refuse from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

  this->sendCommand(receiverId, RosCoordinationCommand::COOPERATION_REFUSE);
}

void RosCommunication::sendCooperationAccept(identifier receiverId)
{
  _log->info("Send cooperation accept from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

  this->sendCommand(receiverId, RosCoordinationCommand::COOPERATION_ACCEPT);
}

void RosCommunication::sendRetryNegotiation(identifier receiverId)
{
  _log->info("Send retry negotiation from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

  this->sendCommand(receiverId, RosCoordinationCommand::NEGOTIATION_RETRY);
}

void RosCommunication::sendCooperationStopped(identifier receiverId)
{
  _log->info("Send cooperation stopped from engine %v to engine %v",
             IDGenerator::toString(this->engineId).c_str(), IDGenerator::toString(receiverId).c_str());

  this->sendCommand(receiverId, RosCoordinationCommand::COOPERATION_STOPPED);
}

std::shared_ptr<BaseInformationSender> RosCommunication::registerStreamAsSender(
    std::shared_ptr<BaseInformationStream> stream)
{
  std::shared_ptr<BaseInformationSender> ptr;

  ptr = this->createSender(stream);

  if (false == ptr)
  {
    auto type = stream->getTypeInfo();
    _log->error("Unknown information type %v of stream %v with type string %v",
                (*type).name(), stream->getName().c_str(), stream->getSpecification()->getTypeString().c_str());
  }

  return ptr;
}

std::shared_ptr<InformationReceiver> RosCommunication::registerStreamAsReceiver(
    std::shared_ptr<BaseInformationStream> stream)
{
  std::shared_ptr<InformationReceiver> ptr;

  ptr = this->createReceiver(stream);

  if (false == ptr)
  {
    auto type = stream->getTypeInfo();
    _log->error("Unknown information type %v of stream %v with type string %v",
                (*type).name(), stream->getName().c_str(), stream->getSpecification()->getTypeString().c_str());
  }

  return ptr;
}

std::shared_ptr<BaseInformationSender> RosCommunication::createSender(std::shared_ptr<BaseInformationStream> stream)
{
  auto type = stream->getTypeInfo();

  if (typeid(Position) == *type)
  { // ice::Position -> ice_msgs::Position
    _log->debug("Creating sender for stream %v with mapping ice::Position -> ice_msgs::Position",
                stream->getName().c_str());
    transformC2M<Position, ice_msgs::Position> method = &RosMessageTransform::transformC2MPosition;
    return this->_createSender<Position, ice_msgs::Position>(stream, method);
  }
  else if (typeid(std::vector<Position>) == *type)
  { // ice::Position[] -> ice_msgs::Positions
    _log->debug("Creating sender for stream %v with mapping ice::Position[] -> ice_msgs::Positions",
                stream->getName().c_str());
    transformC2M<std::vector<Position>, ice_msgs::Positions> method = &RosMessageTransform::transformC2MPositions;
    return this->_createSender<std::vector<Position>, ice_msgs::Positions>(stream, method);
  }

  std::shared_ptr<BaseInformationSender> ptr;
  return ptr;
}

std::shared_ptr<InformationReceiver> RosCommunication::createReceiver(std::shared_ptr<BaseInformationStream> stream)
{
  auto type = stream->getTypeInfo();

  if (typeid(Position) == *type)
  { // ice_msgs::Position -> ice::Position
    _log->debug("Creating receiver for stream %v with mapping ice_msgs::Position -> ice::Position",
                stream->getName().c_str());
    transformM2C<Position, ice_msgs::Position> method = &RosMessageTransform::transformM2CPosition;
    return this->_createReceiver<Position, ice_msgs::Position>(stream, method);
  }
  else if (typeid(std::vector<Position>) == *type)
  { // ice_msgs::Positions -> ice::Position[]
    _log->debug("Creating receiver for stream %v with mapping ice_msgs::Positions -> ice::Position[]",
                stream->getName().c_str());
    transformM2C<std::vector<Position>, ice_msgs::Positions> method = &RosMessageTransform::transformM2CPositions;
    return this->_createReceiver<std::vector<Position>, ice_msgs::Positions>(stream, method);
  }

  std::shared_ptr<InformationReceiver> ptr;
  return ptr;
}

void RosCommunication::sendCommand(const identifier receiverId, const RosCoordinationCommand command) const
{
  if (this->running == false)
  {
    this->_log->warn("Communication not initialized, message will not be send '%v'", RosCoordinationCommandString[command]);
    return;
  }

  ice_msgs::ICECoordination coordinationMsg;
  coordinationMsg.header.senderId.value = this->engineId;
//  coordinationMsg.header.senderId.id.resize(16);
//  std::copy(this->engineId.begin(), this->engineId.end(), coordinationMsg.header.senderId.id.begin());

  ice_msgs::Identifier receiver;
//  receiver.id.resize(16);
//  std::copy(receiverId.begin(), receiverId.end(), receiver.id.begin());
  receiver.value = receiverId;
  coordinationMsg.header.receiverIds.push_back(receiver);

  coordinationMsg.command = command;

  this->coordinationPublisher.publish(coordinationMsg);
}

void RosCommunication::workerTask()
{
  ros::Rate loop_rate(30);

  while (this->running && this->nodeHandel.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

// ROS listener methods

void RosCommunication::onHeartbeat(const ice_msgs::Heartbeat::ConstPtr& msg)
{
  identifier senderId = msg->header.senderId.value;//IDGenerator::getInstance()->getIdentifier(msg->header.senderId.value);

  if (senderId == this->engineId)
    return;

  _log->info("Heartbeat received from engine %v", IDGenerator::toString(senderId).c_str());

  this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
  { return this->coordinator->onEngineHeartbeat(senderId, RosTimeFactory::createTime(msg->header.timestamp));}));
//  this->coordinator->onEngineHeartbeat(senderId, RosTimeFactory::createTime(msg->header.timestamp));
}

void RosCommunication::onCoordination(const ice_msgs::ICECoordination::ConstPtr& msg)
{
  identifier senderId = msg->header.senderId.value;//IDGenerator::getInstance()->getIdentifier(msg->header.senderId.value);

  if (false == this->checkReceiverIds(msg->header) || senderId == this->engineId)
    return;

  _log->info("Coordination command '%v' received from engine '%v'",
             std::to_string(msg->command).c_str(), IDGenerator::toString(senderId).c_str());

  switch (msg->command)
  {
    case RosCoordinationCommand::SYSTEM_SPEC_REQUEST:
      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
      {
        return this->coordinator->onSystemSpecRequest(senderId);
      }));
      break;
    case RosCoordinationCommand::SYSTEM_SPEC_RESPONSE:
      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
      {
        auto content = deserialize<std::tuple<std::string, std::vector<std::string>, std::vector<std::string>>>(msg->bytes);
        return this->coordinator->onSystemSpec(senderId, content);
      }
      ));
      break;
    case RosCoordinationCommand::SUB_MODEL_REQUEST:
      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
      {
        auto content = deserialize<SubModelDesc>(msg->bytes);
        return this->coordinator->onSubModelRequest(senderId, content);
      }
      ));
      break;
    case RosCoordinationCommand::SUB_MODEL_RESPONSE:
      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
      {
        auto content = deserialize<std::tuple<int, bool>>(msg->bytes);
        return this->coordinator->onSubModelResponse(senderId, std::get<0>(content), std::get<1>(content));
      }
      ));
      break;
    case RosCoordinationCommand::NEGOTIATION_FINISHED:
      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
      { return this->coordinator->onNegotiationFinished(senderId);}));
      break;
    case RosCoordinationCommand::STOP_COOPERATION:
      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
      { return this->coordinator->onStopCooperation(senderId);}));
      break;
//    case RosCoordinationCommand::REQUEST_INFORMATION_MODEL:
//      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//      { return this->coordinator->onInformationModelRequest(senderId);}));
//      break;
//    case RosCoordinationCommand::COOPERATION_ACCEPT:
//      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//      { return this->coordinator->onCooperationAccept(senderId);}));
//      break;
//    case RosCoordinationCommand::COOPERATION_REFUSE:
//      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//      { return this->coordinator->onCooperationRefuse(senderId);}));
//      break;
//    case RosCoordinationCommand::NEGOTIATION_FINISHED:
//      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//      { return this->coordinator->onNegotiationFinished(senderId);}));
//      break;
//    case RosCoordinationCommand::NEGOTIATION_RETRY:
//      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//      { return this->coordinator->onRetryNegotiation(senderId);}));
//      break;
//    case RosCoordinationCommand::STOP_COOPERATION:
//      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//      { return this->coordinator->onStopCooperation(senderId);}));
//      break;
//    case RosCoordinationCommand::COOPERATION_STOPPED:
//      this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//      { return this->coordinator->onCooperationStopped(senderId);}));
//      break;
    default:
      _log->error("Unknown coordination command from engine %v with command %v",
                  IDGenerator::toString(senderId).c_str(), msg->command);
  }
}

void RosCommunication::onInformationModel(const ice_msgs::InformationModel::ConstPtr& msg)
{
//  identifier senderId = IDGenerator::getInstance()->getIdentifier(msg->header.senderId.id);
//
//  if (false == this->checkReceiverIds(msg->header) || senderId == this->engineId)
//    return;
//
//  _log->debug("Information model received from engine %v",
//              IDGenerator::toString(senderId).c_str());
//
//  auto model = std::make_shared<InformationModel>();
//
//// Stream descriptions
//  for (auto stream : msg->streamDescriptions)
//  {
//    identifier streamId = IDGenerator::getInstance()->getIdentifier(stream.informationIdentifier.id);
//    auto desc = std::make_shared<StreamDescription>(streamId, stream.shared);
//    model->getStreams()->push_back(desc);
//  }
//
//// Stream template descriptions
//  for (auto streamTemplate : msg->streamTemplateDescriptions)
//  {
//    identifier streamTemplateId = IDGenerator::getInstance()->getIdentifier(streamTemplate.informationIdentifier.id);
//    auto desc = std::make_shared<StreamTemplateDescription>(streamTemplateId);
//    model->getStreamTemplates()->push_back(desc);
//  }
//
//// Node descriptions
//  for (auto node : msg->nodeDescription)
//  {
//    int inputSize = node.inputInformation.size();
//    int inputTemplateSize = node.inputTemplateInformation.size();
//    int outputSize = node.outputInformation.size();
//
//    identifier * inputUuids = new boost::uuids::uuid[inputSize];
//
//    for (int i = 0; i < inputSize; ++i)
//    {
//      identifier id = IDGenerator::getInstance()->getIdentifier(node.inputInformation.at(i).id);
//      inputUuids[i] = id;
//    }
//
//    identifier * inputTeamplateUuids = new boost::uuids::uuid[inputTemplateSize];
//
//    for (int i = 0; i < inputTemplateSize; ++i)
//    {
//      identifier id = IDGenerator::getInstance()->getIdentifier(node.inputTemplateInformation.at(i).id);
//      inputTeamplateUuids[i] = id;
//    }
//
//    identifier * outputUuids = new boost::uuids::uuid[outputSize];
//
//    for (int i = 0; i < outputSize; ++i)
//    {
//      identifier id = IDGenerator::getInstance()->getIdentifier(node.outputInformation.at(i).id);
//      outputUuids[i] = id;
//    }
//
//    auto desc = std::make_shared<NodeDescription>(node.className, inputUuids, inputTeamplateUuids, outputUuids,
//                                                  inputSize, inputTemplateSize, outputSize);
//
//    model->getNodeDescriptions()->push_back(desc);
//  }
//
//  this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//        { return this->coordinator->onInformationModel(senderId, model);}));
}

void RosCommunication::onCooperationRequest(const ice_msgs::CooperationRequest::ConstPtr& msg)
{
//  identifier senderId = IDGenerator::getInstance()->getIdentifier(msg->header.senderId.id);
//
//  if (false == this->checkReceiverIds(msg->header) || senderId == this->engineId)
//    return;
//
//  _log->debug("Cooperation request received from engine %v",
//              IDGenerator::toString(senderId).c_str());
//
//  auto request = std::make_shared<CooperationRequest>(senderId);
//
//// Stream offers
//  for (auto stream : msg->offers)
//  {
//    identifier streamId = IDGenerator::getInstance()->getIdentifier(stream.informationIdentifier.id);
//    auto desc = std::make_shared<StreamDescription>(streamId, stream.shared);
//    request->getOffers()->push_back(desc);
//  }
//
//// Stream template requests
//  for (auto streamTemplate : msg->requests)
//  {
//    identifier streamTemplateId = IDGenerator::getInstance()->getIdentifier(streamTemplate.informationIdentifier.id);
//    auto desc = std::make_shared<StreamTemplateDescription>(streamTemplateId);
//    request->getRequests()->push_back(desc);
//  }
//
//  this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//        { return  this->coordinator->onCooperationRequest(senderId, request);}));
}

void RosCommunication::onCooperationResponse(const ice_msgs::CooperationResponse::ConstPtr& msg)
{
//  identifier senderId = IDGenerator::getInstance()->getIdentifier(msg->header.senderId.id);
//
//  if (false == this->checkReceiverIds(msg->header) || senderId == this->engineId)
//    return;
//
//  _log->debug("Cooperation response received from engine %v",
//              IDGenerator::toString(senderId).c_str());
//
//  auto response = std::make_shared<CooperationResponse>(senderId);
//
//// Stream offers
//  for (auto stream : msg->offers)
//  {
//    identifier streamId = IDGenerator::getInstance()->getIdentifier(stream.informationIdentifier.id);
//    auto desc = std::make_shared<StreamDescription>(streamId, stream.shared);
//    response->getOffersAccepted()->push_back(desc);
//  }
//
//// Stream template requests
//  for (auto streamTemplate : msg->requests)
//  {
//    identifier streamTemplateId = IDGenerator::getInstance()->getIdentifier(streamTemplate.informationIdentifier.id);
//    auto desc = std::make_shared<StreamTemplateDescription>(streamTemplateId);
//    response->getRequestsAccepted()->push_back(desc);
//  }
//
//  this->eventHandler->addTask(std::make_shared<LambdaTask>([=] ()
//        { return this->coordinator->onCooperationResponse(senderId, response);}));
}

bool RosCommunication::checkReceiverIds(ice_msgs::ICEHeader header)
{
  for (auto id : header.receiverIds)
  {
    identifier receiverId = id.value;//IDGenerator::getInstance()->getIdentifier(id.value);

    if (receiverId == this->engineId)
      return true;
  }

  return false;
}

} /* namespace ice */
