/*
 * ComJobBase.h
 *
 *  Created on: Jul 18, 2016
 *      Author: sni
 */

#ifndef COMJOBBASE_H_
#define COMJOBBASE_H_

#include <functional>
#include <map>
#include <memory>
#include "easylogging++.h"

#include "ice/communication/messages/Message.h"
#include "ice/communication/messages/CommandMessage.h"
#include "ice/communication/CommunicationInterface.h"
#include "ice/ICEngine.h"
#include "ice/Entity.h"
#include "ice/Time.h"

namespace ice
{

enum CJState
{
  CJ_CREATED, CJ_INITIALIZED, CJ_ACTIVE, CJ_WAITING, CJ_FINISHED, CJ_ABORTED
};

typedef std::function<std::shared_ptr<ComJobBase>(std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity)> jobCreator;

class ComJobRegistry
{
  static std::map<uint8_t, jobCreator> jobs;

public:
  static std::shared_ptr<ComJobBase> makeInstance(uint8_t id, std::weak_ptr<ICEngine> engine,
                                                  std::shared_ptr<Entity> const &entity)
  {
    auto func = ComJobRegistry::jobs.find(id);
    if (func == ComJobRegistry::jobs.end())
    {
      throw std::runtime_error("No creator for com job " + std::to_string(id));
    }

    return func->second(engine, entity);
  }

  static void put(uint8_t id, jobCreator func)
  {
    if (ComJobRegistry::jobs.find(id) != ComJobRegistry::jobs.end())
    {
      throw std::runtime_error("Duplicated communication job id " + std::to_string(id));
    }

    ComJobRegistry::jobs[id] = func;
  }
};

class ComJobBase
{
public:
  ComJobBase(uint8_t id, std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity, el::Logger *log) :
      id(id), engine(engine), entity(entity), timeout(0), timestampLastActive(0), _log(log), state(CJ_CREATED), ownJob(true)
  {
    this->index = 0; //entity->getNextRequestId();
    auto e = engine.lock();
    this->self = e->getSelf();
    this->com = e->getCommunicationInterface();
    this->timeFactory = e->getTimeFactory();
  }

  virtual ~ComJobBase()
  {
    if (this->state != CJState::CJ_FINISHED && this->state != CJ_ABORTED)
    {
      this->sendCommand(IceMessageIds::IMI_CANCLE_JOB);
    }
  }

  const std::shared_ptr<Entity>& getEntity() const
  {
    return entity;
  }

  void setEntity(const std::shared_ptr<Entity>& entity)
  {
    this->entity = entity;
  }

  bool isOwnJob()
  {
    return ownJob;
  }

  void setOwnJob(bool ownJob)
  {
    this->ownJob = ownJob;
  }

  int getIndex() const
  {
    return index;
  }

  void setIndex(uint8_t index)
  {
    this->index = index;
  }

  CJState getState() const
  {
    return state;
  }

  unsigned long long getTimeout() const
  {
    return timeout;
  }

  void setTimeout(unsigned long long timeout)
  {
    this->timeout = timeout;
  }

  virtual bool checkTimeout()
  {
    if (this->ownJob || this->timestampLastActive == 0 || this->timeout == 0)
    {
      return false;
    }

    return this->timeFactory->checkTimeout(this->timestampLastActive, this->timeout);
  }

  void updateActiveTime()
  {
    this->timestampLastActive = this->timeFactory->createTime();
  }

  void sendCommand(IceMessageIds command)
  {
    auto msg = std::make_shared<CommandMessage>(command);
    this->send(msg);
  }

  void send(std::shared_ptr<Message> msg)
  {
    if (this->ownJob)
      msg->setJobId(this->id + 127);
    else
      msg->setJobId(this->id);

    msg->setJobIndex(this->index);
    msg->setEntity(entity);

    this->com->send(msg);
  }

  bool match(uint8_t id, uint8_t index)
  {
    return (this->id == id && this->index == index);
  }

  virtual void init()
  {
    this->index = entity->getNextIndex();
    this->state = CJ_INITIALIZED;
  }

  void init(std::shared_ptr<Message> const &message)
  {
    this->state = CJ_INITIALIZED;
    this->handleMessage(message);
  }

  virtual void cleanUp()
  {

  }

  virtual void tick()
  {

  }

  virtual void handleMessage(std::shared_ptr<Message> const &message)
  {
    if (message->getId() == IceMessageIds::IMI_CANCLE_JOB)
    {
      this->abort();
    }
    else
    {
      _log->warn("Unknown command '%v' for job '%v', message will be skipped", std::to_string(message->getId()),
                 std::to_string(this->id));
    }
  }

  virtual void abort()
  {
    this->state = CJState::CJ_ABORTED;
    this->sendCommand(IceMessageIds::IMI_CANCLE_JOB);
    this->callCallbackAborted();
  }

  virtual void finish()
  {
    this->state = CJState::CJ_FINISHED;
    this->callCallbackFinished();
  }

protected:
  // callback stuff
  virtual void callCallbackFinished() = 0;
  virtual void callCallbackAborted() = 0;

protected:
  std::weak_ptr<ICEngine>               engine;
  bool                                  ownJob;
  std::shared_ptr<Entity>               self;
  std::shared_ptr<Entity>               entity;
  std::shared_ptr<TimeFactory>          timeFactory;
  uint8_t                               id;
  uint8_t                               index;
  CJState                               state;
  unsigned long long                    timeout;
  time                                  timestampLastActive;
  el::Logger                            *_log;

private:
  std::shared_ptr<CommunicationInterface> com;
};

} /* namespace ice */

#endif /* COMJOBBASE_H_ */
