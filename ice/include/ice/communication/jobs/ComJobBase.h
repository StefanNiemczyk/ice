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

enum CJState {
  CJ_CREATED,
  CJ_INITIALIZED,
  CJ_ACTIVE,
  CJ_WAITING,
  CJ_FINISHED
};

typedef std::function<std::shared_ptr<ComJobBase>(ICEngine* const engine, std::shared_ptr<Entity> const &entity)> jobCreator;

class ComJobRegistry
{
  static std::map<uint8_t, jobCreator> jobs;

public:
  static std::shared_ptr<ComJobBase> makeInstance(uint8_t id, ICEngine* const engine, std::shared_ptr<Entity> const &entity)
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
  ComJobBase(uint8_t id, ICEngine* const engine, std::shared_ptr<Entity> const &entity, el::Logger *log) :
      id(id), engine(engine), entity(entity), timeout(0), lastTimestamp(0), _log(log), state(CJ_CREATED), ownJob(true)
  {
    this->index = 0;//entity->getNextRequestId();
    this->self = engine->getSelf();
    this->com = engine->getCommunicationInterface();
  }
  virtual ~ComJobBase()
  {
    if (this->state != CJState::CJ_FINISHED)
    {
      auto msg = std::make_shared<CommandMessage>(IceCmd::SCMD_CANCLE_JOB);
      msg->setEntity(entity);

      if (this->ownJob)
        msg->setJobId(this->id + 127);
      else
        msg->setJobId(this->id);

      msg->setJobIndex(this->index);
      this->com->send(msg);
    }
  };

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

  time getTimeout() const
  {
    return timeout;
  }

  void setTimeout(time timeout)
  {
    this->timeout = timeout;
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
  };

  virtual void init(std::shared_ptr<Message> const &message) = 0;
  virtual void tick(time timestamp) {};
  virtual void handleMessage(std::shared_ptr<Message> const &message) = 0;
  virtual void cleanUp() {};

protected:
  ICEngine                                    *engine;
  bool                                        ownJob;
  std::shared_ptr<Entity>                     self;
  std::shared_ptr<Entity>                     entity;
  uint8_t                                     id;
  uint8_t                                     index;
  CJState                                     state;
  time                                        timeout;
  time                                        lastTimestamp;
  el::Logger                                  *_log;

private:
  std::shared_ptr<CommunicationInterface>     com;
};

} /* namespace ice */

#endif /* COMJOBBASE_H_ */
