/*
 * ServalCommunication.h
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#ifndef SERVALCOMMUNICATION_H_
#define SERVALCOMMUNICATION_H_

#include <mutex>
#include <thread>
#include <vector>

#include <ice/communication/CommunicationInterface.h>

#define SERVAL_PORT 8845

namespace ice
{
class Entity;
class IceServalBridge;
class MDPSocket;
class serval_interface;
class ServalInformationSender;
class ServalInformationReceiver;

class ServalCommunication : public CommunicationInterface, public std::enable_shared_from_this<ServalCommunication>
{
public:
  ServalCommunication(std::weak_ptr<ICEngine> engine, std::string const configPath, std::string const host,
                      int const port, std::string const authName, std::string const authPass, bool const local = false);
  virtual ~ServalCommunication();

  void checkServal();
  std::shared_ptr<serval_interface> getServalInterface();
  void setOwnSid(std::string const &sid);
  void removeReceiver(std::shared_ptr<InformationReceiver> receiver);

protected:
          void read();
  virtual void initInternal();
  virtual void cleanUpInternal();
  virtual void discover();
  virtual int  readMessage(std::vector<std::shared_ptr<Message>> &outMessages);
  virtual void sendMessage(std::shared_ptr<Message> msg);
  virtual std::shared_ptr<BaseInformationSender> createSender(std::shared_ptr<InformationCollection> collection);
  virtual std::shared_ptr<InformationReceiver> createReceiver(std::shared_ptr<InformationCollection> collection);

private:
  std::shared_ptr<serval_interface>             serval;
  std::string                                   ownSid;
  std::shared_ptr<MDPSocket>                    socket;
  std::thread                                   worker;
  bool                                          running;
  unsigned char                                 buffer[1024];
  std::vector<std::shared_ptr<ServalInformationReceiver>> informationReceivers;

  std::string                           const   configPath;
  std::string                           const   host;
  int                                   const   port;
  std::string                           const   authName;
  std::string                           const   authPass;
  bool                                  const   local;
  el::Logger                                    *_log;
  std::mutex                                    _mtxSend;
  std::mutex                                    _mtxReceiver;
};

} /* namespace ice */

#endif /* SERVALCOMMUNICATION_H_ */
