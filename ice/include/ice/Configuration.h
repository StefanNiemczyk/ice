/*
 * Configuration.h
 *
 *  Created on: May 26, 2014
 *      Author: sni
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <string>
#include <vector>

namespace ice
{

class Configuration
{
public:
  Configuration();
  virtual ~Configuration();

  int getEventHandlerBufferSize() const;

  void setEventHandlerBufferSize(int eventHandlerBufferSize);

  int getEventHandlerThreadCount() const;

  void setEventHandlerThreadCount(int eventHandlerThreadCount);

  int getInformationStreamBufferSize() const;

  void setInformationStreamBufferSize(int informationStreamBufferSize);

  long getHeartbeatTimeout() const;

  void setHeartbeatTimeout(long heartbeatTimeout);

  long getCoordinationMessageTimeout() const;

  void setCoordinationMessageTimeout(long coordinationMessageTimeout);

  int getMaxRetryCount() const;

  void setMaxRetryCount(int maxRetryCount);

public:
  int informationStreamBufferSize;

  int eventHandlerThreadCount;
  int eventHandlerBufferSize;

  long heartbeatTimeout;
  long coordinationMessageTimeout;
  int maxRetryCount;

  std::string ontologyIriMapper;
  std::string ontologyIri;
  std::string ontologyIriOwnEntity;

  bool synthesizeTransformations;
  bool generateInformationProcessing;

  bool  asp_globalOptimization;         /**< True if QoS metadata should be optimized global, false for local */
  int   asp_maxHopCount;            /**< Max count of hops */
  int   asp_maxChainLength;           /**< Maximal length of a node chain */
  bool  asp_useXMLTransformation;
  bool  asp_useAutoTransformation;
  std::vector<std::string> asp_additionalFiles;

  //Static values
public:
  static std::string INFORMATION_TYPE_NAME_SEPERATOR;
};

} /* namespace ice */

#endif /* CONFIGURATION_H_ */
