/*
 * ProcessingModelGenerator.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sni
 */

#ifndef PROCESSINGMODEL_H_
#define PROCESSINGMODEL_H_

#include <memory>
#include <mutex>

//Forward declaration
namespace ice
{
  class ICEngine;
} /* namespace ice */

namespace ice
{

class ProcessingModelGenerator
{
public:
  ProcessingModelGenerator(std::weak_ptr<ICEngine> engine);
  virtual ~ProcessingModelGenerator();

  virtual void init() = 0;
  virtual void cleanUp() = 0;
  virtual void createProcessingModel() = 0;

protected:
  std::weak_ptr<ICEngine> engine; /*< Weak pointer to ice engine */
  std::mutex mtx_; /**< Mutex */
};

} /* namespace ice */

#endif /* PROCESSINGMODEL_H_ */
