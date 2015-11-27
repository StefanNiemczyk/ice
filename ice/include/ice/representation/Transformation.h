/*
 * Transformation.h
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

#include <vector>
#include <memory>

#include "ice/representation/Representation.h"

//Forward declaration
namespace ice
{
class GContainer;
class GContainerFactory;
} /* namespace ice */

namespace ice
{

enum TransformationOperationType
{
  DEFAULT,
  USE,
  FORMULA
};

struct TransformationOperation
{

  ~TransformationOperation()
  {
    switch (this->valueType)
    {
      case BOOL:
        delete (bool*)value;
        break;
      case BYTE:
        delete (int8_t*)value;
        break;
      case UNSIGNED_BYTE:
        delete (uint8_t*)value;
        break;
      case SHORT:
        delete (short*)value;
        break;
      case INT:
        delete (int*)value;
        break;
      case LONG:
        delete (long*)value;
        break;
      case UNSIGNED_SHORT:
        delete (unsigned short*)value;
        break;
      case UNSIGNED_INT:
        delete (unsigned int*)value;
        break;
      case UNSIGNED_LONG:
        delete (unsigned long*)value;
        break;
      case FLOAT:
        delete (float*)value;
        break;
      case DOUBLE:
        delete (double*)value;
        break;
      case STRING:
        delete (std::string*)value;
        break;
    }
  }

  TransformationOperationType type;

  int sourceIndex;
  int* sourceDimension;

  int* targetDimension;

  BasicRepresentationType valueType;
  void* value;
};

class Transformation
{
public:
  Transformation(std::shared_ptr<GContainerFactory> factory, std::string name,
                 std::shared_ptr<Representation> targetRepresentation);
  virtual ~Transformation();

  std::shared_ptr<GContainer> transform(std::shared_ptr<GContainer>* inputs);

  const std::string getName() const;
  std::shared_ptr<Representation> getTargetRepresentation();
  std::vector<TransformationOperation*>& getOperations();
  std::vector<std::shared_ptr<Representation>>& getInputs();

private:
  const std::string name;
  std::shared_ptr<Representation> targetRepresentation;
  std::vector<std::shared_ptr<Representation>> inputs;
  std::vector<TransformationOperation*> operations;
  std::shared_ptr<ice::GContainerFactory> factory;
};

} /* namespace ice */

#endif /* TRANSFORMATION_H_ */
