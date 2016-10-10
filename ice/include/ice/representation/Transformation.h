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
class ICEngine;
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

    if (nullptr != sourceDimension)
      delete sourceDimension;

    if (nullptr != targetDimension)
      delete targetDimension;
  }

  TransformationOperationType type;

  int sourceIndex;
  std::vector<int>* sourceDimension;

  std::vector<int>* targetDimension;

  std::string formula;
  std::string varname;
  std::map<std::string, std::pair<std::vector<int>*, int>> varmap;

  BasicRepresentationType valueType;
  void* value;
};

class Transformation
{
public:
  Transformation(std::weak_ptr<ICEngine> engine, std::string name, std::string scope,
                 std::shared_ptr<Representation> targetRepresentation = nullptr);
  Transformation(std::weak_ptr<GContainerFactory> factory, std::string name, std::string scope,
                 std::shared_ptr<Representation> targetRepresentation = nullptr);
  virtual ~Transformation();

  virtual std::shared_ptr<GContainer> transform(std::shared_ptr<GContainer>* inputs);

  const std::string getName() const;
  const std::string getScope() const;
  std::shared_ptr<Representation> getTargetRepresentation();
  void setTargetRepresentation(std::shared_ptr<Representation> representation);
  std::vector<TransformationOperation*>& getOperations();
  std::vector<std::shared_ptr<Representation>>& getInputs();
  std::string toString();
  void print();
  std::unique_ptr<std::vector<std::string>> getASPRepreentation(std::string system);

protected:
  // Converts the void pointers data from the type given by the parameter
  // type to a double value.
  double convertDouble(void *data, ice::BasicRepresentationType type);

  // Allocates memory for type and returns a void pointer pointing
  // to converted value of parameter val.
  void *convertVoid(double val, ice::BasicRepresentationType type);

protected:
  const std::string                             name;
  const std::string                             scope;
  std::shared_ptr<Representation>               targetRepresentation;
  std::vector<std::shared_ptr<Representation>>  inputs;
  std::vector<TransformationOperation*>         operations;
  std::weak_ptr<ICEngine>                       engine;
  std::weak_ptr<GContainerFactory>              factory;
};

} /* namespace ice */

#endif /* TRANSFORMATION_H_ */
