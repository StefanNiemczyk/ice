/*
 * NodeDescription.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include "ice/processing/NodeDescription.h"

#include <sstream>

namespace ice
{

NodeDescription::NodeDescription(const NodeType type, const std::string className, const std::string name,
                                 const ont::entity entity, const ont::entity entityRelated) :
    type(type), className(className), name(name), entity(entity), entityRelated(entityRelated)
{
  //
}

NodeDescription::~NodeDescription()
{
  //
}

const std::string& NodeDescription::getClassName() const
{
  return this->className;
}

NodeType NodeDescription::getType() const
{
  return this->type;
}

ont::entity NodeDescription::getEntity() const
{
  return this->entity;
}


ont::entityType NodeDescription::getEntityType() const
{
  return this->entityType;
}

void NodeDescription::setEntityType(ont::entityType &type)
{
  this->entityType = type;
}

ont::entity NodeDescription::getEntityRelated() const
{
  return this->entityRelated;
}

std::string NodeDescription::getName() const
{
  return this->name;
}

std::string NodeDescription::getSource() const
{
  return this->source;
}

void NodeDescription::setSource(std::string source)
{
  this->source = source;
}

std::string NodeDescription::toString()
{
  std::stringstream ss;

  ss << "nodeDescription(" << this->name << ",";
  ss << this->className << ",";
  ss << this->source << ",";
  ss << this->entity << ",";
  switch (this->type)
  {
    case (NodeType::PROCESSING):
      ss << "Processing";
      break;
    case (NodeType::SOURCE):
      ss << "Source";
      break;
    case (NodeType::TRANSFORMATION):
      ss << "Transformation";
      break;
    case (NodeType::SET):
      ss << "Set";
      break;
  }
  ss << ")";

  return ss.str();
}

} /* namespace ice */
