/*
 * InformationSpecification.cpp
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#include <sstream>

#include "ice/information/InformationSpecification.h"

namespace ice
{

InformationSpecification::InformationSpecification(ont::entity entity, ont::entityType entityType, ont::scope scope,
                                                   ont::representation representation, ont::entity relatedEntity) :
    entity(entity), entityType(entityType), scope(scope), representation(representation), relatedEntity(relatedEntity)
{
  //
}

InformationSpecification::~InformationSpecification()
{
  // TODO Auto-generated destructor stub
}

const std::string InformationSpecification::getTypeString() const
{
  return this->typeString;
}

const ont::entity InformationSpecification::getEntity() const
{
  return this->entity;
}

const ont::entityType InformationSpecification::getEntityType() const
{
  return this->entityType;
}

const ont::scope InformationSpecification::getScope() const
{
  return this->scope;
}

const ont::representation InformationSpecification::getRepresentation() const
{
  return this->representation;
}

const ont::entity InformationSpecification::getRelatedEntity() const
{
  return this->relatedEntity;
}

void InformationSpecification::setTypeString(std::string type)
{
  this->typeString = type;
}

bool InformationSpecification::operator ==(const InformationSpecification& rhs)
{
  if (this->entity != rhs.entity)
    return false;
  if (this->entityType != rhs.entityType)
    return false;
  if (this->scope != rhs.scope)
    return false;
  if (this->representation != rhs.representation)
    return false;
  if (this->relatedEntity != rhs.relatedEntity)
    return false;

  return true;
}

bool InformationSpecification::operator ==(const std::shared_ptr<InformationSpecification> rhs)
{
  if (this->entity != rhs->entity)
    return false;
  if (this->entityType != rhs->entityType)
    return false;
  if (this->scope != rhs->scope)
    return false;
  if (this->representation != rhs->representation)
    return false;
  if (this->relatedEntity != rhs->relatedEntity)
    return false;

  return true;
}

std::string InformationSpecification::toString()
{
  std::stringstream ss;

  ss << "information(" << this->entity << "," << this->entityType << "," << this->scope << "," << this->representation
      << "," << (this->relatedEntity == "" ? "none" : this->relatedEntity) << ")";

  return ss.str();
}

} /* namespace ice */
