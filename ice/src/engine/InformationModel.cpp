/*
 * Model.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include "ice/coordination/InformationModel.h"

namespace ice
{

InformationModel::InformationModel()
{
  //

}

InformationModel::~InformationModel()
{
  //
}

std::vector<std::shared_ptr<NodeDescription> >* InformationModel::getNodeDescriptions()
{
  return &this->nodeDescriptions;
}

std::vector<std::shared_ptr<StreamDescription>>* InformationModel::getStreams()
{
  return &this->streams;
}

std::vector<std::shared_ptr<StreamTemplateDescription>>* InformationModel::getStreamTemplates()
{
  return &this->streamTemplates;
}

std::shared_ptr<StreamDescription> InformationModel::getStreamByUuid(boost::uuids::uuid uuid) const
{
  for (auto stream : this->streams)
  {
    if (stream->getUuid() == uuid)
      return stream;
  }

  std::shared_ptr<StreamDescription> ptr;
  return ptr;
}

std::shared_ptr<StreamTemplateDescription> InformationModel::getStreamTemplateByUuid(boost::uuids::uuid uuid) const
{
  for (auto streamTemplate : this->streamTemplates)
  {
    if (streamTemplate->getUuid() == uuid)
      return streamTemplate;
  }

  std::shared_ptr<StreamTemplateDescription> ptr;
  return ptr;
}

} /* namespace ice */
