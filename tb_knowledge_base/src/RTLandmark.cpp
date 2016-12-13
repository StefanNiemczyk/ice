/*
 * RTLandmark.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: sni
 */

#include "container/RTLandmark.h"

namespace ice
{

int RTLandmark::LANDMARK_PATH = 0;
int RTLandmark::POSITION_PATH = 1;
int RTLandmark::X_COORDINATE_PATH = 0;
int RTLandmark::Y_COORDINATE_PATH = 1;
int RTLandmark::Z_COORDINATE_PATH = 2;

RTLandmark::RTLandmark(std::shared_ptr<Representation> const &representation) :
    GContainer(representation), x(0), y(0), z(0), landmark("")
{
  //
}

RTLandmark::~RTLandmark()
{
  //
}

GContainer* RTLandmark::clone()
{
  auto pos = new RTLandmark(this->representation);

  pos->landmark = this->landmark;
  pos->x = this->x;
  pos->y = this->y;
  pos->z = this->z;

  return pos;
}

void RTLandmark::print(int level, std::string dimension)
{
  // TODO
}

std::pair<BasicRepresentationType, void*> RTLandmark::getPair(std::vector<int> *indices, int index)
{
  if (indices->at(index) == LANDMARK_PATH)
  {
    return std::make_pair(BasicRepresentationType::STRING, &this->landmark);
  }
  else if (indices->at(index) == POSITION_PATH)
  {
    if (indices->at(index+1) == X_COORDINATE_PATH)
    {
      return std::make_pair(BasicRepresentationType::DOUBLE, &this->x);
    }
    else if (indices->at(index+1) == Y_COORDINATE_PATH)
    {
      return std::make_pair(BasicRepresentationType::DOUBLE, &this->y);
    }
    else if (indices->at(index+1) == Z_COORDINATE_PATH)
    {
      return std::make_pair(BasicRepresentationType::DOUBLE, &this->z);
    }
  }

  return std::make_pair(BasicRepresentationType::UNSET, nullptr);
}

void* RTLandmark::get(std::vector<int> *indices, int index)
{
  if (indices->at(index) == LANDMARK_PATH)
  {
    return &this->landmark;
  }
  else if (indices->at(index) == POSITION_PATH)
  {
    if (indices->at(index+1) == X_COORDINATE_PATH)
    {
      return &this->x;
    }
    else if (indices->at(index+1) == Y_COORDINATE_PATH)
    {
      return &this->y;
    }
    else if(indices->at(index+1) == Z_COORDINATE_PATH)
    {
      return &this->z;
    }
  }

  return nullptr;
}

bool RTLandmark::set(std::vector<int> *indices, int index, const void* value)
{
  if (indices->at(index) == LANDMARK_PATH)
  {
    this->landmark = *((std::string*)value);
  }
  else if (indices->at(index) == POSITION_PATH)
  {
    if (indices->at(index+1) == Z_COORDINATE_PATH)
    {
      this->z = *((double*)value);
      return true;
    }
    else if (indices->at(index+1) == X_COORDINATE_PATH)
    {
      this->x = *((double*)value);
      return true;
    }
    else if (indices->at(index+1) == Y_COORDINATE_PATH)
    {
      this->y = *((double*)value);
      return true;
    }
  }

  return false;
}

std::string RTLandmark::toJSON()
{
  std::stringstream ss;
  ss << "{\"http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark\":{\"http://vs.uni-kassel.de/Ice#Position\":{\"http://vs.uni-kassel.de/Ice#XCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
      << this->x
      << "},\"http://vs.uni-kassel.de/Ice#YCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
      << this->y
      << "},\"http://vs.uni-kassel.de/Ice#ZCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
      << this->z
      << "}},\"http://vs.uni-kassel.de/TurtleBot#LandmarkId\":{\"http://vs.uni-kassel.de/Ice#StringRep\":\""
      << this->landmark
      <<    "\"}}}";

  return ss.str();
}

Value RTLandmark::toJSONValue(Document &d)
{
  // TODO
  Value value;
  return value;
}

bool RTLandmark::readFromJSON(Value& value)
{
  if (false == value.IsObject())
    return false;

  for (auto it = value.MemberBegin(); it != value.MemberEnd(); ++it)
  {
    if (false == it->name.IsString() || false == it->value.IsObject())
    {
      _log->error("Payload could not be parsed: Id is not a string");
      return false;
    }
    auto name = std::string(it->name.GetString());
    auto value = it->value.GetObject();

    if (name == "http://vs.uni-kassel.de/TurtleBot#LandmarkId")
    {
      auto &num = value.MemberBegin()->value;
      if (false == num.IsString())
      {
        _log->error("Payload could not be parsed: LandmarkId is not a string value");
        return false;
      }
      this->landmark = num.GetString();
    }
    else if (name == "http://vs.uni-kassel.de/Ice#Position")
    {
      for (auto it = value.MemberBegin(); it != value.MemberEnd(); ++it)
      {
        if (false == it->name.IsString() || false == it->value.IsObject())
        {
          _log->error("Payload could not be parsed: Id is not a string");
          return false;
        }
        auto name = std::string(it->name.GetString());
        auto value = it->value.GetObject();
        if (name == "http://vs.uni-kassel.de/Ice#XCoordinate")
        {
          auto &num = value.MemberBegin()->value;
          if (false == num.IsDouble())
          {
            _log->error("Payload could not be parsed: XCoordinate is not a double value");
            return false;
          }
          this->x = num.GetDouble();
        }
        else if (name == "http://vs.uni-kassel.de/Ice#YCoordinate")
        {
          auto &num = value.MemberBegin()->value;
          if (false == num.IsDouble())
          {
            _log->error("Payload could not be parsed: YCoordinate is not a double value");
            return false;
          }
          this->y = num.GetDouble();
        }
        else if (name == "http://vs.uni-kassel.de/Ice#ZCoordinate")
        {
          auto &num = value.MemberBegin()->value;
          if (false == num.IsDouble())
          {
            _log->error("Payload could not be parsed: ZCoordinate is not a double value");
            return false;
          }
          this->z = num.GetDouble();
        }
        else
        {
          _log->error("Payload could not be parsed: '%v' is unknown", name);
          return false;
        }
      }
    }
    else
    {
      _log->error("Payload could not be parsed: '%v' is unknown", name);
      return false;
    }
  }

  return true;
}

bool RTLandmark::isGeneric()
{
  return false;
}

} /* namespace ice */
