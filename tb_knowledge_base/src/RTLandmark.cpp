/*
 * RTLandmark.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: sni
 */

#include "container/RTLandmark.h"

#include <iomanip>
#include <limits>

namespace ice
{

void replaceAll(std::string &s, const std::string &search, const std::string &replace ) {
    for( size_t pos = 0; ; pos += replace.length() ) {
        // Locate the substring to replace
        pos = s.find( search, pos );
        if( pos == std::string::npos ) break;
        // Replace by erasing and inserting
        s.erase( pos, search.length() );
        s.insert( pos, replace );
    }
}

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
  std::string landmark = this->landmark;
  replaceAll(landmark, "\"", "\\\"");
  replaceAll(landmark, "\\", "\\");

  std::stringstream ss;
  ss << std::setprecision (std::numeric_limits<double>::digits10 + 1);
  ss << "{\"http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark\":{\"http://vs.uni-kassel.de/Ice#Position\":{\"http://vs.uni-kassel.de/Ice#XCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
      << this->x
      << "},\"http://vs.uni-kassel.de/Ice#YCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
      << this->y
      << "},\"http://vs.uni-kassel.de/Ice#ZCoordinate\":{\"http://vs.uni-kassel.de/Ice#DoubleRep\":"
      << this->z
      << "}},\"http://vs.uni-kassel.de/TurtleBot#LandmarkId\":{\"http://vs.uni-kassel.de/Ice#StringRep\":\""
      << landmark
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
