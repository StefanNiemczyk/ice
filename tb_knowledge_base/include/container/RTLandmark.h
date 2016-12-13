/*
 * RTLandmark.h
 *
 *  Created on: Dec 13, 2016
 *      Author: sni
 */

#ifndef INCLUDE_CONTAINER_RTLANDMARK_H_
#define INCLUDE_CONTAINER_RTLANDMARK_H_

#include <ice/representation/GContainer.h>

namespace ice
{

class RTLandmark : public GContainer
{
private:
  static int LANDMARK_PATH, POSITION_PATH;
  static int X_COORDINATE_PATH, Y_COORDINATE_PATH, Z_COORDINATE_PATH;

public:
  RTLandmark(std::shared_ptr<Representation> const &representation);
  virtual ~RTLandmark();

  virtual GContainer* clone();
  virtual void print(int level, std::string dimension = "");
  virtual std::pair<BasicRepresentationType, void*> getPair(std::vector<int> *indices, int index);
  virtual void* get(std::vector<int> *indices, int index);

  virtual bool set(std::vector<int> *indices, const void* value) {
          return this->set(indices, 0, value);
  }
  virtual bool set(std::vector<int> *indices, int index, const void* value);
  virtual std::string toJSON();
  virtual Value toJSONValue(Document &d);

  virtual bool readFromJSON(Value& value);
  virtual bool isGeneric();

public:
  std::string landmark;
  double x;
  double y;
  double z;
};

} /* namespace ice */

#endif /* INCLUDE_CONTAINER_RTLANDMARK_H_ */
