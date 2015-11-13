#ifndef REPRESENTATION_INSTANCE_H
#define REPRESENTATION_INSTANCE_H

#include <iostream>

namespace ice {

class BaseRepresentationInstance {};

class RepresentationInstance : public BaseRepresentationInstance {
public:

  RepresentationInstance(Representation *rep)
  {
    representation = rep;
  }
  
  template<typename T>
  void setValue(T val) {
  	T *v = new T;
  	*v = val;
  	data = (void*) v;
  }

  template<typename T>
  T* getValue() {
  	return reinterpret_cast<T*>(data);
  }

  void *data;
  Representation *representation;
  std::map<std::string, RepresentationInstance*> subs;
};

}

#endif // REPRESENTATION_INSTANCE_H
