#ifndef REPRESENTATION_INSTANCE_H
#define REPRESENTATION_INSTANCE_H

#include <iostream>
#include <vector>

namespace ice {

class RepresentationInstance {
public:
  RepresentationInstance(Representation *rep)
  {
    this->representation = rep;
  }
  virtual ~RepresentationInstance()
  {

  }

  virtual void* get(int *indecies) = 0;

  virtual void set(int *indecies, const void* value) = 0;

  virtual RepresentationInstance* clone() = 0;

protected:
  Representation *representation;
};

class CompositeRepresentationInstance : public RepresentationInstance {
  friend class RepresentationFactory;

public:
  CompositeRepresentationInstance(Representation *rep) : RepresentationInstance(rep)
  {
    //
  }
  
  virtual ~CompositeRepresentationInstance()
  {
    //
  }

  virtual RepresentationInstance* clone()
  {
    CompositeRepresentationInstance* instance = new CompositeRepresentationInstance(this->representation);

    instance->subs = this->subs;

    return instance;
  }

  virtual void* get(int *indecies)
  {
    return this->subs.at(*indecies)->get(++indecies);
  }

  virtual void set(int *indecies, const void* value)
  {
    return this->subs.at(*indecies)->set(++indecies, value);
  }

  RepresentationInstance* sub(int index)
  {
    return this->subs.at(index);
  }

  RepresentationInstance* sub(std::string name)
  {
    int index = this->representation->mapping.at(name);
    return this->subs.at(index);
  }

private:
  std::vector<RepresentationInstance*> subs;
};


class BasicRepresentationInstance : public RepresentationInstance {
  // TODO
public:
  BasicRepresentationInstance(Representation *rep) : RepresentationInstance(rep)
  {
    //
  }

  virtual ~BasicRepresentationInstance()
  {
    //
  }

  virtual void* get(int *indecies)
  {
    return this->getRaw();
  }

  virtual void set(int *indecies, const void* value)
  {
    return this->setRaw(value);
  }

  virtual void* getRaw() = 0;
  virtual void setRaw(const void* value) = 0;
  virtual RepresentationInstance* clone() = 0;

protected:
  BasicRepresentationType type;
};

//* BoolRepresentationInstance
/**
 * Boolean representation
 */
class BoolRepresentationInstance : public BasicRepresentationInstance {
public:
  BoolRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    BoolRepresentationInstance* instance = new BoolRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((bool*) value);
  }

private:
  bool value;
};

//* ByteRepresentationInstance
/**
 * Byte representation
 */
class ByteRepresentationInstance : public BasicRepresentationInstance {
public:
  ByteRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    ByteRepresentationInstance* instance = new ByteRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((int8_t*) value);
  }

private:
  int8_t value;
};

//* UnsignedByteRepresentationInstance
/**
 * Unsigned byte representation
 */
class UnsignedByteRepresentationInstance : public BasicRepresentationInstance {
public:
  UnsignedByteRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    UnsignedByteRepresentationInstance* instance = new UnsignedByteRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((uint8_t*) value);
  }

private:
  uint8_t value;
};

//* ShortRepresentationInstance
/**
 * Short representation
 */
class ShortRepresentationInstance : public BasicRepresentationInstance {
public:
  ShortRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    ShortRepresentationInstance* instance = new ShortRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((short*) value);
  }

private:
  short value;
};

//* IntegerRepresentationInstance
/**
 * Integer representation
 */
class IntegerRepresentationInstance : public BasicRepresentationInstance {
public:
  IntegerRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    IntegerRepresentationInstance* instance = new IntegerRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((int*) value);
  }

private:
  int value;
};

//* LongRepresentationInstance
/**
 * Long representation
 */
class LongRepresentationInstance : public BasicRepresentationInstance {
public:
  LongRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    LongRepresentationInstance* instance = new LongRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((long*) value);
  }

private:
  long value;
};

//* UnsignedShortRepresentationInstance
/**
 * Unsigned short representation
 */
class UnsignedShortRepresentationInstance : public BasicRepresentationInstance {
public:
  UnsignedShortRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    UnsignedShortRepresentationInstance* instance = new UnsignedShortRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((unsigned short*) value);
  }

private:
  unsigned short value;
};

//* UnsignedIntegerRepresentationInstance
/**
 * Unsigned integer representation
 */
class UnsignedIntegerRepresentationInstance : public BasicRepresentationInstance {
public:
  UnsignedIntegerRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    UnsignedIntegerRepresentationInstance* instance = new UnsignedIntegerRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((unsigned int*) value);
  }

private:
  unsigned int value;
};

//* UnsignedLongRepresentationInstance
/**
 * unsigned long representation
 */
class UnsignedLongRepresentationInstance : public BasicRepresentationInstance {
public:
  UnsignedLongRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    UnsignedLongRepresentationInstance* instance = new UnsignedLongRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((unsigned long*) value);
  }

private:
  unsigned long value;
};

//* FloatRepresentationInstance
/**
 * Float representation
 */
class FloatRepresentationInstance : public BasicRepresentationInstance {
public:
  FloatRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    FloatRepresentationInstance* instance = new FloatRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((float*) value);
  }

private:
  float value;
};

//* DoubleRepresentationInstance
/**
 * Double representation
 */
class DoubleRepresentationInstance : public BasicRepresentationInstance {
public:
  DoubleRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    DoubleRepresentationInstance* instance = new DoubleRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((double*) value);
  }

private:
  double value;
};

//* StringRepresentationInstance
/**
 * String representation
 */
class StringRepresentationInstance : public BasicRepresentationInstance {
public:
  StringRepresentationInstance(Representation *rep) : BasicRepresentationInstance(rep)
  {
//    this->value = false;
  }

  virtual RepresentationInstance* clone()
  {
    StringRepresentationInstance* instance = new StringRepresentationInstance(this->representation);

    instance->value = this->value;

    return instance;
  }

  virtual void* getRaw()
  {
    return &this->value;
  }

  virtual void setRaw(const void* value)
  {
    this->value = *((std::string*) value);
  }

private:
  std::string value;
};

}

#endif // REPRESENTATION_INSTANCE_H
