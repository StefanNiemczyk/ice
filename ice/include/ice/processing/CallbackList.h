/*
 * CallbackList.h
 *
 *  Created on: May 12, 2016
 *      Author: sni
 */

#ifndef CALLBACKLIST_H_
#define CALLBACKLIST_H_

#include <mutex>
#include <vector>

#include <easylogging++.h>

namespace ice
{

template <typename T>
struct CallbackFunction
{
  virtual ~CallbackFunction() {}

  virtual void trigger(T value)
  {
    (*func)(value);
  }

  void (*func)(T);
};


template <typename T, typename C>
struct CallbackClass;

template <typename T>
struct CallbackClassBase
{
  virtual ~CallbackClassBase() {}

  virtual void trigger(T value) = 0;

  void* voidClassPtr;
};

template <typename T, typename C>
struct CallbackClass : CallbackClassBase<T>
{
  CallbackClass(C* cls, void (C::*func)(T))
  {
    this->classPtr = cls;
    this->func = func;
    this->voidClassPtr = cls;
  }

  virtual ~CallbackClass() {}

  virtual void trigger(T value)
  {
    (classPtr->*func)(value);
  }

  C* classPtr;
  void (C::*func)(T);
};

template <typename T>
class CallbackList
{
public:
  CallbackList()
  {
    _log = el::Loggers::getLogger("CallbackList");
  }

  virtual ~CallbackList()
  {
    for (auto &callback : this->callbacks)
    {
      delete callback;
    }

    for (auto &callback : this->callbacksClass)
    {
      delete callback;
    }
  }

  void trigger(T value)
  {
    std::lock_guard<std::mutex> (this->_mtx);

    for (auto &callback : this->callbacks)
    {
      callback->trigger(value);
    }

    for (auto &callback : this->callbacksClass)
    {
      callback->trigger(value);
    }
  }

  void registerCallback(void (*function)(T))
  {
    std::lock_guard<std::mutex> (this->_mtx);

    for (auto &c : this->callbacks)
    {
      if (c->func == function)
        return;
    }

    CallbackFunction<T>* c = new CallbackFunction<T>();
    c->func = function;
    this->callbacks.push_back(c);
  }

  template <typename C>
  void registerCallback(C* cls, void (C::*function)(T))
  {
    std::lock_guard<std::mutex> (this->_mtx);

    for (auto &c : this->callbacksClass)
    {
      if (this->equals(c, cls, function))
        return;
    }

    CallbackClass<T, C>* c = new CallbackClass<T, C>(cls, function);
    this->callbacksClass.push_back(c);
  }

  bool unregisterCallback(void (*function)(T))
  {
    std::lock_guard<std::mutex> (this->_mtx);

    for (int i=0; i < this->callbacks.size(); ++i)
    {
      auto &c = this->callbacks[i];
      if (c->func == function)
      {
        this->callbacks.erase(this->callbacks.begin() + i);
        return true;
      }
    }

    return false;
  }

template<typename C>
  bool unregisterCallback(C* cls, void (C::*function)(T))
  {
    std::lock_guard<std::mutex>(this->_mtx);

    for (int i = 0; i < this->callbacksClass.size(); ++i)
    {
      auto &c = this->callbacksClass[i];
      if (this->equals(c, cls, function))
      {
        this->callbacksClass.erase(this->callbacksClass.begin() + i);
        return true;
      }
    }

    return false;
  }

template <typename C>
  bool equals(CallbackClassBase<T> *cb, C* cls, void (C::*func)(T))
  {
    if (cls != cb->voidClassPtr)
      return false;

    return ( ((CallbackClass<T,C>*) cb)->func == func);
  }

private:
  std::vector<CallbackFunction<T>*>     callbacks;
  std::vector<CallbackClassBase<T>*>    callbacksClass;
  std::mutex                            _mtx;
  el::Logger                            *_log;
};

} /* namespace ice */

#endif /* CALLBACKLIST_H_ */
