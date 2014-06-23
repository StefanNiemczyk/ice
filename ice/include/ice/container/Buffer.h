/*
 * Buffer.h
 *
 *  Created on: May 21, 2014
 *      Author: sni
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <memory>
#include <mutex>

namespace ice
{

//* Buffer
/**
 * Buffer of shared_ptr of a given data type. Thread safety can be activated if required.
 *
 */
template<typename T>
  /*!
   * \brief Default constructor.
   *
   * Default constructor.
   *
   * \param bufferSize Number of elements which can be stored within the buffer
   */
  class Buffer
  {
  public:
    inline Buffer(const int bufferSize, const bool threadSafe)
    {
      this->bufferSize = bufferSize;
      this->threadSafe = threadSafe;
      this->index = -1;
      this->size = 0;
      this->ringBuffer = std::unique_ptr<std::shared_ptr<T>[]>(new std::shared_ptr<T>[this->bufferSize]);
    }

    /*!
     * \brief Default destructor.
     *
     * Default destructor.
     */
    inline virtual ~Buffer()
    {
      //nothing to do
    }

    /*!
     * \brief Returns and removes the last element added to the buffer or NULL if no exists.
     *
     * Returns the last element which was added to the buffer or NULL if no element exists.
     * Afterwards, the returned element will be removed from the buffer.
     */
    std::shared_ptr<T> popLast()
    {
      std::shared_ptr<T> ptr;

      if (this->threadSafe)
        std::lock_guard<std::mutex> guard(mtx_);

      if (this->size <= 0)
      {
        return ptr;
      }

      ptr = this->ringBuffer[this->index];

      //remove element from buffer
      this->index = (this->index - 1 + this->bufferSize) % this->bufferSize;

      --this->size;

      return ptr;
    }

    /*!
     * \brief Returns and removes the oldest element added to the buffer or NULL if no exists.
     *
     * Returns the oldest element which was added to the buffer or NULL if no element exists.
     * Afterwards, the returned element will be removed from the buffer.
     */
    std::shared_ptr<T> popFirst()
    {
      std::shared_ptr<T> ptr;

      if (this->threadSafe)
        std::lock_guard<std::mutex> guard(mtx_);

      if (this->size <= 0)
      {
        return ptr;
      }

      int index = (this->index - this->size + 1 + this->bufferSize) % this->bufferSize;

      ptr = this->ringBuffer[index];

      //remove element from buffer
      --this->size;

      return ptr;
    }

    /*!
     * \brief Returns the n-th last element added to the buffer or NULL if no exists.
     *
     * Returns the n-th last element added to the buffer. Returns NULL if the n-th last
     * element does not exist.
     *
     * \param n The n-th last element, 0 will return the newest one.
     */
    /*std::shared_ptr<T> getLast(int n)
     { UNTESTED!
     std::shared_ptr<T> ptr;
     int index;
     int index2;

     if (this->threadSafe)
     std::lock_guard<std::mutex> guard(mtx_);

     if (this->size >= n)
     {
     return ptr;
     }

     index = (this->index - n) % this->bufferSize;

     ptr = this->ringBuffer[index];

     //remove element from buffer
     if (this->size / 2 < n)
     {
     for (int i = n - 1; i >= 0; --i)
     {
     index2 = (this->index - i) % this->bufferSize;
     this->ringBuffer[index] = this->ringBuffer[index2];

     index = index2;
     }

     this->index = (this->index - 1 + this->bufferSize) % this->bufferSize;
     }
     else
     {
     for (int i = n + 1; i < this->size; ++i)
     {
     index2 = (this->index - i) % this->bufferSize;
     this->ringBuffer[index] = this->ringBuffer[index2];

     index = index2;
     }
     }

     --this->size;

     return ptr;
     }*/

    /*!
     * \brief Adds a new element to the ring buffer.
     *
     * Adds a new element to the ring buffer. If the buffer is full, the removed element
     * will be returned, otherwise a null pointer is returned.
     *
     * \param element The element to add.
     */
    std::shared_ptr<T> add(std::shared_ptr<T> element)
    {
      std::shared_ptr<T> ptr;

      if (this->threadSafe)
        std::lock_guard<std::mutex> guard(mtx_);

      this->index = (this->index + 1) % this->bufferSize;

      if (this->size == this->bufferSize)
      {
        ptr = this->ringBuffer[this->index];
      }
      else
      {
        ++this->size;
      }

      this->ringBuffer[this->index] = element;

      return ptr;
    }

    /*!
     * \brief Return the current buffer size.
     *
     * Return the current buffer size.
     */
    const int getSize() const
    {
      return this->size;
    }

    /*!
     * \brief Return the buffer size.
     *
     * Return the buffer size.
     */
    const int getBufferSize() const
    {
      return this->bufferSize;
    }

    /*!
     * \brief Returns true if the buffer is empty, false otherwise.
     *
     * Returns true if the buffer is empty, false otherwise.
     */
    const bool isEmpty() const
    {
      return this->size == 0;
    }

    /*!
     * \brief Returns true if the buffer is thread safe, else false.
     *
     * Returns true if the buffer is thread safe, else false.
     */
    const bool isThreadSafe() const
    {
      return this->threadSafe;
    }

    /*!
     * \brief Clears the buffer.
     *
     * Clears the buffer. If cleanBuffer is false only the index structure is reseted, but the
     * buffer still exists (old elements are not accessible). If cleanBuffer is true the
     * pointers from the buffer are cleared as well.
     *
     * \param cleanBuffer True to clear the buffer.
     */
    int clear(bool cleanBuffer)
    {
      if (this->threadSafe)
        std::lock_guard<std::mutex> guard(mtx_);

      this->index = -1;

      if (false == cleanBuffer)
        return 0;

      T ptr;
      for (int i = 0; i < this->bufferSize; ++i)
        this->ringBuffer[i] = ptr;

      return 0;
    }

    /*!
     *\brief Returns the type_info of the used template type.
     *
     * Returns the type_info of the used template type.
     */
    const std::type_info* getTypeInfo() const
    {
      return &typeid(T);
    }

  private:
    std::unique_ptr<std::shared_ptr<T>[]> ringBuffer; /**< Ring buffer of elements */
    int bufferSize; /**< number of stored elements */
    int index; /**< Current index of the last added element */
    int size; /**< Current size of the buffer */
    bool threadSafe; /**< True if access should be thread safe, else false */
    std::mutex mtx_; /**< mutex */
  };

} /* namespace ice */

#endif /* BUFFER_H_ */
