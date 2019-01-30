/**
 * @file grabcommon.h
 * @author Simone Comari
 * @date 06 Dec 2018
 * @brief This file collects common utilities which are not specifically related to any of
 * GRAB libraries.
 */

#ifndef GRABCOMMON_H
#define GRABCOMMON_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <stdarg.h>

#include "bitfield.h"

/*---------------------- DEFINES ------------------------*/

#define ANSI_COLOR_RED "\x1b[31m"    /**< ANSI _red_ codex for colorful printings. */
#define ANSI_COLOR_GREEN "\x1b[32m"  /**< ANSI _green_ codex for colorful printings. */
#define ANSI_COLOR_YELLOW "\x1b[33m" /**< ANSI _yellow_ codex for colorful printings. */
#define ANSI_COLOR_BLUE "\x1b[34m"   /**< ANSI _blue_ codex for colorful printings. */
#define ANSI_COLOR_MAGENTA                                                               \
  "\x1b[35m"                       /**< ANSI _magenta_ codex for colorful printings.     \
                                      */
#define ANSI_COLOR_CYAN "\x1b[36m" /**< ANSI _cyan_ codex for colorful printings. */
#define ANSI_COLOR_RESET                                                                 \
  "\x1b[0m" /**< ANSI color reset codex after colorful printings. */

/*---------------------- MACROS ------------------------*/

/**
 * @brief SETUP_ENUM_CLASS_ASSIGNMENTS
 */
#define SETUP_ENUM_CLASS_ASSIGNMENTS(EnumStruct, EType)                                  \
  EType val;                                                                             \
  EnumStruct() {}                                                                        \
  EnumStruct(EType p_eVal) : val(p_eVal) {}                                              \
  operator EType() const { return val; }                                                 \
  void operator=(EType p_eVal) { val = p_eVal; }

/**
 * @brief ENUM_CLASS
 */
#define ENUM_CLASS(EName, ...)                                                           \
  struct EName                                                                           \
  {                                                                                      \
    enum Type                                                                            \
    {                                                                                    \
      __VA_ARGS__                                                                        \
    };                                                                                   \
    SETUP_ENUM_CLASS_ASSIGNMENTS(EName, Type)                                            \
  };

// Make a FOREACH macro
#define FE_1(WHAT, X) WHAT(X)
#define FE_2(WHAT, X, ...) WHAT(X) FE_1(WHAT, __VA_ARGS__)
#define FE_3(WHAT, X, ...) WHAT(X) FE_2(WHAT, __VA_ARGS__)
#define FE_4(WHAT, X, ...) WHAT(X) FE_3(WHAT, __VA_ARGS__)
#define FE_5(WHAT, X, ...) WHAT(X) FE_4(WHAT, __VA_ARGS__)
#define FE_6(WHAT, X, ...) WHAT(X) FE_5(WHAT, __VA_ARGS__)
#define FE_7(WHAT, X, ...) WHAT(X) FE_6(WHAT, __VA_ARGS__)
#define FE_8(WHAT, X, ...) WHAT(X) FE_7(WHAT, __VA_ARGS__)
#define FE_9(WHAT, X, ...) WHAT(X) FE_8(WHAT, __VA_ARGS__)
//... repeat as needed
#define GET_MACRO(_1, _2, _3, _4, _5, _6, _7, _8, _9, NAME, ...) NAME
#define FOR_EACH(action, ...)                                                            \
  GET_MACRO(__VA_ARGS__, FE_9, FE_8, FE_7, FE_6, FE_5, FE_4, FE_3, FE_2, FE_1)(action,         \
                                                                         __VA_ARGS__)

/*---------------------- Generic functions ------------------------*/

/**
 * @brief Handle an error message accorging to @c errno convention and display a message.
 * @note This function immediately terminates the process with an error.
 * @param[in] en Error number.
 * @param[in] msg Message to be displayed before error description.
 * @note For further details about error number convention, click
 * <a href="http://man7.org/linux/man-pages/man3/errno.3.html">here</a>.
 */
[[noreturn]] void HandleErrorEn(const int en, const char* msg);

void PrintColor(const char color, const char* text, ...);
void PrintColor(const char color, const char* text, va_list args);

template <typename T> T sign(const T& value)
{
  return value >= 0 ? static_cast<T>(1) : static_cast<T>(-1);
}

/*---------------------- Generic classes ------------------------*/

/**
 * RingBuffer class
 */
template <typename T> class RingBuffer
{
public:
  RingBuffer(const size_t buffer_size) { buffer_.resize(buffer_size); }
  RingBuffer(const size_t buffer_size, const T& element)
  {
    buffer_.resize(buffer_size, element);
  }
  RingBuffer(const std::vector<T> vector) : buffer_(vector) {}

  /**
   * @brief operator [] Dynamic indexing, that is if given index = k, then the k-th
   * element starting from current ring's head is given.
   * @param index The index of the element from 0 to buffer's size - 1.
   * @return A reference to the desired element.
   */
  T& operator[](const uint64_t& index)
  {
    assert(index <= this->linear_idx_);

    return buffer_[(linear_idx_ + index + 1) % buffer_.size()];
  }
  const T& operator[](const uint64_t& index) const
  {
    assert(index <= this->linear_idx_);

    return buffer_[(linear_idx_ + index + 1) % buffer_.size()];
  }

  /**
   * @brief Returns a reference to the last element in the ring vector.
   * @return A reference to the last element in the ring vector.
   */
  T& Tail() { return buffer_[ring_idx_]; }
  const T& Tail() const { return buffer_[ring_idx_]; }

  /**
   * @brief Returns a reference to the first element in the ring vector.
   * @return A reference to the first element in the ring vector.
   */
  T& Head()
  {
    return buffer_[(linear_idx_ == ring_idx_ | ring_idx_ == buffer_.size() - 1)
                     ? buffer_.front()
                     : buffer_[ring_idx_ + 1]];
  }
  const T& Head() const
  {
    return buffer_[(linear_idx_ == ring_idx_ | ring_idx_ == buffer_.size() - 1)
                     ? buffer_.front()
                     : buffer_[ring_idx_ + 1]];
  }

  /**
   * @brief Data
   * @return
   */
  std::vector<T>& Data() { return buffer_; }
  const std::vector<T>& Data() const { return buffer_; }

  /**
   * @brief IsEmpty
   * @return
   */
  bool IsEmpty() const { return linear_idx_ == 0; }

  /**
   * @brief IsFull
   * @return
   */
  bool IsFull() const { return linear_idx_ >= (buffer_.size() - 1); }

  /**
   * @brief Size
   * @return
   */
  size_t Size() const { return buffer_.size(); }

  /**
   * @brief GetLinearIdx
   * @return
   */
  uint64_t GetLinearIdx() const { return linear_idx_; }

  /**
   * @brief GetRingIdx
   * @return
   */
  uint64_t GetRingIdx() const { return ring_idx_; }

  /**
   * @brief Add
   * @param element
   */
  void Add(const T& element)
  {
    ring_idx_ = linear_idx_ % buffer_.size();
    buffer_[ring_idx_] = element;
    linear_idx_++;
  }

  /**
   * @brief Clear
   */
  void Clear()
  {
    linear_idx_ = 0UL;
    ring_idx_ = 0UL;
    buffer_.clear();
  }

private:
  std::vector<T> buffer_;
  uint64_t linear_idx_ = 0UL;
  uint64_t ring_idx_ = 0UL;
};

using RingBufferD = RingBuffer<double>;

#endif // GRABCOMMON_H
