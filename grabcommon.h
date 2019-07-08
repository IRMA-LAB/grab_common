/**
 * @file grabcommon.h
 * @author Simone Comari
 * @date 08 May 2019
 * @brief This file collects common utilities which are not specifically related to any of
 * GRAB libraries.
 */

#ifndef GRABCOMMON_H
#define GRABCOMMON_H

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

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


// @cond DO_NOT_DOCUMENT
#define SETUP_ENUM_CLASS_ASSIGNMENTS(EnumStruct, EType)                                  \
  EType val;                                                                             \
  EnumStruct() {}                                                                        \
  EnumStruct(EType p_eVal) : val(p_eVal) {}                                              \
  operator EType() const { return val; }                                                 \
  void operator=(EType p_eVal) { val = p_eVal; }
// @endcond

/**
 * @brief An enum-like format to define an enum class.
 * @todo example
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
// @cond DO_NOT_DOCUMENT
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
// @endcond
/**
 * @brief A for loop macro implementation.
 * @warning It works up to 9 iterations, to add more you must extend the list above.
 */
#define FOR_EACH(action, ...)                                                            \
  GET_MACRO(__VA_ARGS__, FE_9, FE_8, FE_7, FE_6, FE_5, FE_4, FE_3, FE_2, FE_1)           \
  (action, __VA_ARGS__)

//---------------------- Generic enums -----------------------------------------------//

/**
 *@brief Return value enum
 */
enum RetVal : uint8_t
{
  OK,       /**< success */
  ECONFIG,  /**< configuration error */
  EREG,     /**< registration error */
  EACTIVE,  /**< activation error */
  EINIT,    /**< initialization error */
  EINV,     /**< invalid error */
  EINT,     /**< interrupt error */
  ETIMEOUT, /**< time-out errro */
  EFAIL     /**< generic error */
};

//---------------------- Generic functions -------------------------------------------//

/**
 * @brief Handle an error message accorging to @c errno convention and display a message.
 * @note This function immediately terminates the process with an error.
 * @param[in] en Error number.
 * @param[in] msg Message to be displayed before error description.
 * @note For further details about error number convention, click
 * <a href="http://man7.org/linux/man-pages/man3/errno.3.html">here</a>.
 */
[[noreturn]] void HandleErrorEn(const int en, const char* msg);

/**
 * @brief Print colored text to shell/console.
 * @param[in] color A letter indicating the text color. Available options are:
 * - 'r' : red
 * - 'y' : yellow
 * - 'g' : green
 * - 'm' : magenta
 * - 'c' : cyan
 * - 'b' : blue
 * - other : white (default)
 * @param[in] text Text to be colored and printed. It can contain format specifiers to be
 * added as variable arguments like in _printf()_.
 */
void PrintColor(const char color, const char* text, ...);
/**
 * @brief Print colored text to shell/console.
 * @param[in] color A letter indicating the text color. Available options are:
 * - 'r' : red
 * - 'y' : yellow
 * - 'g' : green
 * - 'm' : magenta
 * - 'c' : cyan
 * - 'b' : blue
 * - other : white (default)
 * @param[in] text Text to be colored and printed.
 * @param[in] args Additional variable arguments in case text contains format specifiers.
 */
void PrintColor(const char color, const char* text, va_list args);

template <typename T>
/**
 * @brief Return sign of a scalar value.
 * @param value A scalar value.
 * @return Sign of given scalar value.
 */
T sign(const T& value)
{
  return value >= 0 ? static_cast<T>(1) : static_cast<T>(-1);
}

/**
 * @brief Display a message colored depending on the returned value and append the value
 * outcome at the end of it.
 * @param[in] err The error of RetVal type.
 * @param[in] msg The custom message to be prepend before the error identifier.
 */
void DispRetVal(const int err, const char* msg, ...);

/**
 * @brief Get return value's description.
 * @param[in] err The error of RetVal type.
 * @return A string describing the returned value.
 */
std::string GetRetValStr(const int err);

/**
 * @brief Run a Matlab script from shell.
 * @param[in] script_location Filepath of Matlab script.
 * @param[in] display If _true_, display flags will be enabled and Matlab plots will be
 * visible during the execution, if present.
 */
void RunMatlabScript(const std::string& script_location, const bool display = false);

/*---------------------- Generic classes ------------------------*/

template <typename T>
/**
 * @brief A simple implementation of a ring buffer.
 */
class RingBuffer
{
 public:
  /**
   * @brief RingBuffer constructor.
   * @param[in] buffer_size Ring buffer size.
   */
  RingBuffer(const size_t buffer_size) { buffer_.resize(buffer_size); }
  /**
   * @brief RingBuffer constructor with scalar initializer.
   * @param[in] buffer_size Ring buffer size.
   * @param[in] element Scalar whose value will be used to fill the whole buffer.
   */
  RingBuffer(const size_t buffer_size, const T& element)
  {
    buffer_.resize(buffer_size, element);
  }
  /**
   * @brief RingBuffer constructor with vector initializer.
   * @param[in] vector Vector used to inialize buffer content and size.
   */
  RingBuffer(const std::vector<T> vector) : buffer_(vector) {}

  /**
   * @brief operator [] Dynamic indexing, that is if given index = k, then the k-th
   * element starting from current ring's head is given.
   * @param[in] index The index of the element from 0 to buffer's size - 1.
   * @return A reference to the desired element.
   */
  T& operator[](const uint64_t& index)
  {
    assert(index <= this->linear_idx_);

    return buffer_[(linear_idx_ + index + 1) % buffer_.size()];
  }
  /**
   * @brief operator [] Dynamic indexing, that is if given index = k, then the k-th
   * element starting from current ring's head is given.
   * @param[in] index The index of the element from 0 to buffer's size - 1.
   * @return A constant reference to the desired element.
   */
  const T& operator[](const uint64_t& index) const
  {
    assert(index <= this->linear_idx_);

    return buffer_[(linear_idx_ + index + 1) % buffer_.size()];
  }

  /**
   * @brief Returns a constant reference to the last element in the ring vector.
   * @return A reference to the last element in the ring vector.
   */
  T& Tail() { return buffer_[ring_idx_]; }
  /**
   * @brief Returns a constant reference to the last element in the ring vector.
   * @return A reference to the last element in the ring vector.
   */
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
  /**
   * @brief Returns a constant reference to the first element in the ring vector.
   * @return A constant reference to the first element in the ring vector.
   */
  const T& Head() const
  {
    return buffer_[(linear_idx_ == ring_idx_ | ring_idx_ == buffer_.size() - 1)
                     ? buffer_.front()
                     : buffer_[ring_idx_ + 1]];
  }

  /**
   * @brief Return raw data content of the buffer.
   * @return Raw data content of the buffer.
   */
  std::vector<T>& Data() { return buffer_; }
  /**
   * @brief Return a constant reference to raw data content of the buffer.
   * @return A constant reference to raw data content of the buffer.
   */
  const std::vector<T>& Data() const { return buffer_; }

  /**
   * @brief Check if buffer is empty.
   * @return _True_ if buffer is empty, _false_ otherwise.
   */
  bool IsEmpty() const { return linear_idx_ == 0; }

  /**
   * @brief Check if buffer is full.
   * @return _True_ if buffer is full, _false_ otherwise.
   */
  bool IsFull() const { return linear_idx_ >= (buffer_.size() - 1); }

  /**
   * @brief Return buffer size.
   * @return Buffer size.
   */
  size_t Size() const { return buffer_.size(); }

  /**
   * @brief Get current linear index.
   *
   * Linear index is a growing index which increases by 1 for every new element pushed
   * into the buffer.
   * @return Current linear index.
   */
  uint64_t GetLinearIdx() const { return linear_idx_; }

  /**
   * @brief Get ring index.
   *
   * This is a bounded index between 0 and _buffer size_ - 1, which denote the ring
   * position of the latest element pushed into he ring buffer.
   * @return Ring index.
   */
  uint64_t GetRingIdx() const { return ring_idx_; }

  /**
   * @brief Add an element to the buffer.
   * @param[in] element New element to be pushed into the buffer.
   */
  void Add(const T& element)
  {
    ring_idx_          = linear_idx_ % buffer_.size();
    buffer_[ring_idx_] = element;
    linear_idx_++;
  }

  /**
   * @brief Empty the buffer.
   */
  void Clear()
  {
    linear_idx_ = 0UL;
    ring_idx_   = 0UL;
    buffer_.clear();
  }

 private:
  std::vector<T> buffer_;
  uint64_t linear_idx_ = 0UL;
  uint64_t ring_idx_   = 0UL;
};

using RingBufferD = RingBuffer<double>; /**< Alias for double-typed ring buffer. */

#endif // GRABCOMMON_H
