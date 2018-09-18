/**
 * @file grabcommon.h
 * @author Simone Comari
 * @date 18 Sep 2018
 * @brief This file collects common utilities which are not specifically related to any of
 * GRAB
 * libraries.
 */

#ifndef GRABCOMMON_H
#define GRABCOMMON_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <bitset>
#include <cstdint>
#include <type_traits>

#define ANSI_COLOR_RED "\x1b[31m"     /**< ANSI _red_ codex for colorful printings. */
#define ANSI_COLOR_GREEN "\x1b[32m"   /**< ANSI _green_ codex for colorful printings. */
#define ANSI_COLOR_YELLOW "\x1b[33m"  /**< ANSI _yellow_ codex for colorful printings. */
#define ANSI_COLOR_BLUE "\x1b[34m"    /**< ANSI _blue_ codex for colorful printings. */
#define ANSI_COLOR_MAGENTA "\x1b[35m" /**< ANSI _magenta_ codex for colorful printings.  \
                                         */
#define ANSI_COLOR_CYAN "\x1b[36m"    /**< ANSI _cyan_ codex for colorful printings. */
#define ANSI_COLOR_RESET                                                                 \
  "\x1b[0m" /**< ANSI color reset codex after colorful printings. */

/**
 * @brief Handle an error message accorging to @c errno convention and display a message.
 * @note This function immediately terminates the process with an error.
 * @param[in] en Error number.
 * @param[in] msg Message to be displayed before error description.
 * @note For further details about error number convention, click
 * <a href="http://man7.org/linux/man-pages/man3/errno.3.html">here</a>.
 */
[[noreturn]] inline void HandleErrorEn(const int en, const char* msg)
{
  errno = en;
  perror(msg);
  exit(EXIT_FAILURE);
}

/********************************************
 ********* Prototypes and Typedefs *********
 ********************************************/
template <typename T> class Bitfield;

using Bitfield8 = Bitfield<uint8_t>;   /**< 8-bit Bitfield. */
using Bitfield16 = Bitfield<uint16_t>; /**< 16-bit Bitfield. */
using Bitfield32 = Bitfield<uint32_t>; /**< 32-bit Bitfield. */
using Bitfield64 = Bitfield<uint64_t>; /**< 64-bit Bitfield. */

/********************************************
 ************ Class Declaration *************
 ********************************************/

/* Since std::conditional requires both arguments to be true, we need a little wrapper
 * to allow evaluation of underlying_type for non-enum types.
 *
 * If T is an enum, we get the specialized templatization calling the underlying type.
 * If T is not an enum, the default version is called with type == void.
 */
template <typename T, typename = typename std::is_enum<T>::type>
/**
 * @brief The safe_underlying_type struct
 */
struct safe_underlying_type
{
  using type = void; /**< ... */
};

/**
 * @brief templated safe_underlying_type
 */
template <typename T> struct safe_underlying_type<T, std::true_type>
{
  using type = typename std::underlying_type<T>::type;  /**< ... */
};

/**
 * @brief Wrap bitset class to have safe bitfields of full integral type unsigned
 *integrals
 * and enums with unsigned integrals as underlying type.
 *
 * The individual bits are accessed via their position (0, 1, 2, 3) in the case of
 * unsigned integral types, but have to be accessed via enum elements for the
 * enum case. This automatically ensures enum validity.
 * If more types will be wrapped, we should create each case as subclass.
 *
 * Examples:
 * @code{.cpp}
 * Bitfield<uint8_t> b1; // ok
 * Bitfield<int> b2;     // error: only unsigned types supported
 *
 * enum class E1 : uint16_t { A = 0, B, C, D };
 * enum class E2 : int { A = 0, B, C, D };
 * enum class E3 { A = 0, B, C, D };
 *
 * Bitfield<E1> b3;      // ok
 * Bitfield<E2> b4;      // error: underlying type is signed
 * Bitfield<E3> b5;      // error: default underlying type will most likely be
 *                       // signed int
 * @endcode
 *
 */
template <typename T> class Bitfield
{
  // Type checks at compile time, but only when the type is actually used (or inherited).
  // However, this approach allows more readable error feedback.
  static_assert((std::is_integral<T>::value && std::is_unsigned<T>::value) ||
                  (std::is_enum<T>::value &&
                   std::is_unsigned<typename safe_underlying_type<T>::type>::value),
                "Bitfield type need to be unsigned integers or enums with unsigned "
                "underlying type");

public:
  /** Type to use to access positions. */
  using PosType = typename std::conditional<std::is_enum<T>::value, T, size_t>::type;
  /** Underlying type used to define bitset. */
  using UnderlyingBitsetType =
    typename std::conditional<std::is_enum<T>::value,
                              typename safe_underlying_type<T>::type, T>::type;

  // Definition up here for prototypes
  const static size_t bitset_size = (8 * (sizeof(UnderlyingBitsetType)));  /**< ... */

  // operators
  /**
   * @brief For b[i]
   * @param pos
   * @return
   */
  constexpr bool operator[](const PosType pos) const;
  /**
   * @brief For b[i] as l value
   * @param pos
   * @return
   */
  typename std::bitset<Bitfield<T>::bitset_size>::reference
  operator[](const PosType pos);

  /** Set all bits to true. */
  void SetAllTrue();

  /**
   * @brief Set particular bit in bitfield.
   *
   * Value is set to 1 by default, but can be used for clearing too.
   * @param [in] pos bit within bitfield to set.
   * @param [in] value value to which bit should be set, default is true.
   */
  void Set(const PosType pos, const bool value = true);

  /**
   * @brief Set all bits in bitfield to 0.
   */
  void ClearAll();

  /**
   * Set particular bit in bitfield to 0.
   * @param [in] pos bit within bitfield to set.
   */
  void Clear(const PosType pos);

  /**
   * Flip particular bit in bitfield.
   *
   * If bit was 1, set it to 0. If it was 0, set it to 1.
   * @param [in] pos bit within bitfield to set.
   */
  void Flip(const PosType pos);

  /**
   * @brief Check value of particular bit in bitfield.
   *
   * Value is set to true by default, but can be used for clearing too
   * @param [in] pos bit within bitfield to set.
   * @return value of bit, or -1 if bitfield does not contain bit (because it is
   * too small).
   */
  bool CheckBit(const PosType pos) const;

  /**
   * Check how many bits are true.
   * @return size_t number of bits that are true
   */
  size_t Count() const;

  /** Check if any bits are on. */
  bool AnyOn() const;

  /** Check if any bits are off. */
  bool AnyOff() const;

  /** Return whole bitset. */
  typename std::bitset<Bitfield<T>::bitset_size> GetBitset() const;

  /**
   * Return bitfield with bitset cast to underlying type.
   *
   * For unsigned integral types T, this will be of type T.
   * For enums or enum classes E, this will be of std::underlying_type<E>::type.
   */
  typename Bitfield<T>::UnderlyingBitsetType GetBitfield() const;

  /**
   * Set whole bitset.
   * @param [in] bitset desired bitset of same size.
   */
  void SetBitset(const typename std::bitset<bitset_size>& bitset);

  /**
   * Set whole bitset.
   * @param [in] bitset_value type (e.g. number) to be interpreted as bitset,
   * needs to be smaller or equal in size to current bitset type size.
   */
  template <typename U> void SetBitset(const U& bitset_value);

  /** Set whole bitfield.
   * @param [in] bitfield bitfield of same type from which to extract bitset.
   */
  void SetBitfield(const Bitfield<T>& bitfield);

private:
  std::bitset<bitset_size> bitset_;
};

/********************************************
 ************ Template Definition ***********
 ********************************************/
template <typename T> constexpr bool Bitfield<T>::operator[](const PosType pos) const
{
  return bitset_[static_cast<size_t>(pos)];
} // for b[i]

template <typename T>
typename std::bitset<Bitfield<T>::bitset_size>::reference Bitfield<T>::
operator[](const PosType pos)
{
  return bitset_[static_cast<size_t>(pos)];
} // for b[i] as l value

template <typename T> void Bitfield<T>::SetAllTrue() { bitset_.set(); }

template <typename T>
void Bitfield<T>::Set(const PosType pos, const bool value /*= true*/)
{
  bitset_.set(static_cast<size_t>(pos), value);
}

template <typename T> void Bitfield<T>::ClearAll() { bitset_.reset(); }

template <typename T> void Bitfield<T>::Clear(const PosType pos)
{
  bitset_.reset(static_cast<size_t>(pos));
}

template <typename T> void Bitfield<T>::Flip(const PosType pos)
{
  bitset_.flip(static_cast<size_t>(pos));
}

template <typename T> bool Bitfield<T>::CheckBit(const PosType pos) const
{
  return bitset_.test(static_cast<size_t>(pos));
}

template <typename T> size_t Bitfield<T>::Count() const { return bitset_.count(); }

template <typename T> bool Bitfield<T>::AnyOn() const { return bitset_.any(); }

template <typename T> bool Bitfield<T>::AnyOff() const { return bitset_.all(); }

template <typename T>
typename std::bitset<Bitfield<T>::bitset_size> Bitfield<T>::GetBitset() const
{
  return bitset_;
}

template <typename T>
typename Bitfield<T>::UnderlyingBitsetType Bitfield<T>::GetBitfield() const
{
  return static_cast<UnderlyingBitsetType>(bitset_.to_ulong());
}

template <typename T>
void Bitfield<T>::SetBitset(const typename std::bitset<bitset_size>& bitset)
{
  this->bitset_ = bitset;
}

template <typename T>
template <typename U>
void Bitfield<T>::SetBitset(const U& bitset_value)
{
  // Prevent lossy casting. If desired, cast before passing.
  if (this->bitset_size < 8 * sizeof(U))
  {
    return;
  }
  this->SetBitset(std::bitset<bitset_size>(bitset_value));
}

template <typename T> void Bitfield<T>::SetBitfield(const Bitfield<T>& bitfield)
{
  this->bitset_ = bitfield.GetBitset();
}

#endif // GRABCOMMON_H
