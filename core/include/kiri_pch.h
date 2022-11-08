/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-08 12:23:52
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-11-08 15:33:09
 * @FilePath: \Kiri2D\core\include\kiri_pch.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _KIRI_PCH_H_
#define _KIRI_PCH_H_

#pragma once

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>

#include <sstream>
#include <string>

#include <array>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <kiri_define.h>
////////////////////////////////////////////////////////////////////////////////
#if defined(DOUBLE_PRECISION) || defined(HIGH_PRECISION)
#define KIRI_DOUBLE_PRECISION
using Real = double;
#else
#define KIRI_SINGLE_PRECISION
using Real = float;
#endif

////////////////////////////////////////////////////////////////////////////////
using Int8 = int8_t;
using Int16 = int16_t;
using Int = int32_t;
using Int32 = int32_t;
using Int64 = int64_t;

////////////////////////////////////////////////////////////////////////////////
using UChar = unsigned char;
using UInt8 = uint8_t;
using UInt16 = uint16_t;
using UInt = uint32_t;
using UInt32 = uint32_t;
using UInt64 = uint64_t;

////////////////////////////////////////////////////////////////////////////////
using String = std::string;

////////////////////////////////////////////////////////////////////////////////
using PairInt8 = std::pair<Int8, Int8>;
using PairInt16 = std::pair<Int16, Int16>;
using PairInt32 = std::pair<Int, Int>;
using PairInt = std::pair<Int, Int>;
using PairInt64 = std::pair<Int64, Int64>;

////////////////////////////////////////////////////////////////////////////////
using PairUInt8 = std::pair<UInt8, UInt8>;
using PairUInt16 = std::pair<UInt16, UInt16>;
using PairUInt32 = std::pair<UInt, UInt>;
using PairUInt = std::pair<UInt, UInt>;
using PairUInt64 = std::pair<UInt64, UInt64>;

////////////////////////////////////////////////////////////////////////////////
using Pairf = std::pair<float, float>;
using Paird = std::pair<double, double>;
using PairReal = std::pair<Real, Real>;

////////////////////////////////////////////////////////////////////////////////
template <class K, class V> using Map = std::map<K, V>;
template <class K, class V> using UnSortedMap = std::unordered_map<K, V>;

template <class Type> using Vector = std::vector<Type>;
template <class Type> using Set = std::set<Type>;

////////////////////////////////////////////////////////////////////////////////
using Vec_Int8 = Vector<Int8>;
using Vec_Int16 = Vector<Int16>;
using Vec_Int = Vector<Int>;
using Vec_Int32 = Vector<Int>;
using Vec_Int64 = Vector<Int64>;

using Vec_UInt8 = Vector<UInt8>;
using Vec_UInt16 = Vector<UInt16>;
using Vec_UInt = Vector<UInt>;
using Vec_UInt32 = Vector<UInt>;
using Vec_UInt64 = Vector<UInt64>;

using Vec_Char = Vector<char>;
using Vec_UChar = Vector<UChar>;
using Vec_Float = Vector<float>;
using Vec_Double = Vector<double>;
using Vec_Real = Vector<Real>;
using Vec_String = Vector<String>;
////////////////////////////////////////////////////////////////////////////////
template <class Type> using Vec_Vec = Vector<Vector<Type>>;
template <class Type> using SharedPtr = std::shared_ptr<Type>;
template <class Type> using WeakPtr = std::weak_ptr<Type>;
template <class Type> using UniquePtr = std::unique_ptr<Type>;

////////////////////////////////////////////////////////////////////////////////
template <class Type, class UType> using IsSame = std::is_same<Type, UType>;

template <class Type> using IsSame_Float = std::is_same<Type, float>;
template <class Type> using IsSame_Double = std::is_same<Type, double>;
template <class Type> using IsSame_Int = std::is_same<Type, int>;
template <class Type> using IsSame_SizeT = std::is_same<Type, size_t>;
template <class Type> using IsSame_Bool = std::is_same<Type, bool>;

template <class Base, class Type>
constexpr auto IsInstanceOf(const std::shared_ptr<Type> ptr) {
  return std::dynamic_pointer_cast<const Base>(ptr) != nullptr;
}

////////////////////////////////////////////////////////////////////////////////
template <class T> constexpr auto MEpsilon() {
  return std::numeric_limits<T>::epsilon();
}
template <class T> constexpr auto Tiny() {
  return std::numeric_limits<T>::min();
}
template <class T> constexpr auto Huge() {
  return std::numeric_limits<T>::max();
}

template <class T> constexpr auto KIRI_SGN(T val) {
  return (T(0) < val) - (val < T(0));
}
////////////////////////////////////////////////////////////////////////////////

#include <kiri_math_mini/kiri_math_mini.h>

// TODO need support Int type
template <Int N, class Type> using VectorX = kiri_math_mini::Vector<Type, N>;

using Vector2F = kiri_math_mini::Vector2F;
using Vector2D = kiri_math_mini::Vector2D;
using Vector3F = kiri_math_mini::Vector3F;
using Vector3D = kiri_math_mini::Vector3D;
using Vector4F = kiri_math_mini::Vector4F;
using Vector4D = kiri_math_mini::Vector4D;

using Matrix2x2F = kiri_math_mini::Matrix2x2F;
using Matrix3x3F = kiri_math_mini::Matrix3x3F;
using Matrix4x4F = kiri_math_mini::Matrix4x4F;

using Vec_Vec2F = Vector<Vector2F>;
using Vec_Vec3F = Vector<Vector3F>;

template <class T> constexpr auto KIRI_PI() { return kiri_math_mini::pi<T>(); }

////////////////////////////////////////////////////////////////////////////////
template <class T> using Array1 = kiri_math_mini::Array1<T>;
template <class T> using Array2 = kiri_math_mini::Array2<T>;
template <class T> using Array3 = kiri_math_mini::Array3<T>;

template <class T> using ArrayAccessor1 = kiri_math_mini::ArrayAccessor1<T>;

template <class T>
using ConstArrayAccessor1 = kiri_math_mini::ConstArrayAccessor1<T>;

using Array1Vec3F = kiri_math_mini::Array1<Vector3F>;
using Array1Vec4F = kiri_math_mini::Array1<Vector4F>;
using Array1Mat4x4F = kiri_math_mini::Array1<Matrix4x4F>;
using Array2UI = kiri_math_mini::Array2<UInt>;
using Array2F = kiri_math_mini::Array2<float>;
using Array2D = kiri_math_mini::Array2<double>;
using Array3UI = kiri_math_mini::Array3<UInt>;
using Array3F = kiri_math_mini::Array3<float>;
using Array3D = kiri_math_mini::Array3<double>;

using BoundingBox2F = kiri_math_mini::BoundingBox2F;
using BoundingBox3F = kiri_math_mini::BoundingBox3F;

using BoundingBox2D = kiri_math_mini::BoundingBox2D;
using BoundingBox3D = kiri_math_mini::BoundingBox3D;

using Size3 = kiri_math_mini::Size3;
using Point3I = kiri_math_mini::Point3I;

////////////////////////////////////////////////////////////////////////////////
#include <random.hpp>
using Random = effolkronium::random_static;
////////////////////////////////////////////////////////////////////////////////
#include <kiri_log.h>
#include <kiri_timer.h>
////////////////////////////////////////////////////////////////////////////////
#endif