/*  This file is part of the Pinocchio automatic rigging library.
    Copyright (C) 2007 Ilya Baran (ibaran@mit.edu)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef MATHUTILS_H_INCLUDED
#define MATHUTILS_H_INCLUDED

#include <math.h>
#include <functional>
#include <string>
#include "Pinocchio.h"

#ifdef _WIN32
#pragma warning(disable:4355)
#pragma warning(disable:4996)
#pragma warning(disable:4251)
#endif //_WIN32

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

inline int ROUND(double x) { return (int)(x + 0.5); }
inline int SIGN(double x) { return (x > 0.) ? 1 : -1; }
template<class T> T SQR(const T & x) { return x * x; }
template<class T> T CUBE(const T & x) { return x * x * x; }
template<class T> T QUAD(const T & x) { return SQR(SQR(x)); }

template <class T> std::function<T(T, T)> maximum() { return [](const T& a, const T& b) {return std::max(a, b); }; }
template <class T> std::function<T(T, T)> minimum() { return [](const T& a, const T& b) {return std::min(a, b); }; }
template <class A1, class A2, class R> std::function<R(A1, A2)> myMult(){
        return [](const A1& a1, const A2& a2) {return a1 * a2; };
}
template <class A1, class A2, class R> std::function<R(A1, A2)> myDiv() {
    return [](const A1& a1, const A2& a2) {return a1 / a2; };
}

template <class T> class ident
{
public:
    T operator()(const T &v) const { return v; }
};

template<class T> class construct
{
public:
    typedef T result_type;
    template<class From> T operator()(const From &x) const { return T(x); }
};

#ifdef _WIN32
inline bool operator==(const std::string &s1, const std::string &s2) { return s1.compare(s2) == 0; }
inline bool operator<(const std::string &s1, const std::string &s2) { return s1.compare(s2) < 0; }
#endif

#endif //MATHUTILS_H_INCLUDED
