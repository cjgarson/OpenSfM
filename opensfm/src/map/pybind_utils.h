#pragma once
#include <map/shot.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#ifdef PYBIND11_NAMESPACE_BEGIN
#define PYBIND11_NAMESPACE_BEGIN_ PYBIND11_NAMESPACE_BEGIN
#define PYBIND11_NAMESPACE_END_   PYBIND11_NAMESPACE_END
#else
#define PYBIND11_NAMESPACE_BEGIN_ NAMESPACE_BEGIN
#define PYBIND11_NAMESPACE_END_   NAMESPACE_END
#endif

PYBIND11_NAMESPACE_BEGIN_(PYBIND11_NAMESPACE)
PYBIND11_NAMESPACE_BEGIN_(detail)

// A minimal caster for std::vector<sfmmap::Landmark*> used in a few bindings.
using ListCasterBase =
    pybind11::detail::list_caster<std::vector<sfmmap::Landmark*>,
                                  sfmmap::Landmark*>;

template <>
struct type_caster<std::vector<sfmmap::Landmark*>> : ListCasterBase {
  static handle cast(const std::vector<sfmmap::Landmark*>& src,
                     return_value_policy /*pol*/, handle parent) {
    return ListCasterBase::cast(src, return_value_policy::reference, parent);
  }
};

PYBIND11_NAMESPACE_END_(detail)
PYBIND11_NAMESPACE_END_(PYBIND11_NAMESPACE)
