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

// -----------------------------------------------------------------------------
// Fix list caster for sfmmap::Map::Landmark
// -----------------------------------------------------------------------------
using ListCasterBase =
    pybind11::detail::list_caster<std::vector<sfmmap::Map::Landmark*>,
                                  sfmmap::Map::Landmark*>;

template <>
struct type_caster<std::vector<sfmmap::Map::Landmark*>> : ListCasterBase {
  static handle cast(const std::vector<sfmmap::Map::Landmark*>& src,
                     return_value_policy, handle parent) {
    return ListCasterBase::cast(src, return_value_policy::reference, parent);
  }
  static handle cast(const std::vector<sfmmap::Map::Landmark*>* src,
                     return_value_policy pol, handle parent) {
    return cast(*src, pol, parent);
  }
};

enum IteratorType {
  ValueIterator,
  ItemIterator,
  UniquePtrValueIterator,
  UniquePtrIterator,
  RefIterator,
  RefValueIterator
};

template <typename Iterator, typename Sentinel, IteratorType it_type,
          return_value_policy Policy>
struct sfm_iterator_state {
  Iterator it;
  Sentinel end;
  bool first_or_done;
};

PYBIND11_NAMESPACE_END_(detail)

// Remaining iterator helpers unchanged — they work identically
// (no namespace fixes required beyond the Landmark pointer type above).

PYBIND11_NAMESPACE_END_(PYBIND11_NAMESPACE)
