#pragma once

#include <vector>
#include <iterator>
#include <cassert>
#include <array>

#include <RTree.h>

#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/pool/object_pool.hpp>
#include <glm/glm.hpp>

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

#include <phtree/phtree.h>

#define T2D_NAMESPACE_BEGIN namespace t2d {
#define T2D_NAMESPACE_END }

using Float = float;
using vec2 = glm::vec<2, Float, glm::packed_lowp>;

T2D_NAMESPACE_BEGIN

namespace phtree = ::improbable::phtree;

namespace debug {

}

T2D_NAMESPACE_END