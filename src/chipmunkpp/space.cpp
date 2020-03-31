#include <iostream>
#include "space.hpp"
#include "body.hpp"
#include "arbiter.hpp"

#include <algorithm>
#include <cassert>

using namespace std;

namespace cp {
	Space::Space() : space(cpSpaceNew()), staticBody(make_shared<Body>(cpSpaceGetStaticBody(space))) {
	}

	Space::~Space() {
		for (auto& shape : shapes) {
			cpSpaceRemoveShape(space, *shape);
		}
		shapes.clear();
		cpSpaceFree(space);
	}

	Space::operator cpSpace*() {
		return space;
	}

	void Space::add(shared_ptr<Shape> shape) {
		cpSpaceAddShape(space, *shape);
		shapes.push_back(shape);
	}

	void Space::remove(shared_ptr<Shape> shape) {
		cpSpaceRemoveShape(space, *shape);
		shapes.erase(find(shapes.begin(), shapes.end(), shape));
	}

	Vect Space::getGravity() const {
		return cpSpaceGetGravity(space);
	}

	void Space::setGravity(const Vect& vect) {
		cpSpaceSetGravity(space, vect);
	}

	void Space::add(shared_ptr<Body> body) {
		cpSpaceAddBody(space, *body);
		bodies.push_back(body);
	}

	void Space::remove(shared_ptr<Body> body) {
		cpSpaceRemoveBody(space, *body);
		bodies.erase(find(bodies.begin(), bodies.end(), body));
	}

	void Space::step(Float t) {
		cpSpaceStep(space, t);
	}

	shared_ptr<Shape> Space::findPtr(cpShape* shape) const {
		if (!shape) {
			return shared_ptr<Shape>((Shape*)0);
		}
		auto it = find_if(shapes.begin(), shapes.end(),
			[&shape](const shared_ptr<Shape>& s){
				return *s == shape;
			});
		assert(it != shapes.end());
		return *it;
	}

	void Space::segmentQueryFunc(cpShape* shape, cpVect point, cpVect normal, cpFloat alpha, void* data) {
		auto d = reinterpret_cast<SegmentQueryData*>(data);
		d->func(d->self->findPtr(shape), alpha, normal);
	}

	void Space::segmentQuery(Vect a, Vect b, Layers layers, Group group,
	                         SegmentQueryFunc func) const {
		SegmentQueryData data = { this, func };
		cpShapeFilter filter{static_cast<cpGroup>(group),
		                     static_cast<cpBitmask>(layers),
		                     static_cast<cpBitmask>(layers)};
		cpSpaceSegmentQuery(space, a, b, 0, filter, segmentQueryFunc, &data);
	}

	shared_ptr<Shape> Space::segmentQueryFirst(Vect a, Vect b, Layers layers, Group group,
	                                           SegmentQueryInfo* const info) const {
		cpSegmentQueryInfo i;
		cpShapeFilter filter{static_cast<cpGroup>(group),
		                     static_cast<cpBitmask>(layers),
		                     static_cast<cpBitmask>(layers)};
		auto rtn = cpSpaceSegmentQueryFirst(space, a, b, 0, filter, &i);
		if (info) {
			info->t = i.alpha;
			info->n = i.normal;
		}
		return findPtr(rtn);
	}

	shared_ptr<Shape> Space::pointQueryFirst(Vect p, Layers layers, Group group) const {
		cpShapeFilter filter{static_cast<cpGroup>(group),
		                     static_cast<cpBitmask>(layers),
		                     static_cast<cpBitmask>(layers)};
		cpPointQueryInfo i;
		return findPtr(cpSpacePointQueryNearest(space, p, 100, filter, &i));
	}

	cpBool Space::helperBegin(cpArbiter* arb, cpSpace* s, void* d) {
		CollisionHandler& handler = *reinterpret_cast<CollisionHandler*>(d);
		return handler.begin(arb, handler.space);
	}

	cpBool Space::helperPreSolve(cpArbiter* arb, cpSpace* s, void* d) {
		CollisionHandler& handler = *reinterpret_cast<CollisionHandler*>(d);
		return handler.preSolve(arb, handler.space);
	}

	void Space::helperPostSolve(cpArbiter* arb, cpSpace* s, void* d) {
		CollisionHandler& handler = *reinterpret_cast<CollisionHandler*>(d);
		return handler.postSolve(arb, handler.space);
	}

	void Space::helperSeparate(cpArbiter* arb, cpSpace* s, void* d) {
		CollisionHandler& handler = *reinterpret_cast<CollisionHandler*>(d);
		return handler.separate(arb, handler.space);
	}

  cpCollisionHandler* Space::addDefaultCollisionHandler() {
    return cpSpaceAddDefaultCollisionHandler(space);
  }

	void Space::addBeginCollisionHandler(CollisionType a, CollisionType b, std::function<int(Arbiter, Space&)> begin) {
    auto pair = collisionHandlers.emplace(std::make_pair(a, b), std::make_unique<CollisionHandler>(a, b, *this));
    auto& handler = pair.first->second;
    handler->begin = begin;
    auto cpHandler = cpSpaceAddCollisionHandler(handler->space, a, b);
    cpHandler->beginFunc = helperBegin;
  }

  void Space::addBeginCollisionHandler(CollisionType t, std::function<int(Arbiter, Space&)> begin) {
    auto pair = wildcardHandlers.emplace(t, std::make_unique<CollisionHandler>(t, *this));
    auto& handler = pair.first->second;
    handler->begin = begin;
    auto cpWildcard = cpSpaceAddWildcardHandler(space, t);
    cpWildcard->beginFunc = helperBegin;
  }

	void Space::addPreSolveCollisionHandler(CollisionType a, CollisionType b, std::function<int(Arbiter, Space&)> preSolve) {
    auto pair = collisionHandlers.emplace(std::make_pair(a, b), std::make_unique<CollisionHandler>(a, b, *this));
    auto& handler = pair.first->second;
    handler->preSolve = preSolve;
    auto cpHandler = cpSpaceAddCollisionHandler(handler->space, a, b);
    cpHandler->preSolveFunc = helperPreSolve;
  }

  void Space::addPreSolveCollisionHandler(CollisionType t, std::function<int(Arbiter, Space&)> preSolve) {
    auto pair = wildcardHandlers.emplace(t, std::make_unique<CollisionHandler>(t, *this));
    auto& handler = pair.first->second;
    handler->preSolve = preSolve;
    auto cpHandler = cpSpaceAddWildcardHandler(handler->space, t);
    cpHandler->preSolveFunc = helperPreSolve;
  }

	void Space::addPostSolveCollisionHandler(CollisionType a, CollisionType b, std::function<int(Arbiter, Space&)> postSolve) {
    auto pair = collisionHandlers.emplace(std::make_pair(a, b), std::make_unique<CollisionHandler>(a, b, *this));
    auto& handler = pair.first->second;
    handler->postSolve = postSolve;
    auto cpHandler = cpSpaceAddCollisionHandler(handler->space, a, b);
    cpHandler->postSolveFunc = helperPostSolve;
  }

  void Space::addPostSolveCollisionHandler(CollisionType t, std::function<int(Arbiter, Space&)> postSolve) {
    auto pair = wildcardHandlers.emplace(t, std::make_unique<CollisionHandler>(t, *this));
    auto& handler = pair.first->second;
    handler->postSolve = postSolve;
    auto cpHandler = cpSpaceAddWildcardHandler(handler->space, t);
    cpHandler->postSolveFunc = helperPostSolve;
  }

	void Space::addSeparateCollisionHandler(CollisionType a, CollisionType b, std::function<int(Arbiter, Space&)> separate) {
    auto pair = collisionHandlers.emplace(std::make_pair(a, b), std::make_unique<CollisionHandler>(a, b, *this));
    auto& handler = pair.first->second;
    handler->separate = separate;
    auto cpHandler = cpSpaceAddCollisionHandler(handler->space, a, b);
    cpHandler->separateFunc = helperSeparate;
  }

  void Space::addSeparateCollisionHandler(CollisionType t, std::function<int(Arbiter, Space&)> separate) {
    auto pair = wildcardHandlers.emplace(t, std::make_unique<CollisionHandler>(t, *this));
    auto& handler = pair.first->second;
    handler->separate = separate;
    auto cpHandler = cpSpaceAddWildcardHandler(handler->space, t);
    cpHandler->separateFunc = helperSeparate;
  }

  Space::CollisionHandler::CollisionHandler(CollisionType a, CollisionType b, Space& s) : space(s) {
    auto handler = cpSpaceAddCollisionHandler(s, a, b);
    handler->userData = this;
  }

  Space::CollisionHandler::CollisionHandler(CollisionType t, Space& s) : space(s) {
    auto handler = cpSpaceAddWildcardHandler(s, t);
    handler->userData = this;
  }
/*
  Space::CollisionHandler::CollisionHandler(
    CollisionType a,
    CollisionType b,
    std::function<int(Arbiter, Space&)> begin,
    std::function<int(Arbiter, Space&)> preSolve,
    std::function<void(Arbiter, Space&)> postSolve,
    std::function<void(Arbiter, Space&)> separate,
    Space& space) : begin(begin), preSolve(preSolve), postSolve(postSolve), separate(separate), space(space) {
    }
  
  void Space::CollisionHandler::onBegin(std::function<int(Arbiter, Space&)> begin) {
    begin = begin;
    auto handler = cpSpaceAddCollisionHandler(space, a, b);
    handler->beginFunc = helperBegin;
  }
  */
}