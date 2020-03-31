#pragma once

#include "shape.hpp"
#include "vect.hpp"
#include "memory.hpp"

#include <vector>
#include <map>

namespace cp {
	typedef std::function<void(std::shared_ptr<Shape>, Float, Vect)> SegmentQueryFunc;

	class Body;
	class Arbiter;

	/// Basic unit of simulation
	class Space {
	public:
		Space();
		explicit Space(cpSpace*);
		~Space();
		operator cpSpace*();
		void add(std::shared_ptr<Shape>);
		void add(std::shared_ptr<Body>);
		void remove(std::shared_ptr<Shape>);
		void remove(std::shared_ptr<Body>);

		/// Global gravity applied to the space. Defaults to cp::Vect(0, 0). Can be overridden on a per body basis by
		/// writing custom integration functions.
		Vect getGravity() const;
		void setGravity(const Vect&);

    Float getDamping();
    void setDamping(const Float);

		void step(Float);
		void segmentQuery(Vect a, Vect b, Layers, Group, SegmentQueryFunc) const;
		std::shared_ptr<Shape> segmentQueryFirst(Vect a, Vect b, Layers, Group, SegmentQueryInfo* = nullptr) const;
		std::shared_ptr<Shape> pointQueryFirst(Vect p, Layers, Group) const;

		void addBeginCollisionHandler(CollisionType a, CollisionType b, std::function<int(Arbiter, Space&)> begin);
		void addBeginCollisionHandler(CollisionType t, std::function<int(Arbiter, Space&)> begin);
		void addPreSolveCollisionHandler(CollisionType a, CollisionType b, std::function<int(Arbiter, Space&)> preSolve);
		void addPreSolveCollisionHandler(CollisionType t, std::function<int(Arbiter, Space&)> preSolve);
		void addPostSolveCollisionHandler(CollisionType a, CollisionType b, std::function<void(Arbiter, Space&)> postSolve);
		void addPostSolveCollisionHandler(CollisionType t, std::function<void(Arbiter, Space&)> postSolve);
		void addSeparateCollisionHandler(CollisionType a, CollisionType b, std::function<void(Arbiter, Space&)> separate);
		void addSeparateCollisionHandler(CollisionType t, std::function<void(Arbiter, Space&)> separate);
    cpCollisionHandler* addDefaultCollisionHandler();

	private:
		Space(const Space&);
		const Space& operator=(const Space&);
		static void segmentQueryFunc(cpShape*, cpVect, cpVect, cpFloat, void*);
		std::shared_ptr<Shape> findPtr(cpShape*) const;

		cpSpace* space;
		std::vector<std::shared_ptr<Shape>> shapes;
		std::vector<std::shared_ptr<Body>> bodies;

		struct SegmentQueryData {
			const Space* const self;
			SegmentQueryFunc& func;
		};

		struct CollisionHandler {
      CollisionHandler(CollisionType a, CollisionType b, Space& s);
      CollisionHandler(CollisionType t, Space& s);
      std::function<int(Arbiter, Space&)> begin;
      std::function<int(Arbiter, Space&)> preSolve;
      std::function<void(Arbiter, Space&)> postSolve;
      std::function<void(Arbiter, Space&)> separate;
      Space& space;
		};

		std::map<std::pair<CollisionType, CollisionType>, std::unique_ptr<CollisionHandler>> collisionHandlers;
    std::map<CollisionType, std::unique_ptr<CollisionHandler>> wildcardHandlers;

		static cpBool helperBegin(cpArbiter* arb, cpSpace* s, void* d);
		static cpBool helperPreSolve(cpArbiter* arb, cpSpace* s, void* d);
		static void helperPostSolve(cpArbiter* arb, cpSpace* s, void* d);
		static void helperSeparate(cpArbiter* arb, cpSpace* s, void* d);
	public:
		std::shared_ptr<Body> staticBody;
	};
}
