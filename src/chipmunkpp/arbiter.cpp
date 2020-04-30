#include "arbiter.hpp"

namespace cp {
	//Arbiter::Arbiter(cpArbiter* a) : arbiter(a) {}
  Arbiter::Arbiter(cpArbiter* a, Space& s) : arbiter(a), space(s) {}

	Body& Arbiter::getBodyA() {
		cpBody* a;
		cpBody* b;
		cpArbiterGetBodies(arbiter, &a, &b);
		return *static_cast<Body*>(cpBodyGetUserData(a));
	}

	Body& Arbiter::getBodyB() {
		cpBody* a;
		cpBody* b;
		cpArbiterGetBodies(arbiter, &a, &b);
    return *static_cast<Body*>(cpBodyGetUserData(b));
	}

  bool Arbiter::callWildcardBeginA() {
    return cpArbiterCallWildcardBeginA(arbiter, space);
  }

  Float Arbiter::totalKineticEnergy() {
    return cpArbiterTotalKE(arbiter);
  }
}
