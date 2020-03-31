#include "arbiter.hpp"

namespace cp {
	//Arbiter::Arbiter(cpArbiter* a) : arbiter(a) {}
  Arbiter::Arbiter(cpArbiter* a, Space& s) : arbiter(a), space(s) {}

	Body Arbiter::getBodyA() {
		cpBody* a;
		cpBody* b;
		cpArbiterGetBodies(arbiter, &a, &b);
		return Body(a);
	}

	Body Arbiter::getBodyB() {
		cpBody* a;
		cpBody* b;
		cpArbiterGetBodies(arbiter, &a, &b);
		return Body(b);
	}

  bool Arbiter::callWildcardBeginA() {
    return cpArbiterCallWildcardBeginA(arbiter, space);
  }
}
