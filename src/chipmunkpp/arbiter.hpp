#pragma once

#include "body.hpp"
#include "space.hpp"

#include <chipmunk.h>

namespace cp {
	class Arbiter {
	public:
		//Arbiter(cpArbiter*);
		Arbiter(cpArbiter*, Space&);
		Body& getBodyA();
		Body& getBodyB();

    bool callWildcardBeginA();

    Float totalKineticEnergy();

	private:
		cpArbiter* arbiter;
    Space& space;
	};
}
