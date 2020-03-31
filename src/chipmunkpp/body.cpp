#include "body.hpp"

namespace cp {
	Body::Body(cpFloat mass, cpFloat inertia) : body(cpBodyNew(mass, inertia)), owning(true) {
	}

	Body::Body(Body&& other) : body(other.body), owning(other.owning) {
		other.body = nullptr;
		other.owning = false;
	}

	Body::~Body() {
		if (owning) {
			cpBodyFree(body);
		}
	}

	Body::Body(cpBody* body) : body(body), owning(false) {
	}

	Body::operator cpBody*() const {
		return body;
	}

  void Body::setType(cpBodyType type) {
    cpBodySetType(body, type);
  }

	void Body::setPosition(Vect p) {
		cpBodySetPosition(body, p);
	}

	Vect Body::getPosition() const {
		return cpBodyGetPosition(body);
	}

	Vect Body::getVelocity() const {
		return cpBodyGetVelocity(body);
	}

	void Body::setVelocity(Vect velocity) {
		cpBodySetVelocity(body, velocity);
	}

  Float Body::getAngle() const {
    return cpBodyGetAngle(body);
  }

  void Body::setAngle(Float a) {
    cpBodySetAngle(body, a);
  }

  Vect Body::worldToLocal(Vect point) {
    return cpBodyWorldToLocal(body, point);
  }

  void Body::applyForceAtLocalPoint(Vect force, Vect point) {
    cpBodyApplyForceAtLocalPoint(body, force, point);
  }

	DataPointer Body::getUserData() const {
		return cpBodyGetUserData(body);
	}

	void Body::setUserData(DataPointer p) {
		cpBodySetUserData(body, p);
	}


	KinematicBody::KinematicBody(cpFloat mass, cpFloat inertia) : Body(mass, inertia) {
    cpBodySetType(body, cp::KINEMATIC);
  }

  KinematicBody::KinematicBody(cpBody* body) : Body(body) {
    cpBodySetType(body, cp::KINEMATIC);
  }
}