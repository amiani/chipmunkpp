#include "body.hpp"

namespace cp {
	Body::Body(cpFloat mass, cpFloat inertia) : body(cpBodyNew(mass, inertia)), owning(true) {
	  cpBodySetUserData(body, this);
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

	void Body::setTorque(Float torque) {
	  cpBodySetTorque(body, torque);
	}

  Float Body::getAngle() const {
    return cpBodyGetAngle(body);
  }

  void Body::setAngle(Float a) {
    cpBodySetAngle(body, a);
  }

  Float Body::getAngularVelocity() {
	  return cpBodyGetAngularVelocity(body);
	}

  Vect Body::worldToLocal(Vect point) {
    return cpBodyWorldToLocal(body, point);
  }

  void Body::applyForceAtLocalPoint(Vect force, Vect point) {
    cpBodyApplyForceAtLocalPoint(body, force, point);
  }

  void Body::setVelocityUpdateFunc(std::function<void(Body&, Vect, Float, Float)> velocityUpdate) {
	  this->velocityUpdate = velocityUpdate;
	  cpBodySetVelocityUpdateFunc(body, [](cpBody* b, cpVect g, Float d, Float dt) {
      auto self = static_cast<Body*>(cpBodyGetUserData(b));
      self->velocityUpdate(*self, g, d, dt);
	  });
	}

	void Body::updateVelocity(Vect gravity, Float damping, Float dt) {
	  cpBodyUpdateVelocity(body, gravity, damping, dt);
	}

	void Body::velocityUpdateHelper(cpBody* body, cpVect gravity, cpFloat damping, cpFloat dt) {
    //velocityUpdate(this, gravity, damping, dt);
	}

	KinematicBody::KinematicBody(cpFloat mass, cpFloat inertia) : Body(mass, inertia) {
    cpBodySetType(body, cp::KINEMATIC);
  }

  KinematicBody::KinematicBody(cpBody* body) : Body(body) {
    cpBodySetType(body, cp::KINEMATIC);
  }

  StaticBody::StaticBody() : Body(0, 0) {
    cpBodySetType(body, cp::STATIC);
  }
}