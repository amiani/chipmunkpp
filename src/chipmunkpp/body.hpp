#pragma once

#include "vect.hpp"

#include <chipmunk.h>

namespace cp {
	/**
	 * \brief Rigid body
	 *
	 * A rigid body holds the physical properties of an object. (mass, position, rotation, velocity, etc.) It does not
	 * have a shape until you attach one or more collision shapes to it. If you’ve done physics with particles before,
	 * rigid bodies differ in that they are able to rotate. Rigid bodies generally tend to have a 1:1 correlation to
	 * sprites in a game. You should structure your game so that you use the position and rotation of the rigid body
	 * for drawing your sprite.
	 **/
	class Body {
	public:
		Body(Float mass, Float inertia);
		Body(Body&&);
		explicit Body(cpBody*);
		~Body();
		operator cpBody*() const;

    void setType(cpBodyType);

		Vect getPosition() const;
		void setPosition(Vect);

		/// Linear velocity of the center of gravity of the body.
		Vect getVelocity() const;
		void setVelocity(Vect velocity);

    Float getAngle() const;
    void setAngle(Float a);

    Vect worldToLocal(Vect point);

    void applyForceAtLocalPoint(Vect force, Vect point);

		DataPointer getUserData() const;
		void setUserData(DataPointer);
  protected:
		Body(const Body&);
		const Body& operator=(const Body&);
		cpBody* body;
		bool owning;
	};

  class KinematicBody : public Body {
  public:
    KinematicBody(Float mass, Float inertia);
		explicit KinematicBody(cpBody*);
  };

  class StaticBody : public Body {
  public:
    StaticBody();
  };
}
