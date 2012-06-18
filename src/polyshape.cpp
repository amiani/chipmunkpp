#include "chipmunkpp/polyshape.hpp"

namespace cp {
	PolyShape::PolyShape(std::shared_ptr<Body> body, const std::vector<cpVect>& verts, Vect offset)
		: Shape(cpPolyShapeNew(*body, static_cast<int>(verts.size()), const_cast<cpVect*>(&verts[0]), offset), body) {
	}

	int PolyShape::getNumVerts() const {
		return cpPolyShapeGetNumVerts(shape);
	}

	cp::Vect PolyShape::getVert(int i) {
		return cpPolyShapeGetVert(shape, i);
	}
}