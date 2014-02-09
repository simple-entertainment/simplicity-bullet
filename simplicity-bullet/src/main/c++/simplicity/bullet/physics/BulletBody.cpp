/*
 * Copyright © 2014 Simple Entertainment Limited
 *
 * This file is part of The Simplicity Engine.
 *
 * The Simplicity Engine is free software: you can redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * The Simplicity Engine is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with The Simplicity Engine. If not, see
 * <http://www.gnu.org/licenses/>.
 */
#include <simplicity/math/MathFunctions.h>
#include <simplicity/model/Mesh.h>
#include <simplicity/model/Plane.h>
#include <simplicity/model/shape/Box.h>
#include <simplicity/model/shape/Cube.h>
#include <simplicity/model/shape/Sphere.h>

#include "../math/BulletMatrix.h"
#include "../math/BulletVector.h"
#include "BulletBody.h"

using namespace std;

namespace simplicity
{
	namespace bullet
	{
		BulletBody::BulletBody(const Material& material, Model* model, const Matrix44& transform, bool dynamic) :
			body(NULL),
			bulletModel(NULL),
			linearAcceleration(0.0f, 0.0f, 0.0f),
			linearVelocity(0.0f, 0.0f, 0.0f),
			material(material),
			model(model),
			motionState(NULL),
			transform(transform)
		{
			Box* box = dynamic_cast<Box*>(model);
			if (box != NULL)
			{
				bulletModel = new btBoxShape(btVector3(box->getHalfXLength(), box->getHalfYLength(),
						box->getHalfZLength()));
			}
			Cube* cube = dynamic_cast<Cube*>(model);
			if (cube != NULL)
			{
				bulletModel = new btBoxShape(btVector3(cube->getHalfEdgeLength(), cube->getHalfEdgeLength(),
					cube->getHalfEdgeLength()));
			}
			Mesh* mesh = dynamic_cast<Mesh*>(model);
			if (mesh != NULL)
			{
				if (dynamic)
				{
					vector<btVector3> points(mesh->getIndices().size());

					vector<unsigned int> indices = mesh->getIndices();
					vector<Vertex> vertices = mesh->getVertices();
					for (unsigned int index = 0; index < indices.size(); index += 3)
					{
						points[index] = BulletVector::toBtVector3(vertices[indices[index]].position);
						points[index + 1] = BulletVector::toBtVector3(vertices[indices[index + 1]].position);
						points[index + 2] = BulletVector::toBtVector3(vertices[indices[index + 2]].position);

					}

					bulletModel = new btConvexHullShape((btScalar*) points.data(), points.size());
				}
				else
				{
					btTriangleMesh* meshData = new btTriangleMesh;

					vector<unsigned int> indices = mesh->getIndices();
					vector<Vertex> vertices = mesh->getVertices();
					for (unsigned int index = 0; index < indices.size(); index += 3)
					{
						btVector3 vertex0 = BulletVector::toBtVector3(vertices[indices[index]].position);
						btVector3 vertex1 = BulletVector::toBtVector3(vertices[indices[index + 1]].position);
						btVector3 vertex2 = BulletVector::toBtVector3(vertices[indices[index + 2]].position);
						meshData->addTriangle(vertex0, vertex1, vertex2);

					}

					bulletModel = new btBvhTriangleMeshShape(meshData, true);
				}
			}
			Plane* plane = dynamic_cast<Plane*>(model);
			if (plane != NULL)
			{
				bulletModel = new btStaticPlaneShape(BulletVector::toBtVector3(plane->getNormal()), 1.0f);
			}
			Sphere* sphere = dynamic_cast<Sphere*>(model);
			if (sphere != NULL)
			{
				bulletModel = new btSphereShape(sphere->getRadius());
			}

			btVector3 localInertia(0.0f, 0.0f, 0.0f);
			if (dynamic)
			{
				bulletModel->calculateLocalInertia(material.mass, localInertia);
			}

			motionState = new btDefaultMotionState(BulletMatrix::toBtTransform(transform));

			body = new btRigidBody(material.mass, motionState, bulletModel, localInertia);
			body->setFriction(material.friction);
			body->setRestitution(material.restitution);
			body->setRollingFriction(material.friction);
			body->setUserPointer(this);

			if (dynamic)
			{
				if (box != NULL)
				{
					float minEdgeLength = min(box->getHalfXLength(), min(box->getHalfYLength(), box->getHalfZLength()));
					body->setCcdMotionThreshold(minEdgeLength);
					body->setCcdSweptSphereRadius(minEdgeLength * 0.25f);
				}
			}
		}

		BulletBody::~BulletBody()
		{
			if (motionState != NULL)
			{
				delete motionState;
			}

			if (body != NULL)
			{
				delete body;
			}

			if (bulletModel != NULL)
			{
				delete bulletModel;
			}
		}

		void BulletBody::applyForce(const Vector3& force, const Vector3& position)
		{
			body->applyForce(BulletVector::toBtVector3(force), BulletVector::toBtVector3(position));
		}

		void BulletBody::applyTorque(const Vector3& torque)
		{
			body->applyTorque(BulletVector::toBtVector3(torque));
		}

		void BulletBody::clearForces()
		{
			body->clearForces();
		}

		btRigidBody* BulletBody::getBody()
		{
			return body;
		}

		const Vector3& BulletBody::getLinearAcceleration() const
		{
			return linearAcceleration; // TODO ???
		}

		const Vector3& BulletBody::getLinearVelocity() const
		{
			linearVelocity = BulletVector::toVector3(body->getLinearVelocity());
			return linearVelocity;
		}

		float BulletBody::getMass() const
		{
			return body->getInvMass();
		}

		const Body::Material& BulletBody::getMaterial() const
		{
			return material;
		}

		const Model* BulletBody::getModel() const
		{
			return model;
		}

		Matrix44& BulletBody::getTransform()
		{
			transform = BulletMatrix::toMatrix44(body->getWorldTransform());
			return transform;
		}

		const Matrix44& BulletBody::getTransform() const
		{
			transform = BulletMatrix::toMatrix44(body->getWorldTransform());
			return transform;
		}

		bool BulletBody::isDynamic()
		{
			return getMass() != 0.0f;
		}

		void BulletBody::setDynamic(bool dynamic)
		{
			if (!dynamic)
			{
				setMass(0.0f);
			}
		}

		void BulletBody::setLinearVelocity(const Vector3& linearVelocity)
		{
			body->setLinearVelocity(BulletVector::toBtVector3(linearVelocity));
		}

		void BulletBody::setMass(float mass)
		{
			body->setMassProps(mass, body->getInvInertiaDiagLocal());
		}

		void BulletBody::setMaterial(const Material& material)
		{
			this->material = material;

			body->setFriction(material.friction);
			body->setRestitution(material.restitution);
			body->setRollingFriction(material.friction);
		}

		void BulletBody::setTransform(const Matrix44& transform)
		{
			body->setWorldTransform(BulletMatrix::toBtTransform(transform));
		}
	}
}
