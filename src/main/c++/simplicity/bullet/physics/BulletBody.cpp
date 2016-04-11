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
#include <simplicity/model/shape/Box.h>

#include "../math/BulletMatrix.h"
#include "../math/BulletVector.h"
#include "BulletBody.h"

using namespace std;

namespace simplicity
{
	namespace bullet
	{
		BulletBody::BulletBody(const Material& material, const Mesh& mesh, const Shape& bounds,
							   const Matrix44& transform) :
			body(),
			material(material),
			motionState(),
			shape(nullptr)
		{
			createBulletShape(mesh);

			btVector3 localInertia(0.0f, 0.0f, 0.0f);
			if (isDynamic())
			{
				shape->calculateLocalInertia(material.mass, localInertia);
			}

			motionState.reset(new btDefaultMotionState(BulletMatrix::toBtTransform(transform)));

			body.reset(new btRigidBody(material.mass, motionState.get(), shape.get(), localInertia));
			body->setFriction(material.friction);
			body->setRestitution(material.restitution);
			body->setRollingFriction(material.friction);

			// Continuous collision detection.
			if (isDynamic())
			{
				if (bounds.getTypeID() == Box::TYPE_ID)
				{
					const Box& box = static_cast<const Box&>(bounds);
					float minEdgeLength = min(box.getHalfXLength(), min(box.getHalfYLength(), box.getHalfZLength()));
					body->setCcdMotionThreshold(minEdgeLength);
					body->setCcdSweptSphereRadius(minEdgeLength * 0.25f);
				}
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

		void BulletBody::createBulletShape(const Mesh& mesh)
		{
			const MeshData& meshData = mesh.getData();

			if (isDynamic())
			{
				vector<btVector3> points(meshData.size());

				for (unsigned int index = 0; index < meshData.size(); index += 3)
				{
					points[index] = BulletVector::toBtVector3(meshData[index].position);
					points[index + 1] = BulletVector::toBtVector3(meshData[index + 1].position);
					points[index + 2] = BulletVector::toBtVector3(meshData[index + 2].position);
				}

				shape.reset(new btConvexHullShape((btScalar*) points.data(), points.size()));
			}
			else
			{
				btTriangleMesh* bulletMesh = new btTriangleMesh;

				for (unsigned int index = 0; index < meshData.size(); index += 3)
				{
					btVector3 vertex0 = BulletVector::toBtVector3(meshData[index].position);
					btVector3 vertex1 = BulletVector::toBtVector3(meshData[index + 1].position);
					btVector3 vertex2 = BulletVector::toBtVector3(meshData[index + 2].position);
					bulletMesh->addTriangle(vertex0, vertex1, vertex2);
				}

				shape.reset(new btBvhTriangleMeshShape(bulletMesh, true));
			}

			mesh.releaseData();
		}

		btRigidBody* BulletBody::getBody()
		{
			return body.get();
		}

		Vector3 BulletBody::getLinearVelocity() const
		{
			return BulletVector::toVector3(body->getLinearVelocity());
		}

		const Body::Material& BulletBody::getMaterial() const
		{
			return material;
		}

		bool BulletBody::isDynamic()
		{
			// Bullet Physics considers a body static when it has a mass of 0.
			return material.mass != 0.0f;
		}

		void BulletBody::setDynamic(bool dynamic)
		{
			if (!dynamic)
			{
				// Bullet Physics considers a body static when it has a mass of 0.
				material.mass = 0.0f;
				body->setMassProps(material.mass, body->getInvInertiaDiagLocal());
			}
			else
			{
				// We can't really make it dynamic here... we'd have to guess a mass.
				// Set material can be used with a non-zero mass instead.
			}
		}

		void BulletBody::setLinearVelocity(const Vector3& linearVelocity)
		{
			body->setLinearVelocity(BulletVector::toBtVector3(linearVelocity));
		}

		void BulletBody::setMaterial(const Material& material)
		{
			this->material = material;

			body->setFriction(material.friction);
			body->setMassProps(material.mass, body->getInvInertiaDiagLocal());
			body->setRestitution(material.restitution);
			body->setRollingFriction(material.friction);
		}
	}
}
