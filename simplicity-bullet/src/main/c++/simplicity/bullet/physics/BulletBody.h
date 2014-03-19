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
#ifndef BULLETBODY_H_
#define BULLETBODY_H_

#include <bullet/btBulletDynamicsCommon.h>

#include <simplicity/physics/Body.h>

namespace simplicity
{
	namespace bullet
	{
		class BulletBody : public Body
		{
			public:
				BulletBody(const Material& material, Model* model, const Matrix44& transform, bool dynamic);

				~BulletBody();

				void applyForce(const Vector3& force, const Vector3& position);

				void applyTorque(const Vector3& torque);

				void clearForces();

				btRigidBody* getBody();

				const Vector3& getLinearVelocity() const;

				float getMass() const;

				const Material& getMaterial() const;

				const Model* getModel() const;

				Matrix44& getTransform();

				const Matrix44& getTransform() const;

				bool isDynamic();

				void setDynamic(bool dynamic);

				void setLinearVelocity(const Vector3& linearVelocity);

				void setMass(float mass);

				void setMaterial(const Material& material);

				void setTransform(const Matrix44& transform);

			private:
				btRigidBody* body;

				btCollisionShape* bulletModel;

				mutable Vector3 linearVelocity;

				Material material;

				Model* model;

				btMotionState* motionState;

				mutable Matrix44 transform;
		};
	}
}

#endif /* BULLETBODY_H_ */
