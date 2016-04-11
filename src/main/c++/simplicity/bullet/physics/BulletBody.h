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

#include <memory>

#include <btBulletDynamicsCommon.h>

#include <simplicity/model/Mesh.h>
#include <simplicity/physics/Body.h>

namespace simplicity
{
	namespace bullet
	{
		/**
		 * <p>
		 * A physical body implemented using Bullet Physics.
		 * </p>
		 */
		class SIMPLE_API BulletBody : public Body
		{
			public:
				/**
				 * @param material The material this body is constructed from.
				 * @param mesh The geometry of the body.
				 * @param bounds a bounding volume containing the body.
				 * @param transform The position and orientation of the body.
				 */
				BulletBody(const Material& material, const Mesh& mesh, const Shape& bounds, const Matrix44& transform);

				void applyForce(const Vector3& force, const Vector3& position) override;

				void applyTorque(const Vector3& torque) override;

				void clearForces() override;

				/**
				 * <p>
				 * Retrieves the Bullet Physics rigid body used to implement this body.
				 * </p>
				 *
				 * @return
				 */
				btRigidBody* getBody();

				Vector3 getLinearVelocity() const override;

				const Material& getMaterial() const override;

				bool isDynamic() override;

				void setDynamic(bool dynamic) override;

				void setLinearVelocity(const Vector3& linearVelocity) override;

				void setMaterial(const Material& material) override;

			private:
				std::unique_ptr<btRigidBody> body;

				Material material;

				std::unique_ptr<btMotionState> motionState;

				std::unique_ptr<btCollisionShape> shape;

				void createBulletShape(const Mesh& mesh);
		};
	}
}

#endif /* BULLETBODY_H_ */
