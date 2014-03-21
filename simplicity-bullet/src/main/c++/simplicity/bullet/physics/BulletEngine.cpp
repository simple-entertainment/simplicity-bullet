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
#include <simplicity/Simplicity.h>

#include "../math/BulletMatrix.h"
#include "../math/BulletVector.h"
#include "BulletEngine.h"

using namespace std;

namespace simplicity
{
	namespace bullet
	{
		BulletEngine::BulletEngine(const Vector3& gravity, float fixedTimeStep) :
			broadphase(),
			collisionConfiguration(),
			dispatcher(),
			entities(),
			fixedTimeStep(fixedTimeStep),
			gravity(gravity),
			solver(),
			world()
		{
		}

		void BulletEngine::addEntity(Entity& entity)
		{
			for (BulletBody* body : entity.getComponents<BulletBody>())
			{
				entities[body->getBody()] = &entity;
				world->addRigidBody(body->getBody());
			}
		}

		void BulletEngine::advance()
		{
			if (fixedTimeStep == 0.0f)
			{
				world->stepSimulation(Simplicity::getDeltaTime(), 10); // 10 is magic!
			}
			else
			{
				world->stepSimulation(fixedTimeStep, 1, fixedTimeStep);
			}

			// Update entity positions and orientations.
			btCollisionObjectArray& collisionObjects = world->getCollisionObjectArray();
			for (int index = 0; index < collisionObjects.size(); index++)
			{
				Entity* entity = entities[collisionObjects[index]];

				Matrix44 newTransform = BulletMatrix::toMatrix44(collisionObjects[index]->getWorldTransform());
				if (newTransform != entity->getTransform())
				{
					entity->setTransform(newTransform);
					Simplicity::updateWorldRepresentations(*entity);
				}
			}
		}

		void BulletEngine::destroy()
		{
		}

		void BulletEngine::init()
		{
			broadphase.reset(new btDbvtBroadphase());
			collisionConfiguration.reset(new btDefaultCollisionConfiguration());
			dispatcher.reset(new btCollisionDispatcher(collisionConfiguration.get()));
			solver.reset(new btSequentialImpulseConstraintSolver);
			world.reset(new btDiscreteDynamicsWorld(dispatcher.get(), broadphase.get(), solver.get(),
					collisionConfiguration.get()));

			world->setGravity(BulletVector::toBtVector3(gravity));
		}

		void BulletEngine::removeEntity(const Entity& entity)
		{
			for (BulletBody* body : entity.getComponents<BulletBody>())
			{
				world->removeRigidBody(body->getBody());
				entities.erase(body->getBody());
			}
		}
	}
}
