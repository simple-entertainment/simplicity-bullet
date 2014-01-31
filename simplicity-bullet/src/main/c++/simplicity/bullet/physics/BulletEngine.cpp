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

namespace simplicity
{
	namespace bullet
	{
		BulletEngine::BulletEngine(const Vector3& gravity, float fixedTimeStep) :
			broadphase(NULL),
			collisionConfiguration(NULL),
			dispatcher(NULL),
			fixedTimeStep(fixedTimeStep),
			gravity(gravity),
			solver(NULL),
			world(NULL)
		{
		}

		void BulletEngine::addEntity(Entity& entity)
		{
			for (BulletBody* entityBody : entity.getComponents<BulletBody>())
			{
				world->addRigidBody(entityBody->getBody());
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

			btCollisionObjectArray& collisionObjects = world->getCollisionObjectArray();
			for (int index = 0; index < collisionObjects.size(); index++)
			{
				Body* body = static_cast<Body*>(collisionObjects[index]->getUserPointer());
				body->getEntity()->setTransformation(
						BulletMatrix::toMatrix44(collisionObjects[index]->getWorldTransform()));
			}
		}

		void BulletEngine::destroy()
		{
			if (world != NULL)
			{
				delete world;
			}

			if (solver != NULL)
			{
				delete solver;
			}

			if (dispatcher != NULL)
			{
				delete dispatcher;
			}

			if (collisionConfiguration != NULL)
			{
				delete collisionConfiguration;
			}

			if (broadphase != NULL)
			{
				delete broadphase;
			}
		}

		void BulletEngine::init()
		{
			broadphase = new btDbvtBroadphase();
			collisionConfiguration = new btDefaultCollisionConfiguration();
			dispatcher = new btCollisionDispatcher(collisionConfiguration);
			solver = new btSequentialImpulseConstraintSolver;
			world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

			world->setGravity(BulletVector::toBtVector3(gravity));
		}

		void BulletEngine::removeEntity(const Entity& entity)
		{
			vector<BulletBody*> entityBodies = entity.getComponents<BulletBody>();
			for (unsigned int index = 0; index < entityBodies.size(); index++)
			{
				world->removeRigidBody(entityBodies[index]->getBody());
			}
		}
	}
}
