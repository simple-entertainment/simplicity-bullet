/*      _                 _ _      _ _
 *     (_)               | (_)    (_) |
 *  ___ _ _ __ ___  _ __ | |_  ___ _| |_ _   _
 * / __| | '_ ` _ \| '_ \| | |/ __| | __| | | |
 * \__ \ | | | | | | |_) | | | (__| | |_| |_| |
 * |___/_|_| |_| |_| .__/|_|_|\___|_|\__|\__, |
 *                 | |                    __/ |
 *                 |_|                   |___/
 *
 * This file is part of simplicity. See the LICENSE file for the full license governing this code.
 */
#include "BulletBody.h"
#include "BulletEngineState.h"

namespace simplicity
{
	namespace bullet
	{
		BulletEngineState::BulletEngineState() :
			broadphase(new btDbvtBroadphase()),
			collisionConfiguration(new btDefaultCollisionConfiguration()),
			dispatcher(new btCollisionDispatcher(collisionConfiguration.get())),
			entities(),
			solver(new btSequentialImpulseConstraintSolver),
			world(new btDiscreteDynamicsWorld(dispatcher.get(), broadphase.get(), solver.get(),
											  collisionConfiguration.get()))
		{
		}

		BulletEngineState::~BulletEngineState()
		{
			// Manual deletion to ensure correct order.
			world.reset();
			solver.reset();
			dispatcher.reset();
			collisionConfiguration.reset();
			broadphase.reset();
		}

		Entity* BulletEngineState::getEntity(const btCollisionObject& body)
		{
			return entities[&body];
		}

		btDynamicsWorld* BulletEngineState::getWorld()
		{
			return world.get();
		}

		void BulletEngineState::onAddEntity(Entity& entity)
		{
			for (BulletBody* body : entity.getComponents<BulletBody>())
			{
				entities[body->getBody()] = &entity;
				world->addRigidBody(body->getBody());
			}
		}

		void BulletEngineState::onRemoveEntity(Entity& entity)
		{
			for (BulletBody* body : entity.getComponents<BulletBody>())
			{
				world->removeRigidBody(body->getBody());
				entities.erase(body->getBody());
			}
		}
	}
}
