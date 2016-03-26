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
#ifndef BULLETENGINESTATE_H
#define BULLETENGINESTATE_H

#include <map>

#include <btBulletDynamicsCommon.h>

#include <simplicity/scene/SceneState.h>

namespace simplicity
{
	namespace bullet
	{
		class BulletEngineState : public SceneState
		{
			public:
				BulletEngineState();

				~BulletEngineState();

				Entity* getEntity(const btCollisionObject& body);

				btDynamicsWorld* getWorld();

				void onAddEntity(Entity& entity) override;

				void onRemoveEntity(Entity& entity) override;

			private:
				std::unique_ptr<btBroadphaseInterface> broadphase;

				std::unique_ptr<btCollisionConfiguration> collisionConfiguration;

				std::unique_ptr<btDispatcher> dispatcher;

				std::map<const btCollisionObject*, Entity*> entities;

				std::unique_ptr<btConstraintSolver> solver;

				std::unique_ptr<btDynamicsWorld> world;
		};
	}
}

#endif /* BULLETENGINESTATE_H */
