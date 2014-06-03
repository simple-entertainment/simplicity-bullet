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
#ifndef BULLETENGINE_H_
#define BULLETENGINE_H_

#include <map>

#include <simplicity/engine/Engine.h>

#include "BulletBody.h"

namespace simplicity
{
	namespace bullet
	{
		/**
		 * <p>
		 * A physics engine implemented using Bullet Physics.
		 * </p>
		 */
		class SIMPLE_API BulletEngine : public Engine
		{
			public:
				BulletEngine(const Vector3& gravity, float fixedTimeStep = 0.0f);

				void advance() override;

				void onAddEntity(Entity& entity) override;

				void onPlay() override;

				void onRemoveEntity(Entity& entity) override;

				void onStop() override;

			private:
				std::unique_ptr<btBroadphaseInterface> broadphase;

				std::unique_ptr<btCollisionConfiguration> collisionConfiguration;

				std::unique_ptr<btDispatcher> dispatcher;

				std::map<btCollisionObject*, Entity*> entities;

				float fixedTimeStep;

				Vector3 gravity;

				std::unique_ptr<btConstraintSolver> solver;

				std::unique_ptr<btDynamicsWorld> world;
		};
	}
}

#endif /* BULLETENGINE_H_ */
