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

#include <simplicity/engine/Engine.h>

#include "BulletBody.h"

namespace simplicity
{
	namespace bullet
	{
		class BulletEngine : public Engine
		{
			public:
				BulletEngine(const Vector3& gravity, float fixedTimeStep = 0.0f);

				void addEntity(Entity& entity);

				void advance();

				void destroy();

				void init();

				void removeEntity(const Entity& entity);

			private:
				btBroadphaseInterface* broadphase;

				btCollisionConfiguration* collisionConfiguration;

				btDispatcher* dispatcher;

				float fixedTimeStep;

				Vector3 gravity;

				btConstraintSolver* solver;

				btDynamicsWorld* world;
		};
	}
}

#endif /* BULLETENGINE_H_ */
