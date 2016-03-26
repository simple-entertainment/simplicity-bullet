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
		BulletEngine::BulletEngine() :
			fixedTimeStep(0.0f),
			gravity(Vector3(0.0f, -10.0f, 0.0f)),
			state()
		{
		}

		void BulletEngine::advance(Scene& scene)
		{
			BulletEngineState* state = this->state[&scene];

			if (fixedTimeStep == 0.0f)
			{
				state->getWorld()->stepSimulation(Simplicity::getDeltaTime(), 10); // 10 is magic!
			}
			else
			{
				state->getWorld()->stepSimulation(fixedTimeStep, 1, fixedTimeStep);
			}

			// Update entity positions and orientations.
			btCollisionObjectArray& collisionObjects = state->getWorld()->getCollisionObjectArray();
			for (int index = 0; index < collisionObjects.size(); index++)
			{
				Entity* entity = state->getEntity(*collisionObjects[index]);

				Matrix44 newTransform = BulletMatrix::toMatrix44(collisionObjects[index]->getWorldTransform());
				if (newTransform != entity->getTransform())
				{
					entity->setTransform(newTransform);
				}
			}
		}

		float BulletEngine::getFixedTimeStep()
		{
			return fixedTimeStep;
		}

		Vector3 BulletEngine::getGravity()
		{
			return gravity;
		}

		void BulletEngine::onBeforeOpenScene(Scene& scene)
		{
			unique_ptr<BulletEngineState> state(new BulletEngineState);

			this->state[&scene] = state.get();
			scene.addState(move(state));
		}

		void BulletEngine::onCloseScene(Scene& scene)
		{
			scene.removeState(*state[&scene]);
		}

		void BulletEngine::onOpenScene(Scene& scene)
		{
			BulletEngineState* state = this->state[&scene];

			state->getWorld()->setGravity(BulletVector::toBtVector3(gravity));
		}

		void BulletEngine::setFixedTimeStep(float fixedTimeStep)
		{
			this->fixedTimeStep = fixedTimeStep;
		}

		void BulletEngine::setGravity(const Vector3& gravity)
		{
			this->gravity = gravity;

			for (pair<Scene*, BulletEngineState*> state : this->state)
			{
				state.second->getWorld()->setGravity(BulletVector::toBtVector3(gravity));
			}
		}
	}
}
