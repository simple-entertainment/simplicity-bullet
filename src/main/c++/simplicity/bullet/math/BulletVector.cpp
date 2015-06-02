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
#include "BulletVector.h"

namespace simplicity
{
	namespace bullet
	{
		namespace BulletVector
		{
			btVector3 toBtVector3(const Vector3& original)
			{
				return btVector3(original.X(), original.Y(), original.Z());
			}

			Vector3 toVector3(const btVector3& original)
			{
				return Vector3(original.getX(), original.getY(), original.getZ());
			}
		}
	}
}
