/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rrt_planning/map/ROSMap.h"

#include <costmap_2d/cost_values.h>

namespace rrt_planning
{

ROSMap::ROSMap(costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros(costmap_ros),
			costmap(costmap_ros->getCostmap())
{
	std::cerr << "costmap ros recieved" << std::endl;
	std::cerr << costmap_ros->getBaseFrameID() << std::endl;
	std::cerr << costmap_ros->getGlobalFrameID() << std::endl;
	std::cerr << costmap_ros->getLayeredCostmap()->isCurrent() << std::endl;
	std::cerr << costmap_ros->getLayeredCostmap()->isInitialized() << std::endl;

	std::cerr << "costmap recieved" << std::endl;
	std::cerr << costmap->getSizeInMetersX() << std::endl;
	std::cerr << costmap->getSizeInMetersY() << std::endl;
	std::cerr << costmap->getResolution() << std::endl;
	std::cerr << costmap->getOriginX() << std::endl;
	std::cerr << costmap->getOriginY() << std::endl;
	std::cerr << costmap->getSizeInCellsX() << std::endl;
	std::cerr << costmap->getSizeInCellsY() << std::endl;
	//std::cerr << costmap->get << std::endl;

}

bool ROSMap::isFree(const Eigen::VectorXd& p)
{
	return getCost(p) <= costmap_2d::FREE_SPACE;
}

unsigned char ROSMap::getCost(const Eigen::VectorXd& p)
{
	double wx = p(0);
	double wy = p(1);

	unsigned int mx;
	unsigned int my;

	if(costmap->worldToMap(wx, wy, mx, my))
		return costmap->getCost(mx, my);
	else
	{
		std::cerr << "out of map" << std::endl;
		return costmap_2d::NO_INFORMATION;
	}

}

ROSMap::~ROSMap()
{

}

}
