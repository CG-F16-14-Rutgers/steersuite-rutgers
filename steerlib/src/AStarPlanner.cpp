//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm>
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  5000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){epsilon = 10;}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id )
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );

			}
		}

		if ( traversal_cost > COLLISION_COST ) {
			return false;
    }
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*";
		agent_path=computeWeightedAStar(start, goal);
    agent_path.insert(agent_path.end(), goal);
    /*for(std::vector<Util::Point>::iterator iter=agent_path.begin();iter!=agent_path.end();++iter) {
      std::cout<<iter->x << " " << iter->y << "\n";
    }*/
    return true;
  }

  /*
    Gets start position (current point) and destination (goal point) as inputs
    and returns double value containing Manhatten distance (non-diagonal)
   */
  double AStarPlanner::calcHValue(Util::Point start, Util::Point destination) {
    double x = start.x - destination.x;
    double z = start.z - destination.z;
    if(x<0) {
      x=x*-1;
    }
    if(z<0) {
      z=z*-1;
    }
    return x+z;
  }

  /*
    Weighted A* implementation. Epsilon applied to hvalue is 10.
   */
  std::vector<Util::Point> AStarPlanner::computeWeightedAStar(Util::Point start, Util::Point goal) {
    std::multiset<SteerLib::AStarPlannerNode> closedSet;
    std::multiset<SteerLib::AStarPlannerNode> openSet;
    AStarPlannerNode startNode(start,0,epsilon*calcHValue(start,goal),NULL);
    openSet.insert(startNode);
    std::map<Util::Point, Util::Point, AStarPlanner::comparePt> parents;
    std::map<Util::Point, double, comparePt> gscore;
    gscore[start] = 0;
    std::map<Util::Point, double, comparePt> fscore;
    fscore[start] = epsilon*calcHValue(start,goal);
    SteerLib::AStarPlannerNode currentNode;

    int count = 0;
    while(!openSet.empty()) {

      currentNode = *openSet.begin();

      if(((int)currentNode.point.x == (int)goal.x) && ((int)currentNode.point.z == (int)goal.z)) {
        return createPath(parents, currentNode);
      }
      count++;

      closedSet.insert(currentNode);
      openSet.erase(openSet.begin());
      std::multiset<SteerLib::AStarPlannerNode> neighbors;
      populateNeighbors(currentNode, goal, neighbors);
      for(std::multiset<SteerLib::AStarPlannerNode>::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter) {

        if(!checkSet(closedSet,*iter)) {
          int checkCell = gSpatialDatabase->getCellIndexFromLocation(iter->point);
            if(canBeTraversed(checkCell)) {

            if(gscore.count(iter->point) == 0) {
              gscore[iter->point] = 100000000;
              fscore[iter->point] = 100000000;
            }
            if(iter->g < gscore[iter->point]) {
              gscore[iter->point] = iter->g;
              fscore[iter->point] = iter->f;
              parents[iter->point] = currentNode.point;
              if(!checkSet(openSet,*iter)) {
                openSet.insert(*iter);
              }
            }
          }
          else {
            closedSet.insert(*iter);
          }
        }
      }
    }
  }

  /*
    Populates passed in neighbor vector with all points surrounding currentNode at a distance of 1 or 1.4 (one space diagonal).
   */
  void AStarPlanner::populateNeighbors(SteerLib::AStarPlannerNode currentNode, Util::Point goal, std::multiset<AStarPlannerNode>& neighbors) {
    Util::Point center = currentNode.point;
    double add = 1;
    Util::Point east(currentNode.point.x+add,0.0f,currentNode.point.z);
    SteerLib::AStarPlannerNode eastNode(east,currentNode.g+10,currentNode.g+10+epsilon*calcHValue(east,goal),&currentNode);
    neighbors.insert(eastNode);
    Util::Point west(currentNode.point.x-add,0.0f,currentNode.point.z);
    SteerLib::AStarPlannerNode westNode(west,currentNode.g+10,currentNode.g+10+epsilon*calcHValue(west,goal),&currentNode);
    neighbors.insert(westNode);
    Util::Point north(currentNode.point.x,0.0f,currentNode.point.z+add);
    SteerLib::AStarPlannerNode northNode(north,currentNode.g+10,currentNode.g+10+epsilon*calcHValue(north,goal),&currentNode);
    neighbors.insert(northNode);
    Util::Point south(currentNode.point.x,0.0f,currentNode.point.z-add);
    SteerLib::AStarPlannerNode southNode(south,currentNode.g+10,currentNode.g+10+epsilon*calcHValue(south,goal),&currentNode);
    neighbors.insert(southNode);
    Util::Point northeast(currentNode.point.x+add,0.0f,currentNode.point.z+add);
    SteerLib::AStarPlannerNode northEastNode(northeast,currentNode.g+14,currentNode.g+14+epsilon*calcHValue(northeast,goal),&currentNode);
    neighbors.insert(northEastNode);
    Util::Point northwest(currentNode.point.x-add,0.0f,currentNode.point.z+add);
    SteerLib::AStarPlannerNode northWestNode(northwest,currentNode.g+14,currentNode.g+14+epsilon*calcHValue(northwest,goal),&currentNode);
    neighbors.insert(northWestNode);
    Util::Point southeast(currentNode.point.x+add,0.0f,currentNode.point.z-add);
    SteerLib::AStarPlannerNode southEastNode(southeast,currentNode.g+14,currentNode.g+14+epsilon*calcHValue(southeast,goal),&currentNode);
    neighbors.insert(southEastNode);
    Util::Point southwest(currentNode.point.x-add,0.0f,currentNode.point.z-add);
    SteerLib::AStarPlannerNode southWestNode(southwest,currentNode.g+14,currentNode.g+14+epsilon*calcHValue(southwest,goal),&currentNode);
    neighbors.insert(southWestNode);
  }

  /*
    Takes parent node list, and last found point i.e, goal point, and agend_path variables and populates agent_path starting from goal.
   */
  std::vector<Util::Point> AStarPlanner::createPath(std::map<Util::Point, Util::Point, AStarPlanner::comparePt> parents, SteerLib::AStarPlannerNode currentNode) {
    std::vector<Util::Point> path;
    Util::Point currentPoint = currentNode.point;
    while(parents.find(currentPoint) != parents.end()) {
      path.insert(path.begin(), currentPoint);
      currentPoint = parents.at(currentPoint);
    }
    return path;
  }

  bool AStarPlanner::checkSet(std::multiset<AStarPlannerNode> set, SteerLib::AStarPlannerNode node) {
    for(std::multiset<AStarPlannerNode>::iterator iter = set.begin(); iter != set.end(); ++iter) {
      if((iter->point.x == node.point.x) && (iter->point.z == node.point.z)) {
        return true;
      }
    }
    return false;
  }
}
