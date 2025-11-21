#pragma once
#include "Play.h"
#include <vector>

class PathFinding
{
public:
	int** mapGraph;
	std::vector<Play::Point2D> path;
	
	PathFinding(const char** map);

	std::vector<Play::Point2D> AStar(Play::Point2D start, Play::Point2D end, const char** map);
	float ManhattanDistance(Play::Point2D start, Play::Point2D end);
	float EuclideanDistance();
};

