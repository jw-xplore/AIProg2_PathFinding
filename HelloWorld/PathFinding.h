#pragma once
#include "Play.h"
#include <vector>

struct Connection
{
	int x, y;
	float weight;
};

struct Node
{
	int x, y;
	std::vector<Connection*> connections;
};

class PathFinding
{
public:
	std::vector<Node*> mapGraph;
	std::vector<Node*> path;
	
	PathFinding(const char** map, int width, int height);

	void AddConnectionsToNode(Node* node, int x, int y, int width, int height, const char** map);
	std::vector<Node*> AStar(Play::Point2D start, Play::Point2D end, const char** map);
	float ManhattanDistance(Play::Point2D start, Play::Point2D end);
	float EuclideanDistance();
};

