#pragma once
#include "Play.h"
#include <vector>
#include "MapEntity.h"

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
	MapEntity* mapRef;
	std::vector<Node*> mapGraph;
	std::vector<Node*> path;
	
	PathFinding(MapEntity* mapEntity);

	void AddConnectionsToNode(Node* node, int x, int y, int width, int height, const char** map);
	void DrawGraph();

	std::vector<Node*> AStar(Play::Point2D start, Play::Point2D end, const char** map);
	float ManhattanDistance(Play::Point2D start, Play::Point2D end);
	float EuclideanDistance();
};

