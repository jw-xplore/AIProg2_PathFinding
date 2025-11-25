#pragma once
#include "Play.h"
#include <vector>
#include <utility>
#include <queue>
#include "MapEntity.h"

struct Node;

struct Connection
{
	//int x, y;
	Node* node;
	float weight;
};

struct Node
{
	int x, y;
	std::vector<Connection*> connections;
};

struct NodeRecord
{
	Node* node;
	Connection* connection;
	float costSoFar;
	float costEstimated;
};

class PathFinding
{
public:
	MapEntity* mapRef;
	std::vector<Node*> mapGraph;
	std::vector<Node*> path;
	
	PathFinding(MapEntity* mapEntity);

	void AddConnectionsToNode(Node* node);
	void DrawGraph();

	std::vector<Node*> Dijkstra(Play::Point2D start, Play::Point2D end);
	std::vector<Node*> AStar(Play::Point2D start, Play::Point2D end, const char** map);

	Node* NodeFromPostion(int x, int y);
	float ManhattanHeuristics(Node* start, Node* end);
	float EuclideanHeuristic();
};

