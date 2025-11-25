#pragma once
#include "Play.h"
#include <vector>
#include <utility>
#include <queue>
#include <algorithm>
#include "MapEntity.h"

struct Node;

struct Connection
{
	//int x, y;
	Node* fromNode;
	Node* node;
	float weight;
};

struct Node
{
	int x, y;
	std::vector<Connection*> connections;
};

struct NodeRecordAs
{
	Node* node;
	Connection* connection;
	float costSoFar;
	float costEstimated;

	bool operator==(const NodeRecordAs& other) const {
		return node == other.node && costSoFar == other.costSoFar && costEstimated == other.costEstimated;
	}
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
	std::vector<Node*> AStar(Play::Point2D start, Play::Point2D end);

	Node* NodeFromPostion(int x, int y);
	float ManhattanHeuristics(Node* start, Node* end);
	//float EuclideanHeuristic();
	NodeRecordAs SmallestAsRecord(std::vector<NodeRecordAs> list);
	bool ContainsAsRecord(const std::vector<NodeRecordAs>& list, Node* node);
	NodeRecordAs* FindAsRecordFromNode(std::vector<NodeRecordAs>& list, Node* node);
};

