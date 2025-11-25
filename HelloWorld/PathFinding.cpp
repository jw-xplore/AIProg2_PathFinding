#include "PathFinding.h"

PathFinding::PathFinding(MapEntity* mapEntity)
{
	mapRef = mapEntity;

	// Create graph
	for (int y = 0; y < mapEntity->height; y++)
	{
		for (int x = 0; x < mapEntity->width; x++)
		{
			Node* node = new Node();
			node->x = x;
			node->y = y;

			mapGraph.insert(mapGraph.end(), node);
		}
	}

	// Add connections
	for (int i = 0; i < mapGraph.size(); i++)
	{
		if (mapEntity->map[mapGraph[i]->y][mapGraph[i]->x] == 'X')
			continue;

		AddConnectionsToNode(mapGraph[i]);
	}
}

void PathFinding::AddConnectionsToNode(Node* node)
{
	int w = mapRef->width;
	int h = mapRef->height;
	int x = node->x;
	int y = node->y;

	// Top
	if (y < h && mapRef->map[y+1][x] != 'X')
	{
		Connection* link = new Connection();
		//link->x = x;
		//link->y = y + 1;
		link->node = NodeFromPostion(x, y + 1);
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Right
	if (x < w && mapRef->map[y][x+1] != 'X')
	{
		Connection* link = new Connection();
		//link->x = x + 1;
		//link->y = y;
		link->node = NodeFromPostion(x + 1, y);
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Bottom
	if (y > 0 && mapRef->map[y - 1][x] != 'X')
	{
		Connection* link = new Connection();
		//link->x = x;
		//link->y = y - 1;
		link->node = NodeFromPostion(x, y - 1);
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Left
	if (x > 0 && mapRef->map[y][x - 1] != 'X')
	{
		Connection* link = new Connection();
		//link->x = x - 1;
		//link->y = y;
		link->node = NodeFromPostion(x - 1, y);
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}
}

void PathFinding::DrawGraph()
{
	int halfSize = mapRef->tileSize * 0.5f;

	for (int i = 0; i < mapGraph.size(); i++)
	{
		Play::Point2D nodePos = { mapGraph[i]->x * mapRef->tileSize + halfSize, mapGraph[i]->y * mapRef->tileSize + halfSize };
		//Play::DrawCircle(nodePos, 6, cGreen);

		for (int n = 0; n < mapGraph[i]->connections.size(); n++)
		{
			Play::Point2D pos = { mapGraph[i]->connections[n]->node->x * mapRef->tileSize + halfSize, mapGraph[i]->connections[n]->node->y * mapRef->tileSize + halfSize };
			Play::DrawLine(nodePos, pos, cRed);
		}
	}
}

std::vector<Node*> PathFinding::Dijkstra(Play::Point2D start, Play::Point2D end)
{
	int max = 99999999;

	int startI = mapRef->width * start.y + start.x;
	Node* startNode = NodeFromPostion(start.x, start.y);

	int endI = mapRef->width * end.y + end.x;
	Node* endNode = NodeFromPostion(end.x, end.y);

	std::vector<float> distances;
	std::vector<bool> visited;

	std::vector<int> prevNodes;

	for (int i = 0; i < mapGraph.size(); i++)
	{
		distances.insert(distances.end(), max);
		visited.insert(visited.end(), false);
		prevNodes.insert(prevNodes.end(), -1);
	}

	distances[startI] = 0;

	std::priority_queue<
		std::pair<float, Node*>,
		std::vector<std::pair<float, Node*>>,
		std::greater<std::pair<float, Node*>>
	> prioPath;

	prioPath.push({ 0, startNode });

	Node* checking;
	int pathSoFar = 0;

	while (!prioPath.empty())
	{
		checking = prioPath.top().second;
		prioPath.pop();
		int ni = mapRef->width * checking->y + checking->x;
		visited[ni] = true;

		for (int i = 0; i < checking->connections.size(); i++)
		{
			Node* connected = checking->connections[i]->node;
			int ci = mapRef->width * connected->y + connected->x;
			if (visited[ci])
				continue;

			float newDist = distances[ni] + checking->connections[i]->weight;
			if (newDist < distances[ci])
			{
				prevNodes[ci] = ni;
				distances[ci] = newDist;
				prioPath.push({ newDist, connected });
			}
		}
	}

	// Get shortest path
	std::vector<Node*> path;
	if (distances[endI] == max)
		return path;

	for (int at = endI; at != -1; at = prevNodes[at])
	{
		path.push_back(mapGraph[at]);
	}

	// Debug draw
	for (int i = 0; i < path.size(); i++)
	{
		int halfSize = mapRef->tileSize * 0.5f;
		float x = path[i]->x * mapRef->tileSize + halfSize;
		float y = path[i]->y * mapRef->tileSize + halfSize;
		Play:Point2D pos = { x, y };
		Play::DrawCircle(pos, 4, Play::cGreen);
	}


	return path;
}

// Return list of points using A* algorhytm
std::vector<Node*> PathFinding::AStar(Play::Point2D start, Play::Point2D end, const char** map)
{
	// Find start and end
	int startI = mapRef->width * start.y + start.x;
	Node* startNode = NodeFromPostion(start.x, start.y);

	int endI = mapRef->width * end.y + end.x;
	Node* endNode = NodeFromPostion(end.x, end.y);

	// Initialize start node
	NodeRecordAs startRecord;
	startRecord.node = startNode;
	startRecord.costSoFar = 0;
	startRecord.costEstimated = ManhattanHeuristics(startNode, endNode);

	std::vector<NodeRecordAs> records;
	records.insert(records.end(), startRecord);

	// Setup open and closed list
	std::vector<NodeRecordAs> open;
	open.insert(open.end(), startRecord);

	std::vector<NodeRecordAs> closed;

	while (open.size() != 0)
	{
		// Find smallest record - smallest estimated cost
		NodeRecordAs current = SmallestAsRecord(open);


		// Is at the end?
		if (current.node == endNode)
			break;

		// Loop through connections
		for (int i = 0; i < current.node->connections.size(); i++)
		{
			Connection* connection = current.node->connections[i];

			if (current.node->connections[i] == connection)
			{

			}

			Node* endNode = connection->node;
			float endNodeCost = current.costSoFar + connection->weight;

			// Check node in closed list
			/*
			if (std::find(closed.begin(), closed.end(), current) != closed.end())
			{

			}
			*/
		}
	}

	std::vector<Node*> path;
	return path;
}

Node* PathFinding::NodeFromPostion(int x, int y)
{
	return mapGraph[mapRef->width * y + x];
}

float PathFinding::ManhattanHeuristics(Node* start, Node* end)
{
	return abs((end->x - start->x) + (end->y - start->y));
}

NodeRecordAs PathFinding::SmallestAsRecord(std::vector<NodeRecordAs> list)
{
	NodeRecordAs record = list[0];

	for (int i = 1; i < list.size(); i++)
	{
		if (record.costSoFar > list[i].costSoFar)
			record = list[i];
	}

	return record;
}

bool PathFinding::ContainsAsRecord(const std::vector<NodeRecordAs>& list, const NodeRecordAs& record)
{
	return std::find(list.begin(), list.end(), record) != list.end();
}

/*
NodeRecordAs PathFinding::FindAsRecordFromNode(std::vector<NodeRecordAs> list, Node* node)
{

}
*/