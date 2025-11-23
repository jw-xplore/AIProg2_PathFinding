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
	Node* endNode = NodeFromPostion(end.x, end.y);

	std::vector<float> distances;
	std::vector<bool> visited;

	for (int i = 0; i < mapGraph.size(); i++)
	{
		distances.insert(distances.end(), max);
		visited.insert(visited.end(), false);
	}

	distances[startI] = 0;

	std::vector<Node*> path;

	//std::vector <std::pair<Node*, float>> pathPairs;
	//pathPairs.insert(pathPairs.end(), { startNode, 0 });
	std::priority_queue<std::pair<Node*, float>> prioPath;
	prioPath.push({ startNode, 0 });

	Node* checking;
	int pathSoFar = 0;

	while (!prioPath.empty())
	{
		checking = prioPath.top().first;
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
				distances[ci] = newDist;
				prioPath.push({ connected, newDist });
			}
		}
	}

	// Debug draw
	for (int i = 0; i < distances.size(); i++)
	{
		if (distances[i] == max)
			continue;

		int halfSize = mapRef->tileSize * 0.5f;
		Play:Point2D pos = { (i % mapRef->height) * mapRef->tileSize + halfSize, (i / mapRef->width) * mapRef->tileSize + halfSize };
		Play::DrawCircle(pos, 4, Play::cGreen);
	}

	// Get path format
	return path;
	/*
	std::sort(distances.begin(), distances.end());
	for (int i = 0; i < distances.size(); i++)
	{
		if (distances[i] == max)
			break;

		path.insert(path.end(), );
	}
	*/

	/*
	while (checking != endNode)
	{
		Node* bestNode = checking->connections[0]->node;
		int bestI = mapRef->width * bestNode->y + bestNode->x;
		int bestDist = pathSoFar + checking->connections[0]->weight;

		for (int i = 0; i < checking->connections.size(); i++)
		{
			Node* current = checking->connections[i]->node;
			int currentI = mapRef->width * current->y + current->x;

			// Update with shorter weight
			int dist = pathSoFar + checking->connections[i]->weight;
			if (distances[currentI] > dist)
			{
				distances[currentI] = dist;
			}

			// Choose best
			if (bestDist > dist)
			{
				bestDist = dist;
				bestNode = checking->connections[i]->node;
				bestI = mapRef->width * bestNode->y + bestNode->x;
			}
		}

		pathSoFar = bestDist;
		checking = bestNode;
	}

	return path;
	*/
}

// Return list of points using A* algorhytm
std::vector<Node*> PathFinding::AStar(Play::Point2D start, Play::Point2D end, const char** map)
{
	std::vector<Node*> path;
	return path;
}

Node* PathFinding::NodeFromPostion(int x, int y)
{
	return mapGraph[mapRef->width * y + x];
}