#include "PathFinding.h"

PathFinding::PathFinding(MapEntity* mapEntity)
{
	mapRef = mapEntity;

	for (int y = 0; y < mapEntity->height; y++)
	{
		for (int x = 0; x < mapEntity->width; x++)
		{
			if (mapEntity->map[y][x] == 'X')
				continue;

			Node* node = new Node();
			node->x = x;
			node->y = y;

			AddConnectionsToNode(node, x, y, mapEntity->width, mapEntity->height, mapEntity->map);

			mapGraph.insert(mapGraph.end(), node);
		}
	}
}

void PathFinding::AddConnectionsToNode(Node* node, int x, int y, int width, int height, const char** map)
{
	// Top
	if (y < height && map[y+1][x] != 'X')
	{
		Connection* link = new Connection();
		link->x = x;
		link->y = y + 1;
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Right
	if (x < width && map[y][x+1] != 'X')
	{
		Connection* link = new Connection();
		link->x = x + 1;
		link->y = y;
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Bottom
	if (y > 0 && map[y - 1][x] != 'X')
	{
		Connection* link = new Connection();
		link->x = x;
		link->y = y - 1;
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Left
	if (x > 0 && map[y][x - 1] != 'X')
	{
		Connection* link = new Connection();
		link->x = x - 1;
		link->y = y;
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
		Play::DrawCircle(nodePos, 6, cGreen);

		for (int n = 0; n < mapGraph[i]->connections.size(); n++)
		{
			Play::Point2D pos = { mapGraph[i]->connections[n]->x * mapRef->tileSize + halfSize, mapGraph[i]->connections[n]->y * mapRef->tileSize + halfSize };
			Play::DrawLine(nodePos, pos, cRed);
		}
	}
}

// Return list of points using A* algorhytm
std::vector<Node*> PathFinding::AStar(Play::Point2D start, Play::Point2D end, const char** map)
{
	std::vector<Node*> path;
	return path;
}