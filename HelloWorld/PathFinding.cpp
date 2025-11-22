#include "PathFinding.h"

PathFinding::PathFinding(const char** map, int width, int height)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			Node* node = new Node();
			node->x = x;
			node->y = y;

			AddConnectionsToNode(node, x, y, width, height, map);

			mapGraph.insert(mapGraph.end(), node);
		}
	}
}

void PathFinding::AddConnectionsToNode(Node* node, int x, int y, int width, int height, const char** map)
{
	// Top
	if (y < height && map[x][y+1] != 'X')
	{
		Connection* link = new Connection();
		link->x = x;
		link->y = y + 1;
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Right
	if (x < width && map[x+1][y] != 'X')
	{
		Connection* link = new Connection();
		link->x = x + 1;
		link->y = y;
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Bottom
	if (y > 0 && map[x][y - 1] != 'X')
	{
		Connection* link = new Connection();
		link->x = x;
		link->y = y - 1;
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Left
	if (x > 0 && map[x - 1][y] != 'X')
	{
		Connection* link = new Connection();
		link->x = x - 1;
		link->y = y;
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}
}

// Return list of points using A* algorhytm
std::vector<Node*> PathFinding::AStar(Play::Point2D start, Play::Point2D end, const char** map)
{
	std::vector<Node*> path;
	return path;
}