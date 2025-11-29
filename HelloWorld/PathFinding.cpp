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
	if (y < h - 1 && mapRef->map[y+1][x] != 'X')
	{
		Connection* link = new Connection();
		//link->x = x;
		//link->y = y + 1;
		link->fromNode = node;
		link->node = NodeFromPostion(x, y + 1);
		link->weight = 1;

		node->connections.insert(node->connections.end(), link);
	}

	// Right
	if (x < w - 1 && mapRef->map[y][x+1] != 'X')
	{
		Connection* link = new Connection();
		//link->x = x + 1;
		//link->y = y;
		link->fromNode = node;
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
		link->fromNode = node;
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
		link->fromNode = node;
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

	std::reverse(path.begin(), path.end());

	return path;
}

// Return list of points using A* algorhytm
std::vector<Node*> PathFinding::AStar(Play::Point2D start, Play::Point2D end)
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

	// Setup open and closed list
	std::vector<NodeRecordAs> open;
	open.insert(open.end(), startRecord);

	std::vector<NodeRecordAs> closed;
	NodeRecordAs current;

	while (open.size() != 0)
	{
		// Find smallest record - smallest estimated cost
		current = SmallestAsRecord(open);

		// Is at the end?
		if (current.node == endNode)
			break;

		// Loop through connections
		for (int i = 0; i < current.node->connections.size(); i++)
		{
			Connection* connection = current.node->connections[i];

			Node* currentNode = connection->node;
			NodeRecordAs* currentNodeRecord;
			float currentNodeCost = current.costSoFar + connection->weight;

			float currentNodeHeuristics;

			// Check node in closed list
			if (ContainsAsRecord(closed, currentNode))
			{
				// Check if there is shorter route
				currentNodeRecord = FindAsRecordFromNode(closed, currentNode);
				if (currentNodeRecord->costSoFar <= currentNodeCost)
					continue;

				// Remove from closed list if it is shortest path
				closed.erase(std::remove(closed.begin(), closed.end(), *currentNodeRecord), closed.end());

				currentNodeHeuristics = currentNodeRecord->costEstimated - currentNodeRecord->costSoFar;
			}
			else if (ContainsAsRecord(open, currentNode)) // Skip if the node is open and we’ve not found a better route
			{
				currentNodeRecord = FindAsRecordFromNode(open, currentNode);

				// Skip if route is not better
				if (currentNodeRecord->costSoFar <= currentNodeCost)
					continue;

				currentNodeHeuristics = connection->weight - currentNodeRecord->costSoFar;
			}
			else // Record unvisited node
			{
				currentNodeRecord = new NodeRecordAs();
				currentNodeRecord->node = currentNode;

				currentNodeHeuristics = ManhattanHeuristics(currentNode, endNode);
			}

			// Update node cost, estimate and connection
			currentNodeRecord->connection = connection;
			currentNodeRecord->costEstimated = currentNodeCost + currentNodeHeuristics;

			// Add to open list 
			if (!ContainsAsRecord(open, currentNode))
				open.insert(open.end(), *currentNodeRecord);
		}

		// Release current node when done with iterating through connections
		open.erase(std::remove(open.begin(), open.end(), current), open.end());
		closed.insert(closed.end(), current);
	}

	// Format path
	std::vector<Node*> path;

	// Failed to find end?
	if (current.node != endNode)
		return path; // Empty path
	
	// Track path
	while (current.node != startNode)
	{
		path.insert(path.end(), current.node);
		current = *FindAsRecordFromNode(closed, current.connection->fromNode);
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


std::vector<Node*> PathFinding::DepthFirst(Play::Point2D start, Play::Point2D end)
{
	// Find start and end
	int startI = mapRef->width * start.y + start.x;
	Node* startNode = NodeFromPostion(start.x, start.y);

	int endI = mapRef->width * end.y + end.x;
	Node* endNode = NodeFromPostion(end.x, end.y);

	// Initialize start node
	NodeRecordAs startRecord;
	startRecord.node = startNode;

	std::vector<NodeRecordAs> visited;
	//visited.insert(visited.end(), startRecord);

	NodeRecordAs current = startRecord;
	
	/*
	while (current.node != endNode)
	{
		
	}
	*/

	std::vector<Node*> path;
	DepthPath(visited, path, &current, endNode);

	// Debug draw
	for (int i = 0; i < path.size(); i++)
	{
		int halfSize = mapRef->tileSize * 0.5f;
		float x = path[i]->x * mapRef->tileSize + halfSize;
		float y = path[i]->y * mapRef->tileSize + halfSize;
		Play:Point2D pos = { x, y };
		Play::DrawCircle(pos, 4, Play::cGreen);
	}

	std::reverse(path.begin(), path.end());

	return path;
}

bool PathFinding::DepthPath(std::vector<NodeRecordAs>& visited, std::vector<Node*>& path, NodeRecordAs* current, Node* target)
{
	visited.insert(visited.end(), *current);

	for (int i = 0; i < current->node->connections.size(); i++)
	{
		Node* node = current->node->connections[i]->node;
		NodeRecordAs* record = FindAsRecordFromNode(visited, node);
		if (record != nullptr)
			continue;

		NodeRecordAs newRecord;
		newRecord.node = node;
		newRecord.connection = current->node->connections[i];

		bool found = DepthPath(visited, path, &newRecord, target);
		if (found || node == target)
		{
			path.insert(path.end(), newRecord.node);
			return true;
		}
	}

	return false;
}

std::vector<Node*> PathFinding::BreathFirst(Play::Point2D start, Play::Point2D end)
{
	// Find start and end
	int startI = mapRef->width * start.y + start.x;
	Node* startNode = NodeFromPostion(start.x, start.y);

	int endI = mapRef->width * end.y + end.x;
	Node* endNode = NodeFromPostion(end.x, end.y);

	// Solve
	std::vector<Node*> queue;
	queue.insert(queue.end(), startNode);

	std::vector<bool> visited;
	std::vector<Node*> search;

	for (int i = 0; i < mapGraph.size(); i++)
	{
		visited.insert(visited.end(), false);
		search.insert(search.end(), nullptr);
	}

	visited[startI] = true;

	while (queue.size() != 0)
	{
		Node* node = queue[0];
		queue.erase(std::remove(queue.begin(), queue.end(), queue[0]), queue.end());

		for (int i = 0; i < node->connections.size(); i++)
		{
			Node* child = node->connections[i]->node;

			int pos = mapRef->width * child->y + child->x;
			if (visited[pos] == false)
			{
				queue.insert(queue.end(), child);
				visited[pos] = true;
				search[pos] = node;
			}
		}
	}

	// Reconstruct
	std::vector<Node*> path;
	for (Node* at = endNode; at != nullptr; at = at)
	{
		path.push_back(at);
		int i = mapRef->width * at->y + at->x;;
		at = search[i];
	}

	std::reverse(path.begin(), path.end());

	// Debug draw
	for (int i = 0; i < path.size(); i++)
	{
		int halfSize = mapRef->tileSize * 0.5f;
		float x = path[i]->x * mapRef->tileSize + halfSize;
		float y = path[i]->y * mapRef->tileSize + halfSize;
		Play:Point2D pos = { x, y };
		Play::DrawCircle(pos, 4, Play::cBlue);
	}

	return path;
}

std::vector<Node*> PathFinding::RayPathSearch(Play::Point2D start, Play::Point2D end)
{
	// Find start and end
	int startI = mapRef->width * start.y + start.x;
	Node* startNode = NodeFromPostion(start.x, start.y);

	int endI = mapRef->width * end.y + end.x;
	Node* endNode = NodeFromPostion(end.x, end.y);

	int dir = 0;
	float xDir = 0;
	float yDir = 1;

	std::vector<Node*> path;
	path.insert(path.end(), startNode);
	int currentI = 0;

	bool* visited = new bool[mapGraph.size()];
	for (int i = 0; i < mapGraph.size(); i++)
	{
		visited[i] = false;
	}

	visited[startI] = true;

	while (path[currentI] != endNode)
	{
		// Trace back if there are no more nodes to visit
		bool allVisited = true;

		for (int i = 0; i < path[currentI]->connections.size(); i++)
		{
			Node* connected = path[currentI]->connections[i]->node;
			int ci = mapRef->width * connected->y + connected->x;
			if (visited[ci] == false)
			{
				allVisited = false;
				break;
			}
		}

		if (allVisited)
		{
			path.erase(std::remove(path.begin(), path.end(), path[currentI]), path.end());
			currentI--;

			if (currentI < 1)
				break;

			continue;
		}

		// Go in direction until reach end
		Node* inPath = nullptr;

		for (int i = 0; i < path[currentI]->connections.size(); i++)
		{
			Node* connected = path[currentI]->connections[i]->node;

			// Can continue in the direction?
			bool vis = visited[mapRef->width * connected->y + connected->x];

			if (connected->x == path[currentI]->x + xDir && connected->y == path[currentI]->y + yDir && !vis)
			{
				/*
				if (currentI == 15)
				{
					bool vis = visited[mapRef->width * connected->y + connected->x];
					int a = 0;
				}
				*/

				inPath = connected;
				break;
			}
		}
		
		/*
		bool invalid = true;

		if (inPath != nullptr)
			invalid = std::find(path.begin(), path.end(), inPath) == path.end();
			*/

		/*
		if (std::find(path.begin(), path.end(), inPath) == path.end())
		{
			path.erase(std::remove(path.begin(), path.end(), path[currentI]), path.end());
			currentI--;
			break;
		}
		*/


		// Change direction if can't continue
		if (inPath == nullptr)
		{
			int startDir = dir;
			dir++;

			if (dir > 3)
				dir = 0;

			switch (dir)
			{
			case 0: xDir = 0; yDir = 1; break;
			case 1: xDir = 1; yDir = 0; break;
			case 2: xDir = 0; yDir = -1; break;
			case 3: xDir = -1; yDir = 0; break;
			}

			continue;
		}

		// Continue to next
		path.insert(path.end(), inPath);
		int index = mapRef->width * inPath->y + inPath->x;
		visited[index] = true;

		currentI++;
	}

	delete visited;

	// Debug draw
	for (int i = 0; i < path.size(); i++)
	{
		int halfSize = mapRef->tileSize * 0.5f;
		float x = path[i]->x * mapRef->tileSize + halfSize;
		float y = path[i]->y * mapRef->tileSize + halfSize;
		Play:Point2D pos = { x, y };
		Play::DrawCircle(pos, 4, Play::cMagenta);
	}

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

bool PathFinding::ContainsAsRecord(const std::vector<NodeRecordAs>& list, Node* node)
{
	for (int i = 0; i < list.size(); i++)
	{
		if (list[i].node == node)
			return true;
	}

	return false;
}

NodeRecordAs* PathFinding::FindAsRecordFromNode(std::vector<NodeRecordAs>& list, Node* node)
{
	for (auto& record : list)
	{
		if (record.node == node)
			return &record;
	}

	return nullptr; 
}