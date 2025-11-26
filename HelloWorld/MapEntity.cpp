#include "MapEntity.h"

MapEntity::MapEntity(const char* path)
{
	LoadMap(path);
}

void MapEntity::Draw()
{
	for (int y = 0; y < this->height; y++)
	{
		for (int x = 0; x < this->width; x++)
		{
			Colour color = cYellow;
			switch (this->map[y][x])
			{
			case 'X': color = cGrey; break; // Wall
			case 'S': color = cGreen; break; // Start
			case 'G': color = cRed; break; // Goal
			}

			Point2D bl = { x * this->tileSize, y * this->tileSize };
			Point2D tr = { (x+1) * this->tileSize, (y+1) * this->tileSize };
			Play::DrawRect(bl, tr, color, true);
		}
	}
}

bool MapEntity::LoadMap(const char* path)
{
	// Load file
	std::ifstream file;
	file.open(path);

	// Define array
	int lines = std::count(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>(), '\n');
	this->height = lines + 1;
	this->map = new const char* [lines];

	// Restart line reading
	int currentLine = 0;
	file.clear();
	file.seekg(0);

	// Read file
	if (file.is_open())
	{
		std::string line;

		while (std::getline(file, line))
		{
			char* cstr = new char[line.size()];
			for (size_t i = 0; i < line.size(); ++i) {
				cstr[i] = line[i];
			}

			map[currentLine] = cstr;

			currentLine++;
			this->width = line.size();
		}

		file.close();
		return true;
	}

	// Fail to read
	return false;
}