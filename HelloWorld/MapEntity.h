#pragma once
#include "Entity.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class MapEntity : public Entity
{
public:
	const char** map;
	int tileSize = 16;
	int width = 0;
	int height = 0;

	MapEntity(const char* path);

	void Draw() override;

	bool LoadMap(const char* path);
};

