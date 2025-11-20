#pragma once
#include "Entity.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class MapRenderer : public Entity
{
private:
	const char** map;
	int tileSize = 16;
	int width = 0;
	int height = 0;

public:
	MapRenderer(const char* path);

	void Draw() override;

	bool LoadMap(const char* path);
};

