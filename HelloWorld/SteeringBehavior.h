#pragma once
#include "Play.h"
#include "vector"
#include "Entity.h"

using namespace Play;

struct SteerTarget
{
public:
	Point2D position = { 0,0 };
	Point2D prevPosition = { 0,0 };
	Point2D velocity = { 0,0 };
};

class SteeringBehavior
{
public:
	std::vector<Entity*> separationObstacles;

	Point2D seek(SteerTarget* target, Point2D pos, float maxAccel);
	Point2D flee(SteerTarget* target, Point2D pos, float maxAccel);
	Point2D arrive(SteerTarget* target, Point2D pos, Point2D vel, float maxVel, float maxAccel, float radius, float timeToTarget);
	SteerTarget* predictTarget(SteerTarget* target, Point2D pos, Point2D vel, float maxPredict);
};