#pragma once
#include "Entity.h"
#include "SteeringBehavior.h"

enum ESteeringBehavior
{
	None,
	Seek,
	Flee,
	Arrive,
	Pursue,
	Evade,
	Wander,
	FollowPath,
	Separation,
	CollisionAvoidance,
	WallAvoidance,
	GroupMovement,
	SteeringBehaviorCount
};

struct SteeringTarget
{
public:
	Point2D position = { 0,0 };
	Point2D prevPosition = { 0,0 };
	Point2D velocity = { 0,0 };
};

class Agent : public Entity
{
private:
	SteerTarget* predictTarget;
	float targetRadius = 1;
	float timeToTarget = 0.1f;
	SteeringBehavior* steeringBehavior;

	const char* const SPRITE = "ship";

public:
	SteerTarget* target;
	ESteeringBehavior steeringType = ESteeringBehavior::Seek;
	float maxVelocity = 60;
	float maxAcceleration = 100;

	Agent();
	Agent(Point2D startPos, SteeringBehavior* steeringBeh);
	~Agent();
	void Update(float dTime) override;
	void Draw() override;

	// Steering behavior
	void Steer();
	void Wander(float maxRotation);
};