#include "Agent.h"

Agent::Agent()
{
	this->steeringBehavior = new SteeringBehavior();
	this->position = { 100, 100};

	// Setup target
	this->target = new SteerTarget();
	this->target->position = Play::GetMousePos();
	this->target->prevPosition = Play::GetMousePos();
}

Agent::Agent(Point2D startPos, SteeringBehavior* steeringBeh)
{
	this->position = startPos;

	this->steeringBehavior = steeringBeh;
	this->steeringBehavior->separationObstacles.insert(this->steeringBehavior->separationObstacles.end(), this);

	// Setup target
	this->target = new SteerTarget();
	this->target->position = Play::GetMousePos();
	this->target->prevPosition = Play::GetMousePos();
}

Agent::~Agent()
{
	delete this->target;
	delete this->steering;
}

void Agent::Update(float dTime)
{
	// Update target position
	//this->target->position = Play::GetMousePos();
	this->target->velocity = this->target->position - this->target->prevPosition;
	this->target->prevPosition = this->target->position;

	// Update agent
	this->Steer();

	// Rotate
	if (this->velocity.x != 0 && this->velocity.y != 0)
		this->orientation = atan2(this->velocity.y, this->velocity.x);

	Entity::Update(dTime);

	// Velocity cap
	if (this->velocity.Length() > this->maxVelocity)
	{
		this->velocity.Normalize();
		this->velocity *= this->maxVelocity;
	}
}

void Agent::Draw()
{
	DrawSpriteRotated(SPRITE, this->position, 0, this->orientation, 1);
}

/*
* Steering behavior
*/

void Agent::Steer()
{
	switch (this->steeringType)
	{
	case ESteeringBehavior::Seek:
		this->steering->linear = steeringBehavior->seek(this->target, this->position, this->maxAcceleration);
		break;
	case ESteeringBehavior::Flee:
		this->steering->linear = steeringBehavior->flee(this->target, this->position, this->maxAcceleration);
		break;
	case ESteeringBehavior::Arrive:
		this->steering->linear = steeringBehavior->arrive(this->target, this->position, this->velocity, this->maxVelocity, this->maxAcceleration, 1, 0.1f);
		break;
	case ESteeringBehavior::Pursue:
		this->predictTarget = steeringBehavior->predictTarget(this->target, this->position, this->velocity, 10);
		this->steering->linear = steeringBehavior->seek(this->predictTarget, this->position, this->maxAcceleration);
		break;
	case ESteeringBehavior::Evade:
		predictTarget = steeringBehavior->predictTarget(this->target, this->position, this->velocity, 10);
		this->steering->linear = steeringBehavior->flee(predictTarget, this->position, this->maxAcceleration);
		break;
	case ESteeringBehavior::Wander:
		this->Wander(25);
		break;
	}
}

void Agent::Wander(float maxRotation)
{
	float change = RandomRollRange(-maxRotation, maxRotation);
	change = DegToRad(change);
	float targAngle = atan2(this->velocity.y, this->velocity.x);
	targAngle += change;

	Point2D targPoint = this->position + Point2D(cos(targAngle), sin(targAngle));
	Point2D acceleration = targPoint - this->position;
	acceleration.Normalize();

	this->steering->linear = acceleration * maxAcceleration;
}