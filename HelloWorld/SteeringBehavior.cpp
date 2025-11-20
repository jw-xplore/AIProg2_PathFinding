#include "SteeringBehavior.h"


Point2D SteeringBehavior::seek(SteerTarget* target, Point2D pos, float maxAccel)
{
	Point2D acceleration = target->position - pos;
	acceleration.Normalize();

	return acceleration * maxAccel;
}

Point2D SteeringBehavior::flee(SteerTarget* target, Point2D pos, float maxAccel)
{
	Point2D acceleration = pos - target->position;
	acceleration.Normalize();

	return acceleration * maxAccel;
}

Point2D SteeringBehavior::arrive(SteerTarget* target, Point2D pos, Point2D vel, float maxVel, float maxAccel, float radius, float timeToTarget)
{
	Point2D direction = target->position - pos;
	float distance = direction.Length();

	// Stop
	if (distance < radius)
	{
		return Point2D{ 0, 0 };
	}

	// Adjust velocity
	float slowRadius = 100;
	float targetSpeed;

	if (distance > slowRadius)
	{
		targetSpeed = maxVel;
	}
	else
	{
		targetSpeed = maxVel * distance / slowRadius;
	}

	Point2D targetVelocity = direction;
	targetVelocity.Normalize();
	targetVelocity *= targetSpeed;

	// Adjust acceleration
	Point2D acceleration = targetVelocity - vel;
	acceleration /= timeToTarget;

	if (acceleration.Length() > maxAccel)
	{
		acceleration.Normalize();
		acceleration = acceleration * maxAccel;
	}

	Play::DrawLine(pos, pos + acceleration, cOrange);
	return acceleration;
}

SteerTarget* SteeringBehavior::predictTarget(SteerTarget* target, Point2D pos, Point2D vel, float maxPredict)
{
	Point2D direction = target->position - pos;
	float distance = direction.Length();

	float speed = vel.Length();
	float prediction;

	if (speed <= distance / maxPredict)
	{
		prediction = maxPredict;
	}
	else
	{
		prediction = distance / maxPredict;
	}

	SteerTarget* newTarget = new SteerTarget();
	newTarget->position = target->position + target->velocity * prediction;

	// Debug
	Play::DrawCircle(target->position, 10, Play::cRed);
	Play::DrawCircle(newTarget->position, 10, Play::cGreen);

	return newTarget;
}
