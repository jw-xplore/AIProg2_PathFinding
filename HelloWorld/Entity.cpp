#include "Entity.h"

Entity::Entity()
{
	steering = new SteeringOutput();
}

Entity::~Entity()
{
	delete steering;
}

// Update basic entity behavior
// This base should be used when overriding method
void Entity::Update(float dTime)
{
	// Apply velocity on position change
	this->position += this->velocity * dTime;
	this->orientation += this->rotation * dTime;

	this->velocity += this->steering->linear * dTime;
	this->rotation += this->steering->angular * dTime;

	StayInScreenSpace();

	// Base behavior
	Draw();
}

void Entity::Destroy()
{
	delete this;
}

// Set id if it has not been set (this->id == -1)
// Returns true only if id was not set and parameter id is positive
bool Entity::SetId(int id)
{
	if (this->id != -1 or id < 0)
		return false;

	this->id = id;
	return true;
}

int Entity::GetId()
{
	return id;
}

// Returns absoulute position in game world
Point2D Entity::GetPosition()
{
	return position + pivotPoint;
}

// Set absolute position to point
void Entity::SetPosition(Point2D position)
{
	this->position = position;
}

// Set absolute position to x, y coordinates
void Entity::SetPosition(float x, float y)
{
	this->position = { x, y };
}

// Move absolute position by point
void Entity::Translate(Point2D position)
{
	this->position += position;
}

// Move absolute position by x, y coordinates
void Entity::Translate(float x, float y)
{
	this->position += { x, y };
}

Point2D Entity::GetVelocity()
{
	return velocity;
}

void Entity::SetVelocity(Point2D velocity)
{
	this->velocity = velocity;
}

// Return given point as normalized vector without modification of the point
const Point2D Entity::Normalized(Point2D point)
{
	float l = point.Length();
	if (l == 0)
		return { 0, 0 };

	return { point.x / l, point.y / l };
}

void Entity::StayInScreenSpace()
{
	// Check x borders
	if (position.x < 0)
		position.x = DISPLAY_WIDTH;
	else if (position.x > DISPLAY_WIDTH)
		position.x = 0;

	// Check y borders
	if (position.y < 0)
		position.y = DISPLAY_HEIGHT;
	else if (position.y > DISPLAY_HEIGHT)
		position.y = 0;
}