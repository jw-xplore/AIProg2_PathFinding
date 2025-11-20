/*
Entity manager handles creation, update, management and deleting of entities.
Each added entity is added to entities list and has assigned id.
Deleting entity manager handles deleting of registered entities.
*/

#pragma once
#include <vector>
#include "Entity.h"

class EntityManager
{
private:
	int idToAssign = 0; // Last assigned id + 1
protected:
	std::vector<Entity*> entities;

public:
	~EntityManager();
	void UpdateEntities(float dTime);
	void AddEntity(Entity* entity);
	void AddEntities(std::initializer_list<Entity*> entitiesList);
	bool RemoveEntity(Entity* entity);
};