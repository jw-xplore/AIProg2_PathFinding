#include "EntityManager.h"

EntityManager::~EntityManager()
{
	// Delete and clear entity list
	for (int i = 0, size = entities.size(); i < size; i++)
	{
		delete entities[i];
	}

	entities.clear();
}

void EntityManager::UpdateEntities(float dTime)
{
	// Regular update
	for (int i = 0, size = entities.size(); i < size; i++)
	{
		entities[i]->Update(dTime);
	}
}

void EntityManager::AddEntity(Entity* entity)
{
	entity->SetId(idToAssign);
	idToAssign++;

	entities.insert(entities.end(), entity);
}

void EntityManager::AddEntities(std::initializer_list<Entity*> entitiesList)
{
	for (int i = 0; i < entitiesList.size(); i++)
	{
		AddEntity(entitiesList.begin()[i]);
	}
}

// Find selected entity and delete it
bool EntityManager::RemoveEntity(Entity* entity)
{
	for (int i = 0, size = entities.size(); i < size; i++)
	{
		// Remove found enetity from list and delete it
		if (entities[i] == entity)
		{
			entities.erase(entities.begin() + i);
			delete entity;
			return true; // Notify success
		}
	}

	// Entity couldn't be removed 
	return false;
}