#define PLAY_IMPLEMENTATION
#define PLAY_USING_GAMEOBJECT_MANAGER
#include "Play.h"
#include "EntityManager.h"
#include "Agent.h"
#include "MapEntity.h"
#include "PathFinding.h"

EntityManager* entityManager;
Agent* agent;

SteeringBehavior* steeringBehavior;
MapEntity* mapEntity;
PathFinding* pathFinding;

// The entry point for a PlayBuffer program
void MainGameEntry( PLAY_IGNORE_COMMAND_LINE )
{
	Play::CreateManager( DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_SCALE );

	// Setup entity manager and entities
	entityManager = new EntityManager();
	steeringBehavior = new SteeringBehavior();
	mapEntity = new MapEntity("Data/Maps/Map1.txt");

	agent = new Agent({100, 100}, steeringBehavior);
	entityManager->AddEntity(agent);
	entityManager->AddEntity(mapEntity);

	pathFinding = new PathFinding(mapEntity);

	Play::CentreAllSpriteOrigins();
}

// Called by PlayBuffer every frame (60 times a second!)
bool MainGameUpdate( float elapsedTime )
{
	Play::ClearDrawingBuffer( Play::cBlack );
	entityManager->UpdateEntities(elapsedTime);

	pathFinding->DrawGraph();

	Play::PresentDrawingBuffer();
	return Play::KeyDown( KEY_ESCAPE );
}

// Gets called once when the player quits the game 
int MainGameExit( void )
{
	Play::DestroyManager();
	return PLAY_OK;
}

