#include "pch.h"
#include "Physics_World.h"

#pragma warning(push, 0)
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#pragma pop(pop)



#define GRAVITY btVector3(0.0f, -9.81f, 0.0f)

Physics_World::Physics_World()
{
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	broadphase = new btDbvtBroadphase();
	constraintSolver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, constraintSolver, collisionConfiguration);

	//SetUp Physics World
	world->setGravity(GRAVITY);
	world->getDispatchInfo().m_useContinuous = true;
	world->getSolverInfo().m_splitImpulse = false;
	world->getSolverInfo().m_numIterations = 256;

	debugDraw = new PhysicsDebugDraw();
	world->setDebugDrawer(debugDraw);

	filterCB = new PhysicsFilterCallback();
	world->getPairCache()->setOverlapFilterCallback(filterCB);
}

Physics_World::~Physics_World()
{
	SAFE_DELETE(world);
	//SAFE_DELETE(constraintSolver);
	SAFE_DELETE(broadphase);
	SAFE_DELETE(dispatcher);
	SAFE_DELETE(collisionConfiguration);
	SAFE_DELETE(debugDraw);
	SAFE_DELETE(filterCB);
}

void Physics_World::Update()
{
	Timer* timer = Timer::Get();

	float delta_time = timer->GetDeltaTimeSec();
	float internal_time_step = 1.0f / 60.0f;
	int max_substeps = static_cast<int>(delta_time * 60.0f); 

	max_substeps = max_substeps > 1 ? max_substeps : 1;  // timeStep < maxSubSteps * fixedTimeStep

	world->stepSimulation(delta_time, max_substeps, internal_time_step);
	//world->debugDrawWorld();
}

btVector3 Physics_World::GetGravity()
{
	return world->getGravity();
}