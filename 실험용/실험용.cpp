#include "pch.h"

#pragma warning(push, 0)
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <btBulletCollisionCommon.h>
#pragma pop(pop)


void main()
{
	std::vector<RigidBody*> rigidBodys;
	RigidBody* tmp = rigidBodys.emplace_back(new RigidBody(D3DXVECTOR3(1.0f, 3.0f, 1.0f), D3DXQUATERNION(0.0f, 0.0f, 0.0f, 1.0f), D3DXVECTOR3(1.0f, 1.0f, 1.0f)));
	tmp->AddToWorld();
	tmp = rigidBodys.emplace_back(new RigidBody(D3DXVECTOR3(1.0f, 0.0f, 1.0f), D3DXQUATERNION(0.0f, 0.0f, 0.0f, 1.0f), D3DXVECTOR3(1000.0f, 1.0f, 1000.0f)));
	tmp->mass = 0.0f;
	tmp->AddToWorld();
	
	for (int i = 0; i < 300; i++)
	{
		Timer::Get()->Update();
		Physics_World::Get()->Update();

		for (auto& rigidBody : rigidBodys)
		{
			rigidBody->ApplyForce(btVector3(-1.0f, 0.0f, 0.0f));
			D3DXVECTOR3 force = ToD3DXVector3(rigidBody->rigidBody->getTotalForce());
			std::cout << "Total Force wa : " << force.x << " " << force.y << " " << force.z << std::endl;;
		}
	}
}


