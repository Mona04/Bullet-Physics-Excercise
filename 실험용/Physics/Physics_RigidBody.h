#pragma once
#include "pch.h"

class RigidBody
{
public:
	RigidBody(const D3DXVECTOR3& pos, const D3DXQUATERNION& quat, const D3DXVECTOR3& size);
	~RigidBody();

	void SetCollisionShape();
	void AddToWorld();
	
	void ApplyForce(const btVector3& force);
	void Print();

public:
	class btCollisionShape* collisionShape;
	class btRigidBody* rigidBody;
	class MotionState* motionState;

	D3DXVECTOR3 position;
	D3DXQUATERNION rotation;
	D3DXVECTOR3 massCentor;
	D3DXVECTOR3 size;

	btVector3 localInertia;
	
	btScalar mass;
	btScalar friction;
	btScalar localFriction;
	btScalar restitution;
};