#pragma once
#include "pch.h"
#include "Physics_Helper.h"

#pragma warning(push, 0)
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#pragma warning(pop)

using namespace BulletPhysicsHelper;

class MotionState : public btMotionState
{
public:
	MotionState(class RigidBody* rigidBody) : rigidBody(rigidBody) {}

	void getWorldTransform(btTransform& worldTrans) const override
	{
		worldTrans.setOrigin(ToBtVector3(rigidBody->position + rigidBody->rotation * rigidBody->massCentor));
		worldTrans.setRotation(ToBtQuaternion(rigidBody->rotation));

		//rigidBody->Print();
		//rigidBody->SetHasSimulated(true);
	}

	void setWorldTransform(const btTransform& worldTrans) override
	{

		rigidBody->rotation = ToD3DXQuaternion(worldTrans.getRotation());
		rigidBody->position = ToD3DXVector3(worldTrans.getOrigin()) - ToD3DXQuaternion(worldTrans.getRotation()) * rigidBody->massCentor;

		D3DXVECTOR3 position = rigidBody->position;
		D3DXQUATERNION quarternion = rigidBody->rotation;
		// No Scale.  Scale should be implemented in collision shape

		rigidBody->Print();
		//rigidBody->SetHasSimulated(true);
	}

private:
	RigidBody* rigidBody;
};