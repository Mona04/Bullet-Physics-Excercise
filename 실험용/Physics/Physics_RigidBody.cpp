#include "pch.h"
#include "Physics_RigidBody.h"
#include "Physics_MotionState.h"
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <btBulletCollisionCommon.h>

RigidBody::RigidBody(const D3DXVECTOR3& pos, const D3DXQUATERNION& quat, const D3DXVECTOR3& size)
	: position(pos), rotation(quat), size(size)
	, localInertia(0.0f, 0.0f, 0.0f)
	, mass(1.0f)
	, friction(0.2f)
	, localFriction(0.2f)
	, restitution(0.2f)
{
}

RigidBody::~RigidBody()
{
	SAFE_DELETE(collisionShape);
	if (rigidBody)
	{
		Physics_World::Get()->GetWorld()->removeRigidBody(rigidBody);
		delete rigidBody->getMotionState();
		delete rigidBody;

		rigidBody = nullptr;
	}
}

void RigidBody::SetCollisionShape()
{
	collisionShape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
	collisionShape->setLocalScaling(ToBtVector3(size));
	collisionShape->calculateLocalInertia(mass, localInertia);

	collisionShape->setUserPointer(this); // do not used by bullet
}

void RigidBody::AddToWorld()
{
	SetCollisionShape();

	// Make rigidBody
	MotionState* motionState = new MotionState(this);

	btRigidBody::btRigidBodyConstructionInfo constructionInfo(mass, motionState, collisionShape, localInertia);
	constructionInfo.m_motionState = motionState;
	constructionInfo.m_collisionShape = collisionShape;
	constructionInfo.m_mass = mass;
	constructionInfo.m_friction = friction;
	constructionInfo.m_rollingFriction = localFriction;
	constructionInfo.m_restitution = restitution;   // 탄성
	constructionInfo.m_localInertia = localInertia;

	rigidBody = new btRigidBody(constructionInfo);
	rigidBody->setUserPointer(this);  // guid 처럼 identify 용이나 가변적임

	// collisionFlag
	rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
	rigidBody->forceActivationState(DISABLE_DEACTIVATION);
	rigidBody->setDeactivationTime(2000.0f);
	
	rigidBody->setFlags(rigidBody->getFlags() & ~BT_DISABLE_WORLD_GRAVITY);
	rigidBody->setGravity(Physics_World::Get()->GetGravity());
	
	btTransform& worldTrans = rigidBody->getWorldTransform();
	worldTrans.setOrigin(ToBtVector3(position + rotation * massCentor));
	worldTrans.setRotation(ToBtQuaternion(rotation));
	rigidBody->updateInertiaTensor();
	
	if (mass > 0)
	{
		rigidBody->setLinearFactor(btVector3(1.0f, 1.0f, 1.0f));
		rigidBody->setAngularFactor(btVector3(1.0f, 1.0f, 1.0f));
		rigidBody->activate(true);  // only synchronizes 'active' objects
	
	}
	else
	{
		rigidBody->setLinearFactor(btVector3(0.0f, 0.0f, 0.0f));
		rigidBody->setAngularFactor(btVector3(0.0f, 0.0f, 0.0f));
		rigidBody->activate(false); // only synchronizes 'active' objects
	}

	rigidBody->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
	rigidBody->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
	
	static int s = 0;
	//Physics_World::Get()->GetWorld()->addRigidBody(rigidBody); // 충돌 함
	Physics_World::Get()->GetWorld()->addRigidBody(rigidBody, 1, s++);  // 충돌 안함

}

void RigidBody::ApplyForce(const btVector3& force)
{
	rigidBody->applyCentralForce(force);
}

void RigidBody::Print()
{
	btTransform trans;
	if (rigidBody && rigidBody->getMotionState())
	{
		rigidBody->getMotionState()->getWorldTransform(trans);
	}
	printf("world pos object %f,%f,%f\n", float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
}
