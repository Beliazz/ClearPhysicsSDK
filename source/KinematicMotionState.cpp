#include "ClearPhysicsSDK.h"


cpl::CKinematicMotionState::CKinematicMotionState(ActorId actorID, ComponentId componentID, Mat initialTransform)
{
	m_actorID		= 0;
	m_componentID	= 0;

	m_btGlobalTransform = Mat_to_btTransform( initialTransform );
}

cpl::CKinematicMotionState::~CKinematicMotionState()
{

}

void cpl::CKinematicMotionState::setActor(ActorId actorID, ComponentId componentID, Mat initialTransform)
{
	m_actorID		= actorID;
	m_componentID	= componentID;

	m_btGlobalTransform = Mat_to_btTransform( initialTransform );
}

void cpl::CKinematicMotionState::getWorldTransform( btTransform &worldTrans ) const
{
	worldTrans = m_btGlobalTransform;
}

void cpl::CKinematicMotionState::setWorldTransform( const btTransform &worldTrans )
{
}

void cpl::CKinematicMotionState::kinematicMove( Mat transform )
{
	m_btGlobalTransform = Mat_to_btTransform(transform);
}
