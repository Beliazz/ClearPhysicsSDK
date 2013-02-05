#include "ClearPhysicsSDK.h"


cpl::CDynamicMotionState::CDynamicMotionState(ActorId actorID, ComponentId componentID, Mat initialTransform, fastdelegate::FastDelegate4< ActorId, ComponentId, Mat, Mat> onMovedDeligate)
{
	m_actorID		= 0;
	m_componentID	= 0;

	m_btGlobalTransform = Mat_to_btTransform( initialTransform );

	m_onMovedDeligate = onMovedDeligate;
}

cpl::CDynamicMotionState::~CDynamicMotionState()
{

}

void cpl::CDynamicMotionState::setActor(ActorId actorID, ComponentId componentID, Mat initialTransform)
{
	m_actorID		= actorID;
	m_componentID	= componentID;

	m_btGlobalTransform = Mat_to_btTransform( initialTransform );
}

void cpl::CDynamicMotionState::getWorldTransform( btTransform &worldTrans ) const
{
	worldTrans = m_btGlobalTransform;
}

void cpl::CDynamicMotionState::setWorldTransform( const btTransform &worldTrans )
{
	if(m_actorID == 0 || m_componentID == 0)
		return;

	btQuaternion rot = worldTrans.getRotation();
	btVector3	 pos = worldTrans.getOrigin();

	Mat rotation	 = MatRotationQuat( Quat(rot.x(), rot.y(), rot.z(), rot.w()) );
	Mat translation  = MatTranslation(btVector3_to_Vec(pos));

	m_onMovedDeligate( m_actorID, m_componentID, translation, rotation );
}

void cpl::CDynamicMotionState::kinematicMove( Mat transform )
{
	m_btGlobalTransform = Mat_to_btTransform(transform);
}
