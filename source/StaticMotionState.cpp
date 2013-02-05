#include "ClearPhysicsSDK.h"


CDynamicMotionState::CDynamicMotionState( IActor *pActor /*= NULL*/ )
{
	m_pActor		    = pActor;
	m_btGlobalTransform = Mat_to_btTransform( *m_pActor->GetGlobalMatrix() );
}

CDynamicMotionState::~CDynamicMotionState()
{

}

void CDynamicMotionState::setActor( IActor *pActor )
{
	m_pActor			= pActor;
	m_btGlobalTransform = Mat_to_btTransform( *m_pActor->GetGlobalMatrix() );
}

void CDynamicMotionState::getWorldTransform( btTransform &worldTrans ) const
{
	worldTrans = m_btGlobalTransform;
}

void CDynamicMotionState::setWorldTransform( const btTransform &worldTrans )
{
	if(NULL == m_pActor)
		return; // silently return before we set a node

	btTransform local			= Mat_to_btTransform( *m_pActor->GetLocalMatrix() );
	btTransform actorTransform	= worldTrans * local.inverse();


	btQuaternion rot = actorTransform.getRotation();
	btVector3	 pos = actorTransform.getOrigin();

	Mat  translation = MatTranslation( btVector3_to_Vec(pos) );
	Mat  rotation	 = MatRotationQuat( Quat(rot.x(), rot.y(), rot.z(), rot.w()) );

	m_pActor->SetRotationInternal(rotation);
	m_pActor->SetTranslationInternal(translation);
}

void CDynamicMotionState::kinematicMove( Mat transform )
{

}
