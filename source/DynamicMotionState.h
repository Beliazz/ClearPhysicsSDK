#ifndef DynamicMotionState_H__
#define DynamicMotionState_H__

#include "ClearPhysicsSDK.h"

namespace cpl
{

class CLEAR_PHYSICS_API CDynamicMotionState : public btMotionState
{ 
public:
			 CDynamicMotionState(ActorId actorID, ComponentId componentID, Mat initialTransform, fastdelegate::FastDelegate4< ActorId, ComponentId, Mat, Mat> onMoveDeligate);
	virtual ~CDynamicMotionState();

	virtual void getWorldTransform(btTransform &worldTrans) const;
	virtual void setWorldTransform(const btTransform &worldTrans);

	void setActor(ActorId actorID, ComponentId componentID, Mat initialTransform);
	void kinematicMove(Mat transform);


protected:
	ActorId		m_actorID;
	ComponentId m_componentID;
	btTransform	m_btGlobalTransform;

	fastdelegate::FastDelegate4< ActorId, ComponentId, Mat, Mat> m_onMovedDeligate;
};

}

#endif