#ifndef KinematicMotionState_H__
#define KinematicMotionState_H__

#include "ClearPhysicsSDK.h"


namespace cpl
{

class CLEAR_PHYSICS_API CKinematicMotionState : public btMotionState
{ 
public:
			 CKinematicMotionState(ActorId actorID, ComponentId componentID, Mat initialTransform);
	virtual ~CKinematicMotionState();

	virtual void getWorldTransform(btTransform &worldTrans) const;
	virtual void setWorldTransform(const btTransform &worldTrans);

	void setActor(ActorId actorID, ComponentId componentID, Mat initialTransform);
	void kinematicMove(Mat transform);


protected:
	ActorId		m_actorID;
	ComponentId m_componentID;
	btTransform	m_btGlobalTransform;
};

}


#endif