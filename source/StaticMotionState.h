#ifndef DynamicMotionState_H__
#define DynamicMotionState_H__

#include "ClearPhysicsSDK.h"

class CLEAR_PHYSICS_API CDynamicMotionState : public btMotionState
{ 
public:
			 CDynamicMotionState(IActor *pActor = NULL);
	virtual ~CDynamicMotionState();

	virtual void getWorldTransform(btTransform &worldTrans) const;
	virtual void setWorldTransform(const btTransform &worldTrans);

	void setActor(IActor *pActor);
	void kinematicMove(Mat transform);


protected:
	IActor*		m_pActor;
	btTransform	m_btGlobalTransform;
	Mat			m_LocalTransform;
	Mat			m_GlobalTransform;
};

#endif