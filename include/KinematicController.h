#ifndef KinematicController_H__
#define KinematicController_H__

#include "ClearPhysicsSDK.h"

class CLEAR_PHYSICS_API CKinematicController
{ 
public: 
	CKinematicController(shared_ptr<IActor> actor);
	~CKinematicController();

private:
	shared_ptr<IActor> m_actor;
};

#endif