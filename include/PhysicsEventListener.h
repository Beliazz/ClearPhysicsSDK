#ifndef PhysicEventListener_h__
#define PhysicEventListener_h__

#include "ClearPhysicsSDK.h"

class CLEAR_PHYSICS_API PhysicsEventListener : public IEventListener
{
public:
	
	explicit PhysicsEventListener( IGamePhysics *pPhysics );
	virtual ~PhysicsEventListener();
	virtual char const * GetName(void) { return "PhysicsEventListener"; }
	virtual bool HandleEvent( IEventData const & event );

private:

	IGamePhysics *m_pPhysics;
};


#endif