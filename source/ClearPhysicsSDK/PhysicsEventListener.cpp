#include "ClearPhysicsSDK.h"

CLEAR_PHYSICS_API PhysicsEventListener::PhysicsEventListener( IGamePhysics *pPhysics )
	: m_pPhysics(pPhysics)
{
}

CLEAR_PHYSICS_API PhysicsEventListener::~PhysicsEventListener()
{
}

bool CLEAR_PHYSICS_API PhysicsEventListener::HandleEvent( IEventData const & event )
{
	const EventType & eventType = event.VGetEventType();

	if ( EvtData_PhysTrigger_Enter::sk_EventType == eventType )
	{
		printf("EvtData_PhysTrigger_Enter\n");
	}
	else if ( EvtData_PhysTrigger_Leave::sk_EventType == eventType )
	{
		printf("EvtData_PhysTrigger_Leave\n");
	}
	else if ( EvtData_PhysCollision::sk_EventType == eventType )
	{
		printf("EvtData_PhysCollision\n");		
	}
	else if ( EvtData_PhysSeparation::sk_EventType == eventType )
	{
		printf("EvtData_PhysSeparation\n");
	}
	else if ( EvtData_Phys_TogglePause::sk_EventType == eventType )
	{
		m_pPhysics->TogglePause();
	}

	return false;
}

