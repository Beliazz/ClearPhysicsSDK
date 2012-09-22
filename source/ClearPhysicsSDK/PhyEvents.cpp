#include "ClearPhysicsSDK.h"


const EventType EvtData_Move_Actor::sk_EventType( "move_actor" );

const EventType EvtData_RayCast_Result::sk_EventType( "raycast_result" );

const EventType EvtData_PhysTrigger_Enter::sk_EventType( "phys_trigger_enter" );

const EventType EvtData_PhysTrigger_Leave::sk_EventType( "phys_trigger_leave" );

const EventType EvtData_PhysCollision::sk_EventType( "phys_obj_collision" );

const EventType EvtData_PhysSeparation::sk_EventType( "phys_obj_separate" );

const EventType EvtData_Phys_TogglePause::sk_EventType( "phys_toggle_pause" );

const EventType EvtData_Phys_RenderDiagnostic::sk_EventType( "phys_render_diagnostic" );


void CLEAR_PHYSICS_API EvtData_Move_Actor::VBuildLuaEventData( void )
{
	assert( ( false == m_bHasLuaEventData ) && "Already built lua event data!" );

	//Get the global state.
	LuaState * pState = g_pAppLuaStateManager->GetGlobalState().Get();
	m_LuaEventData.AssignNewTable( pState );

	//Now assign the data.
	m_LuaEventData.SetInteger( "ActorId", m_Id );

	//We don't want a whole 4x4 matrix, so just give us the position.
	Vec srcPos = MatGetPosition( m_Mat ); 
	LuaObject posTable = m_LuaEventData.CreateTable( "Pos", 3 );
	posTable.SetNumber( 1, srcPos.GetX() );
	posTable.SetNumber( 2, srcPos.GetY() );
	posTable.SetNumber( 3, srcPos.GetZ() );

	m_bHasLuaEventData = true;
}

void CLEAR_PHYSICS_API EvtData_RayCast_Result::VBuildLuaEventData( void )
{
	assert( ( false == m_bHasLuaEventData ) && "Already built lua event data!" );

	//Get the global state.
	LuaState * pState = g_pAppLuaStateManager->GetGlobalState().Get();
	m_LuaEventData.AssignNewTable( pState );

	//Serialize the data necessary.
	m_LuaEventData.SetInteger( "ActorId", m_id );

	m_LuaEventData.SetNumber( "HitX", m_hitPoint.GetX() );
	m_LuaEventData.SetNumber( "HitY", m_hitPoint.GetY() );
	m_LuaEventData.SetNumber( "HitZ", m_hitPoint.GetZ() );

	m_LuaEventData.SetNumber( "HitNormalX", m_hitNormal.GetX() );
	m_LuaEventData.SetNumber( "HitNormalY", m_hitNormal.GetY() );
	m_LuaEventData.SetNumber( "HitNormalZ", m_hitNormal.GetZ() );

	m_bHasLuaEventData = true;
}

void CLEAR_PHYSICS_API EvtData_PhysSeparation::VBuildLuaEventData( void )
{
	assert( ( false == m_bHasLuaEventData ) && "Already built lua event data!" );

	//Get the global state.
	LuaState * pState = g_pAppLuaStateManager->GetGlobalState().Get();
	m_LuaEventData.AssignNewTable( pState );

	//Now provide the event data necessary.
	m_LuaEventData.SetInteger( "ActorA", m_ActorA );
	m_LuaEventData.SetInteger( "ActorB", m_ActorB );

	m_bHasLuaEventData = true;
}

void CLEAR_PHYSICS_API EvtData_PhysCollision::VBuildLuaEventData( void )
{
	assert( ( false == m_bHasLuaEventData ) && "Already built lua event data!" );

	//Get the global state.
	LuaState * pState = g_pAppLuaStateManager->GetGlobalState().Get();
	m_LuaEventData.AssignNewTable( pState );

	//Now provide the event data necessary.
	m_LuaEventData.SetInteger( "ActorA", m_ActorA );
	m_LuaEventData.SetInteger( "ActorB", m_ActorB );

	LuaObject normalForceObj = m_LuaEventData.CreateTable( "NormalForce" );
	normalForceObj.SetNumber( 1, m_SumNormalForce.GetX() );
	normalForceObj.SetNumber( 2, m_SumNormalForce.GetY() );
	normalForceObj.SetNumber( 3, m_SumNormalForce.GetZ() );

	LuaObject frictionForceObj = m_LuaEventData.CreateTable( "FrictionForce" );
	frictionForceObj.SetNumber( 1, m_SumFrictionForce.GetX() );
	frictionForceObj.SetNumber( 2, m_SumFrictionForce.GetY() );
	frictionForceObj.SetNumber( 3, m_SumFrictionForce.GetZ() );

	m_bHasLuaEventData = true;
}

void CLEAR_PHYSICS_API EvtData_PhysTrigger_Leave::VBuildLuaEventData( void )
{
	assert( ( false == m_bHasLuaEventData ) && "Already built lua event data!" );

	//Get the global state.
	LuaState * pState = g_pAppLuaStateManager->GetGlobalState().Get();
	m_LuaEventData.AssignNewTable( pState );

	//TODO JWC:  Alter this to make it work with new physics.

	m_bHasLuaEventData = true;
}

void CLEAR_PHYSICS_API EvtData_PhysTrigger_Enter::VBuildLuaEventData( void )
{
	assert( ( false == m_bHasLuaEventData ) && "Already built lua event data!" );

	//Get the global state.
	LuaState * pState = g_pAppLuaStateManager->GetGlobalState().Get();
	m_LuaEventData.AssignNewTable( pState );

	//TODO JWC:  Alter this to make it work with new physics.

	m_bHasLuaEventData = true;
}



void CLEAR_PHYSICS_API EvtData_Phys_TogglePause::VBuildLuaEventData( void )
{
	assert( ( false == m_bHasLuaEventData ) && "Already built lua event data!" );

	m_bHasLuaEventData = true;
}

void CLEAR_PHYSICS_API EvtData_Phys_RenderDiagnostic::VBuildLuaEventData( void )
{
	assert( ( false == m_bHasLuaEventData ) && "Already built lua event data!" );

	//Get the global state.
	LuaState * pState = g_pAppLuaStateManager->GetGlobalState().Get();
	m_LuaEventData.AssignNewTable( pState );

	//Now provide the event data necessary.
	m_LuaEventData.RawSetBoolean( "value", m_value );

	m_bHasLuaEventData = true;
}
