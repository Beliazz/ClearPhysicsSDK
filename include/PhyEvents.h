#ifndef PhyEvents_H__
#define PhyEvents_H__

#include "ClearPhysicsSDK.h"

enum NxShapeFlag;
class NxShape;
typedef NxShapeFlag NxTriggerFlag;

struct CLEAR_PHYSICS_API EvtData_PhysTrigger_Enter : public BaseEventData
{
	static const EventType sk_EventType;
	virtual const EventType & VGetEventType( void ) const
	{
		return sk_EventType;
	}

	explicit EvtData_PhysTrigger_Enter( int triggerID,
		optional<ActorId> other )
		: m_triggerID(triggerID),
		m_other(other)
	{}

	IEventDataPtr VCopy() const 
	{
		return IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_PhysTrigger_Enter(m_triggerID, m_other));
	}

	virtual LuaObject VGetLuaEventData(void) const
	{
		assert( ( true == m_bHasLuaEventData ) && "Can't get lua event data because it hasn't been built yet!  Call BulidLuaEventData() first!" );
		return m_LuaEventData;
	}

	virtual void VBuildLuaEventData(void);

	int m_triggerID;
	optional<ActorId> m_other;

private:
	LuaObject	m_LuaEventData;
};

struct CLEAR_PHYSICS_API EvtData_Phys_TogglePause : public BaseEventData
{
	static const EventType sk_EventType;
	virtual const EventType & VGetEventType( void ) const
	{
		return sk_EventType;
	}

	explicit EvtData_Phys_TogglePause()
	{
	}

	explicit EvtData_Phys_TogglePause( LuaObject srcData )
	{
		m_bHasLuaEventData = true;
	}

	IEventDataPtr VCopy() const 
	{
		return IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_Phys_TogglePause());
	}

	virtual LuaObject VGetLuaEventData(void) const
	{
		assert( ( true == m_bHasLuaEventData ) && "Can't get lua event data because it hasn't been built yet!  Call BulidLuaEventData() first!" );
		return m_LuaEventData;
	}

	virtual void VBuildLuaEventData(void);

private:
	LuaObject	m_LuaEventData;
};

struct CLEAR_PHYSICS_API EvtData_Phys_RenderDiagnostic : public BaseEventData
{
	static const EventType sk_EventType;
	virtual const EventType & VGetEventType( void ) const
	{
		return sk_EventType;
	}

	explicit EvtData_Phys_RenderDiagnostic(bool value)
	{
		m_value = value;
	}

	explicit EvtData_Phys_RenderDiagnostic( LuaObject srcData )
	{
		m_value = srcData.GetBoolean();

		m_bHasLuaEventData = true;
	}

	IEventDataPtr VCopy() const 
	{
		return IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_Phys_RenderDiagnostic(m_value));
	}

	virtual LuaObject VGetLuaEventData(void) const
	{
		assert( ( true == m_bHasLuaEventData ) && "Can't get lua event data because it hasn't been built yet!  Call BulidLuaEventData() first!" );
		return m_LuaEventData;
	}

	virtual void VBuildLuaEventData(void);
	bool		m_value;
private:

	LuaObject	m_LuaEventData;
};

struct CLEAR_PHYSICS_API EvtData_PhysTrigger_Leave : public BaseEventData
{
	static const EventType sk_EventType;
	virtual const EventType & VGetEventType( void ) const
	{
		return sk_EventType;
	}


	explicit EvtData_PhysTrigger_Leave( int triggerID,
		optional<ActorId> other )
		: m_triggerID(triggerID),
		m_other(other)
	{}

	virtual IEventDataPtr VCopy() const
	{
		return IEventDataPtr ( DEBUG_CLIENTBLOCK EvtData_PhysTrigger_Leave(m_triggerID, m_other) );
	}

	virtual LuaObject VGetLuaEventData(void) const
	{
		assert( ( true == m_bHasLuaEventData ) && "Can't get lua event data because it hasn't been built yet!  Call BulidLuaEventData() first!" );
		return m_LuaEventData;
	}

	virtual void VBuildLuaEventData(void);

	int m_triggerID;
	optional<ActorId> m_other;

private:
	LuaObject	m_LuaEventData;
};

struct CLEAR_PHYSICS_API EvtData_PhysCollision : public BaseEventData
{
	static const EventType sk_EventType;
	virtual const EventType & VGetEventType( void ) const
	{
		return sk_EventType;
	}

	explicit EvtData_PhysCollision(ActorId actorA, ActorId actorB, ContactInfoList  contactInfoList ) : 
		m_ActorA(actorA),
		m_ActorB(actorB),
		m_contactInfoList(contactInfoList)
	{}


	virtual IEventDataPtr VCopy() const
	{
		return IEventDataPtr ( DEBUG_CLIENTBLOCK EvtData_PhysCollision(m_ActorA, m_ActorB, m_contactInfoList));
	}

	virtual LuaObject VGetLuaEventData(void) const
	{
		assert( ( true == m_bHasLuaEventData ) && "Can't get lua event data because it hasn't been built yet!  Call BulidLuaEventData() first!" );
		return m_LuaEventData;
	}

	virtual void VBuildLuaEventData(void);

	ActorId			 m_ActorA;
	ActorId			 m_ActorB;
	ContactInfoList  m_contactInfoList;

private:
	LuaObject	m_LuaEventData;
};

struct CLEAR_PHYSICS_API EvtData_PhysSeparation : public BaseEventData
{
	static const EventType sk_EventType;
	virtual const EventType & VGetEventType( void ) const
	{
		return sk_EventType;
	}

	explicit EvtData_PhysSeparation(ActorId actorA,
		ActorId actorB
		)
		: m_ActorA(actorA)
		, m_ActorB(actorB)
	{}

	virtual IEventDataPtr VCopy() const
	{
		return IEventDataPtr ( DEBUG_CLIENTBLOCK EvtData_PhysSeparation(m_ActorA, m_ActorB) );
	}

	virtual LuaObject VGetLuaEventData(void) const
	{
		assert( ( true == m_bHasLuaEventData ) && "Can't get lua event data because it hasn't been built yet!  Call BulidLuaEventData() first!" );
		return m_LuaEventData;
	}

	virtual void VBuildLuaEventData(void);

	ActorId m_ActorA;
	ActorId m_ActorB;

private:
	LuaObject	m_LuaEventData;
};

struct CLEAR_PHYSICS_API EvtData_Move_Actor : public BaseEventData
{
	static const EventType sk_EventType;
	virtual const EventType & VGetEventType( void ) const
	{
		return sk_EventType;
	}

	explicit EvtData_Move_Actor( ActorId id, Mat mat)
	{
		m_Id = id;
		m_Mat = mat;
	}

	explicit EvtData_Move_Actor( std::istrstream & in )
	{
		in >> m_Id;
		for (int i=0; i<4; ++i)
		{
			for (int j=0; j<4; ++j)
			{
				in >> m_Mat.GetStorage().m[i][j];
			}
		}
	}

	virtual LuaObject VGetLuaEventData(void) const
	{
		assert( ( true == m_bHasLuaEventData ) && "Can't get lua event data because it hasn't been built yet!  Call BulidLuaEventData() first!" );
		return m_LuaEventData;
	}

	virtual void VBuildLuaEventData(void);

	virtual void VSerialize(std::ostrstream &out) const
	{
		out << m_Id << " ";
		m_Mat.Serialize( out );
	}

	virtual IEventDataPtr VCopy() const
	{
		return IEventDataPtr(DEBUG_CLIENTBLOCK EvtData_Move_Actor(m_Id, m_Mat));
	}

	ActorId m_Id;
	Mat		m_Mat;

private:
	LuaObject	m_LuaEventData;
};

struct CLEAR_PHYSICS_API EvtData_RayCast_Result : public BaseEventData
{
	static const EventType sk_EventType;
	virtual const EventType & VGetEventType( void ) const
	{
		return sk_EventType;
	}

	explicit EvtData_RayCast_Result( ActorId id,  Vec hitPoint,  Vec hitNormal )
		: m_id( id ), m_hitPoint(hitPoint), m_hitNormal(hitNormal)
	{
	}

	explicit EvtData_RayCast_Result( std::istream & in )
	{
		in >> m_id;
		m_hitPoint  = Vec(in);
		m_hitNormal = Vec(in);
	}

	virtual IEventDataPtr VCopy() const
	{
		return IEventDataPtr ( DEBUG_CLIENTBLOCK EvtData_RayCast_Result ( m_id, m_hitPoint, m_hitNormal ) );
	}
	virtual ~EvtData_RayCast_Result() {}
	virtual LuaObject VGetLuaEventData(void) const
	{
		assert( ( true == m_bHasLuaEventData ) && "Can't get lua event data because it hasn't been built yet!  Call BulidLuaEventData() first!" );
		return m_LuaEventData;
	}
	virtual void VBuildLuaEventData(void);
	virtual void VSerialize(std::ostream &out) const
	{		
		out << m_id;
		m_hitPoint.Serialize(out);
		m_hitNormal.Serialize(out);
	}

	ActorId m_id;
	Vec		m_hitPoint;
	Vec		m_hitNormal;

private:
	LuaObject	m_LuaEventData;
};


#endif //PhyMathConversion_H__