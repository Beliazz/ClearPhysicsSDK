#ifndef IPhysic_H__
#define IPhysic_H__


#include "ClearPhysicsSDK.h"

namespace cpl
{

struct ContactInfo
{
	Vec ptA;
	Vec ptB;
	Vec normalOnB;
};

typedef std::list<Vec>		   VecList;
typedef std::list<ContactInfo> ContactInfoList;

typedef unsigned int ActorId;
typedef unsigned int ComponentId;

typedef fastdelegate::FastDelegate4< ActorId, ComponentId, Mat, Mat>								  OnMovedDeligate;
typedef fastdelegate::FastDelegate4< ActorId, ComponentId, ActorId, ComponentId>					  OnPhysTriggerEnterDeligate;
typedef fastdelegate::FastDelegate5< ActorId, ComponentId, ActorId, ComponentId, vector<ContactInfo>> OnCollisionDeligate;

enum ePhysicBodyType
{
	ePhysicBodyType_Dynamic = 0,
	ePhysicBodyType_Kinematic,
};

class IPhysic
{
public:
	// Initialization and Maintenance of the Physics World
	virtual bool VInitialize()=0;
	virtual void VOnUpdate( float deltaSeconds ) = 0; 

	// Initialization of Physics Objects
	virtual void VAddPlane(Vec planeNormal, float planeConstante, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat)=0;
	virtual void VAddPlane(Vec planeNormal, float planeConstante, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat)=0;

	virtual void VAddSphere(float radius, 
							ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat)=0;
	virtual void VAddSphere(float radius, 
							ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat)=0;

	virtual void VAddBox(Vec dimensions, 
							ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat)=0;
	virtual void VAddBox(Vec dimensions, 
							ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat)=0;
	
	virtual void VAddCapsule(float radius, float height, 
							 ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat)=0;
	virtual void VAddCapsule(float radius, float height, 
							 ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat)=0;
	
	virtual void VAddMesh(float* pVertices, unsigned int numVertices, DWORD* pIndices, unsigned int numIndices, 
							ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat)=0;
	virtual void VAddMesh(float* pVertices, unsigned int numVertices, DWORD* pIndices, unsigned int numIndices, 
							ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat)=0;


	virtual void SubscribeOnMoved(string name, OnMovedDeligate onMovedDeligate)=0;
	virtual void UnSubscribeOnMoved(string name)=0;

	virtual void SubscribeOnTriggerEnter(string name, OnPhysTriggerEnterDeligate onPhysTriggerEnterDeligate)=0;
	virtual void UnSubscribeOnTriggerEnter(string name)=0;

	virtual void SubscribeOnCollision(string name, OnCollisionDeligate onCollisionDeligate)=0;
	virtual void UnSubscribeOnCollision(string name)=0;

	//Physics remove actor or single components
	virtual void VRemoveComponent(ActorId id, ComponentId compId )=0;
	virtual void VRemoveActor(ActorId id)=0;

	//Physics world modifiers
	virtual void VCreateTrigger(Vec pos, const float dim, int triggerID)=0;

	virtual void VApplyForce(Vec dir, float newtons, ActorId aid, ComponentId compId)=0;
	virtual void VApplyTorque(Vec dir, float newtons, ActorId aid, ComponentId compId)=0;

	virtual void VSetVelocity(ActorId actorId, ComponentId compId, Vec vel) = 0;
	virtual void VSetAngularVelocity(ActorId actorId, ComponentId compId, Vec vel) = 0;

	virtual bool VKinematicMove(Mat mat, ActorId aid, ComponentId compId) = 0;

	virtual void VTranslate(ActorId actorId, ComponentId compId, Vec vec) = 0;
	virtual void VRotate(ActorId actorId, ComponentId compId, Vec vec) = 0;

	virtual void VStopComponent(ActorId actorId,ComponentId compId) = 0;
	virtual void VStopActor(ActorId actorId) = 0;

	virtual Vec VGetVelocity(ActorId actorId, ComponentId compId) = 0;
	virtual Vec VGetAngularVelocity(ActorId actorId, ComponentId compId) = 0;

	//Physic functions
	virtual Vec  VRayCast( Vec start, Vec end, bool& hit ) =0;
	virtual void TogglePause(){}

	// Debugging
	virtual void DebugDrawWorld(bool value) {};
	virtual void VSetDebugMode( int mode ) {};

	virtual ~IPhysic() { };
};

}

#endif