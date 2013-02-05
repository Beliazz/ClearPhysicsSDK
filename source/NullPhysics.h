#ifndef NullPhysics_H__
#define NullPhysics_H__

#include "ClearPhysicsSDK.h"

namespace cpl
{


class NullPhysics : public IPhysic
{
public:
	NullPhysics() {}
	virtual ~NullPhysics() { }

	// Initialization and Maintenance of the Physics World
	virtual bool VInitialize() { return true; }
	virtual void VOnUpdate( float deltaSeconds ) {}

	// Initialization of Physics Objects
	virtual void VAddPlane(Vec planeNormal, float planeConstante, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat){};
	virtual void VAddPlane(Vec planeNormal, float planeConstante, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat){};

	virtual void VAddSphere(float radius, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat){};
	virtual void VAddSphere(float radius, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat){};

	virtual void VAddBox(Vec dimensions, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat){};
	virtual void VAddBox(Vec dimensions, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat){};

	virtual void VAddCapsule(float radius, float height, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat){};
	virtual void VAddCapsule(float radius, float height, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat){};

	virtual void VAddMesh(float* pVertices, unsigned int numVertices, DWORD* pIndices, unsigned int numIndices, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat){};
	virtual void VAddMesh(float* pVertices, unsigned int numVertices, DWORD* pIndices, unsigned int numIndices, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat){};

		virtual void SubscribeOnMoved(string name, OnMovedDeligate onMovedDeligate){}
	virtual void UnSubscribeOnMoved(string name){}

	virtual void SubscribeOnTriggerEnter(string name, OnPhysTriggerEnterDeligate onPhysTriggerEnterDeligate){}
	virtual void UnSubscribeOnTriggerEnter(string name){}

	virtual void SubscribeOnCollision(string name, OnCollisionDeligate onCollisionDeligate){}
	virtual void UnSubscribeOnCollision(string name){}

	//Physics remove actor or single components
	virtual void VRemoveComponent(ActorId id, ComponentId compId ) {}
	virtual void VRemoveActor(ActorId id) {}

	//Physics world modifiers
	virtual void VCreateTrigger(Vec pos, const float dim, int triggerID) {}

	virtual void VApplyForce(Vec dir, float newtons, ActorId aid, ComponentId compId) {}
	virtual void VApplyTorque(Vec dir, float newtons, ActorId aid, ComponentId compId) {}

	virtual void VSetVelocity(ActorId actorId, ComponentId compId, Vec vel) {}
	virtual void VSetAngularVelocity(ActorId actorId, ComponentId compId, Vec vel) {}

	virtual bool VKinematicMove(Mat mat, ActorId aid, ComponentId compId) { return true; }

	virtual void VTranslate(ActorId actorId, ComponentId compId, Vec vec) {}
	virtual void VRotate(ActorId actorId, ComponentId compId, Vec vec) {}

	virtual void VStopComponent(ActorId actorId,ComponentId compId) {}
	virtual void VStopActor(ActorId actorId) {}

	virtual Vec VGetVelocity(ActorId actorId, ComponentId compId) { return Vec(); }
	virtual Vec VGetAngularVelocity(ActorId actorId, ComponentId compId) { return Vec(); }

	//Physic functions
	virtual Vec  VRayCast( Vec start, Vec end, bool& hit ) { return Vec(); }
};

}

#endif //PhyMathConversion_H__