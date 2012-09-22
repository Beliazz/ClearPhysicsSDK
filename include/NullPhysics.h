#ifndef NullPhysics_H__
#define NullPhysics_H__

#include "ClearPhysicsSDK.h"

class CLEAR_PHYSICS_API NullPhysics : public IGamePhysics
{
public:
	NullPhysics() { }
	virtual ~NullPhysics() { }

	// Initialiazation and Maintenance of the Physics World
	virtual bool VInitialize() { return true; }
	virtual void VSyncVisibleScene() { };
	virtual void VOnUpdate( float ) { }

	void AddShape( IActor * actor, btCollisionShape * shape, btScalar mass, enum PhysicsMaterial mat ) {}

	// Initialization of Physics Objects
	virtual void VAddPlane(Vec planeNormal, float planeConstante, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) { }
	virtual void VAddSphere(float radius, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) { }
	virtual void VAddBox( Vec dimensions, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) { }
	virtual void VAddPointCloud(Vec *verts, int numPoints, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) { }
	virtual void VAddTriangleMesh(btCollisionShape* shape, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat) {}
	virtual void VSetCompoundShapeChild(ActorId parentID, IActor* childActor, unsigned int index) {}
	virtual void VAddCompoundShape(btCompoundShape* shape, IActor *actor, float specificGravity, enum PhysicsMaterial mat) {}
	virtual void VRemoveActor(ActorId id) { }
	virtual Vec  VRayCast( Vec start, Vec end, bool& hit ) {return Vec();}

	// Debugging
	virtual void VRenderDiagnostics(IScene* pScene) { }

	// Physics world modifiers
	virtual void VCreateTrigger(Vec pos, const float dim, int triggerID) { }
	virtual void VApplyForce(Vec dir, float newtons, ActorId aid) { }
	virtual void VApplyTorque(Vec dir, float newtons, ActorId aid) { }
	virtual bool VKinematicMove(Mat mat, ActorId aid) { return true; }

	// Physics actor states
	virtual void VRotateY(ActorId actorId, float angleRadians, float time) { }
	virtual float VGetOrientationY(ActorId actorId) { return 0.0f; }
	virtual void VStopActor(ActorId actorId) { }
	virtual void VSetVelocity(ActorId actorId, Vec vel) { }
	virtual void VTranslate(ActorId actorId, Vec vec) { }
	virtual void VStaticActor( ActorId actorId ) { }
};

#endif //PhyMathConversion_H__