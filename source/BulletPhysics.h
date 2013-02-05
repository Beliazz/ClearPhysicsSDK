#ifndef BulletPhysics_H__
#define BulletPhysics_H__

#include "ClearPhysicsSDK.h"

namespace cpl
{

class CLEAR_PHYSICS_API BulletPhysics : public IPhysic
{
public:
	std::auto_ptr<btDynamicsWorld>				   m_dynamicsWorld;
	std::auto_ptr<btBroadphaseInterface>           m_broadphase;
	std::auto_ptr<btCollisionDispatcher>           m_dispatcher;
	std::auto_ptr<btConstraintSolver>              m_solver;
	std::auto_ptr<btDefaultCollisionConfiguration> m_collisionConfiguration;

	bool m_bRun;
	bool m_bDebugDrawWorld;


	//Profile Times
	float m_fStepSimulationTime;
	float m_fSyncVisualSceneTime;
	float m_fUpdateKinematicControllerTime;
	float m_fDebugDrawWorldTime;

	std::map<ActorId,map<ComponentId,btRigidBody*>> m_actorMap;

	typedef std::map<btRigidBody const *, pair<ActorId,ComponentId>> RigidBodyIDPairMap;
	RigidBodyIDPairMap m_rigidBodyToActorId;

	std::map<string, OnMovedDeligate>			 m_onMovedSubscriberMap;
	std::map<string, OnPhysTriggerEnterDeligate> m_onPhysTriggerEnterSubscriberMap;
	std::map<string, OnCollisionDeligate>		 m_onCollisionSubscriberMap;


	typedef std::pair< btRigidBody const *, btRigidBody const * > CollisionPair;
	typedef std::set< CollisionPair > CollisionPairs;
	CollisionPairs m_previousTickCollisionPairs;

	// helpers for sending events relating to collision pairs
	void SendCollisionPairAddEvent( btPersistentManifold const * manifold, btRigidBody const * body0, btRigidBody const * body1 );
	void SendCollisionPairRemoveEvent( btRigidBody const * body0, btRigidBody const * body1 );


	// helper for cleaning up objects
	void RemoveCollisionObject( btCollisionObject * removeMe );

	// find the BulletActor object with the given actor ID
	btRigidBody* FindComponentBody(ActorId id, ComponentId compId) const;

	// find the actor ID associated with the given body
	ActorId FindActorID( btRigidBody const * ) const;

	// callback from bullet for each physics time step.  set in VInitialize
	static void BulletInternalTickCallback( btDynamicsWorld * const world, btScalar const timeStep );

	void OnMoved( ActorId actorID, ComponentId componentID, Mat translation, Mat rotation);
	void OnPhysTriggerEnter( ActorId actorID_A, ComponentId componentID_A, ActorId actorID_B, ComponentId componentID_B );
	void OnCollision( ActorId actorID_A, ComponentId componentID_A, ActorId actorID_B, ComponentId componentID_B, vector<ContactInfo> contactInfo);

public:
	BulletPhysics();
	virtual ~BulletPhysics();

	// Initialization and Maintenance of the Physics World
	virtual bool VInitialize();
	virtual void VOnUpdate( float deltaSeconds ); 

	void AddShape( ActorId actorID, ComponentId componentID, Mat initialTransform, btCollisionShape * shape, ePhysicBodyType type, btScalar mass, enum PhysicsMaterial mat );
	void AddShape( ActorId actorID, ComponentId componentID, Mat initialTransform, btCollisionShape * shape, ePhysicBodyType type, short group, short mask, btScalar mass, enum PhysicsMaterial mat );

	// Initialization of Physics Objects
	virtual void VAddPlane(Vec planeNormal, float planeConstante, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddPlane(Vec planeNormal, float planeConstante, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat);

	virtual void VAddSphere(float radius, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddSphere(float radius, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat);

	virtual void VAddBox(Vec dimensions, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddBox(Vec dimensions, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat);

	virtual void VAddCapsule(float radius, float height, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddCapsule(float radius, float height, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat);

	virtual void VAddMesh(float* pVertices, unsigned int numVertices, DWORD* pIndices, unsigned int numIndices, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddMesh(float* pVertices, unsigned int numVertices, DWORD* pIndices, unsigned int numIndices, 
						   ActorId actorID, ComponentId componentID, Mat initialTransform, ePhysicBodyType type, short group, short mask, float specificGravity, enum PhysicsMaterial mat);


	//Physics remove actor or single components
	virtual void VRemoveComponent(ActorId id, ComponentId compId );
	virtual void VRemoveActor(ActorId id);


	//Physics world modifiers
	virtual void VCreateTrigger(Vec pos, const float dim, int triggerID);

	virtual void VApplyForce(Vec dir, float newtons, ActorId aid, ComponentId compId);
	virtual void VApplyTorque(Vec dir, float newtons, ActorId aid, ComponentId compId);

	virtual void VSetVelocity(ActorId actorId, ComponentId compId, Vec vel);
	virtual void VSetAngularVelocity(ActorId actorId, ComponentId compId, Vec vel);

	virtual bool VKinematicMove(Mat mat, ActorId aid, ComponentId compId);

	virtual void VTranslate(ActorId actorId, ComponentId compId, Vec vec);
	virtual void VRotate(ActorId actorId, ComponentId compId, Vec vec);

	virtual void VStopComponent(ActorId actorId,ComponentId compId);
	virtual void VStopActor(ActorId actorId);

	virtual Vec VGetVelocity(ActorId actorId, ComponentId compId);
	virtual Vec VGetAngularVelocity(ActorId actorId, ComponentId compId);


	virtual void SubscribeOnMoved(string name, OnMovedDeligate onMovedDeligate);
	virtual void UnSubscribeOnMoved(string name);

	virtual void SubscribeOnTriggerEnter(string name, OnPhysTriggerEnterDeligate onPhysTriggerEnterDeligate);
	virtual void UnSubscribeOnTriggerEnter(string name);

	virtual void SubscribeOnCollision(string name, OnCollisionDeligate onCollisionDeligate);
	virtual void UnSubscribeOnCollision(string name);


	//Physic functions
	virtual Vec  VRayCast( Vec start, Vec end, bool& hit );
	virtual void TogglePause();


	// Debugging
	virtual void DebugDrawWorld(bool value); 
	virtual void VSetDebugMode( int mode );
	virtual void SetDebugDrawer( btIDebugDraw* pDebugDraw );
};

extern IPhysic CLEAR_PHYSICS_API *CreateGamePhysics();
extern IPhysic CLEAR_PHYSICS_API *CreateNullPhysics();

}

#endif //PhyMathConversion_H__