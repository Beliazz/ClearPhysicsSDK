#ifndef BulletPhysics_H__
#define BulletPhysics_H__

#include "ClearPhysicsSDK.h"



struct CLEAR_PHYSICS_API ActorMotionState : public btMotionState
{
	Mat m_worldToPositionTransform;

	ActorMotionState( Mat startingTransform );

	// btMotionState interface:  Bullet calls these
	virtual void getWorldTransform( btTransform& worldTrans ) const;
	virtual void setWorldTransform( const btTransform& worldTrans );
};

typedef btHingeConstraint CLEAR_PHYSICS_API RotatorConstraint;
struct CLEAR_PHYSICS_API BulletActor
{
	btRigidBody* m_pRigidBody;  // the rigid body associted with this actor (should never be NULL)

	float m_desiredDeltaYAngle;
	float m_desiredDeltaYAngleTime;

	Mat*  m_pActorMat;

	explicit BulletActor(btRigidBody* pRigidBody, Mat* pActorMat) 
	{ 
		m_pRigidBody = pRigidBody; 

		m_desiredDeltaYAngle = 0;
		m_desiredDeltaYAngleTime = 0;
		m_pActorMat= pActorMat;
	}
};

struct CLEAR_PHYSICS_API BulletCompoundShapeChildActor
{
	btCompoundShape* m_pCompoundShape;  // the rigid body associted with this actor (should never be NULL)

	int m_compoundShapeIndex;

 	Mat*  m_pActorMat;
// 
// 	explicit BulletCompoundShapeChildActor(btCompoundShape* pRigidBody, Mat* pActorMat) 
// 	{ 
// 		m_pRigidBody = pRigidBody; 
// 
// 		m_desiredDeltaYAngle = 0;
// 		m_desiredDeltaYAngleTime = 0;
// 		m_pActorMat= pActorMat;
// 	}
};

class CLEAR_PHYSICS_API BulletPhysics : public IGamePhysics
{
public:
	std::auto_ptr<btDynamicsWorld>				   m_dynamicsWorld;
	std::auto_ptr<btBroadphaseInterface>           m_broadphase;
	std::auto_ptr<btCollisionDispatcher>           m_dispatcher;
	std::auto_ptr<btConstraintSolver>              m_solver;
	std::auto_ptr<btDefaultCollisionConfiguration> m_collisionConfiguration;

	std::auto_ptr<BulletDebugDrawer> m_debugDrawer;

	EventListenerPtr m_pListener;

	bool m_bRun;

	// keep track of the existing rigid bodies:  To check them for updates
	//   to the actors' positions, and to remove them when their lives are over.
	typedef std::map<ActorId, BulletActor*> ActorIDToBulletActorMap;
	ActorIDToBulletActorMap m_actorBodies;

	typedef std::map<ActorId, btCompoundShape*> ActorIDToBulletbtCompoundShapeActorMap;
	ActorIDToBulletbtCompoundShapeActorMap m_actorCompoundShapes;

	typedef std::map<ActorId, BulletCompoundShapeChildActor*> ActorIDToBulletCSCActor;
	ActorIDToBulletCSCActor m_actorCSC;

	// also keep a map to get the actor id from the btRigidBody*
	typedef std::map<btRigidBody const *, ActorId> RigidBodyToActorIDMap;
	RigidBodyToActorIDMap m_rigidBodyToActorId;

	// data used to store which collision pair (bodies that are touching) need
	//   Collision events sent.  When a new pair of touching bodies are detected,
	//   they are added to m_previousTickCollisionPairs and an event is sent.
	//   When the pair is no longer detected, they are removed and another event
	//   is sent.
	typedef std::pair< btRigidBody const *, btRigidBody const * > CollisionPair;
	typedef std::set< CollisionPair > CollisionPairs;
	CollisionPairs m_previousTickCollisionPairs;

	// helpers for sending events relating to collision pairs
	void SendCollisionPairAddEvent( btPersistentManifold const * manifold, btRigidBody const * body0, btRigidBody const * body1 );
	void SendCollisionPairRemoveEvent( btRigidBody const * body0, btRigidBody const * body1 );


	// helper for cleaning up objects
	void RemoveCollisionObject( btCollisionObject * removeMe );

	// find the BulletActor object with the given actor ID
	BulletActor* FindBulletActor(ActorId id) const;

	// find the actor ID associated with the given body
	optional<ActorId> FindActorID( btRigidBody const * ) const;

	// callback from bullet for each physics time step.  set in VInitialize
	static void BulletInternalTickCallback( btDynamicsWorld * const world, btScalar const timeStep );

	btCompoundShape* FindCompoundShape( ActorId id ) const; 
	BulletCompoundShapeChildActor* FindCompoundShapeChild( ActorId id ) const; 

public:
	BulletPhysics();
	virtual ~BulletPhysics();

	// Initialiazation and Maintenance of the Physics World
	virtual bool VInitialize();
	virtual void VSyncVisibleScene();
	virtual void VOnUpdate( float deltaSeconds ); 
	virtual void TogglePause();
	// common functionality used by VAddSphere, VAddBox, etc
	void AddShape( IActor * actor, btCollisionShape * shape, btScalar mass, enum PhysicsMaterial mat );

	// find the rigid body associated with the given actor ID
	btRigidBody *    FindActorBody( ActorId id ) const;


	// Initialization of Physics Objects
	virtual void VAddPlane(Vec planeNormal, float planeConstante, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddSphere(float radius, IActor *actor, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddBox( Vec dimensions, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat);
	virtual void VAddPointCloud(Vec *verts, int numPoints, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat);
	virtual void VRemoveActor(ActorId id);
	virtual Vec  VRayCast( Vec start, Vec end, bool& hit );
	virtual void VAddTriangleMesh(btCollisionShape* shape, IActor *gameActor, float specificGravity, enum PhysicsMaterial mat);

	virtual void VAddCompoundShape(btCompoundShape* shape, IActor *actor, float specificGravity, enum PhysicsMaterial mat);
	virtual void VSetCompoundShapeChild(ActorId parentID, IActor* childActor, unsigned int index);


	// Debugging
	virtual void VRenderDiagnostics(IScene* pScene);
	virtual void VSetDebugMode( int mode );

	// Physics world modifiers
	virtual void VCreateTrigger(Vec pos, const float dim, int triggerID);
	virtual void VApplyForce(Vec dir, float newtons, ActorId aid);
	virtual void VApplyTorque(Vec dir, float newtons, ActorId aid);
	virtual bool VKinematicMove(Mat mat, ActorId aid);

	virtual void VRotateY(ActorId actorId, float angleRadians, float time);
	virtual float VGetOrientationY(ActorId actorId);
	virtual void VStopActor(ActorId actorId);
	virtual void VSetVelocity(ActorId actorId, Vec vel);
	virtual void VTranslate(ActorId actorId, Vec vec);
	virtual void VStaticActor( ActorId actorId );
};


/////////////////////////////////////////////////////////////////////////////
//
// CreateGamePhysics 
//   The free function that creates an object that implements the IGamePhysics interface.
//
extern IGamePhysics CLEAR_PHYSICS_API *CreateGamePhysics();

extern IGamePhysics CLEAR_PHYSICS_API *CreateNullPhysics();


#endif //PhyMathConversion_H__