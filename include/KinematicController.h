#ifndef KinematicController_H__
#define KinematicController_H__

#include "ClearPhysicsSDK.h"

enum CLEAR_PHYSICS_API BoneCollisionShapeType
{
	eBoneCollisionShape_Box,
	eBoneCollisionShape_Capsul,
	eBoneCollisionShape_Sphere,
};


class CLEAR_PHYSICS_API CBone
{ 
public: 
	CBone(Mat bindposeMatrix, Mat* globalMatrix, string name, string parentName, BoneCollisionShapeType type);
	~CBone();

	string GetName()			  { return m_sName; }
	string GetParentName()		  { return m_sParentName; }
	Mat    GetBindPose()		  { return m_matBindPose; }
	Mat*   GetGlobalMatrix()	  { return m_matGlobalMatrix;}
	btTransform   GetBTBindPose() { return m_btTransformBindPose;}

private:
	string m_sName;
	string m_sParentName;
	Mat m_matBindPose;
	Mat* m_matGlobalMatrix;

	btTransform m_btTransformBindPose;
	BoneCollisionShapeType m_Type;
};


class CLEAR_PHYSICS_API CAnimationsKey
{
public:
	CAnimationsKey( int boneIndex, float timeStamp, Mat bonespaceMatrix, Mat globalMatrix );
	CAnimationsKey() {}
	~CAnimationsKey( );

	float		GetTime()			 { return m_fTimeStamp; }
	DWORD		GetBoneIndex()		 { return m_iBoneIndex; }
	Mat			GetBoneSpaceMatrix() { return m_matBoneSpace; }
	btTransform	GetbtTransform()	 { return m_btTransform; }
private:
	int    m_iBoneIndex;
	float  m_fTimeStamp;
	Mat    m_matBoneSpace;
	btTransform m_btTransform;
};


class CLEAR_PHYSICS_API CAnimationTrack
{
public:
	CAnimationTrack( string name, float startTime, float endTime );
	~CAnimationTrack();

	void   AddAnimationsKey( CAnimationsKey* animationsKey ) { m_animationsKeys.push_back(animationsKey); }
	void   SetAnimationKeysFromBinary( void* keyData, int count );
	void   SetName( string name )							 { m_sName = name; }
	void   SetStartTime( float start )						 { m_fStartTime = start; }
	void   SetEndTime( float end )							 { m_fEndTime = end; }

	string GetName()			  { return m_sName; }
	float  GetStartTime()		  { return m_fStartTime; }
	float  GetEndTime()			  { return m_fEndTime; }
	DWORD  GetKeyCount()		  { return m_animationsKeys.size(); }	
	CAnimationsKey* GetKey(int index ) { return m_animationsKeys[index]; }

private:
	vector<CAnimationsKey*>	m_animationsKeys;

	string m_sName;
	float  m_fStartTime;
	float  m_fEndTime;
};

class CLEAR_PHYSICS_API CKinematicController
{ 
	friend class BulletPhysics;

public: 
	CKinematicController(shared_ptr<IActor> actor);
	~CKinematicController();

	void Update(float const deltaSeconds);

	void AddBone(CBone* pBone, Vec size);
	void AddAnimationTrack(CAnimationTrack* track);

	void SetLocalMatrix(Mat matLocal);

	//Info
	int		GetBoneCount()		 { return m_pBones.size(); }
	int		GetAnimationsCount() { return m_animationTracks.size(); }
	IActor* GetActor()			 { return m_actor.get(); }
	float   GetTime()			 { return m_fTime; }
	float   GetProcessTime()	 { return m_fProcessTime; }
	float   GetAnimationSpeed()	 { return m_fAnimationsSpeed; }

	void   SetTime( float time )			 { m_fTime = time; }
	void   SetAnimationSpeed( float speed )	 { m_fAnimationsSpeed = speed; }

private:
	void SetDynmamicWorld(btDynamicsWorld* world);
	void BuildAnimation();

	//Actor
	shared_ptr<IActor>  m_actor;
	

	//Skeleton Animation member
	std::vector<CBone*>	m_pBones;
	std::vector<Mat>	m_BoneGlobals;
	std::map<string,CAnimationTrack*> m_animationTracks;

	CAnimationTrack* m_pCurrentAnimationsTrack;
	float m_fTime;
	int   m_iLastAnimationsKey;
	float m_fAnimationsSpeed;

	float		   m_fProcessTime;

	//Physic member
	btDynamicsWorld*    m_dynamicsWorld;

	std::vector<btRigidBody*> m_pRigidBodysJoints;

	btRigidBody*		m_pRigidBody;
	btCompoundShape*	m_pCompoundShape;
};

#endif