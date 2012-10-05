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
	CBone(Mat bindposeMatrix, Mat globalMatrix, int index, int parentIndex,BoneCollisionShapeType type);
	~CBone();

private:
	Mat m_matBindPose;
	Mat m_matGlobalMatrix;
	int m_Index;
	int m_ParentIndex;
	BoneCollisionShapeType m_Type;
};


class CLEAR_PHYSICS_API CKinematicController
{ 
	friend class BulletPhysics;

public: 
	CKinematicController(shared_ptr<IActor> actor);
	~CKinematicController();

	void Update(float const deltaSeconds);
	void AddBone(CBone* pBone);

private:
	void SetDynmamicWorld(btDynamicsWorld* world);
	shared_ptr<IActor>  m_actor;
	std::vector<CBone*> m_pBones;

	btDynamicsWorld* m_dynamicsWorld;
};

#endif