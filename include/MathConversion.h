#ifndef PhyMathConversion_H__
#define PhyMathConversion_H__

#include "ClearPhysicsSDK.h"

//Vec to btVector3
extern btVector3 CLEAR_PHYSICS_API Vec_to_btVector3( Vec vec );

//btVector3 to Vec
extern Vec CLEAR_PHYSICS_API btVector3_to_Vec( btVector3 const & btvec );

//Mat to btTransform
extern btTransform CLEAR_PHYSICS_API Mat_to_btTransform( Mat mat );

//btTransform to Mat
extern Mat CLEAR_PHYSICS_API btTransform_to_Mat( btTransform const & trans );

//PhysicsDensity to Gravity 
extern float CLEAR_PHYSICS_API SpecificGravity(enum PhysicsDensity material);

#endif //PhyMathConversion_H__