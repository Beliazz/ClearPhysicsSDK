#include "ClearPhysicsSDK.h"

//Vec to btVector3
btVector3 CLEAR_PHYSICS_API Vec_to_btVector3( Vec vec )
{
	return btVector3( vec.GetX(), vec.GetY(), vec.GetZ() );
}

//btVector3 to Vec
Vec CLEAR_PHYSICS_API btVector3_to_Vec( btVector3 const & btvec )
{
	return Vec( btvec.x(), btvec.y(), btvec.z() );
}




//Mat to btTransform
btTransform CLEAR_PHYSICS_API Mat_to_btTransform( Mat mat )
{
	// convert from Mat (GameCode) to btTransform (Bullet)
	btMatrix3x3 bulletRotation;
	btVector3 bulletPosition;

	XMFLOAT4X4 matData = mat.GetStorage();


	// copy rotation matrix
	for ( int row=0; row<3; ++row )
		for ( int column=0; column<3; ++column )
			bulletRotation[row][column] = matData.m[column][row];	// note the reversed indexing (row/column vs. column/row)
	//  this is because Mats are row-major matrices and
	//  btMatrix3x3 are column-major.  This reversed indexing
	//  implicitly transposes (flips along the diagonal) 
	//  the matrix when it is copied.
	// copy position
	for ( int column=0; column<3; ++column )
		bulletPosition[column] = matData.m[3][column];

	return btTransform( bulletRotation, bulletPosition );
}

//btTransform to Mat
Mat CLEAR_PHYSICS_API btTransform_to_Mat( btTransform const & trans )
{
	Mat mat;
	mat = MatIdentity();

	// convert from btTransform (Bullet) to Mat (GameCode)
	btMatrix3x3 const & bulletRotation = trans.getBasis();
	btVector3 const & bulletPosition = trans.getOrigin();

	float tmp[4][4];
	mat.GetArray((float*)&tmp);

	// copy rotation matrix
	for ( int row=0; row<3; ++row )
		for ( int column=0; column<3; ++column )
			tmp[row][column] = bulletRotation[column][row]; 
	// note the reversed indexing (row/column vs. column/row)
	//  this is because Mats are row-major matrices and
	//  btMatrix3x3 are column-major.  This reversed indexing
	//  implicitly transposes (flips along the diagonal) 
	//  the matrix when it is copied.

	// copy position
	for ( int column=0; column<3; ++column )
		tmp[3][column] = bulletPosition[column];

	return	Mat((float*)tmp);
}

static float g_DensityTable[] = 
{
	// specific gravity (these numbers are easier to find

	.0013f,		// air
	1.000f,		// water

	// Synthetics
	.0100f,		// styrofoam

	// Woods
	.0170f,		// balsa
	.3500f,		// bamboo
	.5000f,		// pine
	.8300f,		// oak
	1.100f,		// ebony

	// Biologic
	1.060f,		// blood
	1.800f,		// bone

	// Metals and Stone
	2.400f,		// silicon
	2.650f,		// aluminum
	2.450f,		// asbestos
	4.500f,		// barite
	3.350f,		// basalt
	9.800f,		// bismuth
	1.750f,		// borax
	2.320f,		// boron
	8.550f,		// brass
	8.640f,		// brick
	8.400f,		// bronze
	4.580f,		// calcium
	1.950f,		// carbon
	7.100f,		// chromium
	2.200f,		// clay - average
	0.800f,		// coal - average
	8.900f,		// cobalt
	8.750f,		// copper - average
	3.510f,		// diamond 
	2.900f,		// dolomite
	1.800f,		// epoxy
	2.600f,		// glass
	2.950f,		// crystal
	2.550f,		// granite
	19.30f,		// gold
	5.200f,		// hematite
	21.60f,		// iridium
	7.200f,		// cast iron
	7.750f,		// wrought iron
	2.400f,		// limestone
	11.34f,		// lead
	3.200f,		// Magnetite
	7.420f,		// Manganese
	1.740f,		// Magnesium
	2.720f,		// Marble
	13.54f,		// Mercury
	10.20f,		// Molybdenum
	8.900f,		// Nickel
	21.45f,		// Platinum
	0.860f,		// Potassium
	2.650f,		// Quartz
	2.300f,		// Sandstone
	2.750f,		// Serpentine
	10.50f,		// Silver
	0.970f,		// Sodium
	7.800f,		// Steel
	2.700f,		// Talc
	1.200f,		// Tar
	6.120f,		// Tellurium
	7.350f,		// Tin
	4.500f,		// Titanium
	19.22f,		// Tungsten
	18.70f,		// Uranium
	5.960f, 	// Vanadium
	1.800f,		// Vinyl
	1.320f,		// Wool
	7.050f,		// Zinc

	0.000f		// Infinite Density - objects will never move
};


//PhysicsDensity to Gravity 

float CLEAR_PHYSICS_API SpecificGravity(enum PhysicsDensity substance)
{
	assert(substance < PhysDens_MaxDensities && _T("Parameter out of range"));
	return g_DensityTable[substance];
}
