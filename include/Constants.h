#ifndef PhyConstatns_H__
#define PhyConstatns_H__

#include "ClearPhysicsSDK.h"

enum CLEAR_PHYSICS_API PhysicsMaterial
{
	PhysMat_Playdough,
	PhysMat_Normal,
	PhysMat_Bouncy,
	PhysMat_Slippery,
	PhysMat_MaxMaterials
};

enum CLEAR_PHYSICS_API PhysicsDensity
{
	// Elemental
	PhysDens_Air,
	PhysDens_Water,

	// Synthetics
	PhysDens_Styrofoam,

	// Woods
	PhysDens_Balsa,
	PhysDens_Bamboo,
	PhysDens_Pine,
	PhysDens_Oak,
	PhysDens_Ebony,

	// Biologic
	PhysDens_Blood,
	PhysDens_Bone,

	// Metals and Stone
	PhysDens_Silicon,
	PhysDens_Aluminum,
	PhysDens_Asbestos,
	PhysDens_Barite,
	PhysDens_Basalt,
	PhysDens_Bismuth,
	PhysDens_Borax,
	PhysDens_Boron,
	PhysDens_Brass,
	PhysDens_Brick,
	PhysDens_Bronze,
	PhysDens_Calcium,
	PhysDens_Carbon,
	PhysDens_Chromium,
	PhysDens_Clay,
	PhysDens_Coal,
	PhysDens_Cobalt,
	PhysDens_Copper,
	PhysDens_Diamond, 
	PhysDens_Dolomite,
	PhysDens_Epoxy,
	PhysDens_Glass,
	PhysDens_Grystal,
	PhysDens_Granite,
	PhysDens_Gold,
	PhysDens_Hematite,
	PhysDens_Iridium,
	PhysDens_Iron_Cast,
	PhysDens_Iron_Wrought,
	PhysDens_Limestone,
	PhysDens_Lead,
	PhysDens_Magnetite,
	PhysDens_Manganese,
	PhysDens_Magnesium,
	PhysDens_Marble,
	PhysDens_Mercury,
	PhysDens_Molybdenum,
	PhysDens_Nickel,
	PhysDens_Platinum,
	PhysDens_Potassium,
	PhysDens_Quartz,
	PhysDens_Sandstone,
	PhysDens_Serpentine,
	PhysDens_Silver,
	PhysDens_Sodium,
	PhysDens_Steel,
	PhysDens_Talc,
	PhysDens_Tar,
	PhysDens_Tellurium,
	PhysDens_Tin,
	PhysDens_Titanium,
	PhysDens_Tungsten,
	PhysDens_Uranium,
	PhysDens_Vanadium,
	PhysDens_Vinyl,
	PhysDens_Wool,
	PhysDens_Zinc,

	PhysDens_Infinite,

	PhysDens_MaxDensities
};



#endif //PhyConstatns_H__