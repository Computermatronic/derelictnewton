/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/
module derelict.newton.types;

enum : int
{
	NEWTON_MAJOR_VERSION = 3,
	NEWTON_MINOR_VERSION = 03,

	NEWTON_PROFILER_WORLD_UPDATE = 0,
	NEWTON_PROFILER_COLLISION_UPDATE = 1,
	NEWTON_PROFILER_COLLISION_UPDATE_BROAD_PHASE = 2,
	NEWTON_PROFILER_COLLISION_UPDATE_NARROW_PHASE = 3,
	NEWTON_PROFILER_DYNAMICS_UPDATE = 4,
	NEWTON_PROFILER_DYNAMICS_CONSTRAINT_GRAPH = 5,
	NEWTON_PROFILER_DYNAMICS_SOLVE_CONSTRAINT_GRAPH = 6,
	NEWTON_PROFILER_FORCE_CALLBACK_UPDATE = 7,
	NEWTON_PRE_LISTERNER_CALLBACK_UPDATE = 8,
	NEWTON_POST_LISTERNER_CALLBACK_UPDATE = 9,
	NEWTON_DYNAMIC_BODY = 0,
	NEWTON_KINEMATIC_BODY = 1,
	NEWTON_DEFORMABLE_BODY = 2,
	SERIALIZE_ID_SPHERE = 0,
	SERIALIZE_ID_CAPSULE = 1,
	SERIALIZE_ID_CHAMFERCYLINDER = 2,
	SERIALIZE_ID_TAPEREDCAPSULE = 3,
	SERIALIZE_ID_CYLINDER = 4,
	SERIALIZE_ID_TAPEREDCYLINDER = 5,
	SERIALIZE_ID_BOX = 6,
	SERIALIZE_ID_CONE = 7,
	SERIALIZE_ID_CONVEXHULL = 8,
	SERIALIZE_ID_NULL = 9,
	SERIALIZE_ID_COMPOUND = 10,
	SERIALIZE_ID_TREE = 11,
	SERIALIZE_ID_HEIGHTFIELD = 12,
	SERIALIZE_ID_DEFORMABLEMESH = 13,
	SERIALIZE_ID_USERMESH = 14,
	SERIALIZE_ID_SCENE = 15,
	SERIALIZE_ID_COMPOUND_BREAKABLE = 16
}

struct NewtonMesh {}
struct NewtonBody {}
struct NewtonWorld {}
struct NewtonJoint {}
struct NewtonMaterial {}
struct NewtonCollision {}
struct NewtonDeformableMeshSegment {}
struct NewtonBreakableComponentMesh {}

struct NewtonBoxParam
{
	 float m_x;
	 float m_y;
	 float m_z;
}

struct NewtonSphereParam
{
	 float m_radio;
}

struct NewtonCylinderParam
{
	 float m_radio;
	 float m_height;
}

struct NewtonCapsuleParam
{
	 float m_radio;
	 float m_height;
}

struct NewtonConeParam
{
	 float m_radio;
	 float m_height;
}

struct NewtonTaperedCapsuleParam
{
	 float m_radio0;
	 float m_radio1;
	 float m_height;
}

struct NewtonTaperedCylinderParam
{
	 float m_radio0;
	 float m_radio1;
	 float m_height;
}

struct NewtonChamferCylinderParam
{
	 float m_radio;
	 float m_height;
}

struct NewtonConvexHullParam
{
	 int m_vertexCount;
	 int m_vertexStrideInBytes;
	 int m_faceCount;
	 float* m_vertex;
}

struct NewtonCompoundCollisionParam
{
	 int m_chidrenCount;
}

struct NewtonCollisionTreeParam
{
	 int m_vertexCount;
	 int m_indexCount;
}

struct NewtonDeformableMeshParam
{
	 int m_vertexCount;
	 int m_triangleCount;
	 int m_vrtexStrideInBytes;
	 ushort* m_indexList;
	 float* m_vertexList;
}

struct NewtonHeightFieldCollisionParam
{
	 int m_width;
	 int m_height;
	 int m_gridsDiagonals;
	 float m_horizonalScale;
	 float m_verticalScale;
	 float* m_elevation;
	 char* m_atributes;
}

struct NewtonSceneCollisionParam
{
	 int m_childrenProxyCount;
}

struct NewtonCollisionInfoRecord
{
	 float[4][4] m_offsetMatrix;
	 int m_collisionType;
	 int m_collisionUserID;
	 union
	 {
		 NewtonBoxParam m_box;
		 NewtonConeParam m_cone;
		 NewtonSphereParam m_sphere;
		 NewtonCapsuleParam m_capsule;
		 NewtonCylinderParam m_cylinder;
		 NewtonTaperedCapsuleParam m_taperedCapsule;
		 NewtonTaperedCylinderParam m_taperedCylinder;
		 NewtonChamferCylinderParam m_chamferCylinder;
		 NewtonConvexHullParam m_convexHull;
		 NewtonDeformableMeshParam m_deformableMesh;
		 NewtonCompoundCollisionParam m_compoundCollision;
		 NewtonCollisionTreeParam m_collisionTree;
		 NewtonHeightFieldCollisionParam m_heightField;
		 NewtonSceneCollisionParam m_sceneCollision;
		 float[64] m_paramArray;
	}
}

struct NewtonJointRecord
{
	 float[4][4] m_attachmenMatrix_0;
	 float[4][4] m_attachmenMatrix_1;
	 float[3] m_minLinearDof;
	 float[3] m_maxLinearDof;
	 float[3] m_minAngularDof;
	 float[3] m_maxAngularDof;
	 const NewtonBody* m_attachBody_0;
	 const NewtonBody* m_attachBody_1;
	 float[16] m_extraParameters;
	 int m_bodiesCollisionOn;
	 char[32] m_descriptionType;
}

struct NewtonUserMeshCollisionCollideDesc
{
	 float[4] m_boxP0;
	 float[4] m_boxP1;
	 float[4] m_boxDistanceTravel;
	 int m_threadNumber;
	 int m_faceCount;
	 int m_vertexStrideInBytes;
	 float m_skinThickness;
	 void* m_userData;
	 NewtonBody* m_objBody;
	 NewtonBody* m_polySoupBody;
	 NewtonCollision* m_objCollision;
	 NewtonCollision* m_polySoupCollision;
	 float* m_vertex;
	 int* m_faceIndexCount;
	 int* m_faceVertexIndex;
}

struct NewtonWorldConvexCastReturnInfo
{
	 float[4] m_point;
	 float[4] m_normal;
	 float[4] m_normalOnHitPoint;
	 float m_penetration;
	 int m_contactID;
	 const NewtonBody* m_hitBody;
}

struct NewtonUserMeshCollisionRayHitDesc
{
	 float[4] m_p0;
	 float[4] m_p1;
	 float[4] m_normalOut;
	 int m_userIdOut;
	 void* m_userData;
}

struct NewtonHingeSliderUpdateDesc
{
	 float m_accel;
	 float m_minFriction;
	 float m_maxFriction;
	 float m_timestep;
}

alias NewtonAllocMemory = extern(C) void* function (int sizeInBytes);
alias NewtonFreeMemory = extern(C) void function (const void* ptr, int sizeInBytes);
alias NewtonWorldDestructorCallback = extern(C) void function (const NewtonWorld* world);
alias NewtonWorldUpdateListenerCallback = extern(C) void function (const NewtonWorld* world, const void* listenerUserData, float timestep);
alias NewtonWorldDestroyListenerCallback = extern(C) void function (const NewtonWorld* world, const void* listenerUserData);
alias NewtonGetTicksCountCallback = extern(C) uint function ();
alias NewtonSerializeCallback = extern(C) void function (const void* serializeHandle, const void* buffer, int size);
alias NewtonDeserializeCallback = extern(C) void function (const void* serializeHandle, const void* buffer, int size);
alias NewtonOnBodySerializationCallback = extern(C) void function (const NewtonBody* body_, NewtonSerializeCallback function_, const void* serializeHandle);
alias NewtonOnBodyDeserializationCallback = extern(C) void function (const NewtonBody* body_, NewtonDeserializeCallback function_, const void* serializeHandle);
alias NewtonOnUserCollisionSerializationCallback = extern(C) void function (const void* userData, NewtonSerializeCallback function_, const void* serializeHandle);
alias NewtonUserMeshCollisionDestroyCallback = extern(C) void function (const void* userData);
alias NewtonUserMeshCollisionCollideCallback = extern(C) void function (const NewtonUserMeshCollisionCollideDesc* collideDescData);
alias NewtonUserMeshCollisionRayHitCallback = extern(C) float function (const NewtonUserMeshCollisionRayHitDesc* lineDescData);
alias NewtonUserMeshCollisionGetCollisionInfo = extern(C) void function (const void* userData, const NewtonCollisionInfoRecord* infoRecord);
alias NewtonUserMeshCollisionAABBTest = extern(C) int function (const void* userData, const float* boxP0, const float* boxP1);
alias NewtonUserMeshCollisionGetFacesInAABB = extern(C) int function (const void* userData, const float* p0, const float* p1, const float** vertexArray, const int* vertexCount, const int* vertexStrideInBytes, const int* indexList, int maxIndexCount, const int* userDataList);
alias NewtonCollisionTreeRayCastCallback = extern(C) float function (const NewtonBody* body_, const NewtonCollision* treeCollision, float interception, const float* normal, int faceId, const void* usedData);
alias NewtonHeightFieldRayCastCallback = extern(C) float function (const NewtonBody* body_, const NewtonCollision* heightFieldCollision, float interception, int row, int col, const float* normal, int faceId, const void* usedData);
alias NewtonTreeCollisionCallback = extern(C) void function (const NewtonBody* bodyWithTreeCollision, const NewtonBody* body_, int faceID, int vertexCount, const float* vertex, int vertexStrideInBytes);
alias NewtonBodyDestructor = extern(C) void function (const NewtonBody* body_);
alias NewtonApplyForceAndTorque = extern(C) void function (const NewtonBody* body_, float timestep, int threadIndex);
alias NewtonSetTransform = extern(C) void function (const NewtonBody* body_, const float* matrix, int threadIndex);
alias NewtonIslandUpdate = extern(C) int function (const NewtonWorld* newtonWorld, const void* islandHandle, int bodyCount);
alias NewtonBodyLeaveWorld = extern(C) void function (const NewtonBody* body_, int threadIndex);
alias NewtonDestroyBodyByExeciveForce = extern(C) void function (const NewtonBody* body_, const NewtonJoint* contact);
alias NewtonCollisionCompoundBreakableCallback = extern(C) int function (const NewtonMesh* mesh, const void* userData, const float* planeMatrixOut);
alias NewtonGetBuoyancyPlane = extern(C) int function (int collisionID, const void* context, const float* globalSpaceMatrix, const float* globalSpacePlane);
alias NewtonWorldRayPrefilterCallback = extern(C) uint function (const NewtonBody* body_, const NewtonCollision* collision, const void* userData);
alias NewtonWorldRayFilterCallback = extern(C) float function (const NewtonBody* body_, const float* hitNormal, int collisionID, const void* userData, float intersectParam);
alias NewtonOnAABBOverlap = extern(C) int function (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1, int threadIndex);
alias NewtonContactsProcess = extern(C) void function (const NewtonJoint* contact, float timestep, int threadIndex);
alias NewtonBodyIterator = extern(C) void function (const NewtonBody* body_, const void* userData);
alias NewtonJointIterator = extern(C) void function (const NewtonJoint* joint, const void* userData);
alias NewtonCollisionIterator = extern(C) void function (const void* userData, int vertexCount, const float* faceArray, int faceId);
alias NewtonBallCallback = extern(C) void function (const NewtonJoint* ball, float timestep);
alias NewtonHingeCallback = extern(C) uint function (const NewtonJoint* hinge, const NewtonHingeSliderUpdateDesc* desc);
alias NewtonSliderCallback = extern(C) uint function (const NewtonJoint* slider, const NewtonHingeSliderUpdateDesc* desc);
alias NewtonUniversalCallback = extern(C) uint function (const NewtonJoint* universal, const NewtonHingeSliderUpdateDesc* desc);
alias NewtonCorkscrewCallback = extern(C) uint function (const NewtonJoint* corkscrew, const NewtonHingeSliderUpdateDesc* desc);
alias NewtonUserBilateralCallback = extern(C) void function (const NewtonJoint* userJoint, float timestep, int threadIndex);
alias NewtonUserBilateralGetInfoCallback = extern(C) void function (const NewtonJoint* userJoint, const NewtonJointRecord* info);
alias NewtonConstraintDestructor = extern(C) void function (const NewtonJoint* me);
alias NewtonJobTask = extern(C) void function (const void* userData, int threadIndex);
alias NewtonReportProgress = extern(C) void function (float progressNormalzedPercent);