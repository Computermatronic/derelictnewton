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
module derelict.newton.newton;
//Dynamic bindings for the Newton Dynamics physics engine for the D Programming Language

public import derelict.newton.types,
              derelict.newton.functions;

private
{
    import derelict.util.loader,
           derelict.util.system;
    
    static if( Derelict_OS_Windows )
        enum libNames = "Newton.dll";
    else static if( Derelict_OS_Mac )
        enum libNames = "/usr/local/lib/libNewton.dylib";
    else static if( Derelict_OS_Posix )
        enum libNames = "libNewton.so";
    else
        static assert( 0, "Need to implement Newton Dynamics libNames for this operating system." );
}

final class DerelictNewtonLoader : SharedLibLoader
{
	public this()
	{
		super( libNames );
	}

    protected override void loadSymbols()
    {
        bindFunc( cast(void** )&NewtonWorldGetVersion, "NewtonWorldGetVersion" );
        bindFunc( cast(void** )&NewtonWorlfloatSize, "NewtonWorlfloatSize" );
        bindFunc( cast(void** )&NewtonGetMemoryUsed, "NewtonGetMemoryUsed" );
        bindFunc( cast(void** )&NewtonSetMemorySystem, "NewtonSetMemorySystem" );
        bindFunc( cast(void** )&NewtonCreate, "NewtonCreate" );
        bindFunc( cast(void** )&NewtonDestroy, "NewtonDestroy" );
        bindFunc( cast(void** )&NewtonDestroyAllBodies, "NewtonDestroyAllBodies" );
        bindFunc( cast(void** )&NewtonEnumrateDevices, "NewtonEnumrateDevices" );
        bindFunc( cast(void** )&NewtonGetCurrentDevice, "NewtonGetCurrentDevice" );
        bindFunc( cast(void** )&NewtonSetCurrentDevice, "NewtonSetCurrentDevice" );
        bindFunc( cast(void** )&NewtonGetDeviceString, "NewtonGetDeviceString" );
        bindFunc( cast(void** )&NewtonInvalidateCache, "NewtonInvalidateCache" );
        bindFunc( cast(void** )&NewtonSetSolverModel, "NewtonSetSolverModel" );
        bindFunc( cast(void** )&NewtonSetMultiThreadSolverOnSingleIsland, "NewtonSetMultiThreadSolverOnSingleIsland" );
        bindFunc( cast(void** )&NewtonGetMultiThreadSolverOnSingleIsland, "NewtonGetMultiThreadSolverOnSingleIsland" );
        bindFunc( cast(void** )&NewtonSetPerformanceClock, "NewtonSetPerformanceClock" );
        bindFunc( cast(void** )&NewtonReadPerformanceTicks, "NewtonReadPerformanceTicks" );
        bindFunc( cast(void** )&NewtonGetBroadphaseAlgorithm, "NewtonGetBroadphaseAlgorithm" );
        bindFunc( cast(void** )&NewtonSelectBroadphaseAlgorithm, "NewtonSelectBroadphaseAlgorithm" );
        bindFunc( cast(void** )&NewtonUpdate, "NewtonUpdate" );
        bindFunc( cast(void** )&NewtonUpdateAsync, "NewtonUpdateAsync" );
        bindFunc( cast(void** )&NewtonWaitForUpdateToFinish, "NewtonWaitForUpdateToFinish" );
        bindFunc( cast(void** )&NewtonSerializeToFile, "NewtonSerializeToFile" );
        bindFunc( cast(void** )&NewtonSerializeBodyArray, "NewtonSerializeBodyArray" );
        bindFunc( cast(void** )&NewtonDeserializeBodyArray, "NewtonDeserializeBodyArray" );
        bindFunc( cast(void** )&NewtonReadThreadPerformanceTicks, "NewtonReadThreadPerformanceTicks" );
        bindFunc( cast(void** )&NewtonWorldCriticalSectionLock, "NewtonWorldCriticalSectionLock" );
        bindFunc( cast(void** )&NewtonWorldCriticalSectionUnlock, "NewtonWorldCriticalSectionUnlock" );
        bindFunc( cast(void** )&NewtonSetThreadsCount, "NewtonSetThreadsCount" );
        bindFunc( cast(void** )&NewtonGetThreadsCount, "NewtonGetThreadsCount" );
        bindFunc( cast(void** )&NewtonGetMaxThreadsCount, "NewtonGetMaxThreadsCount" );
        bindFunc( cast(void** )&NewtonDispachThreadJob, "NewtonDispachThreadJob" );
        bindFunc( cast(void** )&NewtonSyncThreadJobs, "NewtonSyncThreadJobs" );
        bindFunc( cast(void** )&NewtonAtomicAdd, "NewtonAtomicAdd" );
        bindFunc( cast(void** )&NewtonAtomicSwap, "NewtonAtomicSwap" );
        bindFunc( cast(void** )&NewtonYield, "NewtonYield" );
        bindFunc( cast(void** )&NewtonSetFrictionModel, "NewtonSetFrictionModel" );
        bindFunc( cast(void** )&NewtonSetMinimumFrameRate, "NewtonSetMinimumFrameRate" );
        bindFunc( cast(void** )&NewtonSetBodyLeaveWorldEvent, "NewtonSetBodyLeaveWorldEvent" );
        bindFunc( cast(void** )&NewtonSetIslandUpdateEvent, "NewtonSetIslandUpdateEvent" );
        bindFunc( cast(void** )&NewtonSetDestroyBodyByExeciveForce, "NewtonSetDestroyBodyByExeciveForce" );
        bindFunc( cast(void** )&NewtonWorldForEachJointDo, "NewtonWorldForEachJointDo" );
        bindFunc( cast(void** )&NewtonWorldForEachBodyInAABBDo, "NewtonWorldForEachBodyInAABBDo" );
        bindFunc( cast(void** )&NewtonWorldSetUserData, "NewtonWorldSetUserData" );
        bindFunc( cast(void** )&NewtonWorldGetUserData, "NewtonWorldGetUserData" );
        bindFunc( cast(void** )&NewtonWorldGetListenerUserData, "NewtonWorldGetListenerUserData" );
        bindFunc( cast(void** )&NewtonWorldGetPreListener, "NewtonWorldGetPreListener" );
        bindFunc( cast(void** )&NewtonWorldAddPreListener, "NewtonWorldAddPreListener" );
        bindFunc( cast(void** )&NewtonWorldGetPostListener, "NewtonWorldGetPostListener" );
        bindFunc( cast(void** )&NewtonWorldAddPostListener, "NewtonWorldAddPostListener" );
        bindFunc( cast(void** )&NewtonWorldSetDestructorCallback, "NewtonWorldSetDestructorCallback" );
        bindFunc( cast(void** )&NewtonWorldGetDestructorCallback, "NewtonWorldGetDestructorCallback" );
        bindFunc( cast(void** )&NewtonWorldRayCast, "NewtonWorldRayCast" );
        bindFunc( cast(void** )&NewtonWorldCollide, "NewtonWorldCollide" );
        bindFunc( cast(void** )&NewtonWorldConvexCast, "NewtonWorldConvexCast" );
        bindFunc( cast(void** )&NewtonWorldGetBodyCount, "NewtonWorldGetBodyCount" );
        bindFunc( cast(void** )&NewtonWorldGetConstraintCount, "NewtonWorldGetConstraintCount" );
        bindFunc( cast(void** )&NewtonIslandGetBody, "NewtonIslandGetBody" );
        bindFunc( cast(void** )&NewtonIslandGetBodyAABB, "NewtonIslandGetBodyAABB" );
        bindFunc( cast(void** )&NewtonMaterialCreateGroupID, "NewtonMaterialCreateGroupID" );
        bindFunc( cast(void** )&NewtonMaterialGetDefaultGroupID, "NewtonMaterialGetDefaultGroupID" );
        bindFunc( cast(void** )&NewtonMaterialDestroyAllGroupID, "NewtonMaterialDestroyAllGroupID" );
        bindFunc( cast(void** )&NewtonMaterialGetUserData, "NewtonMaterialGetUserData" );
        bindFunc( cast(void** )&NewtonMaterialSetSurfaceThickness, "NewtonMaterialSetSurfaceThickness" );
        bindFunc( cast(void** )&NewtonMaterialSetCollisionCallback, "NewtonMaterialSetCollisionCallback" );
        bindFunc( cast(void** )&NewtonMaterialSetDefaultSoftness, "NewtonMaterialSetDefaultSoftness" );
        bindFunc( cast(void** )&NewtonMaterialSetDefaultElasticity, "NewtonMaterialSetDefaultElasticity" );
        bindFunc( cast(void** )&NewtonMaterialSetDefaultCollidable, "NewtonMaterialSetDefaultCollidable" );
        bindFunc( cast(void** )&NewtonMaterialSetDefaultFriction, "NewtonMaterialSetDefaultFriction" );
        bindFunc( cast(void** )&NewtonWorldGetFirstMaterial, "NewtonWorldGetFirstMaterial" );
        bindFunc( cast(void** )&NewtonWorldGetNextMaterial, "NewtonWorldGetNextMaterial" );
        bindFunc( cast(void** )&NewtonWorldGetFirstBody, "NewtonWorldGetFirstBody" );
        bindFunc( cast(void** )&NewtonWorldGetNextBody, "NewtonWorldGetNextBody" );
        bindFunc( cast(void** )&NewtonMaterialGetMaterialPairUserData, "NewtonMaterialGetMaterialPairUserData" );
        bindFunc( cast(void** )&NewtonMaterialGetContactFaceAttribute, "NewtonMaterialGetContactFaceAttribute" );
        bindFunc( cast(void** )&NewtonMaterialGetBodyCollidingShape, "NewtonMaterialGetBodyCollidingShape" );
        bindFunc( cast(void** )&NewtonMaterialGetContactNormalSpeed, "NewtonMaterialGetContactNormalSpeed" );
        bindFunc( cast(void** )&NewtonMaterialGetContactForce, "NewtonMaterialGetContactForce" );
        bindFunc( cast(void** )&NewtonMaterialGetContactPositionAndNormal, "NewtonMaterialGetContactPositionAndNormal" );
        bindFunc( cast(void** )&NewtonMaterialGetContactTangentDirections, "NewtonMaterialGetContactTangentDirections" );
        bindFunc( cast(void** )&NewtonMaterialGetContactTangentSpeed, "NewtonMaterialGetContactTangentSpeed" );
        bindFunc( cast(void** )&NewtonMaterialSetContactSoftness, "NewtonMaterialSetContactSoftness" );
        bindFunc( cast(void** )&NewtonMaterialSetContactElasticity, "NewtonMaterialSetContactElasticity" );
        bindFunc( cast(void** )&NewtonMaterialSetContactFrictionState, "NewtonMaterialSetContactFrictionState" );
        bindFunc( cast(void** )&NewtonMaterialSetContactFrictionCoef, "NewtonMaterialSetContactFrictionCoef" );
        bindFunc( cast(void** )&NewtonMaterialSetContactNormalAcceleration, "NewtonMaterialSetContactNormalAcceleration" );
        bindFunc( cast(void** )&NewtonMaterialSetContactNormalDirection, "NewtonMaterialSetContactNormalDirection" );
        bindFunc( cast(void** )&NewtonMaterialSetContactTangentAcceleration, "NewtonMaterialSetContactTangentAcceleration" );
        bindFunc( cast(void** )&NewtonMaterialContactRotateTangentDirections, "NewtonMaterialContactRotateTangentDirections" );
        bindFunc( cast(void** )&NewtonCreateNull, "NewtonCreateNull" );
        bindFunc( cast(void** )&NewtonCreateSphere, "NewtonCreateSphere" );
        bindFunc( cast(void** )&NewtonCreateBox, "NewtonCreateBox" );
        bindFunc( cast(void** )&NewtonCreateCone, "NewtonCreateCone" );
        bindFunc( cast(void** )&NewtonCreateCapsule, "NewtonCreateCapsule" );
        bindFunc( cast(void** )&NewtonCreateCylinder, "NewtonCreateCylinder" );
        bindFunc( cast(void** )&NewtonCreateTaperedCapsule, "NewtonCreateTaperedCapsule" );
        bindFunc( cast(void** )&NewtonCreateTaperedCylinder, "NewtonCreateTaperedCylinder" );
        bindFunc( cast(void** )&NewtonCreateChamferCylinder, "NewtonCreateChamferCylinder" );
        bindFunc( cast(void** )&NewtonCreateConvexHull, "NewtonCreateConvexHull" );
        bindFunc( cast(void** )&NewtonCreateConvexHullFromMesh, "NewtonCreateConvexHullFromMesh" );
        bindFunc( cast(void** )&NewtonCollisionGetMode, "NewtonCollisionGetMode" );
        bindFunc( cast(void** )&NewtonCollisionSetCollisonMode, "NewtonCollisionSetCollisonMode" );
        bindFunc( cast(void** )&NewtonConvexHullGetFaceIndices, "NewtonConvexHullGetFaceIndices" );
        bindFunc( cast(void** )&NewtonConvexCollisionCalculateVolume, "NewtonConvexCollisionCalculateVolume" );
        bindFunc( cast(void** )&NewtonConvexCollisionCalculateInertialMatrix, "NewtonConvexCollisionCalculateInertialMatrix" );
        bindFunc( cast(void** )&NewtonCreateCompoundCollision, "NewtonCreateCompoundCollision" );
        bindFunc( cast(void** )&NewtonCreateCompoundCollisionFromMesh, "NewtonCreateCompoundCollisionFromMesh" );
        bindFunc( cast(void** )&NewtonCompoundCollisionBeginAddRemove, "NewtonCompoundCollisionBeginAddRemove" );
        bindFunc( cast(void** )&NewtonCompoundCollisionAddSubCollision, "NewtonCompoundCollisionAddSubCollision" );
        bindFunc( cast(void** )&NewtonCompoundCollisionRemoveSubCollision, "NewtonCompoundCollisionRemoveSubCollision" );
        bindFunc( cast(void** )&NewtonCompoundCollisionRemoveSubCollisionByIndex, "NewtonCompoundCollisionRemoveSubCollisionByIndex" );
        bindFunc( cast(void** )&NewtonCompoundCollisionSetSubCollisionMatrix, "NewtonCompoundCollisionSetSubCollisionMatrix" );
        bindFunc( cast(void** )&NewtonCompoundCollisionEndAddRemove, "NewtonCompoundCollisionEndAddRemove" );
        bindFunc( cast(void** )&NewtonCompoundCollisionGetFirstNode, "NewtonCompoundCollisionGetFirstNode" );
        bindFunc( cast(void** )&NewtonCompoundCollisionGetNextNode, "NewtonCompoundCollisionGetNextNode" );
        bindFunc( cast(void** )&NewtonCompoundCollisionGetNodeByIndex, "NewtonCompoundCollisionGetNodeByIndex" );
        bindFunc( cast(void** )&NewtonCompoundCollisionGetNodeIndex, "NewtonCompoundCollisionGetNodeIndex" );
        bindFunc( cast(void** )&NewtonCompoundCollisionGetCollisionFromNode, "NewtonCompoundCollisionGetCollisionFromNode" );
        bindFunc( cast(void** )&NewtonCreateSceneCollision, "NewtonCreateSceneCollision" );
        bindFunc( cast(void** )&NewtonSceneCollisionBeginAddRemove, "NewtonSceneCollisionBeginAddRemove" );
        bindFunc( cast(void** )&NewtonSceneCollisionAddSubCollision, "NewtonSceneCollisionAddSubCollision" );
        bindFunc( cast(void** )&NewtonSceneCollisionSetSubCollisionMatrix, "NewtonSceneCollisionSetSubCollisionMatrix" );
        bindFunc( cast(void** )&NewtonSceneCollisionEndAddRemove, "NewtonSceneCollisionEndAddRemove" );
        bindFunc( cast(void** )&NewtonSceneCollisionGetFirstNode, "NewtonSceneCollisionGetFirstNode" );
        bindFunc( cast(void** )&NewtonSceneCollisionGetNextNode, "NewtonSceneCollisionGetNextNode" );
        bindFunc( cast(void** )&NewtonSceneCollisionGetCollisionFromNode, "NewtonSceneCollisionGetCollisionFromNode" );
        bindFunc( cast(void** )&NewtonCreateUserMeshCollision, "NewtonCreateUserMeshCollision" );
        bindFunc( cast(void** )&NewtonCreateCollisionFromSerialization, "NewtonCreateCollisionFromSerialization" );
        bindFunc( cast(void** )&NewtonCollisionSerialize, "NewtonCollisionSerialize" );
        bindFunc( cast(void** )&NewtonCollisionGetInfo, "NewtonCollisionGetInfo" );
        bindFunc( cast(void** )&NewtonCreateHeightFieldCollision, "NewtonCreateHeightFieldCollision" );
        bindFunc( cast(void** )&NewtonHeightFieldSetUserRayCastCallback, "NewtonHeightFieldSetUserRayCastCallback" );
        bindFunc( cast(void** )&NewtonCreateTreeCollision, "NewtonCreateTreeCollision" );
        bindFunc( cast(void** )&NewtonCreateTreeCollisionFromMesh, "NewtonCreateTreeCollisionFromMesh" );
        bindFunc( cast(void** )&NewtonTreeCollisionSetUserRayCastCallback, "NewtonTreeCollisionSetUserRayCastCallback" );
        bindFunc( cast(void** )&NewtonTreeCollisionBeginBuild, "NewtonTreeCollisionBeginBuild" );
        bindFunc( cast(void** )&NewtonTreeCollisionAddFace, "NewtonTreeCollisionAddFace" );
        bindFunc( cast(void** )&NewtonTreeCollisionEndBuild, "NewtonTreeCollisionEndBuild" );
        bindFunc( cast(void** )&NewtonTreeCollisionGetFaceAtribute, "NewtonTreeCollisionGetFaceAtribute" );
        bindFunc( cast(void** )&NewtonTreeCollisionSetFaceAtribute, "NewtonTreeCollisionSetFaceAtribute" );
        bindFunc( cast(void** )&NewtonTreeCollisionGetVertexListIndexListInAABB, "NewtonTreeCollisionGetVertexListIndexListInAABB" );
        bindFunc( cast(void** )&NewtonStaticCollisionSetDebugCallback, "NewtonStaticCollisionSetDebugCallback" );
        bindFunc( cast(void** )&NewtonCollisionCreateInstance, "NewtonCollisionCreateInstance" );
        bindFunc( cast(void** )&NewtonCollisionGetType, "NewtonCollisionGetType" );
        bindFunc( cast(void** )&NewtonCollisionSetUserData, "NewtonCollisionSetUserData" );
        bindFunc( cast(void** )&NewtonCollisionGetUserData, "NewtonCollisionGetUserData" );
        bindFunc( cast(void** )&NewtonCollisionSetUserID, "NewtonCollisionSetUserID" );
        bindFunc( cast(void** )&NewtonCollisionGetUserID, "NewtonCollisionGetUserID" );
        bindFunc( cast(void** )&NewtonCollisionSetMatrix, "NewtonCollisionSetMatrix" );
        bindFunc( cast(void** )&NewtonCollisionGetMatrix, "NewtonCollisionGetMatrix" );
        bindFunc( cast(void** )&NewtonCollisionSetScale, "NewtonCollisionSetScale" );
        bindFunc( cast(void** )&NewtonCollisionGetScale, "NewtonCollisionGetScale" );
        bindFunc( cast(void** )&NewtonDestroyCollision, "NewtonDestroyCollision" );
        bindFunc( cast(void** )&NewtonCollisionPointDistance, "NewtonCollisionPointDistance" );
        bindFunc( cast(void** )&NewtonCollisionClosestPoint, "NewtonCollisionClosestPoint" );
        bindFunc( cast(void** )&NewtonCollisionCollide, "NewtonCollisionCollide" );
        bindFunc( cast(void** )&NewtonCollisionCollideContinue, "NewtonCollisionCollideContinue" );
        bindFunc( cast(void** )&NewtonCollisionSupportVertex, "NewtonCollisionSupportVertex" );
        bindFunc( cast(void** )&NewtonCollisionRayCast, "NewtonCollisionRayCast" );
        bindFunc( cast(void** )&NewtonCollisionCalculateAABB, "NewtonCollisionCalculateAABB" );
        bindFunc( cast(void** )&NewtonCollisionForEachPolygonDo, "NewtonCollisionForEachPolygonDo" );
        bindFunc( cast(void** )&NewtonGetEulerAngle, "NewtonGetEulerAngle" );
        bindFunc( cast(void** )&NewtonSetEulerAngle, "NewtonSetEulerAngle" );
        bindFunc( cast(void** )&NewtonCalculateSpringDamperAcceleration, "NewtonCalculateSpringDamperAcceleration" );
        bindFunc( cast(void** )&NewtonCreateDynamicBody, "NewtonCreateDynamicBody" );
        bindFunc( cast(void** )&NewtonCreateKinematicBody, "NewtonCreateKinematicBody" );
        bindFunc( cast(void** )&NewtonDestroyBody, "NewtonDestroyBody" );
        bindFunc( cast(void** )&NewtonBodyGetType, "NewtonBodyGetType" );
        bindFunc( cast(void** )&NewtonBodyAddForce, "NewtonBodyAddForce" );
        bindFunc( cast(void** )&NewtonBodyAddTorque, "NewtonBodyAddTorque" );
        bindFunc( cast(void** )&NewtonBodyCalculateInverseDynamicsForce, "NewtonBodyCalculateInverseDynamicsForce" );
        bindFunc( cast(void** )&NewtonBodySetCentreOfMass, "NewtonBodySetCentreOfMass" );
        bindFunc( cast(void** )&NewtonBodySetMassMatrix___, "NewtonBodySetMassMatrix___" );
        bindFunc( cast(void** )&NewtonBodySetMassProperties, "NewtonBodySetMassProperties" );
        bindFunc( cast(void** )&NewtonBodySetMatrix, "NewtonBodySetMatrix" );
        bindFunc( cast(void** )&NewtonBodySetMatrixRecursive, "NewtonBodySetMatrixRecursive" );
        bindFunc( cast(void** )&NewtonBodySetMaterialGroupID, "NewtonBodySetMaterialGroupID" );
        bindFunc( cast(void** )&NewtonBodySetContinuousCollisionMode, "NewtonBodySetContinuousCollisionMode" );
        bindFunc( cast(void** )&NewtonBodySetJointRecursiveCollision, "NewtonBodySetJointRecursiveCollision" );
        bindFunc( cast(void** )&NewtonBodySetOmega, "NewtonBodySetOmega" );
        bindFunc( cast(void** )&NewtonBodySetVelocity, "NewtonBodySetVelocity" );
        bindFunc( cast(void** )&NewtonBodySetForce, "NewtonBodySetForce" );
        bindFunc( cast(void** )&NewtonBodySetTorque, "NewtonBodySetTorque" );
        bindFunc( cast(void** )&NewtonBodySetLinearDamping, "NewtonBodySetLinearDamping" );
        bindFunc( cast(void** )&NewtonBodySetAngularDamping, "NewtonBodySetAngularDamping" );
        bindFunc( cast(void** )&NewtonBodySetCollision, "NewtonBodySetCollision" );
        bindFunc( cast(void** )&NewtonBodySetCollisionScale, "NewtonBodySetCollisionScale" );
        bindFunc( cast(void** )&NewtonBodyGetSleepState, "NewtonBodyGetSleepState" );
        bindFunc( cast(void** )&NewtonBodySetSleepState, "NewtonBodySetSleepState" );
        bindFunc( cast(void** )&NewtonBodyGetAutoSleep, "NewtonBodyGetAutoSleep" );
        bindFunc( cast(void** )&NewtonBodySetAutoSleep, "NewtonBodySetAutoSleep" );
        bindFunc( cast(void** )&NewtonBodyGetFreezeState, "NewtonBodyGetFreezeState" );
        bindFunc( cast(void** )&NewtonBodySetFreezeState, "NewtonBodySetFreezeState" );
        bindFunc( cast(void** )&NewtonBodySetDestructorCallback, "NewtonBodySetDestructorCallback" );
        bindFunc( cast(void** )&NewtonBodyGetDestructorCallback, "NewtonBodyGetDestructorCallback" );
        bindFunc( cast(void** )&NewtonBodySetTransformCallback, "NewtonBodySetTransformCallback" );
        bindFunc( cast(void** )&NewtonBodyGetTransformCallback, "NewtonBodyGetTransformCallback" );
        bindFunc( cast(void** )&NewtonBodySetForceAndTorqueCallback, "NewtonBodySetForceAndTorqueCallback" );
        bindFunc( cast(void** )&NewtonBodyGetForceAndTorqueCallback, "NewtonBodyGetForceAndTorqueCallback" );
        bindFunc( cast(void** )&NewtonBodyGetID, "NewtonBodyGetID" );
        bindFunc( cast(void** )&NewtonBodySetUserData, "NewtonBodySetUserData" );
        bindFunc( cast(void** )&NewtonBodyGetUserData, "NewtonBodyGetUserData" );
        bindFunc( cast(void** )&NewtonBodyGetWorld, "NewtonBodyGetWorld" );
        bindFunc( cast(void** )&NewtonBodyGetCollision, "NewtonBodyGetCollision" );
        bindFunc( cast(void** )&NewtonBodyGetMaterialGroupID, "NewtonBodyGetMaterialGroupID" );
        bindFunc( cast(void** )&NewtonBodyGetContinuousCollisionMode, "NewtonBodyGetContinuousCollisionMode" );
        bindFunc( cast(void** )&NewtonBodyGetJointRecursiveCollision, "NewtonBodyGetJointRecursiveCollision" );
        bindFunc( cast(void** )&NewtonBodyGetMatrix, "NewtonBodyGetMatrix" );
        bindFunc( cast(void** )&NewtonBodyGetRotation, "NewtonBodyGetRotation" );
        bindFunc( cast(void** )&NewtonBodyGetMassMatrix, "NewtonBodyGetMassMatrix" );
        bindFunc( cast(void** )&NewtonBodyGetInvMass, "NewtonBodyGetInvMass" );
        bindFunc( cast(void** )&NewtonBodyGetInertiaMatrix, "NewtonBodyGetInertiaMatrix" );
        bindFunc( cast(void** )&NewtonBodyGetInvInertiaMatrix, "NewtonBodyGetInvInertiaMatrix" );
        bindFunc( cast(void** )&NewtonBodyGetOmega, "NewtonBodyGetOmega" );
        bindFunc( cast(void** )&NewtonBodyGetVelocity, "NewtonBodyGetVelocity" );
        bindFunc( cast(void** )&NewtonBodyGetForce, "NewtonBodyGetForce" );
        bindFunc( cast(void** )&NewtonBodyGetTorque, "NewtonBodyGetTorque" );
        bindFunc( cast(void** )&NewtonBodyGetForceAcc, "NewtonBodyGetForceAcc" );
        bindFunc( cast(void** )&NewtonBodyGetTorqueAcc, "NewtonBodyGetTorqueAcc" );
        bindFunc( cast(void** )&NewtonBodyGetCentreOfMass, "NewtonBodyGetCentreOfMass" );
        bindFunc( cast(void** )&NewtonBodyGetPointVelocity, "NewtonBodyGetPointVelocity" );
        bindFunc( cast(void** )&NewtonBodyAddImpulse, "NewtonBodyAddImpulse" );
        bindFunc( cast(void** )&NewtonBodyApplyImpulseArray, "NewtonBodyApplyImpulseArray" );
        bindFunc( cast(void** )&NewtonBodyApplyImpulsePair, "NewtonBodyApplyImpulsePair" );
        bindFunc( cast(void** )&NewtonBodyIntegrateVelocity, "NewtonBodyIntegrateVelocity" );
        bindFunc( cast(void** )&NewtonBodyGetLinearDamping, "NewtonBodyGetLinearDamping" );
        bindFunc( cast(void** )&NewtonBodyGetAngularDamping, "NewtonBodyGetAngularDamping" );
        bindFunc( cast(void** )&NewtonBodyGetAABB, "NewtonBodyGetAABB" );
        bindFunc( cast(void** )&NewtonBodyGetFirstJoint, "NewtonBodyGetFirstJoint" );
        bindFunc( cast(void** )&NewtonBodyGetNextJoint, "NewtonBodyGetNextJoint" );
        bindFunc( cast(void** )&NewtonBodyGetFirstContactJoint, "NewtonBodyGetFirstContactJoint" );
        bindFunc( cast(void** )&NewtonBodyGetNextContactJoint, "NewtonBodyGetNextContactJoint" );
        bindFunc( cast(void** )&NewtonBodyAddBuoyancyForce, "NewtonBodyAddBuoyancyForce" );
        bindFunc( cast(void** )&NewtonContactJointGetFirstContact, "NewtonContactJointGetFirstContact" );
        bindFunc( cast(void** )&NewtonContactJointGetNextContact, "NewtonContactJointGetNextContact" );
        bindFunc( cast(void** )&NewtonContactJointGetContactCount, "NewtonContactJointGetContactCount" );
        bindFunc( cast(void** )&NewtonContactJointRemoveContact, "NewtonContactJointRemoveContact" );
        bindFunc( cast(void** )&NewtonContactGetMaterial, "NewtonContactGetMaterial" );
        bindFunc( cast(void** )&NewtonJointGetUserData, "NewtonJointGetUserData" );
        bindFunc( cast(void** )&NewtonJointSetUserData, "NewtonJointSetUserData" );
        bindFunc( cast(void** )&NewtonJointGetBody0, "NewtonJointGetBody0" );
        bindFunc( cast(void** )&NewtonJointGetBody1, "NewtonJointGetBody1" );
        bindFunc( cast(void** )&NewtonJointGetInfo, "NewtonJointGetInfo" );
        bindFunc( cast(void** )&NewtonJointGetCollisionState, "NewtonJointGetCollisionState" );
        bindFunc( cast(void** )&NewtonJointSetCollisionState, "NewtonJointSetCollisionState" );
        bindFunc( cast(void** )&NewtonJointGetStiffness, "NewtonJointGetStiffness" );
        bindFunc( cast(void** )&NewtonJointSetStiffness, "NewtonJointSetStiffness" );
        bindFunc( cast(void** )&NewtonDestroyJoint, "NewtonDestroyJoint" );
        bindFunc( cast(void** )&NewtonJointSetDestructor, "NewtonJointSetDestructor" );
        bindFunc( cast(void** )&NewtonCreateDeformableMesh, "NewtonCreateDeformableMesh" );
        bindFunc( cast(void** )&NewtonDeformableMeshSetPlasticity, "NewtonDeformableMeshSetPlasticity" );
        bindFunc( cast(void** )&NewtonDeformableMeshSetStiffness, "NewtonDeformableMeshSetStiffness" );
        bindFunc( cast(void** )&NewtonDeformableMeshSetSkinThickness, "NewtonDeformableMeshSetSkinThickness" );
        bindFunc( cast(void** )&NewtonCreateDeformableBody, "NewtonCreateDeformableBody" );
        bindFunc( cast(void** )&NewtonDeformableMeshUpdateRenderNormals, "NewtonDeformableMeshUpdateRenderNormals" );
        bindFunc( cast(void** )&NewtonDeformableMeshGetVertexCount, "NewtonDeformableMeshGetVertexCount" );
        bindFunc( cast(void** )&NewtonDeformableMeshGetVertexStreams, "NewtonDeformableMeshGetVertexStreams" );
        bindFunc( cast(void** )&NewtonDeformableMeshGetFirstSegment, "NewtonDeformableMeshGetFirstSegment" );
        bindFunc( cast(void** )&NewtonDeformableMeshGetNextSegment, "NewtonDeformableMeshGetNextSegment" );
        bindFunc( cast(void** )&NewtonDeformableMeshSegmentGetMaterialID, "NewtonDeformableMeshSegmentGetMaterialID" );
        bindFunc( cast(void** )&NewtonDeformableMeshSegmentGetIndexCount, "NewtonDeformableMeshSegmentGetIndexCount" );
        bindFunc( cast(void** )&NewtonDeformableMeshSegmentGetIndexList, "NewtonDeformableMeshSegmentGetIndexList" );
        bindFunc( cast(void** )&NewtonConstraintCreateBall, "NewtonConstraintCreateBall" );
        bindFunc( cast(void** )&NewtonBallSetUserCallback, "NewtonBallSetUserCallback" );
        bindFunc( cast(void** )&NewtonBallGetJointAngle, "NewtonBallGetJointAngle" );
        bindFunc( cast(void** )&NewtonBallGetJointOmega, "NewtonBallGetJointOmega" );
        bindFunc( cast(void** )&NewtonBallGetJointForce, "NewtonBallGetJointForce" );
        bindFunc( cast(void** )&NewtonBallSetConeLimits, "NewtonBallSetConeLimits" );
        bindFunc( cast(void** )&NewtonConstraintCreateHinge, "NewtonConstraintCreateHinge" );
        bindFunc( cast(void** )&NewtonHingeSetUserCallback, "NewtonHingeSetUserCallback" );
        bindFunc( cast(void** )&NewtonHingeGetJointAngle, "NewtonHingeGetJointAngle" );
        bindFunc( cast(void** )&NewtonHingeGetJointOmega, "NewtonHingeGetJointOmega" );
        bindFunc( cast(void** )&NewtonHingeGetJointForce, "NewtonHingeGetJointForce" );
        bindFunc( cast(void** )&NewtonHingeCalculateStopAlpha, "NewtonHingeCalculateStopAlpha" );
        bindFunc( cast(void** )&NewtonConstraintCreateSlider, "NewtonConstraintCreateSlider" );
        bindFunc( cast(void** )&NewtonSliderSetUserCallback, "NewtonSliderSetUserCallback" );
        bindFunc( cast(void** )&NewtonSliderGetJointPosit, "NewtonSliderGetJointPosit" );
        bindFunc( cast(void** )&NewtonSliderGetJointVeloc, "NewtonSliderGetJointVeloc" );
        bindFunc( cast(void** )&NewtonSliderGetJointForce, "NewtonSliderGetJointForce" );
        bindFunc( cast(void** )&NewtonSliderCalculateStopAccel, "NewtonSliderCalculateStopAccel" );
        bindFunc( cast(void** )&NewtonConstraintCreateCorkscrew, "NewtonConstraintCreateCorkscrew" );
        bindFunc( cast(void** )&NewtonCorkscrewSetUserCallback, "NewtonCorkscrewSetUserCallback" );
        bindFunc( cast(void** )&NewtonCorkscrewGetJointPosit, "NewtonCorkscrewGetJointPosit" );
        bindFunc( cast(void** )&NewtonCorkscrewGetJointAngle, "NewtonCorkscrewGetJointAngle" );
        bindFunc( cast(void** )&NewtonCorkscrewGetJointVeloc, "NewtonCorkscrewGetJointVeloc" );
        bindFunc( cast(void** )&NewtonCorkscrewGetJointOmega, "NewtonCorkscrewGetJointOmega" );
        bindFunc( cast(void** )&NewtonCorkscrewGetJointForce, "NewtonCorkscrewGetJointForce" );
        bindFunc( cast(void** )&NewtonCorkscrewCalculateStopAlpha, "NewtonCorkscrewCalculateStopAlpha" );
        bindFunc( cast(void** )&NewtonCorkscrewCalculateStopAccel, "NewtonCorkscrewCalculateStopAccel" );
        bindFunc( cast(void** )&NewtonConstraintCreateUniversal, "NewtonConstraintCreateUniversal" );
        bindFunc( cast(void** )&NewtonUniversalSetUserCallback, "NewtonUniversalSetUserCallback" );
        bindFunc( cast(void** )&NewtonUniversalGetJointAngle0, "NewtonUniversalGetJointAngle0" );
        bindFunc( cast(void** )&NewtonUniversalGetJointAngle1, "NewtonUniversalGetJointAngle1" );
        bindFunc( cast(void** )&NewtonUniversalGetJointOmega0, "NewtonUniversalGetJointOmega0" );
        bindFunc( cast(void** )&NewtonUniversalGetJointOmega1, "NewtonUniversalGetJointOmega1" );
        bindFunc( cast(void** )&NewtonUniversalGetJointForce, "NewtonUniversalGetJointForce" );
        bindFunc( cast(void** )&NewtonUniversalCalculateStopAlpha0, "NewtonUniversalCalculateStopAlpha0" );
        bindFunc( cast(void** )&NewtonUniversalCalculateStopAlpha1, "NewtonUniversalCalculateStopAlpha1" );
        bindFunc( cast(void** )&NewtonConstraintCreateUpVector, "NewtonConstraintCreateUpVector" );
        bindFunc( cast(void** )&NewtonUpVectorGetPin, "NewtonUpVectorGetPin" );
        bindFunc( cast(void** )&NewtonUpVectorSetPin, "NewtonUpVectorSetPin" );
        bindFunc( cast(void** )&NewtonConstraintCreateUserJoint, "NewtonConstraintCreateUserJoint" );
        bindFunc( cast(void** )&NewtonUserJointSetFeedbackCollectorCallback, "NewtonUserJointSetFeedbackCollectorCallback" );
        bindFunc( cast(void** )&NewtonUserJointAddLinearRow, "NewtonUserJointAddLinearRow" );
        bindFunc( cast(void** )&NewtonUserJointAddAngularRow, "NewtonUserJointAddAngularRow" );
        bindFunc( cast(void** )&NewtonUserJointAddGeneralRow, "NewtonUserJointAddGeneralRow" );
        bindFunc( cast(void** )&NewtonUserJointSetRowMinimumFriction, "NewtonUserJointSetRowMinimumFriction" );
        bindFunc( cast(void** )&NewtonUserJointSetRowMaximumFriction, "NewtonUserJointSetRowMaximumFriction" );
        bindFunc( cast(void** )&NewtonUserJointSetRowAcceleration, "NewtonUserJointSetRowAcceleration" );
        bindFunc( cast(void** )&NewtonUserJointSetRowSpringDamperAcceleration, "NewtonUserJointSetRowSpringDamperAcceleration" );
        bindFunc( cast(void** )&NewtonUserJointSetRowStiffness, "NewtonUserJointSetRowStiffness" );
        bindFunc( cast(void** )&NewtonUserJointGetRowForce, "NewtonUserJointGetRowForce" );
        bindFunc( cast(void** )&NewtonUserJointSetSolver, "NewtonUserJointSetSolver" );
        bindFunc( cast(void** )&NewtonMeshCreate, "NewtonMeshCreate" );
        bindFunc( cast(void** )&NewtonMeshCreateFromMesh, "NewtonMeshCreateFromMesh" );
        bindFunc( cast(void** )&NewtonMeshCreateFromCollision, "NewtonMeshCreateFromCollision" );
        bindFunc( cast(void** )&NewtonMeshCreateConvexHull, "NewtonMeshCreateConvexHull" );
        bindFunc( cast(void** )&NewtonMeshCreateDelaunayTetrahedralization, "NewtonMeshCreateDelaunayTetrahedralization" );
        bindFunc( cast(void** )&NewtonMeshCreateVoronoiConvexDecomposition, "NewtonMeshCreateVoronoiConvexDecomposition" );
        bindFunc( cast(void** )&NewtonMeshDestroy, "NewtonMeshDestroy" );
        bindFunc( cast(void** )&NewtonMeshSaveOFF, "NewtonMeshSaveOFF" );
        bindFunc( cast(void** )&NewtonMeshLoadOFF, "NewtonMeshLoadOFF" );
        bindFunc( cast(void** )&NewtonMesApplyTransform, "NewtonMesApplyTransform" );
        bindFunc( cast(void** )&NewtonMeshCalculateOOBB, "NewtonMeshCalculateOOBB" );
        bindFunc( cast(void** )&NewtonMeshCalculateVertexNormals, "NewtonMeshCalculateVertexNormals" );
        bindFunc( cast(void** )&NewtonMeshApplySphericalMapping, "NewtonMeshApplySphericalMapping" );
        bindFunc( cast(void** )&NewtonMeshApplyBoxMapping, "NewtonMeshApplyBoxMapping" );
        bindFunc( cast(void** )&NewtonMeshApplyCylindricalMapping, "NewtonMeshApplyCylindricalMapping" );
        bindFunc( cast(void** )&NewtonMeshIsOpenMesh, "NewtonMeshIsOpenMesh" );
        bindFunc( cast(void** )&NewtonMeshFixTJoints, "NewtonMeshFixTJoints" );
        bindFunc( cast(void** )&NewtonMeshPolygonize, "NewtonMeshPolygonize" );
        bindFunc( cast(void** )&NewtonMeshTriangulate, "NewtonMeshTriangulate" );
        bindFunc( cast(void** )&NewtonMeshUnion, "NewtonMeshUnion" );
        bindFunc( cast(void** )&NewtonMeshDifference, "NewtonMeshDifference" );
        bindFunc( cast(void** )&NewtonMeshIntersection, "NewtonMeshIntersection" );
        bindFunc( cast(void** )&NewtonMeshClip, "NewtonMeshClip" );
        bindFunc( cast(void** )&NewtonMeshSimplify, "NewtonMeshSimplify" );
        bindFunc( cast(void** )&NewtonMeshApproximateConvexDecomposition, "NewtonMeshApproximateConvexDecomposition" );
        bindFunc( cast(void** )&NewtonRemoveUnusedVertices, "NewtonRemoveUnusedVertices" );
        bindFunc( cast(void** )&NewtonMeshBeginFace, "NewtonMeshBeginFace" );
        bindFunc( cast(void** )&NewtonMeshAddFace, "NewtonMeshAddFace" );
        bindFunc( cast(void** )&NewtonMeshEndFace, "NewtonMeshEndFace" );
        bindFunc( cast(void** )&NewtonMeshBuildFromVertexListIndexList, "NewtonMeshBuildFromVertexListIndexList" );
        bindFunc( cast(void** )&NewtonMeshGetVertexStreams, "NewtonMeshGetVertexStreams" );
        bindFunc( cast(void** )&NewtonMeshGetIndirectVertexStreams, "NewtonMeshGetIndirectVertexStreams" );
        bindFunc( cast(void** )&NewtonMeshBeginHandle, "NewtonMeshBeginHandle" );
        bindFunc( cast(void** )&NewtonMeshEndHandle, "NewtonMeshEndHandle" );
        bindFunc( cast(void** )&NewtonMeshFirstMaterial, "NewtonMeshFirstMaterial" );
        bindFunc( cast(void** )&NewtonMeshNextMaterial, "NewtonMeshNextMaterial" );
        bindFunc( cast(void** )&NewtonMeshMaterialGetMaterial, "NewtonMeshMaterialGetMaterial" );
        bindFunc( cast(void** )&NewtonMeshMaterialGetIndexCount, "NewtonMeshMaterialGetIndexCount" );
        bindFunc( cast(void** )&NewtonMeshMaterialGetIndexStream, "NewtonMeshMaterialGetIndexStream" );
        bindFunc( cast(void** )&NewtonMeshMaterialGetIndexStreamShort, "NewtonMeshMaterialGetIndexStreamShort" );
        bindFunc( cast(void** )&NewtonMeshCreateFirstSingleSegment, "NewtonMeshCreateFirstSingleSegment" );
        bindFunc( cast(void** )&NewtonMeshCreateNextSingleSegment, "NewtonMeshCreateNextSingleSegment" );
        bindFunc( cast(void** )&NewtonMeshCreateFirstLayer, "NewtonMeshCreateFirstLayer" );
        bindFunc( cast(void** )&NewtonMeshCreateNextLayer, "NewtonMeshCreateNextLayer" );
        bindFunc( cast(void** )&NewtonMeshGetTotalFaceCount, "NewtonMeshGetTotalFaceCount" );
        bindFunc( cast(void** )&NewtonMeshGetTotalIndexCount, "NewtonMeshGetTotalIndexCount" );
        bindFunc( cast(void** )&NewtonMeshGetFaces, "NewtonMeshGetFaces" );
        bindFunc( cast(void** )&NewtonMeshGetPointCount, "NewtonMeshGetPointCount" );
        bindFunc( cast(void** )&NewtonMeshGetPointStrideInByte, "NewtonMeshGetPointStrideInByte" );
        bindFunc( cast(void** )&NewtonMeshGetPointArray, "NewtonMeshGetPointArray" );
        bindFunc( cast(void** )&NewtonMeshGetNormalArray, "NewtonMeshGetNormalArray" );
        bindFunc( cast(void** )&NewtonMeshGetUV0Array, "NewtonMeshGetUV0Array" );
        bindFunc( cast(void** )&NewtonMeshGetUV1Array, "NewtonMeshGetUV1Array" );
        bindFunc( cast(void** )&NewtonMeshGetVertexCount, "NewtonMeshGetVertexCount" );
        bindFunc( cast(void** )&NewtonMeshGetVertexStrideInByte, "NewtonMeshGetVertexStrideInByte" );
        bindFunc( cast(void** )&NewtonMeshGetVertexArray, "NewtonMeshGetVertexArray" );
        bindFunc( cast(void** )&NewtonMeshGetFirstVertex, "NewtonMeshGetFirstVertex" );
        bindFunc( cast(void** )&NewtonMeshGetNextVertex, "NewtonMeshGetNextVertex" );
        bindFunc( cast(void** )&NewtonMeshGetVertexIndex, "NewtonMeshGetVertexIndex" );
        bindFunc( cast(void** )&NewtonMeshGetFirstPoint, "NewtonMeshGetFirstPoint" );
        bindFunc( cast(void** )&NewtonMeshGetNextPoint, "NewtonMeshGetNextPoint" );
        bindFunc( cast(void** )&NewtonMeshGetPointIndex, "NewtonMeshGetPointIndex" );
        bindFunc( cast(void** )&NewtonMeshGetVertexIndexFromPoint, "NewtonMeshGetVertexIndexFromPoint" );
        bindFunc( cast(void** )&NewtonMeshGetFirstEdge, "NewtonMeshGetFirstEdge" );
        bindFunc( cast(void** )&NewtonMeshGetNextEdge, "NewtonMeshGetNextEdge" );
        bindFunc( cast(void** )&NewtonMeshGetEdgeIndices, "NewtonMeshGetEdgeIndices" );
        bindFunc( cast(void** )&NewtonMeshGetFirstFace, "NewtonMeshGetFirstFace" );
        bindFunc( cast(void** )&NewtonMeshGetNextFace, "NewtonMeshGetNextFace" );
        bindFunc( cast(void** )&NewtonMeshIsFaceOpen, "NewtonMeshIsFaceOpen" );
        bindFunc( cast(void** )&NewtonMeshGetFaceMaterial, "NewtonMeshGetFaceMaterial" );
        bindFunc( cast(void** )&NewtonMeshGetFaceIndexCount, "NewtonMeshGetFaceIndexCount" );
        bindFunc( cast(void** )&NewtonMeshGetFaceIndices, "NewtonMeshGetFaceIndices" );
        bindFunc( cast(void** )&NewtonMeshGetFacePointIndices, "NewtonMeshGetFacePointIndices" );
        bindFunc( cast(void** )&NewtonMeshCalculateFaceNormal, "NewtonMeshCalculateFaceNormal" );
        bindFunc( cast(void** )&NewtonMeshSetFaceMaterial, "NewtonMeshSetFaceMaterial" );
    }
}