/* eslint-disable @typescript-eslint/no-explicit-any */
// This is an adaptation of the Babylon.js HavokPlugin for Three.js.
// All Babylon.js dependencies have been replaced with Three.js equivalents.
// Original authors from Babylon.js team. Adapted with Gemini 2.5 Pro on T3 Chat. Heavily updated and fixed by Gavin Quach.

import { Box3 } from "three/src/math/Box3.js";
import { Quaternion } from "three/src/math/Quaternion.js";
import { Vector3 } from "three/src/math/Vector3.js";
import { Matrix4 } from "three/src/math/Matrix4.js";

import type { PhysicsBody } from "./physicsBody";
import type {
    IPhysicsPointProximityQuery,
    IPhysicsShapeCastQuery,
    IPhysicsShapeProximityCastQuery,
    IRaycastQuery,
    PhysicsRaycastResult,
    ProximityCastResult,
    ShapeCastResult,
} from "./physicsRaycastResult";
import { PhysicsShape } from "./physicsShape";

import {
    type IBasePhysicsCollisionEvent,
    type IPhysicsCollisionEvent,
    PhysicsActivationControl,
    PhysicsConstraintAxis,
    PhysicsConstraintAxisLimitMode,
    PhysicsConstraintMotorType,
    type PhysicsMassProperties,
    PhysicsMaterialCombineMode,
    PhysicsMotionType,
    PhysicsPrestepType,
    PhysicsShapeType,
    type PhysicsEventType,
    type PhysicsMaterial,
    type PhysicsShapeParameters,
} from "@/utils/three/havok/types/havok";
import { Observable } from "@/utils/three/Observable";
import { VECTOR_RIGHT } from "@/utils/three/constants";
import { getVectorNormalToRef } from "@/utils/three/vectorUtils";

import type {
    HavokPhysicsWithBindings,
    MassProperties,
    QTransform,
    Quaternion as QuaternionHavok,
    Vector3 as Vector3Havok,
    PhysicsMaterial as PhysicsMaterialHavok,
    QSTransform,
    RayCastInput,
    HP_BodyId,
    HP_ConstraintId,
    PointProximityInput,
    ShapeProximityInput,
    ShapeCastInput,
    HP_WorldId,
    HP_CollectorId,
} from "@babylonjs/havok";
import type {
    InstancedMesh,
    Mesh,
    Object3D,
} from "three";
import type { Physics6DoFConstraint, PhysicsConstraint } from "./physicsConstraint";

export interface HavokCollisionEvent {
    type: "collision" | "collisionended";
    detail: IPhysicsCollisionEvent;
}

export interface HavokTriggerEvent {
    type: "trigger";
    detail: IBasePhysicsCollisionEvent;
}

const tempMatrixes = Array.from({ length: 4 }, () => new Matrix4());
const tempVecs = Array.from({ length: 3 }, () => new Vector3());
const tempQuats = Array.from({ length: 1 }, () => new Quaternion());

const worldPos = new Vector3();
const worldQuat = new Quaternion();

const isMatrixIdentity = (m: Matrix4): boolean => {
    return m.elements[0] === 1 &&
        m.elements[1] === 0 &&
        m.elements[2] === 0 &&
        m.elements[3] === 0 &&
        m.elements[4] === 0 &&
        m.elements[5] === 1 &&
        m.elements[6] === 0 &&
        m.elements[7] === 0 &&
        m.elements[8] === 0 &&
        m.elements[9] === 0 &&
        m.elements[10] === 1 &&
        m.elements[11] === 0 &&
        m.elements[12] === 0 &&
        m.elements[13] === 0 &&
        m.elements[14] === 0 &&
        m.elements[15] === 1;
};

// Helper class to gather mesh data from a Three.js hierarchy
class MeshAccumulator {
    private _vertices: Array<Vector3>; /// Vertices in body space
    private _indices: Array<number>;
    private _isRightHanded: boolean;
    private _collectIndices: boolean;

    private readonly meshWorldScale: Vector3 = new Vector3();

    /**
     * Constructor of the mesh accumulator
     * @param mesh - The mesh used to compute the world matrix.
     * @param collectIndices - use mesh indices
     * @param scene - The scene used to determine the right handed system.
     *
     * Merge mesh and its children so whole hierarchy can be used as a mesh shape or convex hull
     */
    constructor(collectIndices: boolean) {
        this._vertices = [];
        this._indices = [];
        this._isRightHanded = true;
        this._collectIndices = collectIndices;
    }
    /**
     * Adds a mesh to the physics engine.
     * @param mesh The mesh to add.
     * @param includeChildren Whether to include the children of the mesh.
     *
     * This method adds a mesh to the physics engine by computing the world matrix,
     * multiplying it with the body from world matrix, and then transforming the
     * coordinates of the mesh's vertices. It also adds the indices of the mesh
     * to the physics engine. If includeChildren is true, it will also add the
     * children of the mesh to the physics engine, ignoring any children which
     * have a physics impostor. This is useful for creating a physics engine
     * that accurately reflects the mesh and its children.
     */
    addNodeMeshes(object: Object3D, includeChildren: boolean) {
        object.updateMatrixWorld(true);

        const rootScaled = tempMatrixes[0];
        object.getWorldScale(this.meshWorldScale);
        rootScaled.makeScale(this.meshWorldScale.x, this.meshWorldScale.y, this.meshWorldScale.z);

        if (object.type === "Mesh") this._addMesh(object as Mesh, rootScaled);

        if (includeChildren) {
            object.updateMatrixWorld(true);
            const worldToRoot = tempMatrixes[1];
            worldToRoot.copy(object.matrixWorld.clone().invert());
            const worldToRootScaled = tempMatrixes[2];
            worldToRootScaled.multiplyMatrices(rootScaled, worldToRoot);

            const children: Mesh[] = [];
            object.traverse((node) => {
                if (node.type === "Mesh") children.push(node as Mesh)
            })

            //  Ignore any children which have a physics body.
            //  Other plugin implementations do not have this check, which appears to be
            //  a bug, as otherwise, the mesh will have a duplicate collider
            const transformNodes = children.filter((node) => !node.userData.physicsBody);
            for (const node of transformNodes) {
                node.updateMatrixWorld(true);
                const childToWorld = node.matrixWorld.clone();
                const childToRootScaled = tempMatrixes[3];
                childToRootScaled.multiplyMatrices(worldToRootScaled, childToWorld);
                if (node.type === "Mesh") {
                    this._addMesh(node, childToRootScaled);
                }
            }
        }
    }
    private _addMesh(mesh: Mesh, meshToRoot: Matrix4) {
        // Ensure the world matrix is up-to-date
        mesh.updateMatrixWorld(true);

        const geometry = mesh.geometry;
        const positionAttribute = geometry.attributes.position;

        const numVerts = positionAttribute.count;

        // The offset for indices is the number of vertices we've already collected.
        const indexOffset = this._vertices.length;

        for (let v = 0; v < numVerts; v++) {
            const pos = new Vector3().fromBufferAttribute(positionAttribute, v);
            // Apply the transformation matrix to the vertex
            pos.applyMatrix4(meshToRoot);
            this._vertices.push(pos);
        }

        if (this._collectIndices) {
            const indexAttribute = geometry.index;
            if (indexAttribute) {
                const meshIndices = indexAttribute.array;
                for (let i = 0; i < meshIndices.length; i += 3) {
                    // Havok (and js) uses a right-handed coordinate system.
                    // The winding order determines the "front" face of a triangle.
                    if (this._isRightHanded) {
                        // Standard right-hand rule winding
                        this._indices.push(
                            meshIndices[i + 0] + indexOffset,
                            meshIndices[i + 1] + indexOffset,
                            meshIndices[i + 2] + indexOffset
                        );
                    } else {
                        // Reversed winding for left-handed coordinate systems
                        this._indices.push(
                            meshIndices[i + 2] + indexOffset,
                            meshIndices[i + 1] + indexOffset,
                            meshIndices[i + 0] + indexOffset
                        );
                    }
                }
            }
        }
    }

    /**
     * Allocate and populate the vertex positions inside the physics plugin.
     *
     * @param plugin - The plugin to allocate the memory in.
     * @returns An array of floats, whose backing memory is inside the plugin. The array contains the
     * positions of the mesh vertices, where a position is defined by three floats. You must call
     * freeBuffer() on the returned array once you have finished with it, in order to free the
     * memory inside the plugin..
     */
    getVertices(engine: HavokPhysicsWithBindings) {
        const nFloats = this._vertices.length * 3;
        const bytesPerFloat = 4;
        const nBytes = nFloats * bytesPerFloat;
        const bufferBegin = engine._malloc(nBytes);
        const ret = new Float32Array(engine.HEAPU8.buffer, bufferBegin, nFloats);
        for (let i = 0; i < this._vertices.length; i++) {
            ret[i * 3 + 0] = this._vertices[i].x;
            ret[i * 3 + 1] = this._vertices[i].y;
            ret[i * 3 + 2] = this._vertices[i].z;
        }
        return { offset: bufferBegin, numObjects: nFloats };
    }
    freeBuffer(
        engine: HavokPhysicsWithBindings,
        arr: {
            offset: number;
            numObjects: number;
        }
    ) {
        engine._free(arr.offset);
    }
    /**
     * Allocate and populate the triangle indices inside the physics engine
     *
     * @param engine - The engine to allocate the memory in.
     * @returns A new Int32Array, whose backing memory is inside the engine. The array contains the indices
     * of the triangle positions, where a single triangle is defined by three indices. You must call
     * freeBuffer() on this array once you have finished with it, to free the memory inside the engine..
     */
    getTriangles(engine: HavokPhysicsWithBindings) {
        const bytesPerInt = 4;
        const nBytes = this._indices.length * bytesPerInt;
        const bufferBegin = engine._malloc(nBytes);
        const ret = new Int32Array(
            engine.HEAPU8.buffer,
            bufferBegin,
            this._indices.length
        );
        for (let i = 0; i < this._indices.length; i++) {
            ret[i] = this._indices[i];
        }
        return { offset: bufferBegin, numObjects: this._indices.length };
    }
}

export class BodyPluginData {
    public hpBodyId: HP_BodyId;
    public userMassProps: PhysicsMassProperties;
    public worldTransformOffset: number = 0;

    constructor(bodyId: any) {
        this.hpBodyId = bodyId;
        this.userMassProps = {
            centerOfMass: undefined,
            mass: undefined,
            inertia: undefined,
            inertiaOrientation: undefined,
        };
    }
}

class CollisionContactPoint {
    public bodyId: bigint = BigInt(0);
    public position: Vector3 = new Vector3();
    public normal: Vector3 = new Vector3();
}

class CollisionEvent {
    public contactOnA: CollisionContactPoint = new CollisionContactPoint();
    public contactOnB: CollisionContactPoint = new CollisionContactPoint();
    public impulseApplied: number = 0;
    public type: number = 0;

    public static readToRef(
        buffer: ArrayBuffer,
        offset: number,
        eventOut: CollisionEvent
    ): void {
        const intBuf = new Int32Array(buffer, offset);
        const floatBuf = new Float32Array(buffer, offset);
        const offA = 2;
        eventOut.contactOnA.bodyId = BigInt(intBuf[offA]);
        eventOut.contactOnA.position.set(
            floatBuf[offA + 8],
            floatBuf[offA + 9],
            floatBuf[offA + 10]
        );
        eventOut.contactOnA.normal.set(
            floatBuf[offA + 11],
            floatBuf[offA + 12],
            floatBuf[offA + 13]
        );
        const offB = 18;
        eventOut.contactOnB.bodyId = BigInt(intBuf[offB]);
        eventOut.contactOnB.position.set(
            floatBuf[offB + 8],
            floatBuf[offB + 9],
            floatBuf[offB + 10]
        );
        eventOut.contactOnB.normal.set(
            floatBuf[offB + 11],
            floatBuf[offB + 12],
            floatBuf[offB + 13]
        );
        eventOut.impulseApplied = floatBuf[offB + 13 + 3];
        eventOut.type = intBuf[0];
    }
}

class TriggerEvent {
    public bodyIdA: bigint = BigInt(0);
    public bodyIdB: bigint = BigInt(0);
    public type: number = 0;

    public static readToRef(
        buffer: ArrayBuffer,
        offset: number,
        eventOut: TriggerEvent
    ): void {
        const intBuf = new Int32Array(buffer, offset);
        eventOut.type = intBuf[0];
        eventOut.bodyIdA = BigInt(intBuf[2]);
        eventOut.bodyIdB = BigInt(intBuf[6]);
    }
}

/**
 * The Havok Physics plugin for js
 */
export class HavokPlugin {
    public _hknp: HavokPhysicsWithBindings;
    public name: string = "HavokPlugin";
    public world: HP_WorldId;

    private _useDeltaForWorldStep: boolean;
    private _fixedTimeStep: number = 1 / 60;
    private _bodies: Map<bigint, { body: PhysicsBody; index: number }> =
        new Map();
    private _shapes: Map<number, PhysicsShape> = new Map();
    private _bodyCollisionObservable: Map<bigint, Observable<IPhysicsCollisionEvent>> = new Map();
    private _constraintToBodyIdPair: Map<bigint, [bigint, bigint]> = new Map();
    private _bodyCollisionEndedObservable: Map<bigint, Observable<IPhysicsCollisionEvent>> = new Map();
    private _queryCollector: HP_CollectorId;
    private _bodyBuffer: number = 0;

    /**
     * Observable for collision started and collision continued events
     */
    onCollisionObservable: Observable<IPhysicsCollisionEvent>;
    /**
     * Observable for collision ended events
     */
    onCollisionEndedObservable: Observable<IBasePhysicsCollisionEvent>;
    /**
     * Observable for trigger entered and trigger exited events
     */
    onTriggerCollisionObservable: Observable<IBasePhysicsCollisionEvent>;

    constructor(
        _useDeltaForWorldStep: boolean = true,
        hpInjection: HavokPhysicsWithBindings
    ) {
        this._useDeltaForWorldStep = _useDeltaForWorldStep;
        this._hknp = hpInjection;
        this.world = this._hknp.HP_World_Create()[1];
        this._queryCollector = this._hknp.HP_QueryCollector_Create(1)[1];

        this.onCollisionObservable = new Observable();
        this.onCollisionEndedObservable = new Observable();
        this.onTriggerCollisionObservable = new Observable();
    }
    /**
     * If this plugin is supported
     * @returns true if its supported
     */
    isSupported() {
        return this._hknp !== undefined;
    }
    /**
     * Sets the gravity of the physics world.
     *
     * @param gravity - The gravity vector to set.
     *
     */
    public setGravity(gravity: Vector3): void {
        this._hknp.HP_World_SetGravity(
            this.world,
            this._threeVectorToHavokArray(gravity)
        );
    }
    /**
     * Sets the fixed time step for the physics engine.
     *
     * @param timeStep - The fixed time step to use for the physics engine.
     *
     */
    setTimeStep(timeStep: number) {
        this._fixedTimeStep = timeStep;
    }
    /**
     * Gets the fixed time step used by the physics engine.
     *
     * @returns The fixed time step used by the physics engine.
     *
     */
    getTimeStep(): number {
        return this._fixedTimeStep;
    }

    /**
     * Executes a single step of the physics engine.
     *
     * @param delta The time delta in seconds since the last step.
     * @param physicsBodies An array of physics bodies to be simulated.
     *
     * This method is useful for simulating the physics engine. It sets the physics body transformation,
     * steps the world, syncs the physics body, and notifies collisions. This allows for the physics engine
     * to accurately simulate the physics bodies in the world.
     */
    public executeStep(
        delta: number,
        physicsBodies: PhysicsBody[] = [...this._bodies.values()].map((b) => b.body)
    ): void {
        for (const physicsBody of physicsBodies) {
            if (physicsBody.disablePreStep) {
                continue;
            }
            this.setPhysicsBodyTransformation(physicsBody, physicsBody.node);
        }
        const deltaTime = this._useDeltaForWorldStep ? delta : this._fixedTimeStep;
        this._hknp.HP_World_SetIdealStepTime(this.world, deltaTime);
        this._hknp.HP_World_Step(this.world, deltaTime);
        this._bodyBuffer = this._hknp.HP_World_GetBodyBuffer(this.world)[1];
        for (const physicsBody of physicsBodies) {
            if (!physicsBody.disableSync) {
                this.sync(physicsBody);
            }
        }
        this._notifyCollisions();
        this._notifyTriggers();
    }

    public getPluginVersion(): number {
        return 2;
    }

    /**
     * Set the maximum allowed linear and angular velocities
     * @param maxLinearVelocity maximum allowed linear velocity
     * @param maxAngularVelocity maximum allowed angular velocity
     */
    public setVelocityLimits(
        maxLinearVelocity: number,
        maxAngularVelocity: number
    ): void {
        this._hknp.HP_World_SetSpeedLimit(
            this.world,
            maxLinearVelocity,
            maxAngularVelocity
        );
    }

    /**
     * @returns maximum allowed linear velocity
     */
    public getMaxLinearVelocity(): number {
        const limits = this._hknp.HP_World_GetSpeedLimit(this.world);
        return limits[1];
    }

    /**
     * @returns maximum allowed angular velocity
     */
    public getMaxAngularVelocity(): number {
        const limits = this._hknp.HP_World_GetSpeedLimit(this.world);
        return limits[2];
    }

    /**
     * Initializes a physics body with the given position and orientation.
     *
     * @param body - The physics body to initialize.
     * @param motionType - The motion type of the body.
     * @param position - The position of the body.
     * @param orientation - The orientation of the body.
     * This code is useful for initializing a physics body with the given position and orientation.
     * It creates a plugin data for the body and adds it to the world. It then converts the position
     * and orientation to a transform and sets the body's transform to the given values.
     */
    public initBody(
        body: PhysicsBody,
        motionType: PhysicsMotionType,
        position: Vector3,
        orientation: Quaternion
    ): void {
        body._pluginData = new BodyPluginData(this._hknp.HP_Body_Create()[1]);
        this._internalSetMotionType(body._pluginData, motionType);
        const transform: QTransform = [
            this._threeVectorToHavokArray(position),
            this._threeQuaternionToHavokArray(orientation),
        ];
        this._hknp.HP_Body_SetQTransform(body._pluginData.hpBodyId, transform);
        this._hknp.HP_World_AddBody(
            this.world,
            body._pluginData.hpBodyId,
            body.startAsleep
        );
        this._bodies.set(body._pluginData.hpBodyId[0], { body: body, index: 0 });
    }

    /**
     * Removes a body from the world. To dispose of a body, it is necessary to remove it from the world first.
     *
     * @param body - The body to remove.
     */
    public removeBody(body: PhysicsBody): void {
        if (body._pluginDataInstances && body._pluginDataInstances.length > 0) {
            for (const instance of body._pluginDataInstances) {
                this._hknp.HP_World_RemoveBody(this.world, instance.hpBodyId);
                this._bodies.delete(instance.hpBodyId[0]);
            }
        }
        if (body._pluginData) {
            this._hknp.HP_World_RemoveBody(this.world, body._pluginData.hpBodyId);
            this._bodies.delete(body._pluginData.hpBodyId[0]);
        }
    }

    /**
     * Initializes the body instances for a given physics body and mesh.
     *
     * @param body - The physics body to initialize.
     * @param motionType - How the body will be handled by the engine
     * @param mesh - The mesh to initialize.
     *
     * This code is useful for creating a physics body from a mesh. It creates a
     * body instance for each instance of the mesh and adds it to the world. It also
     * sets the position of the body instance to the position of the mesh instance.
     * This allows for the physics engine to accurately simulate the mesh in the
     * world.
     */
    public initBodyInstances(
        body: PhysicsBody,
        motionType: PhysicsMotionType,
        mesh: InstancedMesh
    ): void {
        const instancesCount = mesh.count;
        const matrixData = mesh.instanceMatrix.array;

        if (!matrixData) {
            return;
        }
        this._createOrUpdateBodyInstances(
            body,
            motionType,
            matrixData as Float32Array,
            0,
            instancesCount,
            false
        );
        for (let index = 0; index < body._pluginDataInstances.length; index++) {
            const bodyId = body._pluginDataInstances[index];
            this._bodies.set(bodyId.hpBodyId[0], { body: body, index: index });
        }
    }

    private _createOrUpdateBodyInstances(
        body: PhysicsBody,
        motionType: PhysicsMotionType,
        matrixData: Float32Array,
        startIndex: number,
        endIndex: number,
        update: boolean
    ): void {
        const rotation = new Quaternion();
        const rotationMatrix = new Matrix4();

        for (let i = startIndex; i < endIndex; i++) {
            const position: Vector3Havok = [
                matrixData[i * 16 + 12],
                matrixData[i * 16 + 13],
                matrixData[i * 16 + 14],
            ];
            const hkbody = update
                ? body._pluginDataInstances[i].hpBodyId
                : this._hknp.HP_Body_Create()[1];

            rotationMatrix.set(
                matrixData[i * 16 + 0],
                matrixData[i * 16 + 4],
                matrixData[i * 16 + 8],
                0,
                matrixData[i * 16 + 1],
                matrixData[i * 16 + 5],
                matrixData[i * 16 + 9],
                0,
                matrixData[i * 16 + 2],
                matrixData[i * 16 + 6],
                matrixData[i * 16 + 10],
                0,
                0,
                0,
                0,
                1
            );
            rotation.setFromRotationMatrix(rotationMatrix);

            const transform: QTransform = [
                position,
                this._threeQuaternionToHavokArray(rotation),
            ];
            this._hknp.HP_Body_SetQTransform(hkbody, transform);

            if (!update) {
                const pluginData = new BodyPluginData(hkbody);
                if (body._pluginDataInstances.length > 0) {
                    pluginData.userMassProps = body._pluginDataInstances[0].userMassProps;
                }
                this._internalSetMotionType(pluginData, motionType);
                this._internalUpdateMassProperties(pluginData);
                body._pluginDataInstances.push(pluginData);
                this._hknp.HP_World_AddBody(this.world, hkbody, body.startAsleep);
                pluginData.worldTransformOffset =
                    this._hknp.HP_Body_GetWorldTransformOffset(hkbody)[1];
            }
        }
    }

    /**
     * Update the internal body instances for a given physics body to match the instances in a mesh.
     * @param body the body that will be updated
     * @param mesh the mesh with reference instances
     */
    public updateBodyInstances(body: PhysicsBody, mesh: InstancedMesh): void {
        const instancesCount = mesh.count;
        const matrixData = mesh.instanceMatrix.array as Float32Array;
        if (!matrixData) {
            return;
        }

        const pluginInstancesCount = body._pluginDataInstances.length;
        const motionType = this.getMotionType(body);

        if (instancesCount > pluginInstancesCount) {
            this._createOrUpdateBodyInstances(
                body,
                motionType,
                matrixData,
                pluginInstancesCount,
                instancesCount,
                false
            );
            const firstBodyShape = this._hknp.HP_Body_GetShape(
                body._pluginDataInstances[0].hpBodyId
            )[1];
            if (!firstBodyShape[0]) {
                firstBodyShape[0] = body.shape?._pluginData[0];
            }
            for (let i = pluginInstancesCount; i < instancesCount; i++) {
                this._hknp.HP_Body_SetShape(
                    body._pluginDataInstances[i].hpBodyId,
                    firstBodyShape
                );
                this._internalUpdateMassProperties(body._pluginDataInstances[i]);
                this._bodies.set(body._pluginDataInstances[i].hpBodyId[0], {
                    body: body,
                    index: i,
                });
            }
        } else if (instancesCount < pluginInstancesCount) {
            const instancesToRemove = pluginInstancesCount - instancesCount;
            for (let i = 0; i < instancesToRemove; i++) {
                const hkbody = body._pluginDataInstances.pop();
                if (hkbody) {
                    this._bodies.delete(hkbody.hpBodyId[0]);
                    this._hknp.HP_World_RemoveBody(this.world, hkbody.hpBodyId);
                    this._hknp.HP_Body_Release(hkbody.hpBodyId);
                }
            }
        }

        // Always update existing instances
        this._createOrUpdateBodyInstances(
            body,
            motionType,
            matrixData,
            0,
            instancesCount,
            true
        );
    }

    public sync(body: PhysicsBody): void {
        this.syncTransform(body, body.node);
    }

    public syncTransform(body: PhysicsBody, node: Object3D): void {
        if (body._pluginDataInstances.length > 0) {
            // Instances
            const m = node as InstancedMesh;
            const matrixData = m.instanceMatrix.array;
            if (!matrixData) {
                return;
            }
            const instancesCount = body._pluginDataInstances.length;
            for (let i = 0; i < instancesCount; i++) {
                const bufOffset = body._pluginDataInstances[i].worldTransformOffset;
                const transformBuffer = new Float32Array(
                    this._hknp.HEAPU8.buffer,
                    this._bodyBuffer + bufOffset,
                    16
                );
                const index = i * 16;
                // Copy rotation and position, preserving scale
                matrixData[index + 0] = transformBuffer[0];
                matrixData[index + 1] = transformBuffer[1];
                matrixData[index + 2] = transformBuffer[2];
                matrixData[index + 4] = transformBuffer[4];
                matrixData[index + 5] = transformBuffer[5];
                matrixData[index + 6] = transformBuffer[6];
                matrixData[index + 8] = transformBuffer[8];
                matrixData[index + 9] = transformBuffer[9];
                matrixData[index + 10] = transformBuffer[10];
                matrixData[index + 12] = transformBuffer[12];
                matrixData[index + 13] = transformBuffer[13];
                matrixData[index + 14] = transformBuffer[14];
            }
            m.instanceMatrix.needsUpdate = true;
        } else {
            try {
                // regular
                const bodyTransform = this._hknp.HP_Body_GetQTransform(body._pluginData.hpBodyId)[1];
                const bodyTranslation = bodyTransform[0];
                const bodyOrientation = bodyTransform[1];
                const quat = tempQuats[0].fromArray(bodyOrientation);
                const parent = node.parent;
                // transform position/orientation in parent space
                if (parent && !isMatrixIdentity(parent.matrixWorld)) {
                    parent.updateMatrixWorld(true);

                    // Save scaling for future use
                    tempVecs[1].copy(node.scale);
                    quat.normalize();
                    const finalTransform = tempMatrixes[0];
                    const finalTranslation = tempVecs[0].fromArray(bodyTranslation);
                    const absoluteScaling = tempVecs[2];
                    node.getWorldScale(absoluteScaling);
                    // Matrix.ComposeToRef(transformNode.absoluteScaling, quat, finalTranslation, finalTransform);
                    finalTransform.compose(finalTranslation, quat, absoluteScaling);

                    const parentInverseTransform = tempMatrixes[1];
                    parentInverseTransform.copy(parent.matrixWorld);
                    parentInverseTransform.invert();
                    const localTransform = tempMatrixes[2];
                    localTransform.multiplyMatrices(parentInverseTransform, finalTransform);
                    localTransform.decompose(node.position, node.quaternion, node.scale);
                    // decomposeToNode(localTransform, node);
                    node.quaternion.normalize();
                    // Keep original scaling. Re-injecting scaling can introduce discontinuity between frames. Basically, it grows or shrinks.
                    node.scale.copy(tempVecs[1]);
                }
                else {
                    node.position.set(bodyTranslation[0], bodyTranslation[1], bodyTranslation[2]);
                    node.quaternion.copy(quat);
                }
            } catch (error) {
                console.error(
                    `Syncing transform failed for node ${node.name}:`,
                    error
                );
            }
        }
    }

    public setShape(body: PhysicsBody, shape: PhysicsShape | null): void {
        const shapeHandle =
            shape && shape._pluginData ? shape._pluginData : BigInt(0);
        if (
            !(
                (body.node as InstancedMesh).isInstancedMesh &&
                (body.node as InstancedMesh).instanceMatrix
            )
        ) {
            this._hknp.HP_Body_SetShape(body._pluginData.hpBodyId, shapeHandle);
            this._internalUpdateMassProperties(body._pluginData);
            return;
        }

        const m = body.node as InstancedMesh;
        const instancesCount = m.count;
        for (let i = 0; i < instancesCount; i++) {
            this._hknp.HP_Body_SetShape(
                body._pluginDataInstances[i].hpBodyId,
                shapeHandle
            );
            this._internalUpdateMassProperties(body._pluginDataInstances[i]);
        }
    }

    private _getPluginReference(
        body: PhysicsBody,
        instanceIndex?: number
    ): BodyPluginData {
        return body._pluginDataInstances?.length
            ? body._pluginDataInstances[instanceIndex ?? 0]
            : body._pluginData;
    }

    public getShape(body: PhysicsBody): PhysicsShape | null {
        const pluginRef = this._getPluginReference(body);
        const shapePluginData = this._hknp.HP_Body_GetShape(pluginRef.hpBodyId)[1];
        if ((shapePluginData as unknown as number) !== 0) {
            return new PhysicsShape({ pluginData: shapePluginData }, this);
        }
        return null;
    }

    public getShapeType(shape: PhysicsShape): PhysicsShapeType {
        // TODO HP_Shape_GetType returns a native type!
        return (
            shape.type ??
            (this._hknp.HP_Shape_GetType(
                shape._pluginData
            ) as unknown as PhysicsShapeType)
        );
    }

    public setEventMask(
        body: PhysicsBody,
        eventMask: number,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (bodyPluginData) => {
                this._hknp.HP_Body_SetEventMask(bodyPluginData.hpBodyId, eventMask);
            },
            instanceIndex
        );
    }

    public getEventMask(body: PhysicsBody, instanceIndex?: number): number {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        return this._hknp.HP_Body_GetEventMask(pluginRef.hpBodyId)[1];
    }

    private _fromMassPropertiesTuple(
        massPropsTuple: any[]
    ): PhysicsMassProperties {
        return {
            centerOfMass: new Vector3().fromArray(massPropsTuple[0]),
            mass: massPropsTuple[1],
            inertia: new Vector3().fromArray(massPropsTuple[2]),
            inertiaOrientation: new Quaternion().fromArray(massPropsTuple[3]),
        };
    }

    private _internalUpdateMassProperties(pluginData: BodyPluginData): void {
        const newProps = this._internalComputeMassProperties(pluginData);
        const massProps = pluginData.userMassProps;

        if (massProps.centerOfMass) {
            newProps[0] = massProps.centerOfMass.toArray();
        }
        if (massProps.mass != undefined) {
            newProps[1] = massProps.mass;
        }
        if (massProps.inertia) {
            newProps[2] = massProps.inertia.toArray();
        }
        if (massProps.inertiaOrientation) {
            newProps[3] = massProps.inertiaOrientation.toArray();
        }
        this._hknp.HP_Body_SetMassProperties(
            pluginData.hpBodyId,
            newProps as MassProperties
        );
    }

    _internalSetMotionType(
        pluginData: BodyPluginData,
        motionType: PhysicsMotionType
    ): void {
        switch (motionType) {
            case PhysicsMotionType.STATIC: {
                this._hknp.HP_Body_SetMotionType(
                    pluginData.hpBodyId,
                    this._hknp.MotionType.STATIC
                );
                break;
            }
            case PhysicsMotionType.ANIMATED: {
                this._hknp.HP_Body_SetMotionType(
                    pluginData.hpBodyId,
                    this._hknp.MotionType.KINEMATIC
                );
                break;
            }
            case PhysicsMotionType.DYNAMIC: {
                this._hknp.HP_Body_SetMotionType(
                    pluginData.hpBodyId,
                    this._hknp.MotionType.DYNAMIC
                );
                break;
            }
        }
    }

    public setMotionType(
        body: PhysicsBody,
        motionType: PhysicsMotionType,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginData) => {
                this._internalSetMotionType(pluginData, motionType);
            },
            instanceIndex
        );
    }

    public getMotionType(
        body: PhysicsBody,
        instanceIndex?: number
    ): PhysicsMotionType {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        const type = this._hknp.HP_Body_GetMotionType(pluginRef.hpBodyId)[1];
        switch (type) {
            case this._hknp.MotionType.STATIC: {
                return PhysicsMotionType.STATIC;
            }
            case this._hknp.MotionType.KINEMATIC: {
                return PhysicsMotionType.ANIMATED;
            }
            case this._hknp.MotionType.DYNAMIC: {
                return PhysicsMotionType.DYNAMIC;
            }
        }
        throw new Error("Unknown motion type: " + type);
    }

    public setActivationControl(
        body: PhysicsBody,
        controlMode: PhysicsActivationControl
    ): void {
        switch (controlMode) {
            case PhysicsActivationControl.ALWAYS_ACTIVE: {
                this._hknp.HP_Body_SetActivationControl(
                    body._pluginData.hpBodyId,
                    this._hknp.ActivationControl.ALWAYS_ACTIVE
                );
                break;
            }
            case PhysicsActivationControl.ALWAYS_INACTIVE: {
                this._hknp.HP_Body_SetActivationControl(
                    body._pluginData.hpBodyId,
                    this._hknp.ActivationControl.ALWAYS_INACTIVE
                );
                break;
            }
            case PhysicsActivationControl.SIMULATION_CONTROLLED: {
                this._hknp.HP_Body_SetActivationControl(
                    body._pluginData.hpBodyId,
                    this._hknp.ActivationControl.SIMULATION_CONTROLLED
                );
                break;
            }
        }
    }

    private _internalComputeMassProperties(pluginData: BodyPluginData): any[] {
        const shapeRes = this._hknp.HP_Body_GetShape(pluginData.hpBodyId);
        if (shapeRes[0] == this._hknp.Result.RESULT_OK) {
            const shapeMass = this._hknp.HP_Shape_BuildMassProperties(shapeRes[1]);
            if (shapeMass[0] == this._hknp.Result.RESULT_OK) {
                return shapeMass[1];
            }
        }
        return [[0, 0, 0], 1, [1, 1, 1], [0, 0, 0, 1]];
    }

    public computeMassProperties(
        body: PhysicsBody,
        instanceIndex?: number
    ): PhysicsMassProperties {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        const computed = this._internalComputeMassProperties(pluginRef);
        return this._fromMassPropertiesTuple(computed);
    }

    public setMassProperties(
        body: PhysicsBody,
        massProps: PhysicsMassProperties,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginData) => {
                pluginData.userMassProps = massProps;
                this._internalUpdateMassProperties(pluginData);
            },
            instanceIndex
        );
    }

    public getMassProperties(
        body: PhysicsBody,
        instanceIndex?: number
    ): PhysicsMassProperties {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        const massPropsTuple = this._hknp.HP_Body_GetMassProperties(
            pluginRef.hpBodyId
        )[1];
        return this._fromMassPropertiesTuple(massPropsTuple);
    }

    public setLinearDamping(
        body: PhysicsBody,
        damping: number,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginData) => {
                this._hknp.HP_Body_SetLinearDamping(pluginData.hpBodyId, damping);
            },
            instanceIndex
        );
    }

    public getLinearDamping(body: PhysicsBody, instanceIndex?: number): number {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        return this._hknp.HP_Body_GetLinearDamping(pluginRef.hpBodyId)[1];
    }

    public setAngularDamping(
        body: PhysicsBody,
        damping: number,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginData) => {
                this._hknp.HP_Body_SetAngularDamping(pluginData.hpBodyId, damping);
            },
            instanceIndex
        );
    }

    public getAngularDamping(body: PhysicsBody, instanceIndex?: number): number {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        return this._hknp.HP_Body_GetAngularDamping(pluginRef.hpBodyId)[1];
    }

    public setLinearVelocity(
        body: PhysicsBody,
        linVel: Vector3,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginData) => {
                this._hknp.HP_Body_SetLinearVelocity(
                    pluginData.hpBodyId,
                    this._threeVectorToHavokArray(linVel)
                );
            },
            instanceIndex
        );
    }

    public getLinearVelocityToRef(
        body: PhysicsBody,
        linVel: Vector3,
        instanceIndex?: number
    ): void {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        const lv = this._hknp.HP_Body_GetLinearVelocity(pluginRef.hpBodyId)[1];
        this._havokArrayToThreeVectorRef(lv, linVel);
    }

    private _applyToBodyOrInstances(
        body: PhysicsBody,
        fnToApply: (pluginData: BodyPluginData) => void,
        instanceIndex?: number
    ): void {
        if (body._pluginDataInstances?.length > 0 && instanceIndex === undefined) {
            for (let i = 0; i < body._pluginDataInstances.length; i++) {
                fnToApply(body._pluginDataInstances[i]);
            }
        } else {
            fnToApply(this._getPluginReference(body, instanceIndex));
        }
    }

    public applyImpulse(
        body: PhysicsBody,
        impulse: Vector3,
        location: Vector3,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginRef) => {
                this._hknp.HP_Body_ApplyImpulse(
                    pluginRef.hpBodyId,
                    this._threeVectorToHavokArray(location),
                    this._threeVectorToHavokArray(impulse)
                );
            },
            instanceIndex
        );
    }

    public applyAngularImpulse(
        body: PhysicsBody,
        angularImpulse: Vector3,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginRef) => {
                this._hknp.HP_Body_ApplyAngularImpulse(
                    pluginRef.hpBodyId,
                    this._threeVectorToHavokArray(angularImpulse)
                );
            },
            instanceIndex
        );
    }

    public applyForce(
        body: PhysicsBody,
        force: Vector3,
        location: Vector3,
        instanceIndex?: number
    ): void {
        force.clone().multiplyScalar(this.getTimeStep());
        this.applyImpulse(body, tempVecs[0], location, instanceIndex);
    }

    public setAngularVelocity(
        body: PhysicsBody,
        angVel: Vector3,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginRef) => {
                this._hknp.HP_Body_SetAngularVelocity(
                    pluginRef.hpBodyId,
                    this._threeVectorToHavokArray(angVel)
                );
            },
            instanceIndex
        );
    }

    public getAngularVelocityToRef(
        body: PhysicsBody,
        angVel: Vector3,
        instanceIndex?: number
    ): void {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        const av = this._hknp.HP_Body_GetAngularVelocity(pluginRef.hpBodyId)[1];
        this._havokArrayToThreeVectorRef(av, angVel);
    }

    public setPhysicsBodyTransformation(body: PhysicsBody, node: Object3D): void {
        if (body.getPrestepType() == PhysicsPrestepType.TELEPORT) {
            if (body.numInstances > 0) {
                const m = node as InstancedMesh;
                const matrixData = m.instanceMatrix.array as Float32Array;
                if (!matrixData) {
                    return;
                }
                this._createOrUpdateBodyInstances(
                    body,
                    body.getMotionType(),
                    matrixData,
                    0,
                    body.numInstances,
                    true
                );
            } else {
                this._hknp.HP_Body_SetQTransform(
                    body._pluginData.hpBodyId,
                    this._getTransformInfos(node)
                );
            }
        } else if (body.getPrestepType() == PhysicsPrestepType.ACTION) {
            worldPos.set(0, 0, 0);
            worldQuat.set(0, 0, 0, 1);
            node.getWorldPosition(worldPos);
            node.getWorldQuaternion(worldQuat);
            this.setTargetTransform(body, worldPos, worldQuat);
        } else if (body.getPrestepType() == PhysicsPrestepType.DISABLED) {
            console.warn(
                "Prestep type is set to DISABLED. Unable to set physics body transformation."
            );
        } else {
            console.warn("Invalid prestep type set to physics body.");
        }
    }

    public setTargetTransform(
        body: PhysicsBody,
        position: Vector3,
        rotation: Quaternion,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginRef) => {
                this._hknp.HP_Body_SetTargetQTransform(pluginRef.hpBodyId, [
                    this._threeVectorToHavokArray(position),
                    this._threeQuaternionToHavokArray(rotation),
                ]);
            },
            instanceIndex
        );
    }

    public setGravityFactor(
        body: PhysicsBody,
        factor: number,
        instanceIndex?: number
    ): void {
        this._applyToBodyOrInstances(
            body,
            (pluginRef) => {
                this._hknp.HP_Body_SetGravityFactor(pluginRef.hpBodyId, factor);
            },
            instanceIndex
        );
    }

    public getGravityFactor(body: PhysicsBody, instanceIndex?: number): number {
        const pluginRef = this._getPluginReference(body, instanceIndex);
        return this._hknp.HP_Body_GetGravityFactor(pluginRef.hpBodyId)[1];
    }

    public disposeBody(body: PhysicsBody): void {
        if (body._pluginDataInstances && body._pluginDataInstances.length > 0) {
            for (const instance of body._pluginDataInstances) {
                this._hknp.HP_Body_Release(instance.hpBodyId);
                instance.hpBodyId = undefined!;
            }
        }
        if (body._pluginData) {
            this._hknp.HP_Body_Release(body._pluginData.hpBodyId);
            body._pluginData.hpBodyId = undefined!;
        }
    }

    // There's no GroundMesh in three.js
    // private _createOptionsFromGroundMesh(options: PhysicsShapeParameters): void {
    //     const mesh = options.groundMesh;
    //     if (!mesh || !mesh.geometry.attributes.position) {
    //         return;
    //     }

    //     const pos = mesh.geometry.attributes.position.array as Float32Array;
    //     mesh.updateWorldMatrix(true, false);
    //     const transform = mesh.matrixWorld;

    //     const transformedVertices: number[] = [];
    //     const tempVec = new Vector3();
    //     for (let i = 0; i < pos.length; i += 3) {
    //         tempVec.fromArray(pos, i);
    //         tempVec.applyMatrix4(transform);
    //         tempVec.toArray(transformedVertices, i);
    //     }

    //     mesh.geometry.computeBoundingBox();
    //     const boundingBox = mesh.geometry
    //         .boundingBox!.clone()
    //         .applyMatrix4(transform);
    //     const extendSizeWorld = new Vector3();
    //     boundingBox.getSize(extendSizeWorld);

    //     const arraySize = Math.trunc(Math.sqrt(pos.length / 3) - 1);
    //     const dim = Math.min(extendSizeWorld.x, extendSizeWorld.z);
    //     const minX = boundingBox.min.x;
    //     const minY = boundingBox.min.y;
    //     const minZ = boundingBox.min.z;

    //     const matrix = new Float32Array((arraySize + 1) * (arraySize + 1));
    //     const elementSize = dim / arraySize;
    //     matrix.fill(minY);

    //     for (let i = 0; i < transformedVertices.length; i = i + 3) {
    //         const x = Math.round((transformedVertices[i + 0] - minX) / elementSize);
    //         const z =
    //             arraySize -
    //             Math.round((transformedVertices[i + 2] - minZ) / elementSize);
    //         const y = transformedVertices[i + 1] - minY;
    //         matrix[z * (arraySize + 1) + x] = y;
    //     }

    //     options.numHeightFieldSamplesX = arraySize + 1;
    //     options.numHeightFieldSamplesZ = arraySize + 1;
    //     options.heightFieldSizeX = extendSizeWorld.x;
    //     options.heightFieldSizeZ = extendSizeWorld.z;
    //     options.heightFieldData = matrix;
    // }

    public initShape(
        shape: PhysicsShape,
        type: PhysicsShapeType,
        options: PhysicsShapeParameters
    ): void {
        switch (type) {
            case PhysicsShapeType.SPHERE: {
                const radius = options.radius || 1;
                const center: Vector3Havok = options.center
                    ? this._threeVectorToHavokArray(options.center)
                    : [0, 0, 0];
                shape._pluginData = this._hknp.HP_Shape_CreateSphere(center, radius)[1];
                break;
            }
            case PhysicsShapeType.BOX: {
                const rotation: QuaternionHavok = options.rotation
                    ? this._threeQuaternionToHavokArray(options.rotation)
                    : [0, 0, 0, 1];
                const extent: Vector3Havok = options.extents
                    ? this._threeVectorToHavokArray(options.extents)
                    : [1, 1, 1];
                const center: Vector3Havok = options.center
                    ? this._threeVectorToHavokArray(options.center)
                    : [0, 0, 0];
                shape._pluginData = this._hknp.HP_Shape_CreateBox(
                    center,
                    rotation,
                    extent
                )[1];
                break;
            }
            case PhysicsShapeType.CAPSULE: {
                const pointA: Vector3Havok = options.pointA
                    ? this._threeVectorToHavokArray(options.pointA)
                    : [0, 0, 0];
                const pointB: Vector3Havok = options.pointB
                    ? this._threeVectorToHavokArray(options.pointB)
                    : [0, 1, 0];
                const radius = options.radius || 0;
                shape._pluginData = this._hknp.HP_Shape_CreateCapsule(
                    pointA,
                    pointB,
                    radius
                )[1];
                break;
            }
            case PhysicsShapeType.CONTAINER: {
                shape._pluginData = this._hknp.HP_Shape_CreateContainer()[1];
                break;
            }
            case PhysicsShapeType.CYLINDER: {
                const pointA: Vector3Havok = options.pointA
                    ? this._threeVectorToHavokArray(options.pointA)
                    : [0, 0, 0];
                const pointB: Vector3Havok = options.pointB
                    ? this._threeVectorToHavokArray(options.pointB)
                    : [0, 1, 0];
                const radius = options.radius || 0;
                shape._pluginData = this._hknp.HP_Shape_CreateCylinder(
                    pointA,
                    pointB,
                    radius
                )[1];
                break;
            }
            case PhysicsShapeType.CONVEX_HULL:
            case PhysicsShapeType.MESH: {
                const mesh = options.mesh;
                if (mesh) {
                    const includeChildMeshes = !!options.includeChildMeshes;
                    const needIndices = type != PhysicsShapeType.CONVEX_HULL;
                    const accum = new MeshAccumulator(needIndices);
                    accum.addNodeMeshes(mesh, includeChildMeshes);
                    const positions = accum.getVertices(this._hknp);
                    const numVec3s = positions.numObjects / 3;
                    if (type == PhysicsShapeType.CONVEX_HULL) {
                        shape._pluginData = this._hknp.HP_Shape_CreateConvexHull(
                            positions.offset,
                            numVec3s
                        )[1];
                    } else {
                        const triangles = accum.getTriangles(this._hknp);
                        const numTriangles = triangles.numObjects / 3;
                        shape._pluginData = this._hknp.HP_Shape_CreateMesh(
                            positions.offset,
                            numVec3s,
                            triangles.offset,
                            numTriangles
                        )[1];
                        accum.freeBuffer(this._hknp, triangles);
                    }
                    accum.freeBuffer(this._hknp, positions);
                } else {
                    throw new Error("No mesh provided to create physics shape.");
                }
                break;
            }
            case PhysicsShapeType.HEIGHTFIELD: {
                // if (options.groundMesh) {
                //     this._createOptionsFromGroundMesh(options);
                // }
                if (
                    options.numHeightFieldSamplesX &&
                    options.numHeightFieldSamplesZ &&
                    options.heightFieldSizeX &&
                    options.heightFieldSizeZ &&
                    options.heightFieldData
                ) {
                    const totalNumHeights =
                        options.numHeightFieldSamplesX * options.numHeightFieldSamplesZ;
                    const numBytes = totalNumHeights * 4;
                    const bufferBegin = this._hknp._malloc(numBytes);
                    const heightBuffer = new Float32Array(
                        this._hknp.HEAPU8.buffer,
                        bufferBegin,
                        totalNumHeights
                    );
                    for (let x = 0; x < options.numHeightFieldSamplesX; x++) {
                        for (let z = 0; z < options.numHeightFieldSamplesZ; z++) {
                            const hkBufferIndex = z * options.numHeightFieldSamplesX + x;
                            const bjsBufferIndex =
                                (options.numHeightFieldSamplesX - 1 - x) *
                                options.numHeightFieldSamplesZ +
                                z;
                            heightBuffer[hkBufferIndex] =
                                options.heightFieldData[bjsBufferIndex];
                        }
                    }
                    const scaleX =
                        options.heightFieldSizeX / (options.numHeightFieldSamplesX - 1);
                    const scaleZ =
                        options.heightFieldSizeZ / (options.numHeightFieldSamplesZ - 1);
                    shape._pluginData = this._hknp.HP_Shape_CreateHeightField(
                        options.numHeightFieldSamplesX,
                        options.numHeightFieldSamplesZ,
                        [scaleX, 1, scaleZ],
                        bufferBegin
                    )[1];
                    this._hknp._free(bufferBegin);
                } else {
                    throw new Error("Missing required heightfield parameters");
                }
                break;
            }
            default: {
                throw new Error("Unsupported Shape Type.");
            }
        }
        this._shapes.set(shape._pluginData[0], shape);
    }

    /**
     * Sets the shape filter membership mask of a body
     * @param shape - The physics body to set the shape filter membership mask for.
     * @param membershipMask - The shape filter membership mask to set.
     */
    setShapeFilterMembershipMask(shape: PhysicsShape, membershipMask: number) {
        const collideWith = this._hknp.HP_Shape_GetFilterInfo(
            shape._pluginData
        )[1][1];
        this._hknp.HP_Shape_SetFilterInfo(shape._pluginData, [
            membershipMask,
            collideWith,
        ]);
    }
    /**
     * Gets the shape filter membership mask of a body
     * @param shape - The physics body to get the shape filter membership mask from.
     * @returns The shape filter membership mask of the given body.
     */
    getShapeFilterMembershipMask(shape: PhysicsShape) {
        return this._hknp.HP_Shape_GetFilterInfo(shape._pluginData)[1][0];
    }
    /**
     * Sets the shape filter collide mask of a body
     * @param shape - The physics body to set the shape filter collide mask for.
     * @param collideMask - The shape filter collide mask to set.
     */
    setShapeFilterCollideMask(shape: PhysicsShape, collideMask: number) {
        const membership = this._hknp.HP_Shape_GetFilterInfo(
            shape._pluginData
        )[1][0];
        this._hknp.HP_Shape_SetFilterInfo(shape._pluginData, [
            membership,
            collideMask,
        ]);
    }
    /**
     * Gets the shape filter collide mask of a body
     * @param shape - The physics body to get the shape filter collide mask from.
     * @returns The shape filter collide mask of the given body.
     */
    getShapeFilterCollideMask(shape: PhysicsShape) {
        return this._hknp.HP_Shape_GetFilterInfo(shape._pluginData)[1][1];
    }
    /**
     * Sets the material of a physics shape.
     * @param shape - The physics shape to set the material of.
     * @param material - The material to set.
     *
     */
    setMaterial(shape: PhysicsShape, material: PhysicsMaterial) {
        const dynamicFriction = material.friction ?? 0.5;
        const staticFriction = material.staticFriction ?? dynamicFriction;
        const restitution = material.restitution ?? 0;
        const frictionCombine =
            material.frictionCombine ?? 1; /* PhysicsMaterialCombineMode.MINIMUM */
        const restitutionCombine =
            material.restitutionCombine ?? 2; /* PhysicsMaterialCombineMode.MAXIMUM */
        const hpMaterial: PhysicsMaterialHavok = [
            staticFriction,
            dynamicFriction,
            restitution,
            this._materialCombineToNative(frictionCombine),
            this._materialCombineToNative(restitutionCombine),
        ];
        this._hknp.HP_Shape_SetMaterial(shape._pluginData, hpMaterial);
    }
    /**
     * Gets the material associated with a physics shape.
     * @param shape - The shape to get the material from.
     * @returns The material associated with the shape.
     */
    getMaterial(shape: PhysicsShape) {
        const hkMaterial = this._hknp.HP_Shape_GetMaterial(shape._pluginData)[1];
        return {
            staticFriction: hkMaterial[0],
            friction: hkMaterial[1],
            restitution: hkMaterial[2],
            frictionCombine: this._nativeToMaterialCombine(hkMaterial[3]),
            restitutionCombine: this._nativeToMaterialCombine(hkMaterial[4]),
        };
    }
    /**
     * Sets the density of a physics shape.
     * @param shape - The physics shape to set the density of.
     * @param density - The density to set.
     *
     */
    setDensity(shape: PhysicsShape, density: number) {
        this._hknp.HP_Shape_SetDensity(shape._pluginData, density);
    }
    /**
     * Calculates the density of a given physics shape.
     *
     * @param shape - The physics shape to calculate the density of.
     * @returns The density of the given physics shape.
     *
     */
    getDensity(shape: PhysicsShape) {
        return this._hknp.HP_Shape_GetDensity(shape._pluginData)[1];
    }
    /**
     * Releases a physics shape from the physics engine.
     *
     * @param shape - The physics shape to be released.
     *
     * This method is useful for releasing a physics shape from the physics engine, freeing up resources and preventing memory leaks.
     */
    public disposeShape(shape: PhysicsShape): void {
        this._shapes.delete(shape._pluginData[0]);
        this._hknp.HP_Shape_Release(shape._pluginData);
        shape._pluginData = undefined;
    }

    public getBodyGeometry(body: PhysicsBody): {
        positions: Float32Array | number[];
        indices: Uint32Array | number[];
    } {
        const dataInfo = body._pluginData;
        const shape = this._hknp.HP_Body_GetShape(dataInfo.hpBodyId)[1];
        const geometryRes = this._hknp.HP_Shape_CreateDebugDisplayGeometry(shape);
        if (geometryRes[0] != this._hknp.Result.RESULT_OK) {
            return { positions: [], indices: [] };
        }
        const geometryInfo = this._hknp.HP_DebugGeometry_GetInfo(geometryRes[1])[1];
        const positionsInPlugin = new Float32Array(
            this._hknp.HEAPU8.buffer,
            geometryInfo[0],
            geometryInfo[1] * 3
        );
        const indicesInPlugin = new Uint32Array(
            this._hknp.HEAPU8.buffer,
            geometryInfo[2],
            geometryInfo[3] * 3
        );
        const positions = [...positionsInPlugin];
        const indices = [...indicesInPlugin];
        this._hknp.HP_DebugGeometry_Release(geometryRes[1]);
        return { positions, indices };
    }

    /**
     * Adds a child shape to the given shape.
     * @param shape - The parent shape.
     * @param newChild - The child shape to add.
     * @param translation - The relative translation of the child from the parent shape
     * @param rotation - The relative rotation of the child from the parent shape
     * @param scale - The relative scale scale of the child from the parent shaep
     *
     */
    addChild(
        shape: PhysicsShape,
        newChild: PhysicsShape,
        translation: Vector3,
        rotation: Quaternion,
        scale: Vector3
    ) {
        const transformNative: QSTransform = [
            [translation.x, translation.y, translation.z],
            [rotation.x, rotation.y, rotation.z, rotation.w],
            [scale.x, scale.y, scale.z],
        ];
        this._hknp.HP_Shape_AddChild(
            shape._pluginData,
            newChild._pluginData,
            transformNative
        );
    }
    /**
     * Removes a child shape from a parent shape.
     * @param shape - The parent shape.
     * @param childIndex - The index of the child shape to remove.
     *
     */
    removeChild(shape: PhysicsShape, childIndex: number) {
        this._hknp.HP_Shape_RemoveChild(shape._pluginData, childIndex);
    }
    /**
     * Returns the number of children of the given shape.
     *
     * @param shape - The shape to get the number of children from.
     * @returns The number of children of the given shape.
     *
     */
    getNumChildren(shape: PhysicsShape) {
        return this._hknp.HP_Shape_GetNumChildren(shape._pluginData)[1];
    }
    /**
     * Marks the shape as a trigger
     * @param shape the shape to mark as a trigger
     * @param isTrigger if the shape is a trigger
     */
    setTrigger(shape: PhysicsShape, isTrigger: boolean) {
        this._hknp.HP_Shape_SetTrigger(shape._pluginData, isTrigger);
    }
    public getBoundingBox(shape: PhysicsShape): Box3 {
        const aabb = this._hknp.HP_Shape_GetBoundingBox(shape._pluginData, [
            [0, 0, 0],
            [0, 0, 0, 1],
        ])[1];
        const min = new Vector3(aabb[0][0], aabb[0][1], aabb[0][2]);
        const max = new Vector3(aabb[1][0], aabb[1][1], aabb[1][2]);
        return new Box3(min, max);
    }

    public getBodyBoundingBox(body: PhysicsBody): Box3 {
        if (!body.shape) {
            return new Box3();
        }
        const aabb = this.getBoundingBox(body.shape);
        body.node.updateWorldMatrix(true, false);
        return aabb.clone().applyMatrix4(body.node.matrixWorld);
    }

    // constraint
    /**
     * Initializes a physics constraint with the given parameters.
     *
     * @param constraint - The physics constraint to be initialized.
     * @param body - The main body
     * @param childBody - The child body.
     * @param instanceIndex - If this body is instanced, the index of the instance to which the constraint will be applied. If not specified, no constraint will be applied.
     * @param childInstanceIndex - If the child body is instanced, the index of the instance to which the constraint will be applied. If not specified, no constraint will be applied.
     *
     * This function is useful for setting up a physics constraint in a physics engine.
     */
    initConstraint(
        constraint: PhysicsConstraint,
        body: PhysicsBody,
        childBody: PhysicsBody,
        instanceIndex?: number,
        childInstanceIndex?: number
    ) {
        const type = constraint.type;
        const options = constraint.options;
        if (!type || !options) {
            console.warn("No constraint type or options. Constraint is invalid.");
            return;
        }
        if (
            (body._pluginDataInstances.length > 0 && instanceIndex === undefined) ||
            (childBody._pluginDataInstances.length > 0 &&
                childInstanceIndex === undefined)
        ) {
            console.warn(
                "Body is instanced but no instance index was specified. Constraint will not be applied."
            );
            return;
        }
        constraint._pluginData = constraint._pluginData ?? [];
        const jointId = this._hknp.HP_Constraint_Create()[1];
        constraint._pluginData.push(jointId);
        // body parenting
        const bodyA = this._getPluginReference(body, instanceIndex).hpBodyId;
        const bodyB = this._getPluginReference(
            childBody,
            childInstanceIndex
        ).hpBodyId;
        this._hknp.HP_Constraint_SetParentBody(jointId, bodyA);
        this._hknp.HP_Constraint_SetChildBody(jointId, bodyB);
        this._constraintToBodyIdPair.set(jointId[0], [bodyA[0], bodyB[0]]);
        // anchors
        const pivotA: Vector3Havok = options.pivotA
            ? [options.pivotA.x, options.pivotA.y, options.pivotA.z]
            : [0, 0, 0];
        const axisA = options.axisA ?? VECTOR_RIGHT;
        const perpAxisA = tempVecs[0];
        if (options.perpAxisA) {
            perpAxisA.copy(options.perpAxisA);
        } else {
            getVectorNormalToRef(axisA, perpAxisA);
        }
        this._hknp.HP_Constraint_SetAnchorInParent(
            jointId,
            pivotA,
            [axisA.x, axisA.y, axisA.z],
            [perpAxisA.x, perpAxisA.y, perpAxisA.z]
        );
        const pivotB: Vector3Havok = options.pivotB
            ? [options.pivotB.x, options.pivotB.y, options.pivotB.z]
            : [0, 0, 0];
        const axisB = options.axisB ?? VECTOR_RIGHT;
        const perpAxisB = tempVecs[0];
        if (options.perpAxisB) {
            perpAxisB.copy(options.perpAxisB);
        } else {
            getVectorNormalToRef(axisB, perpAxisB);
        }
        this._hknp.HP_Constraint_SetAnchorInChild(
            jointId,
            pivotB,
            [axisB.x, axisB.y, axisB.z],
            [perpAxisB.x, perpAxisB.y, perpAxisB.z]
        );
        // Save the options that were used for initializing the constraint for debugging purposes
        // Check first to avoid copying the same options multiple times
        if (!constraint._initOptions) {
            constraint._initOptions = {
                axisA: axisA.clone(),
                axisB: axisB.clone(),
                perpAxisA: perpAxisA.clone(),
                perpAxisB: perpAxisB.clone(),
                pivotA: new Vector3(pivotA[0], pivotA[1], pivotA[2]),
                pivotB: new Vector3(pivotB[0], pivotB[1], pivotB[2]),
            };
        }
        if (type == 5 /* PhysicsConstraintType.LOCK */) {
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_X,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_X,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
        } else if (type == 2 /* PhysicsConstraintType.DISTANCE */) {
            const distance = options.maxDistance || 0;
            const dist3d = this._hknp.ConstraintAxis.LINEAR_DISTANCE;
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                dist3d,
                this._hknp.ConstraintAxisLimitMode.LIMITED
            );
            this._hknp.HP_Constraint_SetAxisMinLimit(jointId, dist3d, distance);
            this._hknp.HP_Constraint_SetAxisMaxLimit(jointId, dist3d, distance);
        } else if (type == 3 /* PhysicsConstraintType.HINGE */) {
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_X,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
        } else if (type == 6 /* PhysicsConstraintType.PRISMATIC */) {
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_X,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
        } else if (type == 4 /* PhysicsConstraintType.SLIDER */) {
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.ANGULAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
        } else if (type == 1 /* PhysicsConstraintType.BALL_AND_SOCKET */) {
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_X,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Y,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._hknp.ConstraintAxis.LINEAR_Z,
                this._hknp.ConstraintAxisLimitMode.LOCKED
            );
        } else if (type == 7 /* PhysicsConstraintType.SIX_DOF */) {
            const sixdofData = constraint as Physics6DoFConstraint;
            for (const l of sixdofData.limits) {
                const axId = this._constraintAxisToNative(l.axis);
                if ((l.minLimit ?? -1) == 0 && (l.maxLimit ?? -1) == 0) {
                    this._hknp.HP_Constraint_SetAxisMode(
                        jointId,
                        axId,
                        this._hknp.ConstraintAxisLimitMode.LOCKED
                    );
                } else {
                    if (l.minLimit != undefined) {
                        this._hknp.HP_Constraint_SetAxisMode(
                            jointId,
                            axId,
                            this._hknp.ConstraintAxisLimitMode.LIMITED
                        );
                        this._hknp.HP_Constraint_SetAxisMinLimit(jointId, axId, l.minLimit);
                    }
                    if (l.maxLimit != undefined) {
                        this._hknp.HP_Constraint_SetAxisMode(
                            jointId,
                            axId,
                            this._hknp.ConstraintAxisLimitMode.LIMITED
                        );
                        this._hknp.HP_Constraint_SetAxisMaxLimit(jointId, axId, l.maxLimit);
                    }
                }
                if (l.stiffness) {
                    this._hknp.HP_Constraint_SetAxisStiffness(jointId, axId, l.stiffness);
                }
                if (l.damping) {
                    this._hknp.HP_Constraint_SetAxisDamping(jointId, axId, l.damping);
                }
            }
        } else {
            throw new Error("Unsupported Constraint Type.");
        }
        const collisionEnabled = +!!options.collision;
        this._hknp.HP_Constraint_SetCollisionsEnabled(jointId, collisionEnabled);
        this._hknp.HP_Constraint_SetEnabled(jointId, 1);
    }
    /**
     * Get a list of all the pairs of bodies that are connected by this constraint.
     * @param constraint the constraint to search from
     * @returns a list of parent, child pairs
     */
    getBodiesUsingConstraint(constraint: PhysicsConstraint) {
        const pairs = [];
        for (const jointId of constraint._pluginData) {
            const bodyIds = this._constraintToBodyIdPair.get(jointId[0]);
            if (bodyIds) {
                const parentBodyInfo = this._bodies.get(bodyIds[0]);
                const childBodyInfo = this._bodies.get(bodyIds[1]);
                if (parentBodyInfo && childBodyInfo) {
                    pairs.push({
                        parentBody: parentBodyInfo.body,
                        parentBodyIndex: parentBodyInfo.index,
                        childBody: childBodyInfo.body,
                        childBodyIndex: childBodyInfo.index,
                    });
                }
            }
        }
        return pairs;
    }
    /**
     * Adds a constraint to the physics engine.
     *
     * @param body - The main body to which the constraint is applied.
     * @param childBody - The body to which the constraint is applied.
     * @param constraint - The constraint to be applied.
     * @param instanceIndex - If this body is instanced, the index of the instance to which the constraint will be applied. If not specified, no constraint will be applied.
     * @param childInstanceIndex - If the child body is instanced, the index of the instance to which the constraint will be applied. If not specified, no constraint will be applied.
     */
    addConstraint(
        body: PhysicsBody,
        childBody: PhysicsBody,
        constraint: PhysicsConstraint,
        instanceIndex?: number,
        childInstanceIndex?: number
    ) {
        //<todo It's real weird that initConstraint() is called only after adding to a body!
        this.initConstraint(
            constraint,
            body,
            childBody,
            instanceIndex,
            childInstanceIndex
        );
    }
    /**
     * Enables or disables a constraint in the physics engine.
     * @param constraint - The constraint to enable or disable.
     * @param isEnabled - Whether the constraint should be enabled or disabled.
     *
     */
    setEnabled(constraint: PhysicsConstraint, isEnabled: boolean) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetEnabled(jointId, isEnabled ? 1 : 0);
        }
    }
    /**
     * Gets the enabled state of the given constraint.
     * @param constraint - The constraint to get the enabled state from.
     * @returns The enabled state of the given constraint.
     *
     */
    getEnabled(constraint: PhysicsConstraint): boolean {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            return this._hknp.HP_Constraint_GetEnabled(firstId)[1] === 0 ? false : true;
        }
        return false;
    }
    /**
     * Enables or disables collisions for the given constraint.
     * @param constraint - The constraint to enable or disable collisions for.
     * @param isEnabled - Whether collisions should be enabled or disabled.
     *
     */
    setCollisionsEnabled(constraint: PhysicsConstraint, isEnabled: boolean) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetCollisionsEnabled(jointId, isEnabled ? 1 : 0);
        }
    }
    /**
     * Gets whether collisions are enabled for the given constraint.
     * @param constraint - The constraint to get collisions enabled for.
     * @returns Whether collisions are enabled for the given constraint.
     *
     */
    getCollisionsEnabled(constraint: PhysicsConstraint): boolean {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            return this._hknp.HP_Constraint_GetCollisionsEnabled(firstId)[1] === 0 ? false : true;
        }
        return false;
    }
    /**
     * Sets the friction of the given axis of the given constraint.
     *
     * @param constraint - The constraint to set the friction of.
     * @param axis - The axis of the constraint to set the friction of.
     * @param friction - The friction to set.
     *
     */
    setAxisFriction(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis,
        friction: number
    ) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetAxisFriction(
                jointId,
                this._constraintAxisToNative(axis),
                friction
            );
        }
    }
    /**
     * Gets the friction value of the specified axis of the given constraint.
     *
     * @param constraint - The constraint to get the axis friction from.
     * @param axis - The axis to get the friction from.
     * @returns The friction value of the specified axis.
     *
     */
    getAxisFriction(constraint: PhysicsConstraint, axis: PhysicsConstraintAxis) {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            return this._hknp.HP_Constraint_GetAxisFriction(
                firstId,
                this._constraintAxisToNative(axis)
            )[1];
        }
        return null;
    }
    /**
     * Sets the limit mode of the specified axis of the given constraint.
     * @param constraint - The constraint to set the axis mode of.
     * @param axis - The axis to set the limit mode of.
     * @param limitMode - The limit mode to set.
     */
    setAxisMode(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis,
        limitMode: PhysicsConstraintAxisLimitMode
    ) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetAxisMode(
                jointId,
                this._constraintAxisToNative(axis),
                this._limitModeToNative(limitMode)
            );
        }
    }
    /**
     * Gets the axis limit mode of the given constraint.
     *
     * @param constraint - The constraint to get the axis limit mode from.
     * @param axis - The axis to get the limit mode from.
     * @returns The axis limit mode of the given constraint.
     *
     */
    getAxisMode(constraint: PhysicsConstraint, axis: PhysicsConstraintAxis) {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            const mode = this._hknp.HP_Constraint_GetAxisMode(
                firstId,
                this._constraintAxisToNative(axis)
            )[1];
            return this._nativeToLimitMode(mode);
        }
        return null;
    }
    /**
     * Sets the minimum limit of the given axis of the given constraint.
     * @param constraint - The constraint to set the minimum limit of.
     * @param axis - The axis to set the minimum limit of.
     * @param limit - The minimum limit to set.
     *
     */
    setAxisMinLimit(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis,
        limit: number
    ) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetAxisMinLimit(
                jointId,
                this._constraintAxisToNative(axis),
                limit
            );
        }
    }
    /**
     * Gets the minimum limit of the specified axis of the given constraint.
     * @param constraint - The constraint to get the minimum limit from.
     * @param axis - The axis to get the minimum limit from.
     * @returns The minimum limit of the specified axis of the given constraint.
     *
     */
    getAxisMinLimit(constraint: PhysicsConstraint, axis: PhysicsConstraintAxis) {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            return this._hknp.HP_Constraint_GetAxisMinLimit(
                firstId,
                this._constraintAxisToNative(axis)
            )[1];
        }
        return null;
    }
    /**
     * Sets the maximum limit of the given axis of the given constraint.
     * @param constraint - The constraint to set the maximum limit of the given axis.
     * @param axis - The axis to set the maximum limit of.
     * @param limit - The maximum limit to set.
     *
     */
    setAxisMaxLimit(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis,
        limit: number
    ) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetAxisMaxLimit(
                jointId,
                this._constraintAxisToNative(axis),
                limit
            );
        }
    }
    /**
     * Gets the maximum limit of the given axis of the given constraint.
     *
     * @param constraint - The constraint to get the maximum limit from.
     * @param axis - The axis to get the maximum limit from.
     * @returns The maximum limit of the given axis of the given constraint.
     *
     */
    getAxisMaxLimit(constraint: PhysicsConstraint, axis: PhysicsConstraintAxis) {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            return this._hknp.HP_Constraint_GetAxisMaxLimit(
                firstId,
                this._constraintAxisToNative(axis)
            )[1];
        }
        return null;
    }
    /**
     * Sets the motor type of the given axis of the given constraint.
     * @param constraint - The constraint to set the motor type of.
     * @param axis - The axis of the constraint to set the motor type of.
     * @param motorType - The motor type to set.
     *
     */
    setAxisMotorType(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis,
        motorType: PhysicsConstraintMotorType
    ) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetAxisMotorType(
                jointId,
                this._constraintAxisToNative(axis),
                this._constraintMotorTypeToNative(motorType)
            );
        }
    }
    /**
     * Gets the motor type of the specified axis of the given constraint.
     * @param constraint - The constraint to get the motor type from.
     * @param axis - The axis of the constraint to get the motor type from.
     * @returns The motor type of the specified axis of the given constraint.
     *
     */
    getAxisMotorType(constraint: PhysicsConstraint, axis: PhysicsConstraintAxis) {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            return this._nativeToMotorType(
                this._hknp.HP_Constraint_GetAxisMotorType(
                    firstId,
                    this._constraintAxisToNative(axis)
                )[1]
            );
        }
        return null;
    }
    /**
     * Sets the target of an axis motor of a constraint.
     *
     * @param constraint - The constraint to set the axis motor target of.
     * @param axis - The axis of the constraint to set the motor target of.
     * @param target - The target of the axis motor.
     *
     */
    setAxisMotorTarget(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis,
        target: number
    ) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetAxisMotorTarget(
                jointId,
                this._constraintAxisToNative(axis),
                target
            );
        }
    }
    /**
     * Gets the target of the motor of the given axis of the given constraint.
     *
     * @param constraint - The constraint to get the motor target from.
     * @param axis - The axis of the constraint to get the motor target from.
     * @returns The target of the motor of the given axis of the given constraint.
     *
     */
    getAxisMotorTarget(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis
    ) {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            return this._hknp.HP_Constraint_GetAxisMotorTarget(
                constraint._pluginData as HP_ConstraintId,
                this._constraintAxisToNative(axis)
            )[1];
        }
        return null;
    }
    /**
     * Sets the maximum force that can be applied by the motor of the given constraint axis.
     * @param constraint - The constraint to set the motor max force for.
     * @param axis - The axis of the constraint to set the motor max force for.
     * @param maxForce - The maximum force that can be applied by the motor.
     *
     */
    setAxisMotorMaxForce(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis,
        maxForce: number
    ) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetAxisMotorMaxForce(
                jointId,
                this._constraintAxisToNative(axis),
                maxForce
            );
        }
    }
    /**
     * Gets the maximum force of the motor of the given constraint axis.
     *
     * @param constraint - The constraint to get the motor maximum force from.
     * @param axis - The axis of the constraint to get the motor maximum force from.
     * @returns The maximum force of the motor of the given constraint axis.
     *
     */
    getAxisMotorMaxForce(
        constraint: PhysicsConstraint,
        axis: PhysicsConstraintAxis
    ) {
        const firstId = constraint._pluginData && constraint._pluginData[0];
        if (firstId) {
            return this._hknp.HP_Constraint_GetAxisMotorMaxForce(
                firstId,
                this._constraintAxisToNative(axis)
            )[1];
        }
        return null;
    }
    /**
     * Disposes a physics constraint.
     *
     * @param constraint - The physics constraint to dispose.
     *
     * This method is useful for releasing the resources associated with a physics constraint, such as
     * the Havok constraint, when it is no longer needed. This is important for avoiding memory leaks.
     */
    disposeConstraint(constraint: PhysicsConstraint) {
        for (const jointId of constraint._pluginData) {
            this._hknp.HP_Constraint_SetEnabled(jointId, 0);
            this._hknp.HP_Constraint_Release(jointId);
        }
        constraint._pluginData.length = 0;
    }
    _populateHitData(hitData: any, result: any) {
        const hitBody = this._bodies.get(hitData[0][0]);
        result.body = hitBody?.body;
        result.bodyIndex = hitBody?.index;
        const hitShape = this._shapes.get(hitData[1][0]);
        result.shape = hitShape;
        const hitPos = hitData[3];
        const hitNormal = hitData[4];
        const hitTriangle = hitData[5];
        result.setHitData(
            { x: hitNormal[0], y: hitNormal[1], z: hitNormal[2] },
            { x: hitPos[0], y: hitPos[1], z: hitPos[2] },
            hitTriangle
        );
    }
    /**
     * Performs a raycast from a given start point to a given end point and stores the result in a given PhysicsRaycastResult object.
     *
     * @param from - The start point of the raycast.
     * @param to - The end point of the raycast.
     * @param result - The PhysicsRaycastResult object to store the result of the raycast.
     * @param query - The raycast query options. See [[IRaycastQuery]] for more information.
     *
     * Performs a raycast. It takes in two points, from and to, and a PhysicsRaycastResult object to store the result of the raycast.
     * It then performs the raycast and stores the hit data in the PhysicsRaycastResult object.
     */
    raycast(
        from: Vector3,
        to: Vector3,
        result: PhysicsRaycastResult,
        query?: IRaycastQuery
    ) {
        const queryMembership = query?.membership ?? ~0;
        const queryCollideWith = query?.collideWith ?? ~0;
        const shouldHitTriggers = query?.shouldHitTriggers ?? false;
        result.reset(from, to);
        const bodyToIgnore = [BigInt(0)];
        const hkQuery: RayCastInput = [
            from.toArray(),
            to.toArray(),
            [queryMembership, queryCollideWith],
            shouldHitTriggers,
            bodyToIgnore as HP_BodyId,
        ];
        this._hknp.HP_World_CastRayWithCollector(
            this.world,
            this._queryCollector,
            hkQuery
        );
        if (this._hknp.HP_QueryCollector_GetNumHits(this._queryCollector)[1] > 0) {
            const [, hitData] = this._hknp.HP_QueryCollector_GetCastRayResult(
                this._queryCollector,
                0
            )[1];
            this._populateHitData(hitData, result);
            result.calculateHitDistance();
        }
    }
    /**
     * Given a point, returns the closest physics
     * body to that point.
     * @param query the query to perform. @see IPhysicsPointProximityQuery
     * @param result contact point on the hit shape, in world space
     */
    pointProximity(
        query: IPhysicsPointProximityQuery,
        result: ProximityCastResult
    ) {
        const queryMembership = query?.collisionFilter?.membership ?? ~0;
        const queryCollideWith = query?.collisionFilter?.collideWith ?? ~0;
        result.reset();
        const bodyToIgnore = query.ignoreBody
            ? [BigInt(query.ignoreBody._pluginData.hpBodyId[0])]
            : [BigInt(0)];
        const hkQuery: PointProximityInput = [
            query.position.toArray(),
            query.maxDistance,
            [queryMembership, queryCollideWith],
            query.shouldHitTriggers,
            bodyToIgnore as HP_BodyId,
        ];
        this._hknp.HP_World_PointProximityWithCollector(
            this.world,
            this._queryCollector,
            hkQuery
        );
        if (this._hknp.HP_QueryCollector_GetNumHits(this._queryCollector)[1] > 0) {
            const [distance, hitData] =
                this._hknp.HP_QueryCollector_GetPointProximityResult(
                    this._queryCollector,
                    0
                )[1];
            this._populateHitData(hitData, result);
            result.setHitDistance(distance);
        }
    }
    /**
     * Given a shape in a specific position and orientation, returns the closest point to that shape.
     * @param query the query to perform. @see IPhysicsShapeProximityCastQuery
     * @param inputShapeResult contact point on input shape, in input shape space
     * @param hitShapeResult contact point on hit shape, in world space
     */
    shapeProximity(
        query: IPhysicsShapeProximityCastQuery,
        inputShapeResult: ProximityCastResult,
        hitShapeResult: ProximityCastResult
    ) {
        inputShapeResult.reset();
        hitShapeResult.reset();
        const shapeId = query.shape._pluginData;
        const bodyToIgnore = query.ignoreBody
            ? [BigInt(query.ignoreBody._pluginData.hpBodyId[0])]
            : [BigInt(0)];
        const hkQuery: ShapeProximityInput = [
            shapeId,
            query.position.toArray(),
            query.rotation.toArray(),
            query.maxDistance,
            query.shouldHitTriggers,
            bodyToIgnore as HP_BodyId,
        ];
        this._hknp.HP_World_ShapeProximityWithCollector(
            this.world,
            this._queryCollector,
            hkQuery
        );
        if (this._hknp.HP_QueryCollector_GetNumHits(this._queryCollector)[1] > 0) {
            const [distance, hitInputData, hitShapeData] =
                this._hknp.HP_QueryCollector_GetShapeProximityResult(
                    this._queryCollector,
                    0
                )[1];
            this._populateHitData(hitInputData, inputShapeResult);
            this._populateHitData(hitShapeData, hitShapeResult);
            inputShapeResult.setHitDistance(distance);
            hitShapeResult.setHitDistance(distance);
        }
    }
    /**
     * Given a shape in a specific orientation, cast it from the start to end position specified by the query, and return the first hit.
     * @param query the query to perform. @see IPhysicsShapeCastQuery
     * @param inputShapeResult contact point on input shape, in input shape space
     * @param hitShapeResult contact point on hit shape, in world space
     */
    shapeCast(
        query: IPhysicsShapeCastQuery,
        inputShapeResult: ShapeCastResult,
        hitShapeResult: ShapeCastResult
    ) {
        inputShapeResult.reset();
        hitShapeResult.reset();
        const shapeId = query.shape._pluginData;
        const bodyToIgnore = query.ignoreBody
            ? [BigInt(query.ignoreBody._pluginData.hpBodyId[0])]
            : [BigInt(0)];
        const hkQuery: ShapeCastInput = [
            shapeId,
            [query.rotation.x, query.rotation.y, query.rotation.z, query.rotation.w],
            [query.startPosition.x, query.startPosition.y, query.startPosition.z],
            [query.endPosition.x, query.endPosition.y, query.endPosition.z],
            query.shouldHitTriggers,
            bodyToIgnore as HP_BodyId,
        ];
        this._hknp.HP_World_ShapeCastWithCollector(
            this.world,
            this._queryCollector,
            hkQuery
        );
        if (this._hknp.HP_QueryCollector_GetNumHits(this._queryCollector)[1] > 0) {
            const [fractionAlongRay, hitInputData, hitShapeData] =
                this._hknp.HP_QueryCollector_GetShapeCastResult(
                    this._queryCollector,
                    0
                )[1];
            this._populateHitData(hitInputData, inputShapeResult);
            this._populateHitData(hitShapeData, hitShapeResult);
            inputShapeResult.setHitFraction(fractionAlongRay);
            hitShapeResult.setHitFraction(fractionAlongRay);
        }
    }
    /**
     * Return the collision observable for a particular physics body.
     * @param body the physics body
     * @returns the collision observable for the body
     */
    getCollisionObservable(body: PhysicsBody) {
        const bodyId = body._pluginData.hpBodyId[0];
        let observable = this._bodyCollisionObservable.get(bodyId);
        if (!observable) {
            observable = new Observable();
            this._bodyCollisionObservable.set(bodyId, observable);
        }
        return observable;
    }
    /**
     * Return the collision ended observable for a particular physics body.
     * @param body the physics body
     * @returns
     */
    getCollisionEndedObservable(body: PhysicsBody) {
        const bodyId = body._pluginData.hpBodyId[0];
        let observable = this._bodyCollisionEndedObservable.get(bodyId);
        if (!observable) {
            observable = new Observable();
            this._bodyCollisionEndedObservable.set(bodyId, observable);
        }
        return observable;
    }
    /**
     * Enable collision to be reported for a body when a callback is setup on the world
     * @param body the physics body
     * @param enabled whether to enable or disable collision events
     */
    setCollisionCallbackEnabled(body: PhysicsBody, enabled: boolean) {
        // Register for collide events by default
        const collideEvents =
            // @ts-expect-error ignore this
            this._hknp.EventType.COLLISION_STARTED.value |
            // @ts-expect-error ignore this
            this._hknp.EventType.COLLISION_CONTINUED.value |
            // @ts-expect-error ignore this
            this._hknp.EventType.COLLISION_FINISHED.value;
        if (body._pluginDataInstances && body._pluginDataInstances.length > 0) {
            for (let index = 0; index < body._pluginDataInstances.length; index++) {
                const bodyId = body._pluginDataInstances[index];
                this._hknp.HP_Body_SetEventMask(
                    bodyId.hpBodyId,
                    enabled ? collideEvents : 0
                );
            }
        } else if (body._pluginData) {
            this._hknp.HP_Body_SetEventMask(
                body._pluginData.hpBodyId,
                enabled ? collideEvents : 0
            );
        }
    }

    /**
     * Enable collision ended to be reported for a body when a callback is setup on the world
     * @param body the physics body
     * @param enabled whether to enable or disable collision ended events
     */
    setCollisionEndedCallbackEnabled(body: PhysicsBody, enabled: boolean) {
        // Register to collide ended events
        const pluginRef = this._getPluginReference(body);
        let currentCollideEvents = this._hknp.HP_Body_GetEventMask(pluginRef.hpBodyId)[1];
        // update with the ended mask
        currentCollideEvents = enabled
            // @ts-expect-error ignore this
            ? currentCollideEvents | this._hknp.EventType.COLLISION_FINISHED.value
            // @ts-expect-error ignore this
            : currentCollideEvents & ~this._hknp.EventType.COLLISION_FINISHED.value;
        if (body._pluginDataInstances && body._pluginDataInstances.length > 0) {
            for (let index = 0; index < body._pluginDataInstances.length; index++) {
                const bodyId = body._pluginDataInstances[index];
                this._hknp.HP_Body_SetEventMask(bodyId.hpBodyId, currentCollideEvents);
            }
        }
        else if (body._pluginData) {
            this._hknp.HP_Body_SetEventMask(body._pluginData.hpBodyId, currentCollideEvents);
        }
    }

    /**
     * Runs thru all detected collisions and filter by body
     */
    _notifyCollisions() {
        let eventAddress = this._hknp.HP_World_GetCollisionEvents(this.world)[1];
        const event = new CollisionEvent();
        const worldAddr = Number(this.world);
        while (eventAddress) {
            CollisionEvent.readToRef(this._hknp.HEAPU8.buffer as ArrayBuffer, eventAddress, event);
            const bodyInfoA = this._bodies.get(event.contactOnA.bodyId as HP_BodyId[0]);
            const bodyInfoB = this._bodies.get(event.contactOnB.bodyId as HP_BodyId[0]);
            // Bodies may have been disposed between events. Check both still exist.
            if (bodyInfoA && bodyInfoB) {
                // @ts-expect-error ignore this
                const collisionInfo: IPhysicsCollisionEvent = {
                    collider: bodyInfoA.body,
                    colliderIndex: bodyInfoA.index,
                    collidedAgainst: bodyInfoB.body,
                    collidedAgainstIndex: bodyInfoB.index,
                    type: this._nativeCollisionValueToCollisionType(event.type),
                };
                if (collisionInfo.type === "COLLISION_FINISHED" /* PhysicsEventType.COLLISION_FINISHED */) {
                    this.onCollisionEndedObservable.notifyObservers(collisionInfo);
                } else {
                    tempVecs[0].subVectors(event.contactOnB.position, event.contactOnA.position);
                    const distance = tempVecs[0].dot(event.contactOnA.normal);
                    collisionInfo.point = event.contactOnA.position;
                    collisionInfo.distance = distance;
                    collisionInfo.impulse = event.impulseApplied;
                    collisionInfo.normal = event.contactOnA.normal;
                    this.onCollisionObservable.notifyObservers(collisionInfo);
                    this.onCollisionEndedObservable.notifyObservers(collisionInfo);
                }
                if (this._bodyCollisionObservable.size > 0 && collisionInfo.type !== "COLLISION_FINISHED" /* PhysicsEventType.COLLISION_FINISHED */) {
                    const observableA = this._bodyCollisionObservable.get(event.contactOnA.bodyId);
                    const observableB = this._bodyCollisionObservable.get(event.contactOnB.bodyId);
                    tempVecs[0].subVectors(event.contactOnA.position, event.contactOnB.position);
                    const distance = tempVecs[0].dot(event.contactOnB.normal);
                    if (observableA) {
                        observableA.notifyObservers(collisionInfo);
                    }
                    if (observableB) {
                        const collisionInfoB = {
                            collider: bodyInfoB.body,
                            colliderIndex: bodyInfoB.index,
                            collidedAgainst: bodyInfoA.body,
                            collidedAgainstIndex: bodyInfoA.index,
                            point: event.contactOnB.position,
                            distance: distance,
                            impulse: event.impulseApplied,
                            normal: event.contactOnB.normal,
                            type: this._nativeCollisionValueToCollisionType(event.type),
                        };
                        observableB.notifyObservers(collisionInfoB);
                    }
                }
                else if (this._bodyCollisionEndedObservable.size > 0) {
                    const observableA = this._bodyCollisionEndedObservable.get(event.contactOnA.bodyId);
                    const observableB = this._bodyCollisionEndedObservable.get(event.contactOnB.bodyId);
                    tempVecs[0].subVectors(event.contactOnA.position, event.contactOnB.position);
                    const distance = tempVecs[0].dot(event.contactOnB.normal);
                    if (observableA) {
                        observableA.notifyObservers(collisionInfo);
                    }
                    if (observableB) {
                        const collisionInfoB = {
                            collider: bodyInfoB.body,
                            colliderIndex: bodyInfoB.index,
                            collidedAgainst: bodyInfoA.body,
                            collidedAgainstIndex: bodyInfoA.index,
                            point: event.contactOnB.position,
                            distance: distance,
                            impulse: event.impulseApplied,
                            normal: event.contactOnB.normal,
                            type: this._nativeCollisionValueToCollisionType(event.type),
                        };
                        observableB.notifyObservers(collisionInfoB);
                    }
                }
            }
            eventAddress = this._hknp.HP_World_GetNextCollisionEvent(worldAddr, eventAddress);
        }
    }

    private _notifyTriggers(): void {
        let eventAddress = this._hknp.HP_World_GetTriggerEvents(this.world)[1];
        const event = new TriggerEvent();
        while (eventAddress) {
            TriggerEvent.readToRef(
                this._hknp.HEAPU8.buffer as ArrayBuffer,
                eventAddress,
                event
            );
            const bodyInfoA = this._bodies.get(event.bodyIdA as HP_BodyId[0]);
            const bodyInfoB = this._bodies.get(event.bodyIdB as HP_BodyId[0]);
            if (bodyInfoA && bodyInfoB) {
                const triggerCollisionInfo: IBasePhysicsCollisionEvent = {
                    collider: bodyInfoA.body,
                    colliderIndex: bodyInfoA.index,
                    collidedAgainst: bodyInfoB.body,
                    collidedAgainstIndex: bodyInfoB.index,
                    type: this._nativeTriggerCollisionValueToCollisionType(event.type),
                };
                this.onTriggerCollisionObservable.notifyObservers(triggerCollisionInfo);
            }
            eventAddress = this._hknp.HP_World_GetNextTriggerEvent(
                this.world,
                eventAddress
            );
        }
    }

    get numBodies(): number {
        return this._hknp.HP_World_GetNumBodies(this.world)[1];
    }

    public dispose(): void {
        this._bodyCollisionObservable.clear();
        this._bodyCollisionEndedObservable.clear();
        this.onCollisionObservable.clear();
        this.onCollisionEndedObservable.clear();
        this.onTriggerCollisionObservable.clear();
        this._bodies.clear();
        this._shapes.clear();

        if (this._queryCollector) {
            this._hknp.HP_QueryCollector_Release(this._queryCollector);
            this._queryCollector = undefined!;
        }
        if (this.world) {
            this._hknp.HP_World_Release(this.world);
            this.world = undefined!;
        }
    }

    // --- HELPER FUNCTIONS ---

    // private getPerpendicular(v: Vector3, result: Vector3): void {
    //     if (Math.abs(v.x) > Math.abs(v.y)) {
    //         const invLen = 1 / Math.hypot(v.x, v.z);
    //         result.set(-v.z * invLen, 0, v.x * invLen);
    //     } else {
    //         const invLen = 1 / Math.hypot(v.y, v.z);
    //         result.set(0, v.z * invLen, -v.y * invLen);
    //     }
    // }

    private _getTransformInfos(node: Object3D): QTransform {
        node.updateWorldMatrix(true, false);
        worldPos.set(0, 0, 0);
        worldQuat.set(0, 0, 0, 1);
        node.matrixWorld.decompose(worldPos, worldQuat, tempVecs[1]); // scale is ignored

        return [
            this._threeVectorToHavokArray(worldPos),
            this._threeQuaternionToHavokArray(worldQuat),
        ];
    }

    private _havokArrayToThreeVectorRef(v: number[], vec3: Vector3): void {
        vec3.set(v[0], v[1], v[2]);
    }

    private _threeVectorToHavokArray(v: Vector3): Vector3Havok {
        return [v.x, v.y, v.z];
    }

    private _threeQuaternionToHavokArray(q: Quaternion): QuaternionHavok {
        return [q.x, q.y, q.z, q.w];
    }

    private _constraintMotorTypeToNative(motorType: PhysicsConstraintMotorType) {
        switch (motorType) {
            case PhysicsConstraintMotorType.POSITION: {
                return this._hknp.ConstraintMotorType.POSITION;
            }
            case PhysicsConstraintMotorType.VELOCITY: {
                return this._hknp.ConstraintMotorType.VELOCITY;
            }
        }
        return this._hknp.ConstraintMotorType.NONE;
    }

    private _nativeToMotorType(motorType: any): PhysicsConstraintMotorType {
        switch (motorType) {
            case this._hknp.ConstraintMotorType.POSITION: {
                return PhysicsConstraintMotorType.POSITION;
            }
            case this._hknp.ConstraintMotorType.VELOCITY: {
                return PhysicsConstraintMotorType.VELOCITY;
            }
        }
        return PhysicsConstraintMotorType.NONE;
    }

    private _materialCombineToNative(mat: PhysicsMaterialCombineMode) {
        switch (mat) {
            case PhysicsMaterialCombineMode.GEOMETRIC_MEAN: {
                return this._hknp.MaterialCombine.GEOMETRIC_MEAN;
            }
            case PhysicsMaterialCombineMode.MINIMUM: {
                return this._hknp.MaterialCombine.MINIMUM;
            }
            case PhysicsMaterialCombineMode.MAXIMUM: {
                return this._hknp.MaterialCombine.MAXIMUM;
            }
            case PhysicsMaterialCombineMode.ARITHMETIC_MEAN: {
                return this._hknp.MaterialCombine.ARITHMETIC_MEAN;
            }
            case PhysicsMaterialCombineMode.MULTIPLY: {
                return this._hknp.MaterialCombine.MULTIPLY;
            }
        }
    }

    private _nativeToMaterialCombine(mat: any): PhysicsMaterialCombineMode {
        switch (mat) {
            case this._hknp.MaterialCombine.GEOMETRIC_MEAN: {
                return PhysicsMaterialCombineMode.GEOMETRIC_MEAN;
            }
            case this._hknp.MaterialCombine.MINIMUM: {
                return PhysicsMaterialCombineMode.MINIMUM;
            }
            case this._hknp.MaterialCombine.MAXIMUM: {
                return PhysicsMaterialCombineMode.MAXIMUM;
            }
            case this._hknp.MaterialCombine.ARITHMETIC_MEAN: {
                return PhysicsMaterialCombineMode.ARITHMETIC_MEAN;
            }
            case this._hknp.MaterialCombine.MULTIPLY: {
                return PhysicsMaterialCombineMode.MULTIPLY;
            }
            default: {
                return PhysicsMaterialCombineMode.MINIMUM;
            }
        }
    }

    private _constraintAxisToNative(axId: PhysicsConstraintAxis) {
        switch (axId) {
            case PhysicsConstraintAxis.LINEAR_X: {
                return this._hknp.ConstraintAxis.LINEAR_X;
            }
            case PhysicsConstraintAxis.LINEAR_Y: {
                return this._hknp.ConstraintAxis.LINEAR_Y;
            }
            case PhysicsConstraintAxis.LINEAR_Z: {
                return this._hknp.ConstraintAxis.LINEAR_Z;
            }
            case PhysicsConstraintAxis.ANGULAR_X: {
                return this._hknp.ConstraintAxis.ANGULAR_X;
            }
            case PhysicsConstraintAxis.ANGULAR_Y: {
                return this._hknp.ConstraintAxis.ANGULAR_Y;
            }
            case PhysicsConstraintAxis.ANGULAR_Z: {
                return this._hknp.ConstraintAxis.ANGULAR_Z;
            }
            case PhysicsConstraintAxis.LINEAR_DISTANCE: {
                return this._hknp.ConstraintAxis.LINEAR_DISTANCE;
            }
        }
    }

    private _nativeToLimitMode(mode: any): PhysicsConstraintAxisLimitMode {
        switch (mode) {
            case this._hknp.ConstraintAxisLimitMode.FREE: {
                return PhysicsConstraintAxisLimitMode.FREE;
            }
            case this._hknp.ConstraintAxisLimitMode.LIMITED: {
                return PhysicsConstraintAxisLimitMode.LIMITED;
            }
            case this._hknp.ConstraintAxisLimitMode.LOCKED: {
                return PhysicsConstraintAxisLimitMode.LOCKED;
            }
        }
        return PhysicsConstraintAxisLimitMode.FREE;
    }

    private _limitModeToNative(mode: PhysicsConstraintAxisLimitMode) {
        switch (mode) {
            case PhysicsConstraintAxisLimitMode.FREE: {
                return this._hknp.ConstraintAxisLimitMode.FREE;
            }
            case PhysicsConstraintAxisLimitMode.LIMITED: {
                return this._hknp.ConstraintAxisLimitMode.LIMITED;
            }
            case PhysicsConstraintAxisLimitMode.LOCKED: {
                return this._hknp.ConstraintAxisLimitMode.LOCKED;
            }
        }
    }

    private _nativeCollisionValueToCollisionType(type: number): PhysicsEventType {
        switch (type) {
            // @ts-expect-error ignore this
            case this._hknp.EventType.COLLISION_STARTED.value: {
                return "COLLISION_STARTED";
            }
            // @ts-expect-error ignore this
            case this._hknp.EventType.COLLISION_FINISHED.value: {
                return "COLLISION_FINISHED";
            }
            // @ts-expect-error ignore this
            case this._hknp.EventType.COLLISION_CONTINUED.value: {
                return "COLLISION_CONTINUED";
            }
        }
        return "COLLISION_STARTED";
    }

    private _nativeTriggerCollisionValueToCollisionType(
        type: number
    ): PhysicsEventType {
        switch (type) {
            case 8: {
                // TRIGGER_ENTERED
                return "TRIGGER_ENTERED";
            }
            case 16: {
                // TRIGGER_EXITED
                return "TRIGGER_EXITED";
            }
        }
        return "TRIGGER_ENTERED";
    }
}
