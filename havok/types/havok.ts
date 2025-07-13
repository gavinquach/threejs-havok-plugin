/* eslint-disable @typescript-eslint/no-explicit-any */

import type { PhysicsBody } from "@/utils/three/havok/physicsBody";
import type { Group, Mesh, Object3D, Quaternion, Vector3 } from "three";

/**
 * Determines how values from the PhysicsMaterial are combined when
 * two objects are in contact. When each PhysicsMaterial specifies
 * a different combine mode for some property, the combine mode which
 * is used will be selected based on their order in this enum - i.e.
 * a value later in this list will be preferentially used.
 */
export enum PhysicsMaterialCombineMode {
    /**
     * The final value will be the geometric mean of the two values:
     * sqrt( valueA *  valueB )
     */
    GEOMETRIC_MEAN = 0,
    /**
     * The final value will be the smaller of the two:
     * min( valueA , valueB )
     */
    MINIMUM = 1,
    MAXIMUM = 2,
    ARITHMETIC_MEAN = 3,
    /**
     * The final value will be the product of the two values:
     * valueA * valueB
     */
    MULTIPLY = 4
}

/**
 * Physics material class
 * Helps setting friction and restitution that are used to compute responding forces in collision response
 */
export interface PhysicsMaterial {
    /**
     * Sets the friction used by this material
     *
     * The friction determines how much an object will slow down when it is in contact with another object.
     * This is important for simulating realistic physics, such as when an object slides across a surface.
     *
     * If not provided, a default value of 0.5 will be used.
     */
    friction?: number;
    /**
     * Sets the static friction used by this material.
     *
     * Static friction is the friction that must be overcome before a pair of objects can start sliding
     * relative to each other; for physically-realistic behaviour, it should be at least as high as the
     * normal friction value. If not provided, the friction value will be used
     */
    staticFriction?: number;
    /**
     * Sets the restitution of the physics material.
     *
     * The restitution is a factor which describes, the amount of energy that is retained after a collision,
     * which should be a number between 0 and 1..
     *
     * A restitution of 0 means that no energy is retained and the objects will not bounce off each other,
     * while a restitution of 1 means that all energy is retained and the objects will bounce.
     *
     * Note, though, due that due to the simulation implementation, an object with a restitution of 1 may
     * still lose energy over time.
     *
     * If not provided, a default value of 0 will be used.
     */
    restitution?: number;
    /**
     * Describes how two different friction values should be combined. See PhysicsMaterialCombineMode for
     * more details.
     *
     * If not provided, will use PhysicsMaterialCombineMode.MINIMUM
     */
    frictionCombine?: PhysicsMaterialCombineMode;
    /**
     * Describes how two different restitution values should be combined. See PhysicsMaterialCombineMode for
     * more details.
     *
     * If not provided, will use PhysicsMaterialCombineMode.MAXIMUM
     */
    restitutionCombine?: PhysicsMaterialCombineMode;
}

export interface PhysicShapeOptions {
    /**
     * The type of the shape. This can be one of the following: SPHERE, BOX, CAPSULE, CYLINDER, CONVEX_HULL, MESH, HEIGHTFIELD, CONTAINER
     */
    type?: PhysicsShapeType;
    /**
     * The parameters of the shape. Varies depending of the shape type.
     */
    parameters?: PhysicsShapeParameters;
    /**
     * Reference to an already existing physics shape in the plugin.
     */
    pluginData?: any;
}

export interface PhysicsShapeParameters {
    /**
     * Shape center position
     */
    center?: Vector3;
    /**
     * Radius for cylinder, shape and capsule
     */
    radius?: number;
    /**
     * First point position that defines the cylinder or capsule
     */
    pointA?: Vector3;
    /**
     * Second point position that defines the cylinder or capsule
     */
    pointB?: Vector3;
    /**
     * Shape orientation
     */
    rotation?: Quaternion;
    /**
     * Dimesion extention for the box
     */
    extents?: Vector3;
    /**
     * Mesh used for Mesh shape or convex hull. It can be different than the mesh the body is attached to.
     */
    mesh?: Object3D | Group | Mesh;
    /**
     * Use children hierarchy
     */
    includeChildMeshes?: boolean;
    /**
     * The size of the heightfield in the X axis
     */
    heightFieldSizeX?: number;
    /**
     * The size of the heightfield in the Z axis
     */
    heightFieldSizeZ?: number;
    /**
     * The number of samples along the X axis
     */
    numHeightFieldSamplesX?: number;
    /**
     * The number of samples along the Z axis
     */
    numHeightFieldSamplesZ?: number;
    /**
     * The data for the heightfield
     */
    heightFieldData?: Float32Array;
}


export enum PhysicsShapeType {
    SPHERE,
    CAPSULE,
    CYLINDER,
    BOX,
    CONVEX_HULL,
    CONTAINER,
    MESH,
    HEIGHTFIELD,
}

export enum PhysicsActivationControl {
    SIMULATION_CONTROLLED,
    ALWAYS_ACTIVE,
    ALWAYS_INACTIVE,
}

export enum PhysicsConstraintType {
    BALL_AND_SOCKET = 1,
    DISTANCE = 2,
    HINGE = 3,
    SLIDER = 4,
    LOCK = 5,
    PRISMATIC = 6,
    SIX_DOF = 7,
}

export enum PhysicsConstraintAxis {
    LINEAR_X,
    LINEAR_Y,
    LINEAR_Z,
    ANGULAR_X,
    ANGULAR_Y,
    ANGULAR_Z,
    LINEAR_DISTANCE,
}

export enum PhysicsConstraintAxisLimitMode {
    FREE,
    LIMITED,
    LOCKED,
}

export enum PhysicsConstraintMotorType {
    NONE,
    VELOCITY,
    POSITION,
}

export type PhysicsEventType =
    | "COLLISION_STARTED"
    | "COLLISION_CONTINUED"
    | "COLLISION_FINISHED"
    | "TRIGGER_ENTERED"
    | "TRIGGER_EXITED";

export interface PhysicsMassProperties {
    centerOfMass?: Vector3;
    mass?: number;
    inertia?: Vector3;
    inertiaOrientation?: Quaternion;
}

export interface IBasePhysicsCollisionEvent {
    collider: PhysicsBody;
    collidedAgainst: PhysicsBody;
    colliderIndex?: number;
    collidedAgainstIndex?: number;
    type: PhysicsEventType;
}

export interface IPhysicsCollisionEvent extends IBasePhysicsCollisionEvent {
    point: Vector3 | undefined;
    distance: number;
    impulse: number;
    normal: Vector3 | undefined;
}

/**
 * Represents a pair of bodies connected by a constraint.
 */
export type ConstrainedBodyPair = {
    parentBody: PhysicsBody;
    parentBodyIndex: number;
    childBody: PhysicsBody;
    childBodyIndex: number;
};

/**
 * Parameters used to describe a Constraint
 */
export interface PhysicsConstraintParameters {
    /**
     * Location of the constraint pivot in the space of first body
     */
    pivotA?: Vector3;
    /**
     * Location of the constraint pivot in the space of the second body
     */
    pivotB?: Vector3;
    /**
     * An axis in the space of the first body which determines how
     * distances/angles are measured for LINEAR_X/ANGULAR_X limits.
     */
    axisA?: Vector3;
    /**
     * An axis in the space of the second body which determines how
     * distances/angles are measured for LINEAR_X/ANGULAR_X limits.
     */
    axisB?: Vector3;
    /**
     * An axis in the space of the first body which determines how
     * distances/angles are measured for LINEAR_Y/ANGULAR_Y limits.
     */
    perpAxisA?: Vector3;
    /**
     * An axis in the space of the second body which determines how
     * distances/angles are measured for LINEAR_Y/ANGULAR_Y limits.
     */
    perpAxisB?: Vector3;
    /**
     * The maximum distance that can seperate the two pivots.
     * Only used for DISTANCE constraints
     */
    maxDistance?: number;
    /**
     * Determines if the connected bodies should collide. Generally,
     * it is preferable to set this to false, especially if the constraint
     * positions the bodies so that they overlap. Otherwise, the constraint
     * will "fight" the collision detection and may cause jitter.
     */
    collision?: boolean;
}

/**
 * Indicates how to handle position/rotation change of transform node attached to a physics body
 */
export enum PhysicsPrestepType {
    DISABLED = 0,
    TELEPORT = 1,
    ACTION = 2
}
export enum PhysicsMotionType {
    STATIC = 0,
    ANIMATED = 1,
    DYNAMIC = 2
}