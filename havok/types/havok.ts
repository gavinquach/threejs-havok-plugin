/* eslint-disable @typescript-eslint/no-explicit-any */

import type { HavokPlugin } from "@/utils/three/havok/havokPlugin";
import type { PhysicsBody } from "@/utils/three/havok/physicsBody";
import type { Group, Mesh, Object3D, Quaternion, Scene, Vector3 } from "three";

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
 * This is a holder class for the physics constraint created by the physics plugin
 * It holds a set of functions to control the underlying constraint
 * @see https://doc.babylonjs.com/features/featuresDeepDive/physics/usingPhysicsEngine
 */
export declare class PhysicsConstraint {
    /**
     * V2 Physics plugin private data for a physics material
     */
    _pluginData: any;
    /**
     * The V2 plugin used to create and manage this Physics Body
     */
    protected _physicsPlugin: HavokPlugin;
    protected _options: PhysicsConstraintParameters;
    protected _type: PhysicsConstraintType;
    /**
     * @internal
     * The internal options that were used to init the constraint
     */
    _initOptions?: PhysicsConstraintParameters;
    /**
     * Constructs a new constraint for the physics constraint.
     * @param type The type of constraint to create.
     * @param options The options for the constraint.
     * @param scene The scene the constraint belongs to.
     *
     * This code is useful for creating a new constraint for the physics engine. It checks if the scene has a physics engine, and if the plugin version is correct.
     * If all checks pass, it initializes the constraint with the given type and options.
     */
    constructor(type: PhysicsConstraintType, options: PhysicsConstraintParameters, scene: Scene);
    /**
     * Gets the type of the constraint.
     *
     * @returns The type of the constraint.
     *
     */
    get type(): PhysicsConstraintType;
    /**
     * Retrieves the options of the physics constraint.
     *
     * @returns The physics constraint parameters.
     *
     */
    get options(): PhysicsConstraintParameters;
    /**
     * Enable/disable the constraint
     * @param isEnabled value for the constraint
     */
    set isEnabled(isEnabled: boolean);
    /**
     *
     * @returns true if constraint is enabled
     */
    get isEnabled(): boolean;
    /**
     * Enables or disables collisions for the physics engine.
     *
     * @param isEnabled - A boolean value indicating whether collisions should be enabled or disabled.
     *
     */
    set isCollisionsEnabled(isEnabled: boolean);
    /**
     * Gets whether collisions are enabled for this physics object.
     *
     * @returns `true` if collisions are enabled, `false` otherwise.
     *
     */
    get isCollisionsEnabled(): boolean;
    /**
     * Gets all bodies that are using this constraint
     * @returns
     */
    getBodiesUsingConstraint(): ConstrainedBodyPair[];
    /**
     * Disposes the constraint from the physics engine.
     *
     * This method is useful for cleaning up the physics engine when a body is no longer needed. Disposing the body will free up resources and prevent memory leaks.
     */
    dispose(): void;
}
export declare class Physics6DoFLimit {
    /**
     * The axis ID to limit
     */
    axis: PhysicsConstraintAxis;
    /**
     * An optional minimum limit for the axis.
     * Corresponds to a distance in meters for linear axes, an angle in radians for angular axes.
     */
    minLimit?: number;
    /**
     * An optional maximum limit for the axis.
     * Corresponds to a distance in meters for linear axes, an angle in radians for angular axes.
     */
    maxLimit?: number;
    /**
     * The stiffness of the constraint.
     */
    stiffness?: number;
    /**
     * A constraint parameter that specifies damping.
     */
    damping?: number;
}

// export interface PhysicsConstraint {
//     type: PhysicsConstraintType;
//     _pluginData: any[];
//     options: any;
//     _initOptions?: any;
// }

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
 * A generic constraint, which can be used to build more complex constraints than those specified
 * in PhysicsConstraintType. The axis and pivot options in PhysicsConstraintParameters define the space
 * the constraint operates in. This constraint contains a set of limits, which restrict the
 * relative movement of the bodies in that coordinate system
 */
export declare class Physics6DoFConstraint extends PhysicsConstraint {
    /**
     * The collection of limits which this constraint will apply
     */
    limits: Physics6DoFLimit[];
    constructor(constraintParams: PhysicsConstraintParameters, limits: Physics6DoFLimit[], scene: Scene);
    /**
     * Sets the friction of the given axis of the physics engine.
     * @param axis - The axis of the physics engine to set the friction for.
     * @param friction - The friction to set for the given axis.
     *
     */
    setAxisFriction(axis: PhysicsConstraintAxis, friction: number): void;
    /**
     * Gets the friction of the given axis of the physics engine.
     * @param axis - The axis of the physics engine.
     * @returns The friction of the given axis, or null if the constraint hasn't been initialized yet.
     *
     */
    getAxisFriction(axis: PhysicsConstraintAxis): number | null;
    /**
     * Sets the limit mode for the given axis of the constraint.
     * @param axis The axis to set the limit mode for.
     * @param limitMode The limit mode to set.
     *
     * This method is useful for setting the limit mode for a given axis of the constraint. This is important for
     * controlling the behavior of the physics engine when the constraint is reached. By setting the limit mode,
     * the engine can be configured to either stop the motion of the objects, or to allow them to continue
     * moving beyond the constraint.
     */
    setAxisMode(axis: PhysicsConstraintAxis, limitMode: PhysicsConstraintAxisLimitMode): void;
    /**
     * Gets the limit mode of the given axis of the constraint.
     *
     * @param axis - The axis of the constraint.
     * @returns The limit mode of the given axis, or null if the constraint hasn't been initialized yet.
     *
     */
    getAxisMode(axis: PhysicsConstraintAxis): PhysicsConstraintAxisLimitMode | null;
    /**
     * Sets the minimum limit of a given axis of a constraint.
     * @param axis - The axis of the constraint.
     * @param minLimit - The minimum limit of the axis.
     *
     */
    setAxisMinLimit(axis: PhysicsConstraintAxis, minLimit: number): void;
    /**
     * Gets the minimum limit of the given axis of the physics engine.
     * @param axis - The axis of the physics engine.
     * @returns The minimum limit of the given axis, or null if the constraint hasn't been initialized yet.
     *
     */
    getAxisMinLimit(axis: PhysicsConstraintAxis): number | null;
    /**
     * Sets the maximum limit of the given axis for the physics engine.
     * @param axis - The axis to set the limit for.
     * @param limit - The maximum limit of the axis.
     *
     * This method is useful for setting the maximum limit of the given axis for the physics engine,
     * which can be used to control the movement of the physics object. This helps to ensure that the
     * physics object does not move beyond the given limit.
     */
    setAxisMaxLimit(axis: PhysicsConstraintAxis, limit: number): void;
    /**
     * Gets the maximum limit of the given axis of the physics engine.
     * @param axis - The axis of the physics engine.
     * @returns The maximum limit of the given axis, or null if the constraint hasn't been initialized yet.
     *
     */
    getAxisMaxLimit(axis: PhysicsConstraintAxis): number | null;
    /**
     * Sets the motor type of the given axis of the constraint.
     * @param axis - The axis of the constraint.
     * @param motorType - The type of motor to use.
     */
    setAxisMotorType(axis: PhysicsConstraintAxis, motorType: PhysicsConstraintMotorType): void;
    /**
     * Gets the motor type of the specified axis of the constraint.
     *
     * @param axis - The axis of the constraint.
     * @returns The motor type of the specified axis, or null if the constraint hasn't been initialized yet.
     *
     */
    getAxisMotorType(axis: PhysicsConstraintAxis): PhysicsConstraintMotorType | null;
    /**
     * Sets the target velocity of the motor associated with the given axis of the constraint.
     * @param axis - The axis of the constraint.
     * @param target - The target velocity of the motor.
     *
     * This method is useful for setting the target velocity of the motor associated with the given axis of the constraint.
     */
    setAxisMotorTarget(axis: PhysicsConstraintAxis, target: number): void;
    /**
     * Gets the target velocity of the motor associated to the given constraint axis.
     * @param axis - The constraint axis associated to the motor.
     * @returns The target velocity of the motor, or null if the constraint hasn't been initialized yet.
     *
     */
    getAxisMotorTarget(axis: PhysicsConstraintAxis): number | null;
    /**
     * Sets the maximum force of the motor of the given axis of the constraint.
     * @param axis - The axis of the constraint.
     * @param maxForce - The maximum force of the motor.
     *
     */
    setAxisMotorMaxForce(axis: PhysicsConstraintAxis, maxForce: number): void;
    /**
     * Gets the maximum force of the motor of the given axis of the constraint.
     * @param axis - The axis of the constraint.
     * @returns The maximum force of the motor, or null if the constraint hasn't been initialized yet.
     *
     */
    getAxisMotorMaxForce(axis: PhysicsConstraintAxis): number | null;
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