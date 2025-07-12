import { Vector3 } from "three/src/math/Vector3.js";

import type { PhysicsShape } from "@/utils/three/havok/physicsShape";
import type { PhysicsBody } from "@/utils/three/havok/physicsBody";

import type { Quaternion } from "three";

/**
 * Interface for point proximity query.
 */
export interface IPhysicsPointProximityQuery {
    /**
     * The position of the query
     */
    position: Vector3;
    /**
     * Maximum distance to check for collisions. Can be set to 0 to check for overlaps.
     */
    maxDistance: number;
    /**
     * Collision filter for the query.
     */
    collisionFilter: IRaycastQuery;
    /**
     * Should trigger collisions be considered in the query?
     */
    shouldHitTriggers: boolean;
    /**
     * Should the query ignore the body that is passed in?
     */
    ignoreBody?: PhysicsBody;
}

/**
 * Query for shape proximity.
 */
export interface IPhysicsShapeProximityCastQuery {
    /**
     * The shape to test proximity against
     */
    shape: PhysicsShape;
    /**
     * The position of shape
     */
    position: Vector3;
    /**
     * The rotation of shape
     */
    rotation: Quaternion;
    /**
     * Maximum distance to check for collisions. Can be set to 0 to check for overlaps.
     */
    maxDistance: number;
    /**
     * Should trigger collisions be considered in the query?
     */
    shouldHitTriggers: boolean;
    /**
     * Ignores the body passed if it is in the query
     */
    ignoreBody?: PhysicsBody;
}

/**
 * Shape cast query
 */
export interface IPhysicsShapeCastQuery {
    /**
     * The shape to query with
     */
    shape: PhysicsShape;
    /**
     * The rotation of the shape
     */
    rotation: Quaternion;
    /**
     * The start position of the query
     */
    startPosition: Vector3;
    /**
     * The end position of the query
     */
    endPosition: Vector3;
    /**
     * Should trigger collisions be considered in the query?
     */
    shouldHitTriggers: boolean;
    /**
     * Ignores the body passed if it is in the query
     */
    ignoreBody?: PhysicsBody;
}

/**
 * Interface for query parameters in the raycast function.
 * @see the "Collision Filtering" section in https://github.com/eoineoineoin/glTF/tree/MSFT_RigidBodies/extensions/2.0/Vendor/MSFT_collision_primitives
 */
export interface IRaycastQuery {
    /** Membership mask */
    membership?: number;
    /** CollideWith mask */
    collideWith?: number;
    /** Should trigger collisions be considered in the query? */
    shouldHitTriggers?: boolean;
}

/**
 * Base class for results of casts.
 */
export class CastingResult {
    private _hasHit;
    protected _hitNormal: Vector3;
    protected _hitPoint: Vector3;
    private _triangleIndex;
    /**
     * The Physics body that the query hit.
     */
    body?: PhysicsBody;
    /**
     * The body Index in case the Physics body is using instances
     */
    bodyIndex?: number;
    /**
     * The shape hit by the query.
     */
    shape?: PhysicsShape;

    constructor() {
        this._hasHit = false;
        this._hitNormal = new Vector3();
        this._hitPoint = new Vector3();
        this._triangleIndex = -1;
    }

    /**
     * Gets the hit point.
     */
    get hitPoint(): Vector3 {
        return this._hitPoint;
    }
    /**
     * Gets the hit normal.
     */
    get hitNormal(): Vector3 {
        return this._hitNormal;
    }
    /**
     * Gets if there was a hit
     */
    get hasHit(): boolean {
        return this._hasHit;
    }
    /*
     * The index of the original triangle which was hit. Will be -1 if contact point is not on a mesh shape
     */
    get triangleIndex(): number {
        return this._triangleIndex;
    }

    /**
     * Sets the hit data
     * @param hitNormal defines the normal in world space
     * @param hitPoint defines the point in world space
     * @param triangleIndex defines the index of the triangle in case of mesh shape
     */
    setHitData(hitNormal: IXYZ, hitPoint: IXYZ, triangleIndex?: number): void {
        this._hasHit = true;
        this._hitNormal.set(hitNormal.x, hitNormal.y, hitNormal.z);
        this._hitPoint.set(hitPoint.x, hitPoint.y, hitPoint.z);
        this._triangleIndex = triangleIndex ?? -1;
    }
    /**
     * Resets all the values to default
     */
    reset(): void {
        this._hasHit = false;
        this._hitNormal.set(0, 0, 0);
        this._hitPoint.set(0, 0, 0);
        this._triangleIndex = -1;
        this.body = undefined;
        this.bodyIndex = undefined;
        this.shape = undefined;
    }
}
/**
 * Interface for the size containing width and height
 */
interface IXYZ {
    /**
     * X
     */
    x: number;
    /**
     * Y
     */
    y: number;
    /**
     * Z
     */
    z: number;
}
export { };

/**
 * Holds the data for the raycast result
 * @see https://doc.babylonjs.com/features/featuresDeepDive/physics/usingPhysicsEngine
 */
export class PhysicsRaycastResult extends CastingResult {
    private _hitDistance;
    private _rayFromWorld;
    private _rayToWorld;

    constructor() {
        super();
        this._hitDistance = 0;
        this._rayFromWorld = new Vector3();
        this._rayToWorld = new Vector3();
    }
    /**
     * Gets the distance from the hit
     */
    get hitDistance() {
        return this._hitDistance;
    }
    /**
     * Gets the hit normal/direction in the world
     */
    get hitNormalWorld() {
        return this._hitNormal;
    }
    /**
     * Gets the hit point in the world
     */
    get hitPointWorld() {
        return this._hitPoint;
    }
    /**
     * Gets the ray "start point" of the ray in the world
     */
    get rayFromWorld() {
        return this._rayFromWorld;
    }
    /**
     * Gets the ray "end point" of the ray in the world
     */
    get rayToWorld() {
        return this._rayToWorld;
    }
    /**
     * Sets the distance from the start point to the hit point
     * @param distance defines the distance to set
     */
    setHitDistance(distance: number): void {
        this._hitDistance = distance;
    }
    /**
     * Calculates the distance manually
     */
    calculateHitDistance(): void {
        this._hitDistance = this._rayFromWorld.distanceTo(this._hitPoint);
    }
    /**
     * Resets all the values to default
     * @param from The from point on world space
     * @param to The to point on world space
     */
    reset(from: Vector3 = new Vector3(), to: Vector3 = new Vector3()): void {
        super.reset();
        this._rayFromWorld.copy(from);
        this._rayToWorld.copy(to);
        this._hitDistance = 0;
    }
}

/**
 * Class representing a contact point produced in a proximity cast
 */
export class ProximityCastResult extends CastingResult {
    protected _hitDistance: number;
    constructor() {
        super();
        this._hitDistance = 0;
    }
    /**
     * Gets the distance from the hit
     */
    get hitDistance(): number {
        return this._hitDistance;
    }
    /**
     * Sets the distance from the start point to the hit point
     * @param distance
     */
    setHitDistance(distance: number): void {
        this._hitDistance = distance;
    }
    /**
     * Resets all the values to default
     */
    reset(): void {
        super.reset();
        this._hitDistance = 0;
    }
}

/**
 * Class representing a contact point produced in a shape cast
 */
export class ShapeCastResult extends CastingResult {
    private _hitFraction;
    constructor() {
        super();
        this._hitFraction = 0;
    }
    /**
     * Gets the hit fraction along the casting ray
     */
    get hitFraction(): number {
        return this._hitFraction;
    }
    /**
     * Sets the hit fraction along the casting ray
     * @param fraction
     */
    setHitFraction(fraction: number): void {
        this._hitFraction = fraction;
    }
}
