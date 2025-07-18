/* eslint-disable @typescript-eslint/no-explicit-any */

import { Matrix4 } from "three/src/math/Matrix4.js";
import { Vector3 } from "three/src/math/Vector3.js";
import { Quaternion } from "three/src/math/Quaternion.js";

import type { HavokPlugin } from "./havokPlugin";

import {
    PhysicsShapeType,
    type PhysicShapeOptions,
    type PhysicsMaterial,
} from "@/utils/three/havok/types/havok";
import { QUATERNION_IDENTITY, VECTOR_ONE, VECTOR_ZERO } from "@/utils/three/constants";

import type { Object3D, Mesh } from "three";

const tempMats = Array.from({ length: 2 }, () => new Matrix4());
const tempVecs = Array.from({ length: 2 }, () => new Vector3());
const tempQuats = Array.from({ length: 1 }, () => new Quaternion());

/**
 * PhysicsShape class.
 * This class is useful for creating a physics shape that can be used in a physics engine.
 * A Physic Shape determine how collision are computed. It must be attached to a body.
 */
export class PhysicsShape {
    /**
     * V2 Physics plugin private data for single shape
     */
    _pluginData?: any;
    /**
     * The V2 plugin used to create and manage this Physics Body
     */
    private _physicsPlugin: HavokPlugin;
    private _type;
    private _material!: PhysicsMaterial;
    private _isTrigger;
    private _isDisposed;

    /**
     * Constructs a new physics shape.
     * @param options The options for the physics shape. These are:
     *  * type: The type of the shape. This can be one of the following: SPHERE, BOX, CAPSULE, CYLINDER, CONVEX_HULL, MESH, HEIGHTFIELD, CONTAINER
     *  * parameters: The parameters of the shape.
     *  * pluginData: The plugin data of the shape. This is used if you already have a reference to the object on the plugin side.
     * You need to specify either type or pluginData.
     * @param scene The scene the shape belongs to.
     *
     * This code is useful for creating a new physics shape with the given type, options, and scene.
     * It also checks that the physics engine and plugin version are correct.
     * If not, it throws an error. This ensures that the shape is created with the correct parameters and is compatible with the physics engine.
     */
    constructor(options: PhysicShapeOptions, physicsPlugin: HavokPlugin) {
        /**
         * V2 Physics plugin private data for single shape
         */
        this._isTrigger = false;
        this._isDisposed = false;
        this._physicsPlugin = physicsPlugin;
        if (options.pluginData !== undefined && options.pluginData !== null) {
            this._pluginData = options.pluginData;
            this._type = this._physicsPlugin.getShapeType(this);
        } else if (options.type !== undefined && options.type !== null) {
            this._type = options.type;
            const parameters = options.parameters ?? {};
            this._physicsPlugin.initShape(this, options.type, parameters);
        }
    }
    /**
     * Returns the string "PhysicsShape".
     * @returns "PhysicsShape"
     */
    getClassName() {
        return "PhysicsShape";
    }
    /**
     * Returns the type of the physics shape.
     * @returns The type of the physics shape.
     */
    get type() {
        return this._type;
    }
    /**
     * Set the membership mask of a shape. This is a bitfield of arbitrary
     * "categories" to which the shape is a member. This is used in combination
     * with the collide mask to determine if this shape should collide with
     * another.
     *
     * @param membershipMask Bitfield of categories of this shape.
     */
    set filterMembershipMask(membershipMask) {
        this._physicsPlugin.setShapeFilterMembershipMask(this, membershipMask);
    }
    /**
     * Get the membership mask of a shape.
     * @returns Bitmask of categories which this shape is a member of.
     */
    get filterMembershipMask() {
        return this._physicsPlugin.getShapeFilterMembershipMask(this);
    }
    /**
     * Sets the collide mask of a shape. This is a bitfield of arbitrary
     * "categories" to which this shape collides with. Given two shapes,
     * the engine will check if the collide mask and membership overlap:
     * shapeA.filterMembershipMask & shapeB.filterCollideMask
     *
     * If this value is zero (i.e. shapeB only collides with categories
     * which shapeA is _not_ a member of) then the shapes will not collide.
     *
     * Note, the engine will also perform the same test with shapeA and
     * shapeB swapped; the shapes will not collide if either shape has
     * a collideMask which prevents collision with the other shape.
     *
     * @param collideMask Bitmask of categories this shape should collide with
     */
    set filterCollideMask(collideMask) {
        this._physicsPlugin.setShapeFilterCollideMask(this, collideMask);
    }
    /**
     *
     * @returns Bitmask of categories that this shape should collide with
     */
    get filterCollideMask() {
        return this._physicsPlugin.getShapeFilterCollideMask(this);
    }
    /**
     *
     * @param material
     */
    set material(material) {
        this._physicsPlugin.setMaterial(this, material);
        this._material = material;
    }
    /**
     * Returns the material of the physics shape.
     * @returns The material of the physics shape.
     */
    get material() {
        if (!this._material) {
            this._material = this._physicsPlugin.getMaterial(this);
        }
        return this._material;
    }
    /**
     * Sets the density of the physics shape.
     * @param density The density of the physics shape.
     */
    set density(density) {
        this._physicsPlugin.setDensity(this, density);
    }
    /**
     * Returns the density of the physics shape.
     * @returns The density of the physics shape.
     */
    get density() {
        return this._physicsPlugin.getDensity(this);
    }
    set isTrigger(isTrigger) {
        if (this._isTrigger === isTrigger) {
            return;
        }
        this._isTrigger = isTrigger;
        this._physicsPlugin.setTrigger(this, isTrigger);
    }
    get isTrigger() {
        return this._isTrigger;
    }
    /**
     * Utility to add a child shape to this container,
     * automatically computing the relative transform between
     * the container shape and the child instance.
     *
     * @param parentTransform The transform node associated with this shape
     * @param newChild The new PhysicsShape to add
     * @param childTransform The transform node associated with the child shape
     */
    addChildFromParent(
        parentTransform: Object3D,
        newChild: PhysicsShape,
        childTransform: Object3D
    ) {
        childTransform.updateMatrixWorld(true);
        parentTransform.updateMatrixWorld(true);
        const childToWorld = childTransform.matrixWorld;
        const parentToWorld = tempMats[1].copy(parentTransform.matrixWorld).invert();
        const childToParent = tempMats[0];
        childToParent.multiplyMatrices(parentToWorld.invert(), childToWorld);
        const translation = tempVecs[0];
        const rotation = tempQuats[0];
        const scale = tempVecs[1];
        childToParent.decompose(translation, rotation, scale);
        this._physicsPlugin.addChild(this, newChild, translation, rotation, scale);
    }
    /**
     * Adds a child shape to a container with an optional transform
     * @param newChild The new PhysicsShape to add
     * @param translation Optional position of the child shape relative to this shape
     * @param rotation Optional rotation of the child shape relative to this shape
     * @param scale Optional scale of the child shape relative to this shape
     */
    addChild(
        newChild: PhysicsShape,
        translation: Vector3 = VECTOR_ZERO,
        rotation: Quaternion = QUATERNION_IDENTITY,
        scale: Vector3 = VECTOR_ONE
    ) {
        this._physicsPlugin.addChild(this, newChild, translation, rotation, scale);
    }
    /**
     * Removes a child shape from this shape.
     * @param childIndex The index of the child shape to remove
     */
    removeChild(childIndex: number) {
        this._physicsPlugin.removeChild(this, childIndex);
    }
    /**
     * Returns the number of children of a physics shape.
     * @returns The number of children of a physics shape.
     */
    getNumChildren() {
        return this._physicsPlugin.getNumChildren(this);
    }
    /**
     * Returns the bounding box of the physics shape.
     * @returns The bounding box of the physics shape.
     */
    getBoundingBox() {
        return this._physicsPlugin.getBoundingBox(this);
    }
    /**
     * Dispose the shape and release its associated resources.
     */
    dispose() {
        if (this._isDisposed) return;
        this._physicsPlugin.disposeShape(this);
        this._isDisposed = true;
    }
}
/**
 * Helper object to create a sphere shape
 */
export class PhysicsShapeSphere extends PhysicsShape {
    /**
     * Constructor for the Sphere Shape
     * @param center local center of the sphere
     * @param radius radius
     * @param havokPlugin Havok physics plugin
     */
    constructor(center: Vector3, radius: number, havokPlugin: HavokPlugin) {
        super(
            {
                type: 0 /* PhysicsShapeType.SPHERE */,
                parameters: { center: center, radius: radius },
            },
            havokPlugin
        );
    }
    // /**
    //  * Derive an approximate sphere from the mesh.
    //  * @param mesh node from which to derive the sphere shape
    //  * @returns PhysicsShapeSphere
    //  */
    // static FromMesh(mesh: Mesh) {
    //     if (!mesh.geometry) {
    //         throw new Error("Object does not have geometry");
    //     }
    //     const geometry = mesh.geometry;
    //     if (!geometry.boundingSphere) geometry.computeBoundingSphere();
    //     return new PhysicsShapeSphere(
    //         geometry.boundingSphere!.center,
    //         geometry.boundingSphere!.radius,
    //         this._physicsPlugin
    //     );
    // }
}
/**
 * Helper object to create a capsule shape
 */
export class PhysicsShapeCapsule extends PhysicsShape {
    /**
     *
     * @param pointA Starting point that defines the capsule segment
     * @param pointB ending point of that same segment
     * @param radius radius
     * @param havokPlugin Havok physics plugin
     */
    constructor(
        pointA: Vector3,
        pointB: Vector3,
        radius: number,
        havokPlugin: HavokPlugin
    ) {
        super(
            {
                type: 1 /* PhysicsShapeType.CAPSULE */,
                parameters: { pointA: pointA, pointB: pointB, radius: radius },
            },
            havokPlugin
        );
    }
    // /**
    //  * Derive an approximate capsule from the mesh. Note, this is
    //  * not the optimal bounding capsule.
    //  * @param mesh Node from which to derive a cylinder shape
    //  * @returns Physics Shape Capsule
    //  */
    // static FromMesh(mesh) {
    //     const boundsLocal = mesh.getBoundingInfo();
    //     const radius = boundsLocal.boundingBox.extendSize.x;
    //     const pointFromCenter = new Vector3(
    //         0,
    //         boundsLocal.boundingBox.extendSize.y - radius,
    //         0
    //     );
    //     const pointA = boundsLocal.boundingBox.center.add(pointFromCenter);
    //     const pointB = boundsLocal.boundingBox.center.subtract(pointFromCenter);
    //     return new PhysicsShapeCapsule(pointA, pointB, radius, mesh.getScene());
    // }
}
/**
 * Helper object to create a cylinder shape
 */
export class PhysicsShapeCylinder extends PhysicsShape {
    /**
     *
     * @param pointA Starting point that defines the cylinder segment
     * @param pointB ending point of that same segment
     * @param radius radius
     * @param havokPlugin Havok physics plugin
     */
    constructor(
        pointA: Vector3,
        pointB: Vector3,
        radius: number,
        havokPlugin: HavokPlugin
    ) {
        super(
            {
                type: 2 /* PhysicsShapeType.CYLINDER */,
                parameters: { pointA: pointA, pointB: pointB, radius: radius },
            },
            havokPlugin
        );
    }
    // /**
    //  * Derive an approximate cylinder from the mesh. Note, this is
    //  * not the optimal bounding cylinder.
    //  * @param mesh Node from which to derive a cylinder shape
    //  * @returns Physics Shape Cylinder
    //  */
    // static FromMesh(mesh) {
    //     const boundsLocal = mesh.getBoundingInfo();
    //     const radius = boundsLocal.boundingBox.extendSize.x;
    //     const pointFromCenter = new Vector3(
    //         0,
    //         boundsLocal.boundingBox.extendSize.y,
    //         0
    //     );
    //     const pointA = boundsLocal.boundingBox.center.add(pointFromCenter);
    //     const pointB = boundsLocal.boundingBox.center.subtract(pointFromCenter);
    //     return new PhysicsShapeCylinder(pointA, pointB, radius, mesh.getScene());
    // }
}
/**
 * Helper object to create a box shape
 */
export class PhysicsShapeBox extends PhysicsShape {
    /**
     *
     * @param center local center of the box
     * @param rotation local orientation
     * @param extents size of the box in each direction
     * @param havokPlugin Havok physics plugin
     */
    constructor(
        center: Vector3,
        rotation: Quaternion,
        extents: Vector3,
        havokPlugin: HavokPlugin
    ) {
        super(
            {
                type: 3 /* PhysicsShapeType.BOX */,
                parameters: { center: center, rotation: rotation, extents: extents },
            },
            havokPlugin
        );
    }
    // /**
    //  *
    //  * @param mesh
    //  * @returns PhysicsShapeBox
    //  */
    // static FromMesh(mesh) {
    //     const bounds = mesh.getBoundingInfo();
    //     const centerLocal = bounds.boundingBox.center;
    //     const extents = bounds.boundingBox.extendSize.scale(2.0); //<todo.eoin extendSize seems to really be half-extents?
    //     return new PhysicsShapeBox(
    //         centerLocal,
    //         Quaternion.Identity(),
    //         extents,
    //         mesh.getScene()
    //     );
    // }
}
/**
 * Helper object to create a convex hull shape
 */
export class PhysicsShapeConvexHull extends PhysicsShape {
    /**
     *
     * @param mesh the mesh to be used as topology infos for the convex hull
     * @param havokPlugin Havok physics plugin
     */
    constructor(mesh: Mesh, havokPlugin: HavokPlugin) {
        super(
            {
                type: PhysicsShapeType.CONVEX_HULL,
                parameters: { mesh: mesh },
            },
            havokPlugin
        );
    }
}
/**
 * Helper object to create a mesh shape
 */
export class PhysicsShapeMesh extends PhysicsShape {
    /**
     *
     * @param mesh the mesh topology that will be used to create the shape
     * @param havokPlugin Havok physics plugin
     */
    constructor(mesh: Mesh, havokPlugin: HavokPlugin) {
        super(
            { type: 6 /* PhysicsShapeType.MESH */, parameters: { mesh: mesh } },
            havokPlugin
        );
    }
}
/**
 * A shape container holds a variable number of shapes. Use AddChild to append to newly created parent container.
 */
export class PhysicsShapeContainer extends PhysicsShape {
    /**
     * Constructor of the Shape container
     * @param havokPlugin Havok physics plugin
     */
    constructor(havokPlugin: HavokPlugin) {
        super(
            { type: 5 /* PhysicsShapeType.CONTAINER */, parameters: {} },
            havokPlugin
        );
    }
}
