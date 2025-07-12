/* eslint-disable unicorn/no-null */
/* eslint-disable unicorn/prefer-ternary */
/* eslint-disable unicorn/numeric-separators-style */

import { EdgesGeometry, Group, InstancedMesh, Mesh, MeshBasicMaterial, Object3D, Vector3, type Scene, Matrix4, Euler, BoxGeometry, LineSegments, LineBasicMaterial, CylinderGeometry, BufferGeometry, Quaternion, Float32BufferAttribute } from "three";

import { PhysicsConstraintAxis, PhysicsConstraintAxisLimitMode, type PhysicsConstraint, type PhysicsMassProperties } from "@/models/havok";
import type { HavokPlugin } from "@/utils/three/havok/havokPlugin";
import type { PhysicsBody } from "@/utils/three/havok/physicsBody";


const tempMatrixes = Array.from({ length: 10 }, () => new Matrix4());
const tempVecs = Array.from({ length: 10 }, () => new Vector3());

/**
 * A custom AxesViewer that uses meshes for a more distinct look, similar to the Babylon.js version.
 */
class CustomAxesViewer extends Group {
	public xAxis: Mesh;
	public yAxis: Mesh;
	public zAxis: Mesh;

	constructor(size = 1) {
		super();

		const radius = 0.01 * size;
		const height = 1 * size;

		const axisGeom = new CylinderGeometry(radius, radius, height, 8);
		axisGeom.translate(0, height / 2, 0); // Pivot at the base

		const redMat = new MeshBasicMaterial({ color: 0xFF0000 });
		const greenMat = new MeshBasicMaterial({ color: 0x00FF00 });
		const blueMat = new MeshBasicMaterial({ color: 0x0000FF });

		this.xAxis = new Mesh(axisGeom, redMat);
		this.xAxis.rotation.z = -Math.PI / 2;
		this.add(this.xAxis);

		this.yAxis = new Mesh(axisGeom, greenMat);
		this.add(this.yAxis);

		this.zAxis = new Mesh(axisGeom, blueMat);
		this.zAxis.rotation.x = Math.PI / 2;
		this.add(this.zAxis);
	}

	dispose() {
		(this.xAxis.material as MeshBasicMaterial).dispose();
		(this.yAxis.material as MeshBasicMaterial).dispose();
		(this.zAxis.material as MeshBasicMaterial).dispose();
		this.xAxis.geometry.dispose();
		// yAxis and zAxis share geometry, so only dispose once.
	}
}

/**
 * Used to show the physics impostor around the specific mesh
 */
class PhysicsViewer {
    private _scene: Scene;
    private _physicsPlugin: HavokPlugin;

    private _bodies: Array<PhysicsBody> = [];
    private _inertiaBodies: Array<PhysicsBody> = [];
    private _constraints: Array<PhysicsConstraint> = [];

    private _bodyMeshes: Array<Object3D> = [];
    private _inertiaMeshes: Array<Object3D> = [];
    private _constraintMeshes: Array<Array<Object3D>> = [];

    private _debugMaterial: MeshBasicMaterial | null = null;
    private _debugInertiaMaterial: MeshBasicMaterial | null = null;

    private _constraintAxesSize: number = 0.4;
    private _constraintAngularSize: number = 0.4;

    /**
     * Creates a new PhysicsViewer
     * @param scene defines the hosting scene
     * @param physicsPlugin defines the physics plugin to use
     * @param size defines a scaling factor for the debug visuals
     */
    constructor(
        scene: Scene,
        physicsPlugin: HavokPlugin,
        size: number = 1
    ) {
        this._scene = scene;
        this._physicsPlugin = physicsPlugin;
        this._constraintAxesSize = 0.4 * size;
        this._constraintAngularSize = 0.4 * size;
    }

    /**
     * Updates the debug meshes of the physics engine.
     * This method should be called in your render loop to keep the visuals in sync with the physics simulation.
     */
    public update(): void {
        this._updateDebugMeshes();
        this._updateInertiaMeshes();
        this._updateDebugConstraints();
    }

    /**
     * Shows a debug mesh for a given physics body.
     * @param body The physics body to show.
     * @returns The debug mesh, or null if the body is already shown.
     */
    public showBody(body: PhysicsBody): Object3D | null {
        if (this._bodies.includes(body)) {
            return null;
        }

        const debugMesh = this._getDebugBodyMesh(body);
        console.log("create debug mesh", debugMesh);

        if (debugMesh) {
            this._bodies.push(body);
            this._bodyMeshes.push(debugMesh);
            this._scene.add(debugMesh);
        }
        return debugMesh;
    }

    /**
     * Hides a body from the physics engine.
     * @param body - The body to hide.
     * @returns true if body actually removed
     */
    public hideBody(body: PhysicsBody | null): boolean {
        if (!body) {
            return false;
        }

        const index = this._bodies.indexOf(body);
        if (index !== -1) {
            const mesh = this._bodyMeshes[index];
            if (mesh) {
                this._scene.remove(mesh);
                if ((mesh as Mesh).geometry) {
                    (mesh as Mesh).geometry.dispose();
                }
                if ((mesh as Mesh).material) {
                    if (Array.isArray((mesh as Mesh).material)) {
                        ((mesh as Mesh).material as MeshBasicMaterial[]).forEach((m) =>
                            m.dispose()
                        );
                    } else {
                        ((mesh as Mesh).material as MeshBasicMaterial).dispose();
                    }
                }
            }

            this._bodies.splice(index, 1);
            this._bodyMeshes.splice(index, 1);
            return true;
        }
        return false;
    }

    /**
     * Shows a debug box corresponding to the inertia of a given body
     * @param body the physics body used to get the inertia
     * @returns the debug mesh used to show the inertia, or null if the body is already shown
     */
    public showInertia(body: PhysicsBody): Object3D | null {
        if (this._inertiaBodies.includes(body)) {
            return null;
        }

        const debugMesh = this._getDebugInertiaMesh(body);
        if (debugMesh) {
            this._inertiaBodies.push(body);
            this._inertiaMeshes.push(debugMesh);
            this._scene.add(debugMesh);
        }
        return debugMesh;
    }

    /**
     * Hides a body's inertia from the viewer.
     * @param body the body to hide
     * @returns true if inertia actually removed
     */
    public hideInertia(body: PhysicsBody | null): boolean {
        if (!body) {
            return false;
        }
        const index = this._inertiaBodies.indexOf(body);
        if (index !== -1) {
            const mesh = this._inertiaMeshes[index];
            if (mesh) {
                this._scene.remove(mesh);
                // Dispose logic for group
                mesh.traverse((child) => {
                    if ((child as Mesh).isMesh) {
                        (child as Mesh).geometry.dispose();
                        if (Array.isArray((child as Mesh).material)) {
                            ((child as Mesh).material as MeshBasicMaterial[]).forEach(
                                (m) => m.dispose()
                            );
                        } else {
                            ((child as Mesh).material as MeshBasicMaterial).dispose();
                        }
                    }
                });
            }
            this._inertiaBodies.splice(index, 1);
            this._inertiaMeshes.splice(index, 1);
            return true;
        }
        return false;
    }

    /**
     * Shows a debug mesh for a given physics constraint.
     * @param constraint the physics constraint to show
     * @returns the debug mesh, or null if the constraint is already shown
     */
    public showConstraint(constraint: PhysicsConstraint): Object3D | null {
        if (this._constraints.includes(constraint)) {
            return null;
        }

        const debugMesh = this._getDebugConstraintMesh(constraint);
        if (debugMesh) {
            this._constraints.push(constraint);
            this._constraintMeshes.push(debugMesh);
            this._scene.add(debugMesh[0]); // Add the main parent mesh
        }
        return debugMesh ? debugMesh[0] : null;
    }

    /**
     * Hide a physics constraint from the viewer.
     * @param constraint the constraint to hide
     */
    public hideConstraint(constraint: PhysicsConstraint | null): void {
        if (!constraint) {
            return;
        }
        const index = this._constraints.indexOf(constraint);
        if (index !== -1) {
            const meshes = this._constraintMeshes[index];
            if (meshes) {
                for (const mesh of meshes) {
                    this._scene.remove(mesh);
                    // Dispose logic for group/mesh
                    mesh.traverse((child) => {
                        if ((child as Mesh).isMesh) {
                            (child as Mesh).geometry.dispose();
                            if (Array.isArray((child as Mesh).material)) {
                                ((child as Mesh).material as MeshBasicMaterial[]).forEach(
                                    (m) => m.dispose()
                                );
                            } else {
                                ((child as Mesh).material as MeshBasicMaterial).dispose();
                            }
                        }
                    });
                }
            }
            this._constraints.splice(index, 1);
            this._constraintMeshes.splice(index, 1);
        }
    }

    /**
     * Clean up all physics debug display.
     */
    public dispose(): void {
        // bodies
        [...this._bodies].forEach((body) => this.hideBody(body));
        // inertia
        [...this._inertiaBodies].forEach((body) => this.hideInertia(body));
        // constraints
        [...this._constraints].forEach((constraint) =>
            this.hideConstraint(constraint)
        );

        this._debugMaterial?.dispose();
        this._debugInertiaMaterial?.dispose();

        this._bodies.length = 0;
        this._bodyMeshes.length = 0;
        this._inertiaBodies.length = 0;
        this._inertiaMeshes.length = 0;
        this._constraints.length = 0;
        this._constraintMeshes.length = 0;
    }

    private _updateDebugMeshes(): void {
        for (let i = 0; i < this._bodies.length;) {
            const body = this._bodies[i];
            if (body && body.isDisposed && this.hideBody(body)) {
                continue; // hideBody has shifted the array, so we re-evaluate the same index
            }
            const transform = this._bodyMeshes[i];
            if (body && transform) {
                this._physicsPlugin.syncTransform(body, transform);
            }
            i++;
        }
    }

    private _updateInertiaMeshes(): void {
        for (let i = 0; i < this._inertiaBodies.length;) {
            const body = this._inertiaBodies[i];
            if (body && body.isDisposed && this.hideInertia(body)) {
                continue;
            }
            const mesh = this._inertiaMeshes[i];
            if (body && mesh) {
                this._updateDebugInertia(body, mesh);
            }
            i++;
        }
    }

    private _updateDebugInertia(body: PhysicsBody, inertiaMesh: Object3D): void {
        const inertiaMatrixRef = new Matrix4();
        const transformMatrixRef = new Matrix4();
        const finalMatrixRef = new Matrix4();

        if (body._pluginDataInstances.length > 0) {
            const inertiaAsMesh = inertiaMesh as InstancedMesh;
            for (let i = 0; i < body._pluginDataInstances.length; i++) {
                const props = body.getMassProperties(i);
                this._getMeshDebugInertiaMatrixToRef(props, inertiaMatrixRef);

                body.node.updateMatrixWorld(true);
                (body.node as InstancedMesh).getMatrixAt(i, transformMatrixRef);

                finalMatrixRef.multiplyMatrices(transformMatrixRef, inertiaMatrixRef);
                inertiaAsMesh.setMatrixAt(i, finalMatrixRef);
            }
            inertiaAsMesh.instanceMatrix.needsUpdate = true;
        } else {
            const props = body.getMassProperties();
            this._getMeshDebugInertiaMatrixToRef(props, inertiaMatrixRef);

            body.node.updateMatrixWorld(true);
            transformMatrixRef.copy(body.node.matrixWorld);

            finalMatrixRef.multiplyMatrices(transformMatrixRef, inertiaMatrixRef);
            finalMatrixRef.decompose(
                inertiaMesh.position,
                inertiaMesh.quaternion,
                inertiaMesh.scale
            );
        }
    }

    private _updateDebugConstraints(): void {
        for (let i = 0; i < this._constraints.length; i++) {
            const constraint = this._constraints[i];
            const mesh = this._constraintMeshes[i];
            if (constraint && mesh) {
                this._updateDebugConstraint(constraint, mesh[0]);
            }
        }
    }

    private _makeScalingUnitInPlace(scaling: Vector3): void {
        const epsilon = 1e-6;
        if (Math.abs(scaling.x - 1) > epsilon) {
            scaling.x = 1 * Math.sign(scaling.x);
        }
        if (Math.abs(scaling.y - 1) > epsilon) {
            scaling.y = 1 * Math.sign(scaling.y);
        }
        if (Math.abs(scaling.z - 1) > epsilon) {
            scaling.z = 1 * Math.sign(scaling.z);
        }
    }

    private _updateDebugConstraint(
        constraint: PhysicsConstraint,
        parentingMesh: Object3D
    ): void {
        if (!constraint._initOptions) {
            return;
        }

        const { pivotA, pivotB, axisA, axisB, perpAxisA, perpAxisB } =
            constraint._initOptions;
        if (!pivotA || !pivotB || !axisA || !axisB || !perpAxisA || !perpAxisB) {
            return;
        }

        parentingMesh.children.forEach((parentOfPair) => {
            const parentCoordSystemNode = parentOfPair.children[0] as Group;
            const childCoordSystemNode = parentOfPair.children[1] as Group;

            const { parentBody, parentBodyIndex } = parentCoordSystemNode.userData;
            const { childBody, childBodyIndex } = childCoordSystemNode.userData;

            const parentTransform = this._getTransformFromBodyToRef(
                parentBody,
                tempMatrixes[0],
                parentBodyIndex
            );
            const childTransform = this._getTransformFromBodyToRef(
                childBody,
                tempMatrixes[1],
                childBodyIndex
            );

            parentTransform.decompose(
                parentCoordSystemNode.position,
                parentCoordSystemNode.quaternion,
                parentCoordSystemNode.scale
            );
            this._makeScalingUnitInPlace(parentCoordSystemNode.scale);

            childTransform.decompose(
                childCoordSystemNode.position,
                childCoordSystemNode.quaternion,
                childCoordSystemNode.scale
            );
            this._makeScalingUnitInPlace(childCoordSystemNode.scale);

            const parentTransformNode = parentCoordSystemNode.children[0] as Group;
            parentTransformNode.position.copy(pivotA);
            const childTransformNode = childCoordSystemNode.children[0] as Group;
            childTransformNode.position.copy(pivotB);

            const zAxisParent = new Vector3().crossVectors(axisA, perpAxisA);
            tempMatrixes[0].makeBasis(axisA, perpAxisA, zAxisParent);
            parentTransformNode.quaternion.setFromRotationMatrix(tempMatrixes[0]);

            const zAxisChild = new Vector3().crossVectors(axisB, perpAxisB);
            tempMatrixes[1].makeBasis(axisB, perpAxisB, zAxisChild);
            childTransformNode.quaternion.setFromRotationMatrix(tempMatrixes[1]);
        });
    }

    private _getDebugMaterial(): MeshBasicMaterial {
        if (!this._debugMaterial) {
            this._debugMaterial = new MeshBasicMaterial({
                wireframe: true,
                color: 0xFFFFFF,
            });
        }
        return this._debugMaterial;
    }

    private _getDebugInertiaMaterial(): MeshBasicMaterial {
        if (!this._debugInertiaMaterial) {
            this._debugInertiaMaterial = new MeshBasicMaterial({
                color: 0xFF00FF,
                opacity: 0.2,
                transparent: true,
            });
        }
        return this._debugInertiaMaterial;
    }

    private _getDebugAxisColoredMaterial(axisNumber: number): MeshBasicMaterial {
        const color =
            axisNumber === 0
                ? 0xFF0000
                : (axisNumber === 1
                    ? 0x00FF00
                    : 0x0000FF);
        return new MeshBasicMaterial({ color });
    }

    private _getDebugBodyMesh(body: PhysicsBody): Object3D | null {
        const { positions, indices } = this._physicsPlugin.getBodyGeometry(body);
        if (positions.length === 0 || indices.length === 0) {
            return null;
        }

        const geometry = new BufferGeometry();
        geometry.setAttribute(
            "position",
            new Float32BufferAttribute(positions, 3)
        );
        geometry.setIndex([...indices]);

        let mesh: Mesh | InstancedMesh;
        if (body._pluginDataInstances?.length > 0) {
            mesh = new InstancedMesh(
                geometry,
                this._getDebugMaterial(),
                body._pluginDataInstances.length
            );
        } else {
            mesh = new Mesh(geometry, this._getDebugMaterial());
        }

        return mesh;
    }

    private _getMeshDebugInertiaMatrixToRef(
        massProps: PhysicsMassProperties,
        matrix: Matrix4
    ): Matrix4 {
        const orientation = massProps.inertiaOrientation ?? new Quaternion();
        const inertiaLocal = massProps.inertia ?? new Vector3();
        const center = massProps.centerOfMass ?? new Vector3();

        const betaSqrd = (inertiaLocal.x - inertiaLocal.y + inertiaLocal.z) * 6;
        const beta = Math.sqrt(Math.max(betaSqrd, 0));
        const gammaSqrd = inertiaLocal.x * 12 - betaSqrd;
        const gamma = Math.sqrt(Math.max(gammaSqrd, 0));
        const alphaSqrd = inertiaLocal.z * 12 - betaSqrd;
        const alpha = Math.sqrt(Math.max(alphaSqrd, 0));

        const extents = tempVecs[0].set(alpha, beta, gamma);

        const scaling = new Matrix4().makeScale(extents.x, extents.y, extents.z);
        const rotation = new Matrix4().makeRotationFromQuaternion(orientation);
        const translation = new Matrix4().makeTranslation(
            center.x,
            center.y,
            center.z
        );

        matrix.copy(scaling).premultiply(rotation).premultiply(translation);
        return matrix;
    }

    private _getDebugInertiaMesh(body: PhysicsBody): Object3D | null {
        const boxGeom = new BoxGeometry(1, 1, 1);
        const matrixRef = new Matrix4();

        if (body._pluginDataInstances.length > 0) {
            const inertiaBoxMesh = new InstancedMesh(
                boxGeom,
                this._getDebugInertiaMaterial(),
                body._pluginDataInstances.length
            );
            for (let i = 0; i < body._pluginDataInstances.length; ++i) {
                const props = body.getMassProperties(i);
                this._getMeshDebugInertiaMatrixToRef(props, matrixRef);
                inertiaBoxMesh.setMatrixAt(i, matrixRef);
            }
            inertiaBoxMesh.instanceMatrix.needsUpdate = true;
            return inertiaBoxMesh;
        } else {
            const inertiaBoxMesh = new Mesh(boxGeom, this._getDebugInertiaMaterial());
            const props = body.getMassProperties();
            this._getMeshDebugInertiaMatrixToRef(props, matrixRef);
            matrixRef.decompose(
                inertiaBoxMesh.position,
                inertiaBoxMesh.quaternion,
                inertiaBoxMesh.scale
            );

            const edges = new EdgesGeometry(inertiaBoxMesh.geometry);
            const line = new LineSegments(
                edges,
                new LineBasicMaterial({ color: 0xFF00FF, linewidth: 2 })
            );
            inertiaBoxMesh.add(line);

            return inertiaBoxMesh;
        }
    }

    private _getTransformFromBodyToRef(
        body: PhysicsBody,
        matrix: Matrix4,
        instanceIndex?: number
    ): Matrix4 {
        const tnode = body.node;
        if (
            instanceIndex !== undefined &&
            instanceIndex >= 0 &&
            (tnode as InstancedMesh).isInstancedMesh
        ) {
            (tnode as InstancedMesh).getMatrixAt(instanceIndex, matrix);
            return matrix;
        } else {
            tnode.updateWorldMatrix(true, false);
            return matrix.copy(tnode.matrixWorld);
        }
    }

    private _createAngularConstraintMesh(
        minLimit: number,
        maxLimit: number,
        axisNumber: number,
        parent: Object3D
    ): Mesh {
        const arcAngle = maxLimit - minLimit;
        const mesh = new Mesh(
            new CylinderGeometry(
                this._constraintAngularSize * 1.5,
                this._constraintAngularSize * 1.5,
                0.001,
                32,
                1,
                false,
                minLimit,
                arcAngle
            ),
            this._getDebugAxisColoredMaterial(axisNumber)
        );
        mesh.material.side = 2; // DoubleSide
        parent.add(mesh);

        switch (axisNumber) {
            // ANGULAR_X
            case 0: {
                mesh.quaternion.setFromEuler(new Euler(0, Math.PI / 2, 0));
                break;
            }
            // ANGULAR_Y
            case 1: {
                mesh.quaternion.setFromEuler(new Euler(-Math.PI / 2, 0, 0));
                break;
            }
            // ANGULAR_Z
            case 2: {
                // No rotation needed
                break;
            }
        }
        return mesh;
    }

    private _createCage(parent: Object3D): Group {
        const cage = new Group();
        const box = new Mesh(
            new BoxGeometry(1, 1, 1),
            new MeshBasicMaterial({ transparent: true, opacity: 0 })
        );
        box.position.set(0.5, 0.5, 0.5); // pivot at corner
        const edges = new EdgesGeometry(box.geometry);
        const line = new LineSegments(
            edges,
            new LineBasicMaterial({ color: 0xFFFFFF, linewidth: 2 })
        );
        box.add(line);
        cage.add(box);
        parent.add(cage);
        return cage;
    }

    private _getDebugConstraintMesh(
        constraint: PhysicsConstraint
    ): Array<Object3D> | null {
        if (!constraint._initOptions) {
            return null;
        }

        const { pivotA, pivotB, axisA, axisB, perpAxisA, perpAxisB } =
            constraint._initOptions;
        if (!pivotA || !pivotB || !axisA || !axisB || !perpAxisA || !perpAxisB) {
            return null;
        }

        const parentingMesh = new Group();
        parentingMesh.name = "parentingDebugConstraint";
        const parentedConstraintMeshes: Array<Object3D> = [parentingMesh];

        const bodiesUsingConstraint =
            this._physicsPlugin.getBodiesUsingConstraint(constraint);

        for (const bodyPairInfo of bodiesUsingConstraint) {
            const parentOfPair = new Group();
            parentOfPair.name = "parentOfPair";
            parentingMesh.add(parentOfPair);

            const { parentBody, parentBodyIndex, childBody, childBodyIndex } =
                bodyPairInfo;

            const parentCoordSystemNode = new Group();
            parentCoordSystemNode.name = "parentCoordSystem";
            parentOfPair.add(parentCoordSystemNode);
            parentCoordSystemNode.userData = { parentBody, parentBodyIndex };

            const childCoordSystemNode = new Group();
            childCoordSystemNode.name = "childCoordSystem";
            parentOfPair.add(childCoordSystemNode);
            childCoordSystemNode.userData = { childBody, childBodyIndex };

            const parentTransformNode = new Group();
            parentTransformNode.name = "constraint_parent";
            parentCoordSystemNode.add(parentTransformNode);

            const childTransformNode = new Group();
            childTransformNode.name = "constraint_child";
            childCoordSystemNode.add(childTransformNode);

            const parentAxes = new CustomAxesViewer(this._constraintAxesSize);
            parentTransformNode.add(parentAxes);

            const childAxes = new CustomAxesViewer(this._constraintAxesSize);
            childTransformNode.add(childAxes);

            // Any free/limited Linear axis
            const cage = this._createCage(parentTransformNode);
            const min = new Vector3();
            const max = new Vector3();

            const limited = [false, false, false];
            limited[0] =
                this._physicsPlugin.getAxisMode(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_X
                ) === PhysicsConstraintAxisLimitMode.LIMITED;
            limited[1] =
                this._physicsPlugin.getAxisMode(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_Y
                ) === PhysicsConstraintAxisLimitMode.LIMITED;
            limited[2] =
                this._physicsPlugin.getAxisMode(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_Z
                ) === PhysicsConstraintAxisLimitMode.LIMITED;

            min.x = limited[0]
                ? this._physicsPlugin.getAxisMinLimit(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_X
                ) ?? 0
                : 0;
            max.x = limited[0]
                ? this._physicsPlugin.getAxisMaxLimit(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_X
                ) ?? 0
                : 0;
            min.y = limited[1]
                ? this._physicsPlugin.getAxisMinLimit(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_Y
                ) ?? 0
                : 0;
            max.y = limited[1]
                ? this._physicsPlugin.getAxisMaxLimit(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_Y
                ) ?? 0
                : 0;
            min.z = limited[2]
                ? this._physicsPlugin.getAxisMinLimit(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_Z
                ) ?? 0
                : 0;
            max.z = limited[2]
                ? this._physicsPlugin.getAxisMaxLimit(
                    constraint,
                    PhysicsConstraintAxis.LINEAR_Z
                ) ?? 0
                : 0;

            cage.position.copy(min);
            cage.scale.set(max.x - min.x, max.y - min.y, max.z - min.z);
            parentedConstraintMeshes.push(cage);

            // Angular limits
            const constraintAxisAngular = [
                PhysicsConstraintAxis.ANGULAR_X,
                PhysicsConstraintAxis.ANGULAR_Y,
                PhysicsConstraintAxis.ANGULAR_Z,
            ];
            for (let axisIndex = 0; axisIndex < 3; axisIndex++) {
                const axis = constraintAxisAngular[axisIndex];
                const axisMode = this._physicsPlugin.getAxisMode(constraint, axis);
                if (axisMode === PhysicsConstraintAxisLimitMode.LIMITED) {
                    const minLimit = this._physicsPlugin.getAxisMinLimit(
                        constraint,
                        axis
                    ) ?? 0;
                    const maxLimit = this._physicsPlugin.getAxisMaxLimit(
                        constraint,
                        axis
                    ) ?? 0;
                    const mesh = this._createAngularConstraintMesh(
                        minLimit,
                        maxLimit,
                        axisIndex,
                        childTransformNode
                    );
                    parentedConstraintMeshes.push(mesh);
                }
            }
        }

        this._updateDebugConstraint(constraint, parentingMesh);
        return parentedConstraintMeshes;
    }
}

export default PhysicsViewer;
