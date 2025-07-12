import { Euler, type Object3D, type Vector3 } from "three";

/**
 * Sets the world position of a THREE.Object3D.
 * @param {Object3D} object The object to move.
 * @param {Vector3} worldPosition The target world position.
 */
export const setObjectWorldPosition = (object: Object3D, worldPosition: Vector3) => {
    if (object.parent) {
        // First, ensure the parent's world matrix is up-to-date
        object.parent.updateWorldMatrix(true, false);

        // Use a clone of the world position vector to avoid modifying the original
        const localPosition = worldPosition.clone();

        // Transform the world position to the parent's local space
        object.parent.worldToLocal(localPosition);

        // Set the object's local position
        object.position.copy(localPosition);
    } else {
        // If there's no parent, world position is the same as local position
        object.position.copy(worldPosition);
    }
};

export const setObjectDirection = (
    object: Object3D,
    direction: Vector3,
    yawCor: number = 0,
    pitchCor: number = 0,
    rollCor: number = 0
) => {
    const yaw = -Math.atan2(direction.z, direction.x) + Math.PI / 2;
    const len = Math.hypot(direction.x, direction.z);
    const pitch = -Math.atan2(direction.y, len);

    if (object.quaternion) {
        object.quaternion.setFromEuler(
            new Euler(pitch + pitchCor, yaw + yawCor, rollCor, "YXZ")
        );
    } else {
        object.rotation.x = pitch + pitchCor;
        object.rotation.y = yaw + yawCor;
        object.rotation.z = rollCor;
    }
};
