import { Vector3 } from "three";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

export const setOrbitRadius = (controls: OrbitControls, radius: number) => {
    // Get the current direction vector from target to camera
    const direction = new Vector3();
    direction.subVectors(controls.object.position, controls.target);

    // Set the length of that vector to the new radius
    direction.setLength(radius);

    // Add the direction vector to the target to get the new camera position
    controls.object.position.copy(controls.target).add(direction);

    // Update the controls to reflect the change
    controls.update();
}

export const getOrbitRadius = (controls: OrbitControls) => {
    return controls.object.position.distanceTo(controls.target);
}
