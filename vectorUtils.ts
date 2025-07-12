import type { Vector3 } from "three";

/**
 * Original author: Babylon.js team 
 * 
 * From file: @babylonjs/core/Maths/math.vector
 * 
 * Creates a vector normal (perpendicular) to the current Vector3 and stores the result in the given vector
 * Out of the infinite possibilities the normal chosen is the one formed by rotating the current vector
 * 90 degrees about an axis which lies perpendicular to the current vector
 * and its projection on the xz plane. In the case of a current vector in the xz plane
 * the normal is calculated to be along the y axis.
 * Example Playground https://playground.babylonjs.com/#R1F8YU#230
 * Example Playground https://playground.babylonjs.com/#R1F8YU#231
 * @param result defines the Vector3 object where to store the resultant normal
 * @returns the result
 */
export const getVectorNormalToRef = (fromVector: Vector3, toVector: Vector3): Vector3 => {
    /**
     * Calculates the spherical coordinates of the current vector
     */
    const radius = fromVector.length();
    let theta = Math.acos(fromVector.y / radius);
    const phi = Math.atan2(fromVector.z, fromVector.x);
    // makes angle 90 degs to current vector
    if (theta > Math.PI / 2) {
        theta -= Math.PI / 2;
    } else {
        theta += Math.PI / 2;
    }
    //Calculates resutant normal vector from spherical coordinate of perpendicular vector
    const x = radius * Math.sin(theta) * Math.cos(phi);
    const y = radius * Math.cos(theta);
    const z = radius * Math.sin(theta) * Math.sin(phi);
    toVector.set(x, y, z);
    return toVector;
};
