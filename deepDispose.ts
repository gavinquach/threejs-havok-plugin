/* eslint-disable @typescript-eslint/no-explicit-any */
import { BufferGeometry } from "three/src/core/BufferGeometry.js";
import { Material } from "three/src/materials/Material.js";
import { Texture } from "three/src/textures/Texture.js";

import type { Object3D, ShaderMaterial } from "three";

/**
 * Traverse material or array of materials and all nested textures
 * executing there respective callback
 *
 * @param material          Three js Material or array of material
 * @param materialCallback  Material callback
 * @param textureCallback   Texture callback
 */
function traverseMaterialsTextures(
    material: Material | Material[],
    materialCallback?: (material: any) => void,
    textureCallback?: (texture: any) => void
) {
    const traverseMaterial = (mat: Material) => {
        if (materialCallback) materialCallback(mat);

        if (!textureCallback) return;

        for (const texture of Object.values(mat)
            .filter((value) => value instanceof Texture)) textureCallback(texture);

        if ((mat as ShaderMaterial).uniforms)
            for (const { value } of Object.values((mat as ShaderMaterial).uniforms)
                .filter(({ value }) => value instanceof Texture)) textureCallback(value);
    };

    if (Array.isArray(material)) {
        for (const mat of material) traverseMaterial(mat);
    } else traverseMaterial(material);
}

/**
 * Dispose of all Object3D`s nested Geometries, Materials and Textures
 *
 * @param object  Object3D, BufferGeometry, Material or Texture
 */
function deepDispose(object: Object3D | BufferGeometry | Material | Texture) {
    const dispose = (object: BufferGeometry | Material | Texture) =>
        object.dispose();
    const disposeObject = (object: any) => {
        if (object.geometry) dispose(object.geometry);
        if (object.material)
            traverseMaterialsTextures(object.material, dispose, dispose);
    };

    if (object instanceof BufferGeometry || object instanceof Texture)
        return dispose(object);

    if (object instanceof Material)
        return traverseMaterialsTextures(object, dispose, dispose);

    disposeObject(object);

    if (object.traverse) object.traverse((obj) => disposeObject(obj));
}

export { deepDispose };
