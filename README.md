Also need to use `@babylonjs-havok` npm package
```
const { default: HavokPhysics } = await import("@babylonjs/havok");
const path = new URL(
	"/havok/HavokPhysics.wasm", // your .wasm file path here
	globalThis.self.location.origin
).toString();
const response = await fetch(path);
if (!response.ok) {
	throw new Error(`HTTP error! Status: ${response.status} - ${response.statusText}`);
}
const binary = await response.arrayBuffer();
const havok = await HavokPhysics({ locateFile: () => path, wasmBinary: binary });
```
