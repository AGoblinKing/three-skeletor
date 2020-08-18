'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var three = require('three');

/**
 * A collection of utilities.
 * @module utils
 */

const t1 = new three.Vector3();
const t2 = new three.Vector3();
const t3 = new three.Vector3();
const m1 = new three.Matrix4();

/**
 * Returns the world position of object and sets
 * it on target.
 *
 * @param {THREE.Object3D} object
 * @param {THREE.Vector3} target
 */
function getWorldPosition(object, target) {
	return target.setFromMatrixPosition(object.matrixWorld)
}

/**
 * Returns the distance between two objects.
 *
 * @param {THREE.Object3D} obj1
 * @param {THREE.Object3D} obj2
 * @return {number}
 */
function getWorldDistance(obj1, obj2) {
	getWorldPosition(obj1, t1);
	getWorldPosition(obj2, t2);
	return a.distanceTo(b)
}

/**
 * Sets the target to the centroid position between all passed in
 * positions.
 *
 * @param {Array<THREE.Vector3>} positions
 * @param {THREE.Vector3} target
 */
function getCentroid(positions, target) {
	target.set(0, 0, 0);
	for (let position of positions) {
		target.add(position);
	}
	target.divideScalar(positions.length);

	return target
}

/**
 * Takes a direction vector and an up vector and sets
 * `target` quaternion to the rotation. Similar to THREE.Matrix4's
 * `lookAt` function, except rather than taking two Vector3 points,
 * we've already calculaeld the direction earlier so skip the first half.
 *
 * @param {THREE.Vector3} direction
 * @param {THREE.Vector3} up
 * @param {THREE.Quaternion} target
 */
function setQuaternionFromDirection(
	direction,
	up,
	target
) {
	const x = t1;
	const y = t2;
	const z = t3;
	const m = m1;
	const el = m1.elements;

	z.copy(direction);
	x.crossVectors(up, z);

	if (x.lengthSq() === 0) {
		// parallel
		if (Math.abs(up.z) === 1) {
			z.x += 0.0001;
		} else {
			z.z += 0.0001;
		}
		z.normalize();
		x.crossVectors(up, z);
	}

	x.normalize();
	y.crossVectors(z, x);

	el[0] = x.x;
	el[4] = y.x;
	el[8] = z.x;
	el[1] = x.y;
	el[5] = y.y;
	el[9] = z.y;
	el[2] = x.z;
	el[6] = y.z;
	el[10] = z.z;

	target.setFromRotationMatrix(m);
}

/**
 * Implementation of Unity's Transform.transformPoint, which is similar
 * to three's Vector3.transformDirection, except we want to take scale into account,
 * as we're not transforming a direction. Function taken from BabylonJS.
 *
 * From BabylonJS's `Vector3.transformCoordinates`:
 * Sets the passed vector coordinates with the result of the transformation by the
 * passed matrix of the passed vector. This method computes tranformed coordinates only,
 * not transformed direction vectors (ie. it takes translation in account)
 *
 * @see https://docs.unity3d.com/ScriptReference/Transform.TransformPoint.html
 * @see https://github.com/BabylonJS/Babylon.js/blob/6050288da37623088d5f613ca2d85aef877c5cd5/src/Math/babylon.math.ts#L1936
 * @param {THREE.Vector3} vector
 * @param {THREE.Matrix4} matrix
 * @param {THREE.Vector3} target
 */
function transformPoint(
	vector,
	matrix,
	target
) {
	const e = matrix.elements;

	const x = vector.x * e[0] + vector.y * e[4] + vector.z * e[8] + e[12];
	const y = vector.x * e[1] + vector.y * e[5] + vector.z * e[9] + e[13];
	const z = vector.x * e[2] + vector.y * e[6] + vector.z * e[10] + e[14];
	const w = vector.x * e[3] + vector.y * e[7] + vector.z * e[11] + e[15];
	target.set(x / w, y / w, z / w);
}

const Y_AXIS = new three.Vector3(0, 1, 0);

/**
 * A class for a joint.
 */
class IKJoint {
	/**
	 * @param {THREE.Bone} bone
	 * @param {Object} config
	 * @param {Array<IKConstraint>} [config.constraints]
	 */
	constructor(bone, { constraints } = {}) {
		this.constraints = constraints || [];

		this.bone = bone;

		this.distance = 0;

		this._originalDirection = new three.Vector3();
		this._originalHinge = new three.Vector3();
		this._direction = new three.Vector3();
		this._worldPosition = new three.Vector3();
		this._isSubBase = false;
		this._subBasePositions = null;
		this.isIKJoint = true;

		this._originalUp = new three.Vector3(0, 1, 0);
		this._originalUp.applyQuaternion(this.bone.quaternion).normalize();
		this._updateWorldPosition();
	}

	/**
	 * @private
	 */
	_setIsSubBase() {
		this._isSubBase = true;
		this._subBasePositions = [];
	}

	/**
	 * Consumes the stored sub base positions and apply it as this
	 * joint's world position, clearing the sub base positions.
	 *
	 * @private
	 */
	_applySubBasePositions() {
		if (this._subBasePositions.length === 0) {
			return
		}
		getCentroid(this._subBasePositions, this._worldPosition);
		this._subBasePositions.length = 0;
	}

	/**
	 * @private
	 */
	_applyConstraints() {
		if (!this.constraints) {
			return
		}

		let constraintApplied = false;
		for (let constraint of this.constraints) {
			if (constraint && constraint._apply) {
				let applied = constraint._apply(this);
				constraintApplied = constraintApplied || applied;
			}
		}
		return constraintApplied
	}

	/**
	 * Set the distance.
	 * @private
	 * @param {number} distance
	 */
	_setDistance(distance) {
		this.distance = distance;
	}

	/**
	 * @private
	 */
	_getDirection() {
		return this._direction
	}

	/**
	 * @private
	 */
	_setDirection(direction) {
		this._direction.copy(direction);
	}

	/**
	 * Gets the distance.
	 * @private
	 * @return {THREE.Vector3}
	 */
	_getDistance() {
		return this.distance
	}

	/**
	 * @private
	 */
	_updateMatrixWorld() {
		this.bone.updateMatrixWorld(true);
	}

	/**
	 * @private
	 * @return {THREE.Vector3}
	 */
	_getWorldPosition() {
		return this._worldPosition
	}

	/**
	 * @private
	 */
	_getWorldDirection(joint) {
		return new three.Vector3()
			.subVectors(this._getWorldPosition(), joint._getWorldPosition())
			.normalize()
	}

	/**
	 * @private
	 */
	_updateWorldPosition() {
		getWorldPosition(this.bone, this._worldPosition);
	}

	/**
	 * @private
	 */
	_setWorldPosition(position) {
		this._worldPosition.copy(position);
	}

	/**
	 * @private
	 */
	_localToWorldDirection(direction) {
		if (this.bone.parent) {
			const parent = this.bone.parent.matrixWorld;
			direction.transformDirection(parent);
		}
		return direction
	}

	/**
	 * @private
	 */
	_worldToLocalDirection(direction) {
		if (this.bone.parent) {
			const inverseParent = new three.Matrix4().getInverse(
				this.bone.parent.matrixWorld
			);
			direction.transformDirection(inverseParent);
		}
		return direction
	}

	/**
	 * @private
	 */
	_applyWorldPosition() {
		let direction = new three.Vector3().copy(this._direction);
		let position = new three.Vector3().copy(this._getWorldPosition());

		const parent = this.bone.parent;

		if (parent) {
			this._updateMatrixWorld();
			let inverseParent = new three.Matrix4().getInverse(this.bone.parent.matrixWorld);
			transformPoint(position, inverseParent, position);
			this.bone.position.copy(position);

			this._updateMatrixWorld();

			this._worldToLocalDirection(direction);
			setQuaternionFromDirection(direction, Y_AXIS, this.bone.quaternion);
		} else {
			this.bone.position.copy(position);
		}

		// Update the world matrix so the next joint can properly transform
		// with this world matrix
		this.bone.updateMatrix();
		this._updateMatrixWorld();
	}

	/**
	 * @param {IKJoint|THREE.Vector3}
	 * @private
	 * @return {THREE.Vector3}
	 */
	_getWorldDistance(joint) {
		return this._worldPosition.distanceTo(
			joint.isIKJoint
				? joint._getWorldPosition()
				: getWorldPosition(joint, new three.Vector3())
		)
	}
}

/**
 * Mesh for representing an IKJoint.
 * @private
 * @extends {THREE.Object3d}
 */
class BoneHelper extends three.Object3D {
	/**
	 * @param {number} height
	 * @param {number?} boneSize
	 * @param {number?} axesSize
	 */
	constructor(height, boneSize, axesSize) {
		super();

		// If our bone has 0 height (like an end effector),
		// use a dummy Object3D instead, otherwise the ConeBufferGeometry
		// will fall back to its default and not use 0 height.
		if (height !== 0) {
			const geo = new three.ConeBufferGeometry(boneSize, height, 4);
			geo.applyMatrix(
				new three.Matrix4().makeRotationAxis(new three.Vector3(1, 0, 0), Math.PI / 2)
			);
			this.boneMesh = new three.Mesh(
				geo,
				new three.MeshBasicMaterial({
					color: 0xff0000,
					wireframe: true,
					depthTest: false,
					depthWrite: false,
				})
			);
		} else {
			this.boneMesh = new three.Object3D();
		}

		// Offset the bone so that its rotation point is at the base of the bone
		this.boneMesh.position.z = height / 2;
		this.add(this.boneMesh);

		this.axesHelper = new three.AxesHelper(axesSize);
		this.add(this.axesHelper);
	}
}

/**
 * Class for visualizing an IK system.
 * @extends {THREE.Object3d}
 */
class IKHelper extends three.Object3D {
	/**
	 * Creates a visualization for an IK.
	 *
	 * @param {IK} ik
	 * @param {Object} config
	 * @param {THREE.Color} [config.color]
	 * @param {boolean} [config.showBones]
	 * @param {boolean} [config.showAxes]
	 * @param {boolean} [config.wireframe]
	 * @param {number} [config.axesSize]
	 * @param {number} [config.boneSize]
	 */
	constructor(
		ik,
		{ color, showBones, boneSize, showAxes, axesSize, wireframe } = {}
	) {
		super();

		boneSize = boneSize || 0.1;
		axesSize = axesSize || 0.2;

		if (!ik.isIK) {
			throw new Error('IKHelper must receive an IK instance.')
		}

		this.ik = ik;

		this._meshes = new Map();

		for (let rootChain of this.ik.chains) {
			const chainsToMeshify = [rootChain];
			while (chainsToMeshify.length) {
				const chain = chainsToMeshify.shift();
				for (let i = 0; i < chain.joints.length; i++) {
					const joint = chain.joints[i];
					const nextJoint = chain.joints[i + 1];
					const distance = nextJoint ? nextJoint.distance : 0;

					// If a sub base, don't make another bone
					if (chain.base === joint && chain !== rootChain) {
						continue
					}
					const mesh = new BoneHelper(distance, boneSize, axesSize);
					mesh.matrixAutoUpdate = false;
					this._meshes.set(joint, mesh);
					this.add(mesh);
				}
				for (let subChains of chain.chains.values()) {
					for (let subChain of subChains) {
						chainsToMeshify.push(subChain);
					}
				}
			}
		}

		/**
		 * Whether this IKHelper's bones are visible or not.
		 *
		 * @name IKHelper#showBones
		 * @type boolean
		 * @default true
		 */
		this.showBones = showBones !== undefined ? showBones : true;

		/**
		 * Whether this IKHelper's axes are visible or not.
		 *
		 * @name IKHelper#showAxes
		 * @type boolean
		 * @default true
		 */
		this.showAxes = showAxes !== undefined ? showAxes : true;

		/**
		 * Whether this IKHelper should be rendered as wireframes or not.
		 *
		 * @name IKHelper#wireframe
		 * @type boolean
		 * @default true
		 */
		this.wireframe = wireframe !== undefined ? wireframe : true;

		/**
		 * The color of this IKHelper's bones.
		 *
		 * @name IKHelper#color
		 * @type THREE.Color
		 * @default new THREE.Color(0xff0077)
		 */
		this.color = color || new three.Color(0xff0077);
	}

	get showBones() {
		return this._showBones
	}
	set showBones(showBones) {
		if (showBones === this._showBones) {
			return
		}
		for (let [joint, mesh] of this._meshes) {
			if (showBones) {
				mesh.add(mesh.boneMesh);
			} else {
				mesh.remove(mesh.boneMesh);
			}
		}
		this._showBones = showBones;
	}

	get showAxes() {
		return this._showAxes
	}
	set showAxes(showAxes) {
		if (showAxes === this._showAxes) {
			return
		}
		for (let [joint, mesh] of this._meshes) {
			if (showAxes) {
				mesh.add(mesh.axesHelper);
			} else {
				mesh.remove(mesh.axesHelper);
			}
		}
		this._showAxes = showAxes;
	}

	get wireframe() {
		return this._wireframe
	}
	set wireframe(wireframe) {
		if (wireframe === this._wireframe) {
			return
		}
		for (let [joint, mesh] of this._meshes) {
			if (mesh.boneMesh.material) {
				mesh.boneMesh.material.wireframe = wireframe;
			}
		}
		this._wireframe = wireframe;
	}

	get color() {
		return this._color
	}
	set color(color) {
		if (this._color && this._color.equals(color)) {
			return
		}
		color = color && color.isColor ? color : new three.Color(color);
		for (let [joint, mesh] of this._meshes) {
			if (mesh.boneMesh.material) {
				mesh.boneMesh.material.color = color;
			}
		}
		this._color = color;
	}

	updateMatrixWorld(force) {
		for (let [joint, mesh] of this._meshes) {
			mesh.matrix.copy(joint.bone.matrixWorld);
		}
		super.updateMatrixWorld(force);
	}
}

class IK {
	/**
	 * Create an IK structure.
	 *
	 */
	constructor() {
		this.chains = [];
		this._needsRecalculated = true;

		this.isIK = true;

		// this.iterations = 1;
		// this.tolerance = 0.05;

		/**
		 * An array of root chains for this IK system, each containing
		 * an array of all subchains, including the root chain, for that
		 * root chain, in descending-depth order.
		 * @private
		 */
		this._orderedChains = null;
	}

	/**
	 * Adds an IKChain to the IK system.
	 *
	 * @param {IKChain} chain
	 */
	add(chain) {
		if (!chain.isIKChain) {
			throw new Error('Argument is not an IKChain.')
		}

		this.chains.push(chain);
	}

	/**
	 * Called if there's been any changes to an IK structure.
	 * Called internally. Not sure if this should be supported externally.
	 * @private
	 */
	recalculate() {
		this._orderedChains = [];

		for (let rootChain of this.chains) {
			const orderedChains = [];
			this._orderedChains.push(orderedChains);

			const chainsToSave = [rootChain];
			while (chainsToSave.length) {
				const chain = chainsToSave.shift();
				orderedChains.push(chain);
				for (let subChains of chain.chains.values()) {
					for (let subChain of subChains) {
						if (chainsToSave.indexOf(subChain) !== -1) {
							throw new Error('Recursive chain structure detected.')
						}
						chainsToSave.push(subChain);
					}
				}
			}
		}
	}

	/**
	 * Performs the IK solution and updates bones.
	 */
	solve() {
		// If we don't have a depth-sorted array of chains, generate it.
		// This is from the first `update()` call after creating.
		if (!this._orderedChains) {
			this.recalculate();
		}

		for (let subChains of this._orderedChains) {
			// Hardcode to one for now
			let iterations = 1; // this.iterations;

			while (iterations > 0) {
				for (let i = subChains.length - 1; i >= 0; i--) {
					subChains[i]._updateJointWorldPositions();
				}

				// Run the chain's forward step starting with the deepest chains.
				for (let i = subChains.length - 1; i >= 0; i--) {
					subChains[i]._forward();
				}

				// Run the chain's backward step starting with the root chain.
				let withinTolerance = true;
				for (let i = 0; i < subChains.length; i++) {
					const distanceFromTarget = subChains[i]._backward();
					if (distanceFromTarget > this.tolerance) {
						withinTolerance = false;
					}
				}

				if (withinTolerance) {
					break
				}

				iterations--;
			}
		}
	}

	/**
	 * Returns the root bone of this structure. Currently
	 * only returns the first root chain's bone.
	 *
	 * @return {THREE.Bone}
	 */
	getRootBone() {
		return this.chains[0].base.bone
	}
}

const Z_AXIS = new three.Vector3(0, 0, -1);
const X_AXIS = new three.Vector3(1, 0, 0);

const t1$1 = new three.Vector3();
const t2$1 = new three.Vector3();
const t3$1 = new three.Vector3();
const t4 = new three.Vector3();

const { DEG2RAD, RAD2DEG } = three.Math;

/**
 * A class for a constraint.
 */
class IKHingeConstraint {
	/**
	 * Pass in an angle value in degrees,
	 * Axis of rotation for the constraint is calculated from initial bone positions.
	 *
	 * @param {number} angle
	 */
	constructor(angle) {
		this.angle = angle;
		this.rotationPlane = new three.Plane();
	}

	/**
	 * Applies a hinge constraint to passed in IKJoint. The direction will always be updated
	 * with this constraint, because it will always be projected onto the rotation plane.
	 * Additionally, an angle constraint will be applied if necessary.
	 *
	 * @param {IKJoint} joint
	 * @private
	 */
	_apply(joint) {
		// Get direction of joint and parent in world space
		const direction = new three.Vector3().copy(joint._getDirection());
		const parentDirection = joint
			._localToWorldDirection(t1$1.copy(Z_AXIS))
			.normalize();
		const rotationPlaneNormal = joint
			._localToWorldDirection(t2$1.copy(joint._originalHinge))
			.normalize();
		this.rotationPlane.normal = rotationPlaneNormal;
		var projectedDir = this.rotationPlane.projectPoint(direction, new three.Vector3());

		var parentDirectionProjected = this.rotationPlane.projectPoint(
			parentDirection,
			t3$1
		);
		var currentAngle = projectedDir.angleTo(parentDirectionProjected) * RAD2DEG;

		//apply adjustment to angle if it is "negative"
		var cross = t4.crossVectors(projectedDir, parentDirectionProjected);
		if (cross.dot(rotationPlaneNormal) > 0) {
			currentAngle += 180;
		}

		if (currentAngle > this.angle) {
			parentDirectionProjected.applyAxisAngle(
				rotationPlaneNormal,
				this.angle / RAD2DEG
			);
			joint._setDirection(parentDirectionProjected);
		} else {
			joint._setDirection(projectedDir);
		}
	}
}

/**
 * Class representing an IK chain, comprising multiple IKJoints.
 */
class IKChain {
	/**
	 * Create an IKChain.
	 */
	constructor() {
		this.isIKChain = true;
		this.totalLengths = 0;
		this.base = null;
		this.effector = null;
		this.effectorIndex = null;
		this.chains = new Map();

		/* THREE.Vector3 world position of base node */
		this.origin = null;

		this.iterations = 100;
		this.tolerance = 0.01;

		this._depth = -1;
		this._targetPosition = new three.Vector3();
	}

	/**
	 * Add an IKJoint to the end of this chain.
	 *
	 * @param {IKJoint} joint
	 * @param {Object} config
	 * @param {THREE.Object3D} [config.target]
	 */

	add(joint, { target } = {}) {
		if (this.effector) {
			throw new Error(
				'Cannot add additional joints to a chain with an end effector.'
			)
		}

		if (!joint.isIKJoint) {
			joint = new IKJoint(joint);
		}

		this.joints = this.joints || [];
		this.joints.push(joint);

		// If this is the first joint, set as base.
		if (this.joints.length === 1) {
			this.base = this.joints[0];
			this.origin = new three.Vector3().copy(this.base._getWorldPosition());
		}
		// Otherwise, calculate the distance for the previous joint,
		// and update the total length.
		else {
			const previousJoint = this.joints[this.joints.length - 2];
			const previousPreviousJoint = this.joints[this.joints.length - 3];

			previousJoint._updateMatrixWorld();
			previousJoint._updateWorldPosition();
			joint._updateWorldPosition();

			const distance = previousJoint._getWorldDistance(joint);
			if (distance === 0) {
				throw new Error('bone with 0 distance between adjacent bone found')
			}
			joint._setDistance(distance);

			joint._updateWorldPosition();
			const direction = previousJoint._getWorldDirection(joint);
			previousJoint._originalDirection = new three.Vector3().copy(direction);
			joint._originalDirection = new three.Vector3().copy(direction);

			if (previousPreviousJoint) {
				previousJoint._originalHinge = previousJoint._worldToLocalDirection(
					previousJoint._originalDirection
						.clone()
						.cross(previousPreviousJoint._originalDirection)
						.normalize()
				);
			}
			this.totalLengths += distance;
		}

		if (target) {
			this.effector = joint;
			this.effectorIndex = joint;
			this.target = target;
		}

		return this
	}

	/**
	 * Returns a boolean indicating whether or not this chain has an end effector.
	 *
	 * @private
	 * @return {boolean}
	 */
	_hasEffector() {
		return !!this.effector
	}

	/**
	 * Returns the distance from the end effector to the target. Returns -1 if
	 * this chain does not have an end effector.
	 *
	 * @private
	 * @return {number}
	 */
	_getDistanceFromTarget() {
		return this._hasEffector()
			? this.effector._getWorldDistance(this.target)
			: -1
	}

	/**
	 * Connects another IKChain to this chain. The additional chain's root
	 * joint must be a member of this chain.
	 *
	 * @param {IKChain} chain
	 */
	connect(chain) {
		if (!chain.isIKChain) {
			throw new Error('Invalid connection in an IKChain. Must be an IKChain.')
		}

		if (!chain.base.isIKJoint) {
			throw new Error('Connecting chain does not have a base joint.')
		}

		const index = this.joints.indexOf(chain.base);

		// If we're connecting to the last joint in the chain, ensure we don't
		// already have an effector.
		if (this.target && index === this.joints.length - 1) {
			throw new Error(
				'Cannot append a chain to an end joint in a chain with a target.'
			)
		}

		if (index === -1) {
			throw new Error(
				'Cannot connect chain that does not have a base joint in parent chain.'
			)
		}

		this.joints[index]._setIsSubBase();

		let chains = this.chains.get(index);
		if (!chains) {
			chains = [];
			this.chains.set(index, chains);
		}
		chains.push(chain);

		return this
	}

	/**
	 * Update joint world positions for this chain.
	 *
	 * @private
	 */
	_updateJointWorldPositions() {
		for (let joint of this.joints) {
			joint._updateWorldPosition();
		}
	}

	/**
	 * Runs the forward pass of the FABRIK algorithm.
	 *
	 * @private
	 */
	_forward() {
		// Copy the origin so the forward step can use before `_backward()`
		// modifies it.
		this.origin.copy(this.base._getWorldPosition());

		// Set the effector's position to the target's position.

		if (this.target) {
			this._targetPosition.setFromMatrixPosition(this.target.matrixWorld);
			this.effector._setWorldPosition(this._targetPosition);
		} else if (!this.joints[this.joints.length - 1]._isSubBase) {
			// If this chain doesn't have additional chains or a target,
			// not much to do here.
			return
		}

		// Apply sub base positions for all joints except the base,
		// as we want to possibly write to the base's sub base positions,
		// not read from it.
		for (let i = 1; i < this.joints.length; i++) {
			const joint = this.joints[i];
			if (joint._isSubBase) {
				joint._applySubBasePositions();
			}
		}

		for (let i = this.joints.length - 1; i > 0; i--) {
			const joint = this.joints[i];
			const prevJoint = this.joints[i - 1];
			const direction = prevJoint._getWorldDirection(joint);

			const worldPosition = direction
				.multiplyScalar(joint.distance)
				.add(joint._getWorldPosition());

			// If this chain's base is a sub base, set it's position in
			// `_subBaseValues` so that the forward step of the parent chain
			// can calculate the centroid and clear the values.
			// @TODO Could this have an issue if a subchain `x`'s base
			// also had its own subchain `y`, rather than subchain `x`'s
			// parent also being subchain `y`'s parent?
			if (prevJoint === this.base && this.base._isSubBase) {
				this.base._subBasePositions.push(worldPosition);
			} else {
				prevJoint._setWorldPosition(worldPosition);
			}
		}
	}

	/**
	 * Runs the backward pass of the FABRIK algorithm.
	 *
	 * @private
	 */
	_backward() {
		// If base joint is a sub base, don't reset it's position back
		// to the origin, but leave it where the parent chain left it.
		if (!this.base._isSubBase) {
			this.base._setWorldPosition(this.origin);
		}

		for (let i = 0; i < this.joints.length - 1; i++) {
			const joint = this.joints[i];
			const nextJoint = this.joints[i + 1];
			const jointWorldPosition = joint._getWorldPosition();

			const direction = nextJoint._getWorldDirection(joint);
			joint._setDirection(direction);

			joint._applyConstraints();

			direction.copy(joint._direction);

			// Now apply the world position to the three.js matrices. We need
			// to do this before the next joint iterates so it can generate rotations
			// in local space from its parent's matrixWorld.
			// If this is a chain sub base, let the parent chain apply the world position
			if (!(this.base === joint && joint._isSubBase)) {
				joint._applyWorldPosition();
			}

			nextJoint._setWorldPosition(
				direction.multiplyScalar(nextJoint.distance).add(jointWorldPosition)
			);

			// Since we don't iterate over the last joint, handle the applying of
			// the world position. If it's also a non-effector, then we must orient
			// it to its parent rotation since otherwise it has nowhere to point to.
			if (i === this.joints.length - 2) {
				if (nextJoint !== this.effector) {
					nextJoint._setDirection(direction);
				}
				nextJoint._applyWorldPosition();
			}
		}

		return this._getDistanceFromTarget()
	}
}

const Z_AXIS$1 = new three.Vector3(0, 0, 1);
const { DEG2RAD: DEG2RAD$1, RAD2DEG: RAD2DEG$1 } = three.Math;

/**
 * A class for a constraint.
 */
class IKBallConstraint {
	

	constructor(angle) {
		this.angle = angle;
	}

	/**
	 * Applies a constraint to passed in IKJoint, updating
	 * its direction if necessary. Returns a boolean indicating
	 * if the constraint was applied or not.
	 *
	 * @param {IKJoint} joint
	 * @private
	 * @return {boolean}
	 */
	_apply(joint) {
		// Get direction of joint and parent in world space
		const direction = new three.Vector3().copy(joint._getDirection());
		const parentDirection = joint
			._localToWorldDirection(new three.Vector3().copy(Z_AXIS$1))
			.normalize();

		// Find the current angle between them
		const currentAngle = direction.angleTo(parentDirection) * RAD2DEG$1;

		if (this.angle / 2 < currentAngle) {
			direction.normalize();
			// Find the correction axis and rotate around that point to the
			// largest allowed angle
			const correctionAxis = new three.Vector3()
				.crossVectors(parentDirection, direction)
				.normalize();

			parentDirection.applyAxisAngle(correctionAxis, this.angle * DEG2RAD$1 * 0.5);
			joint._setDirection(parentDirection);
			return true
		}

		return false
	}
}

exports.BoneHelper = BoneHelper;
exports.IK = IK;
exports.IKBallConstraint = IKBallConstraint;
exports.IKChain = IKChain;
exports.IKHelper = IKHelper;
exports.IKHingeConstraint = IKHingeConstraint;
exports.IKJoint = IKJoint;
exports.getCentroid = getCentroid;
exports.getWorldDistance = getWorldDistance;
exports.getWorldPosition = getWorldPosition;
exports.setQuaternionFromDirection = setQuaternionFromDirection;
exports.transformPoint = transformPoint;
