(function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(_dereq_,module,exports){
/**
 *
 * FULL TILT
 * http://github.com/richtr/Full-Tilt
 *
 * A standalone DeviceOrientation + DeviceMotion JavaScript library that
 * normalises orientation sensor input, applies relevant screen orientation
 * transforms, returns Euler Angle, Quaternion and Rotation
 * Matrix representations back to web developers and provides conversion
 * between all supported orientation representation types.
 *
 * Copyright: 2014 Rich Tibbett
 * License:   MIT
 *
 */

var Promise = _dereq_('es6-promise').Promise;

var M_PI   = Math.PI;
var M_PI_2 = M_PI / 2;
var M_2_PI = 2 * M_PI;

// Degree to Radian conversion
var degToRad = M_PI / 180;
var radToDeg = 180 / M_PI;

// Internal device orientation + motion variables
var sensors = {
	"orientation": {
		active:    false,
		callbacks: [],
		data:      undefined
	},
	"motion": {
		active:    false,
		callbacks: [],
		data:      undefined
	}
};
var screenActive = false;

// Internal screen orientation variables
var hasScreenOrientationAPI = window.screen && window.screen.orientation && window.screen.orientation.angle !== undefined && window.screen.orientation.angle !== null ? true : false;
var screenOrientationAngle = ( hasScreenOrientationAPI ? window.screen.orientation.angle : ( window.orientation || 0 ) ) * degToRad;

var SCREEN_ROTATION_0        = 0,
    SCREEN_ROTATION_90       = M_PI_2,
    SCREEN_ROTATION_180      = M_PI,
    SCREEN_ROTATION_270      = M_2_PI / 3,
    SCREEN_ROTATION_MINUS_90 = - M_PI_2;

// Math.sign polyfill
function sign(x) {
	x = +x; // convert to a number
	if (x === 0 || isNaN(x))
		return x;
	return x > 0 ? 1 : -1;
}

///// Promise-based Sensor Data checker //////

function SensorCheck(sensorRootObj) {

	var promise = new Promise(function(resolve, reject) {

		var runCheck = function (tries) {

			setTimeout(function() {

				if (sensorRootObj && sensorRootObj.data) {

					resolve();

				} else if (tries >= 5) {

					reject();

				} else {

					runCheck(++tries);

				}

			}, 50);

		};

		runCheck(0);

	});

	return promise;

}

////// Internal Event Handlers //////

function handleScreenOrientationChange () {

	if ( hasScreenOrientationAPI ) {

		screenOrientationAngle = ( window.screen.orientation.angle || 0 ) * degToRad;

	} else {

		screenOrientationAngle = ( window.orientation || 0 ) * degToRad;

	}

}

function handleDeviceOrientationChange ( event ) {

	sensors.orientation.data = event;

	// Fire every callback function each time deviceorientation is updated
	for ( var i in sensors.orientation.callbacks ) {

		sensors.orientation.callbacks[ i ].call( this );

	}

}

function handleDeviceMotionChange ( event ) {

	sensors.motion.data = event;

	// Fire every callback function each time devicemotion is updated
	for ( var i in sensors.motion.callbacks ) {

		sensors.motion.callbacks[ i ].call( this );

	}

}

///// FULLTILT API Root Object /////

var FULLTILT = {};

FULLTILT.version = "0.5.1";

///// FULLTILT API Root Methods /////

FULLTILT.getDeviceOrientation = function(options) {

	var promise = new Promise(function(resolve, reject) {

		var control = new FULLTILT.DeviceOrientation(options);

		control.start();

		var orientationSensorCheck = new SensorCheck(sensors.orientation);

		orientationSensorCheck.then(function() {

			control._alphaAvailable = (sensors.orientation.data.alpha && sensors.orientation.data.alpha !== null);
			control._betaAvailable = (sensors.orientation.data.beta && sensors.orientation.data.beta !== null);
			control._gammaAvailable = (sensors.orientation.data.gamma && sensors.orientation.data.gamma !== null);

			resolve(control);

		}).catch(function() {

			control.stop();
			reject('DeviceOrientation is not supported');

		});

	});

	return promise;

};

FULLTILT.getDeviceMotion = function(options) {

	var promise = new Promise(function(resolve, reject) {

		var control = new FULLTILT.DeviceMotion(options);

		control.start();

		var motionSensorCheck = new SensorCheck(sensors.motion);

		motionSensorCheck.then(function() {

			control._accelerationXAvailable = (sensors.motion.data.acceleration && sensors.motion.data.acceleration.x);
			control._accelerationYAvailable = (sensors.motion.data.acceleration && sensors.motion.data.acceleration.y);
			control._accelerationZAvailable = (sensors.motion.data.acceleration && sensors.motion.data.acceleration.z);

			control._accelerationIncludingGravityXAvailable = (sensors.motion.data.accelerationIncludingGravity && sensors.motion.data.accelerationIncludingGravity.x);
			control._accelerationIncludingGravityYAvailable = (sensors.motion.data.accelerationIncludingGravity && sensors.motion.data.accelerationIncludingGravity.y);
			control._accelerationIncludingGravityZAvailable = (sensors.motion.data.accelerationIncludingGravity && sensors.motion.data.accelerationIncludingGravity.z);

			control._rotationRateAlphaAvailable = (sensors.motion.data.rotationRate && sensors.motion.data.rotationRate.alpha);
			control._rotationRateBetaAvailable = (sensors.motion.data.rotationRate && sensors.motion.data.rotationRate.beta);
			control._rotationRateGammaAvailable = (sensors.motion.data.rotationRate && sensors.motion.data.rotationRate.gamma);

			resolve(control);

		}).catch(function() {

			control.stop();
			reject('DeviceMotion is not supported');

		});

	});

	return promise;

};

////// FULLTILT.Quaternion //////

FULLTILT.Quaternion = function ( x, y, z, w ) {

	var quat, outQuat;

	this.set = function ( x, y, z, w ) {

		this.x = x || 0;
		this.y = y || 0;
		this.z = z || 0;
		this.w = w || 1;

	};

	this.copy = function ( quaternion ) {

		this.x = quaternion.x;
		this.y = quaternion.y;
		this.z = quaternion.z;
		this.w = quaternion.w;

	};

	this.setFromEuler = (function () {

		var _x, _y, _z;
		var _x_2, _y_2, _z_2;
		var cX, cY, cZ, sX, sY, sZ;

		return function ( euler ) {

			euler = euler || {};

			_z = ( euler.alpha || 0 ) * degToRad;
			_x = ( euler.beta || 0 ) * degToRad;
			_y = ( euler.gamma || 0 ) * degToRad;

			_z_2 = _z / 2;
			_x_2 = _x / 2;
			_y_2 = _y / 2;

			cX = Math.cos( _x_2 );
			cY = Math.cos( _y_2 );
			cZ = Math.cos( _z_2 );
			sX = Math.sin( _x_2 );
			sY = Math.sin( _y_2 );
			sZ = Math.sin( _z_2 );

			this.set(
				sX * cY * cZ - cX * sY * sZ, // x
				cX * sY * cZ + sX * cY * sZ, // y
				cX * cY * sZ + sX * sY * cZ, // z
				cX * cY * cZ - sX * sY * sZ  // w
			);

			this.normalize();

			return this;

		};

	})();

	this.setFromRotationMatrix = (function () {

		var R;

		return function( matrix ) {

			R = matrix.elements;

			this.set(
				0.5 * Math.sqrt( 1 + R[0] - R[4] - R[8] ) * sign( R[7] - R[5] ), // x
				0.5 * Math.sqrt( 1 - R[0] + R[4] - R[8] ) * sign( R[2] - R[6] ), // y
				0.5 * Math.sqrt( 1 - R[0] - R[4] + R[8] ) * sign( R[3] - R[1] ), // z
				0.5 * Math.sqrt( 1 + R[0] + R[4] + R[8] )                        // w
			);

			return this;

		};

	})();

	this.multiply = function ( quaternion ) {

		outQuat = FULLTILT.Quaternion.prototype.multiplyQuaternions( this, quaternion );
		this.copy( outQuat );

		return this;

	};

	this.rotateX = function ( angle ) {

		outQuat = FULLTILT.Quaternion.prototype.rotateByAxisAngle( this, [ 1, 0, 0 ], angle );
		this.copy( outQuat );

		return this;

	};

	this.rotateY = function ( angle ) {

		outQuat = FULLTILT.Quaternion.prototype.rotateByAxisAngle( this, [ 0, 1, 0 ], angle );
		this.copy( outQuat );

		return this;

	};

	this.rotateZ = function ( angle ) {

		outQuat = FULLTILT.Quaternion.prototype.rotateByAxisAngle( this, [ 0, 0, 1 ], angle );
		this.copy( outQuat );

		return this;

	};

	this.normalize = function () {

		return FULLTILT.Quaternion.prototype.normalize( this );

	};

	// Initialize object values
	this.set( x, y, z, w );

};

FULLTILT.Quaternion.prototype = {

	constructor: FULLTILT.Quaternion,

	multiplyQuaternions: function () {

		var multipliedQuat = new FULLTILT.Quaternion();

		return function ( a, b ) {

			var qax = a.x, qay = a.y, qaz = a.z, qaw = a.w;
			var qbx = b.x, qby = b.y, qbz = b.z, qbw = b.w;

			multipliedQuat.set(
				qax * qbw + qaw * qbx + qay * qbz - qaz * qby, // x
				qay * qbw + qaw * qby + qaz * qbx - qax * qbz, // y
				qaz * qbw + qaw * qbz + qax * qby - qay * qbx, // z
				qaw * qbw - qax * qbx - qay * qby - qaz * qbz  // w
			);

			return multipliedQuat;

		};

	}(),

	normalize: function( q ) {

		var len = Math.sqrt( q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w );

		if ( len === 0 ) {

			q.x = 0;
			q.y = 0;
			q.z = 0;
			q.w = 1;

		} else {

			len = 1 / len;

			q.x *= len;
			q.y *= len;
			q.z *= len;
			q.w *= len;

		}

		return q;

	},

	rotateByAxisAngle: function () {

		var outputQuaternion = new FULLTILT.Quaternion();
		var transformQuaternion = new FULLTILT.Quaternion();

		var halfAngle, sA;

		return function ( targetQuaternion, axis, angle ) {

			halfAngle = ( angle || 0 ) / 2;
			sA = Math.sin( halfAngle );

			transformQuaternion.set(
				( axis[ 0 ] || 0 ) * sA, // x
				( axis[ 1 ] || 0 ) * sA, // y
				( axis[ 2 ] || 0 ) * sA, // z
				Math.cos( halfAngle )    // w
			);

			// Multiply quaternion by q
			outputQuaternion = FULLTILT.Quaternion.prototype.multiplyQuaternions( targetQuaternion, transformQuaternion );

			return FULLTILT.Quaternion.prototype.normalize( outputQuaternion );

		};

	}()

};

////// FULLTILT.RotationMatrix //////

FULLTILT.RotationMatrix = function ( m11, m12, m13, m21, m22, m23, m31, m32, m33 ) {

	var outMatrix;

	this.elements = new Float32Array( 9 );

	this.identity = function () {

		this.set(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		);

		return this;

	};

	this.set = function ( m11, m12, m13, m21, m22, m23, m31, m32, m33 ) {

		this.elements[ 0 ] = m11 || 1;
		this.elements[ 1 ] = m12 || 0;
		this.elements[ 2 ] = m13 || 0;
		this.elements[ 3 ] = m21 || 0;
		this.elements[ 4 ] = m22 || 1;
		this.elements[ 5 ] = m23 || 0;
		this.elements[ 6 ] = m31 || 0;
		this.elements[ 7 ] = m32 || 0;
		this.elements[ 8 ] = m33 || 1;

	};

	this.copy = function ( matrix ) {

		this.elements[ 0 ] = matrix.elements[ 0 ];
		this.elements[ 1 ] = matrix.elements[ 1 ];
		this.elements[ 2 ] = matrix.elements[ 2 ];
		this.elements[ 3 ] = matrix.elements[ 3 ];
		this.elements[ 4 ] = matrix.elements[ 4 ];
		this.elements[ 5 ] = matrix.elements[ 5 ];
		this.elements[ 6 ] = matrix.elements[ 6 ];
		this.elements[ 7 ] = matrix.elements[ 7 ];
		this.elements[ 8 ] = matrix.elements[ 8 ];

	};

	this.setFromEuler = (function() {

		var _x, _y, _z;
		var cX, cY, cZ, sX, sY, sZ;

		return function ( euler ) {

			euler = euler || {};

			_z = ( euler.alpha || 0 ) * degToRad;
			_x = ( euler.beta || 0 ) * degToRad;
			_y = ( euler.gamma || 0 ) * degToRad;

			cX = Math.cos( _x );
			cY = Math.cos( _y );
			cZ = Math.cos( _z );
			sX = Math.sin( _x );
			sY = Math.sin( _y );
			sZ = Math.sin( _z );

			//
			// ZXY-ordered rotation matrix construction.
			//

			this.set(
				cZ * cY - sZ * sX * sY, // 1,1
				- cX * sZ,              // 1,2
				cY * sZ * sX + cZ * sY, // 1,3

				cY * sZ + cZ * sX * sY, // 2,1
				cZ * cX,                // 2,2
				sZ * sY - cZ * cY * sX, // 2,3

				- cX * sY,              // 3,1
				sX,                     // 3,2
				cX * cY                 // 3,3
			);

			this.normalize();

			return this;

		};

	})();

	this.setFromQuaternion = (function() {

		var sqw, sqx, sqy, sqz;

		return function( q ) {

			sqw = q.w * q.w;
			sqx = q.x * q.x;
			sqy = q.y * q.y;
			sqz = q.z * q.z;

			this.set(
				sqw + sqx - sqy - sqz,       // 1,1
				2 * (q.x * q.y - q.w * q.z), // 1,2
				2 * (q.x * q.z + q.w * q.y), // 1,3

				2 * (q.x * q.y + q.w * q.z), // 2,1
				sqw - sqx + sqy - sqz,       // 2,2
				2 * (q.y * q.z - q.w * q.x), // 2,3

				2 * (q.x * q.z - q.w * q.y), // 3,1
				2 * (q.y * q.z + q.w * q.x), // 3,2
				sqw - sqx - sqy + sqz        // 3,3
			);

			return this;

		};

	})();

	this.multiply = function ( m ) {

		outMatrix = FULLTILT.RotationMatrix.prototype.multiplyMatrices( this, m );
		this.copy( outMatrix );

		return this;

	};

	this.rotateX = function ( angle ) {

		outMatrix = FULLTILT.RotationMatrix.prototype.rotateByAxisAngle( this, [ 1, 0, 0 ], angle );
		this.copy( outMatrix );

		return this;

	};

	this.rotateY = function ( angle ) {

		outMatrix = FULLTILT.RotationMatrix.prototype.rotateByAxisAngle( this, [ 0, 1, 0 ], angle );
		this.copy( outMatrix );

		return this;

	};

	this.rotateZ = function ( angle ) {

		outMatrix = FULLTILT.RotationMatrix.prototype.rotateByAxisAngle( this, [ 0, 0, 1 ], angle );
		this.copy( outMatrix );

		return this;

	};

	this.normalize = function () {

		return FULLTILT.RotationMatrix.prototype.normalize( this );

	};

	// Initialize object values
	this.set( m11, m12, m13, m21, m22, m23, m31, m32, m33 );

};

FULLTILT.RotationMatrix.prototype = {

	constructor: FULLTILT.RotationMatrix,

	multiplyMatrices: function () {

		var matrix = new FULLTILT.RotationMatrix();

		var aE, bE;

		return function ( a, b ) {

			aE = a.elements;
			bE = b.elements;

			matrix.set(
				aE[0] * bE[0] + aE[1] * bE[3] + aE[2] * bE[6],
				aE[0] * bE[1] + aE[1] * bE[4] + aE[2] * bE[7],
				aE[0] * bE[2] + aE[1] * bE[5] + aE[2] * bE[8],

				aE[3] * bE[0] + aE[4] * bE[3] + aE[5] * bE[6],
				aE[3] * bE[1] + aE[4] * bE[4] + aE[5] * bE[7],
				aE[3] * bE[2] + aE[4] * bE[5] + aE[5] * bE[8],

				aE[6] * bE[0] + aE[7] * bE[3] + aE[8] * bE[6],
				aE[6] * bE[1] + aE[7] * bE[4] + aE[8] * bE[7],
				aE[6] * bE[2] + aE[7] * bE[5] + aE[8] * bE[8]
			);

			return matrix;

		};

	}(),

	normalize: function( matrix ) {

		var R = matrix.elements;

		// Calculate matrix determinant
		var determinant = R[0] * R[4] * R[8] - R[0] * R[5] * R[7] - R[1] * R[3] * R[8] + R[1] * R[5] * R[6] + R[2] * R[3] * R[7] - R[2] * R[4] * R[6];

		// Normalize matrix values
		R[0] /= determinant;
		R[1] /= determinant;
		R[2] /= determinant;
		R[3] /= determinant;
		R[4] /= determinant;
		R[5] /= determinant;
		R[6] /= determinant;
		R[7] /= determinant;
		R[8] /= determinant;

		matrix.elements = R;

		return matrix;

	},

	rotateByAxisAngle: function () {

		var outputMatrix = new FULLTILT.RotationMatrix();
		var transformMatrix = new FULLTILT.RotationMatrix();

		var sA, cA;
		var validAxis = false;

		return function ( targetRotationMatrix, axis, angle ) {

			transformMatrix.identity(); // reset transform matrix

			validAxis = false;

			sA = Math.sin( angle );
			cA = Math.cos( angle );

			if ( axis[ 0 ] === 1 && axis[ 1 ] === 0 && axis[ 2 ] === 0 ) { // x

				validAxis = true;

				transformMatrix.elements[4] = cA;
				transformMatrix.elements[5] = -sA;
				transformMatrix.elements[7] = sA;
				transformMatrix.elements[8] = cA;

	 		} else if ( axis[ 1 ] === 1 && axis[ 0 ] === 0 && axis[ 2 ] === 0 ) { // y

				validAxis = true;

				transformMatrix.elements[0] = cA;
				transformMatrix.elements[2] = sA;
				transformMatrix.elements[6] = -sA;
				transformMatrix.elements[8] = cA;

	 		} else if ( axis[ 2 ] === 1 && axis[ 0 ] === 0 && axis[ 1 ] === 0 ) { // z

				validAxis = true;

				transformMatrix.elements[0] = cA;
				transformMatrix.elements[1] = -sA;
				transformMatrix.elements[3] = sA;
				transformMatrix.elements[4] = cA;

	 		}

			if ( validAxis ) {

				outputMatrix = FULLTILT.RotationMatrix.prototype.multiplyMatrices( targetRotationMatrix, transformMatrix );

				outputMatrix = FULLTILT.RotationMatrix.prototype.normalize( outputMatrix );

			} else {

				outputMatrix = targetRotationMatrix;

			}

			return outputMatrix;

		};

	}()

};

////// FULLTILT.Euler //////

FULLTILT.Euler = function ( alpha, beta, gamma ) {

	this.set = function ( alpha, beta, gamma ) {

		this.alpha = alpha || 0;
		this.beta  = beta  || 0;
		this.gamma = gamma || 0;

	};

	this.copy = function ( inEuler ) {

		this.alpha = inEuler.alpha;
		this.beta  = inEuler.beta;
		this.gamma = inEuler.gamma;

	};

	this.setFromRotationMatrix = (function () {

		var R, _alpha, _beta, _gamma;

		return function ( matrix ) {

			R = matrix.elements;

			if (R[8] > 0) { // cos(beta) > 0

				_alpha = Math.atan2(-R[1], R[4]);
				_beta  = Math.asin(R[7]); // beta (-pi/2, pi/2)
				_gamma = Math.atan2(-R[6], R[8]); // gamma (-pi/2, pi/2)

			} else if (R[8] < 0) {  // cos(beta) < 0

				_alpha = Math.atan2(R[1], -R[4]);
				_beta  = -Math.asin(R[7]);
				_beta  += (_beta >= 0) ? - M_PI : M_PI; // beta [-pi,-pi/2) U (pi/2,pi)
				_gamma = Math.atan2(R[6], -R[8]); // gamma (-pi/2, pi/2)

			} else { // R[8] == 0

				if (R[6] > 0) {  // cos(gamma) == 0, cos(beta) > 0

					_alpha = Math.atan2(-R[1], R[4]);
					_beta  = Math.asin(R[7]); // beta [-pi/2, pi/2]
					_gamma = - M_PI_2; // gamma = -pi/2

				} else if (R[6] < 0) { // cos(gamma) == 0, cos(beta) < 0

					_alpha = Math.atan2(R[1], -R[4]);
					_beta  = -Math.asin(R[7]);
					_beta  += (_beta >= 0) ? - M_PI : M_PI; // beta [-pi,-pi/2) U (pi/2,pi)
					_gamma = - M_PI_2; // gamma = -pi/2

				} else { // R[6] == 0, cos(beta) == 0

					// gimbal lock discontinuity
					_alpha = Math.atan2(R[3], R[0]);
					_beta  = (R[7] > 0) ? M_PI_2 : - M_PI_2; // beta = +-pi/2
					_gamma = 0; // gamma = 0

				}

			}

			// alpha is in [-pi, pi], make sure it is in [0, 2*pi).
			if (_alpha < 0) {
				_alpha += M_2_PI; // alpha [0, 2*pi)
			}

			// Convert to degrees
			_alpha *= radToDeg;
			_beta  *= radToDeg;
			_gamma *= radToDeg;

			// apply derived euler angles to current object
			this.set( _alpha, _beta, _gamma );

		};

	})();

	this.setFromQuaternion = (function () {

		var _alpha, _beta, _gamma;

		return function ( q ) {

			var sqw = q.w * q.w;
			var sqx = q.x * q.x;
			var sqy = q.y * q.y;
			var sqz = q.z * q.z;

			var unitLength = sqw + sqx + sqy + sqz; // Normalised == 1, otherwise correction divisor.
			var wxyz = q.w * q.x + q.y * q.z;
			var epsilon = 1e-6; // rounding factor

			if (wxyz > (0.5 - epsilon) * unitLength) {

				_alpha = 2 * Math.atan2(q.y, q.w);
				_beta = M_PI_2;
				_gamma = 0;

			} else if (wxyz < (-0.5 + epsilon) * unitLength) {

				_alpha = -2 * Math.atan2(q.y, q.w);
				_beta = -M_PI_2;
				_gamma = 0;

			} else {

				var aX = sqw - sqx + sqy - sqz;
				var aY = 2 * (q.w * q.z - q.x * q.y);

				var gX = sqw - sqx - sqy + sqz;
				var gY = 2 * (q.w * q.y - q.x * q.z);

				if (gX > 0) {

					_alpha = Math.atan2(aY, aX);
					_beta  = Math.asin(2 * wxyz / unitLength);
					_gamma = Math.atan2(gY, gX);

				} else {

					_alpha = Math.atan2(-aY, -aX);
					_beta  = -Math.asin(2 * wxyz / unitLength);
					_beta  += _beta < 0 ? M_PI : - M_PI;
					_gamma = Math.atan2(-gY, -gX);

				}

			}

			// alpha is in [-pi, pi], make sure it is in [0, 2*pi).
			if (_alpha < 0) {
				_alpha += M_2_PI; // alpha [0, 2*pi)
			}

			// Convert to degrees
			_alpha *= radToDeg;
			_beta  *= radToDeg;
			_gamma *= radToDeg;

			// apply derived euler angles to current object
			this.set( _alpha, _beta, _gamma );

		};

	})();

	this.rotateX = function ( angle ) {

		FULLTILT.Euler.prototype.rotateByAxisAngle( this, [ 1, 0, 0 ], angle );

		return this;

	};

	this.rotateY = function ( angle ) {

		FULLTILT.Euler.prototype.rotateByAxisAngle( this, [ 0, 1, 0 ], angle );

		return this;

	};

	this.rotateZ = function ( angle ) {

		FULLTILT.Euler.prototype.rotateByAxisAngle( this, [ 0, 0, 1 ], angle );

		return this;

	};

	// Initialize object values
	this.set( alpha, beta, gamma );

};

FULLTILT.Euler.prototype = {

	constructor: FULLTILT.Euler,

	rotateByAxisAngle: function () {

		var _matrix = new FULLTILT.RotationMatrix();
		var outEuler;

		return function ( targetEuler, axis, angle ) {

			_matrix.setFromEuler( targetEuler );

			_matrix = FULLTILT.RotationMatrix.prototype.rotateByAxisAngle( _matrix, axis, angle );

			targetEuler.setFromRotationMatrix( _matrix );

			return targetEuler;

		};

	}()

};

///// FULLTILT.DeviceOrientation //////

FULLTILT.DeviceOrientation = function (options) {

	this.options = options || {}; // by default use UA deviceorientation 'type' ("game" on iOS, "world" on Android)

	var tries = 0;
	var maxTries = 200;
	var successCount = 0;
	var successThreshold = 10;

	this.alphaOffsetScreen = 0;
	this.alphaOffsetDevice = undefined;

	// Create a game-based deviceorientation object (initial alpha === 0 degrees)
	if (this.options.type === "game") {

		var setGameAlphaOffset = function(evt) {

			if (evt.alpha !== null) { // do regardless of whether 'evt.absolute' is also true
				this.alphaOffsetDevice = new FULLTILT.Euler(evt.alpha, 0, 0);
				this.alphaOffsetDevice.rotateZ( -screenOrientationAngle );

				// Discard first {successThreshold} responses while a better compass lock is found by UA
				if(++successCount >= successThreshold) {
					window.removeEventListener( 'deviceorientation', setGameAlphaOffset, false );
					return;
				}
			}

			if(++tries >= maxTries) {
				window.removeEventListener( 'deviceorientation', setGameAlphaOffset, false );
			}

		}.bind(this);

		window.addEventListener( 'deviceorientation', setGameAlphaOffset, false );

	// Create a compass-based deviceorientation object (initial alpha === compass degrees)
	} else if (this.options.type === "world") {

		var setCompassAlphaOffset = function(evt) {

			if (evt.absolute !== true && evt.webkitCompassAccuracy !== undefined && evt.webkitCompassAccuracy !== null && +evt.webkitCompassAccuracy >= 0 && +evt.webkitCompassAccuracy < 50) {
				this.alphaOffsetDevice = new FULLTILT.Euler(evt.webkitCompassHeading, 0, 0);
				this.alphaOffsetDevice.rotateZ( screenOrientationAngle );
				this.alphaOffsetScreen = screenOrientationAngle;

				// Discard first {successThreshold} responses while a better compass lock is found by UA
				if(++successCount >= successThreshold) {
					window.removeEventListener( 'deviceorientation', setCompassAlphaOffset, false );
					return;
				}
			}

			if(++tries >= maxTries) {
				window.removeEventListener( 'deviceorientation', setCompassAlphaOffset, false );
			}

		}.bind(this);

		window.addEventListener( 'deviceorientation', setCompassAlphaOffset, false );

	} // else... use whatever orientation system the UA provides ("game" on iOS, "world" on Android)

};

FULLTILT.DeviceOrientation.prototype = {

	constructor: FULLTILT.DeviceOrientation,

	start: function ( callback ) {

		if ( callback && Object.prototype.toString.call( callback ) == '[object Function]' ) {

			sensors.orientation.callbacks.push( callback );

		}

		if( !screenActive ) {

			if ( hasScreenOrientationAPI ) {

			window.screen.orientation.addEventListener( 'change', handleScreenOrientationChange, false );

			} else {

				window.addEventListener( 'orientationchange', handleScreenOrientationChange, false );

			}

		}

		if ( !sensors.orientation.active ) {

			window.addEventListener( 'deviceorientation', handleDeviceOrientationChange, false );

			sensors.orientation.active = true;

		}

	},

	stop: function () {

		if ( sensors.orientation.active ) {

			window.removeEventListener( 'deviceorientation', handleDeviceOrientationChange, false );

			sensors.orientation.active = false;

		}

	},

	listen: function( callback ) {

		this.start( callback );

	},

	getFixedFrameQuaternion: (function () {

		var euler = new FULLTILT.Euler();
		var matrix = new FULLTILT.RotationMatrix();
		var quaternion = new FULLTILT.Quaternion();

		return function() {

			var orientationData = sensors.orientation.data || { alpha: 0, beta: 0, gamma: 0 };

			var adjustedAlpha = orientationData.alpha;

			if (this.alphaOffsetDevice) {
				matrix.setFromEuler( this.alphaOffsetDevice );
				matrix.rotateZ( - this.alphaOffsetScreen );
				euler.setFromRotationMatrix( matrix );

				if (euler.alpha < 0) {
					euler.alpha += 360;
				}

				adjustedAlpha -= euler.alpha;
			}

			euler.set(
				adjustedAlpha,
				orientationData.beta,
				orientationData.gamma
			);

			quaternion.setFromEuler( euler );

			return quaternion;

		};

	})(),

	getScreenAdjustedQuaternion: (function () {

		var quaternion;

		return function() {

			quaternion = this.getFixedFrameQuaternion();

			// Automatically apply screen orientation transform
			quaternion.rotateZ( - screenOrientationAngle );

			return quaternion;

		};

	})(),

	getFixedFrameMatrix: (function () {

		var euler = new FULLTILT.Euler();
		var matrix = new FULLTILT.RotationMatrix();

		return function () {

			var orientationData = sensors.orientation.data || { alpha: 0, beta: 0, gamma: 0 };

			var adjustedAlpha = orientationData.alpha;

			if (this.alphaOffsetDevice) {
				matrix.setFromEuler( this.alphaOffsetDevice );
				matrix.rotateZ( - this.alphaOffsetScreen );
				euler.setFromRotationMatrix( matrix );

				if (euler.alpha < 0) {
					euler.alpha += 360;
				}

				adjustedAlpha -= euler.alpha;
			}

			euler.set(
				adjustedAlpha,
				orientationData.beta,
				orientationData.gamma
			);

			matrix.setFromEuler( euler );

			return matrix;

		};

	})(),

	getScreenAdjustedMatrix: (function () {

		var matrix;

		return function () {

			matrix = this.getFixedFrameMatrix();

			// Automatically apply screen orientation transform
			matrix.rotateZ( - screenOrientationAngle );

			return matrix;

		};

	})(),

	getFixedFrameEuler: (function () {

		var euler = new FULLTILT.Euler();
		var matrix;

		return function () {

			matrix = this.getFixedFrameMatrix();

			euler.setFromRotationMatrix( matrix );

			return euler;

		};

	})(),

	getScreenAdjustedEuler: (function () {

		var euler = new FULLTILT.Euler();
		var matrix;

		return function () {

			matrix = this.getScreenAdjustedMatrix();

			euler.setFromRotationMatrix( matrix );

			return euler;

		};

	})(),

	isAbsolute: function () {

		if ( sensors.orientation.data && sensors.orientation.data.absolute === true ) {
			return true;
		}

		return false;

	},

	getLastRawEventData: function () {

		return sensors.orientation.data || {};

	},

	_alphaAvailable: false,
	_betaAvailable: false,
	_gammaAvailable: false, 

	isAvailable: function(_valueType){

		switch(_valueType){
			case this.ALPHA:
				return this._alphaAvailable;

			case this.BETA:
				return this._betaAvailable;

			case this.GAMMA:
				return this._gammaAvailable;
		}
		
	},

	ALPHA: 'alpha',
	BETA: 'beta',
	GAMMA: 'gamma'

};

///// FULLTILT.DeviceMotion //////

FULLTILT.DeviceMotion = function (options) {

	this.options = options || {}; // placeholder object since no options are currently supported

};

FULLTILT.DeviceMotion.prototype = {

	constructor: FULLTILT.DeviceMotion,

	start: function ( callback ) {

		if ( callback && Object.prototype.toString.call( callback ) == '[object Function]' ) {

			sensors.motion.callbacks.push( callback );

		}

		if( !screenActive ) {

			if ( hasScreenOrientationAPI ) {

				window.screen.orientation.addEventListener( 'change', handleScreenOrientationChange, false );

			} else {

				window.addEventListener( 'orientationchange', handleScreenOrientationChange, false );

			}

		}

		if ( !sensors.motion.active ) {

			window.addEventListener( 'devicemotion', handleDeviceMotionChange, false );

			sensors.motion.active = true;

		}

	},

	stop: function () {

		if ( sensors.motion.active ) {

			window.removeEventListener( 'devicemotion', handleDeviceMotionChange, false );

			sensors.motion.active = false;

		}

	},

	listen: function( callback ) {

		this.start( callback );

	},

	getScreenAdjustedAcceleration: function () {

		var accData = sensors.motion.data && sensors.motion.data.acceleration ? sensors.motion.data.acceleration : { x: 0, y: 0, z: 0 };
		var screenAccData = {};

		switch ( screenOrientationAngle ) {
			case SCREEN_ROTATION_90:
				screenAccData.x = - accData.y;
				screenAccData.y =   accData.x;
				break;
			case SCREEN_ROTATION_180:
				screenAccData.x = - accData.x;
				screenAccData.y = - accData.y;
				break;
			case SCREEN_ROTATION_270:
			case SCREEN_ROTATION_MINUS_90:
				screenAccData.x =   accData.y;
				screenAccData.y = - accData.x;
				break;
			default: // SCREEN_ROTATION_0
				screenAccData.x =   accData.x;
				screenAccData.y =   accData.y;
				break;
		}

		screenAccData.z = accData.z;

		return screenAccData;

	},

	getScreenAdjustedAccelerationIncludingGravity: function () {

		var accGData = sensors.motion.data && sensors.motion.data.accelerationIncludingGravity ? sensors.motion.data.accelerationIncludingGravity : { x: 0, y: 0, z: 0 };
		var screenAccGData = {};

		switch ( screenOrientationAngle ) {
			case SCREEN_ROTATION_90:
				screenAccGData.x = - accGData.y;
				screenAccGData.y =   accGData.x;
				break;
			case SCREEN_ROTATION_180:
				screenAccGData.x = - accGData.x;
				screenAccGData.y = - accGData.y;
				break;
			case SCREEN_ROTATION_270:
			case SCREEN_ROTATION_MINUS_90:
				screenAccGData.x =   accGData.y;
				screenAccGData.y = - accGData.x;
				break;
			default: // SCREEN_ROTATION_0
				screenAccGData.x =   accGData.x;
				screenAccGData.y =   accGData.y;
				break;
		}

		screenAccGData.z = accGData.z;

		return screenAccGData;

	},

	getScreenAdjustedRotationRate: function () {

		var rotRateData = sensors.motion.data && sensors.motion.data.rotationRate ? sensors.motion.data.rotationRate : { alpha: 0, beta: 0, gamma: 0 };
		var screenRotRateData = {};

		switch ( screenOrientationAngle ) {
			case SCREEN_ROTATION_90:
				screenRotRateData.beta  = - rotRateData.gamma;
				screenRotRateData.gamma =   rotRateData.beta;
				break;
			case SCREEN_ROTATION_180:
				screenRotRateData.beta  = - rotRateData.beta;
				screenRotRateData.gamma = - rotRateData.gamma;
				break;
			case SCREEN_ROTATION_270:
			case SCREEN_ROTATION_MINUS_90:
				screenRotRateData.beta  =   rotRateData.gamma;
				screenRotRateData.gamma = - rotRateData.beta;
				break;
			default: // SCREEN_ROTATION_0
				screenRotRateData.beta  =   rotRateData.beta;
				screenRotRateData.gamma =   rotRateData.gamma;
				break;
		}

		screenRotRateData.alpha = rotRateData.alpha;

		return screenRotRateData;

	},

	getLastRawEventData: function () {

		return sensors.motion.data || {};

	},

	_accelerationXAvailable: false,
	_accelerationYAvailable: false,
	_accelerationZAvailable: false,

	_accelerationIncludingGravityXAvailable: false,
	_accelerationIncludingGravityYAvailable: false,
	_accelerationIncludingGravityZAvailable: false,

	_rotationRateAlphaAvailable: false,
	_rotationRateBetaAvailable: false,
	_rotationRateGammaAvailable: false,

	isAvailable: function(_valueType){

		switch(_valueType){
			case this.ACCELERATION_X:
				return this._accelerationXAvailable;

			case this.ACCELERATION_Y:
				return this._accelerationYAvailable;

			case this.ACCELERATION_Z:
				return this._accelerationZAvailable;

			case this.ACCELERATION_INCLUDING_GRAVITY_X:
				return this._accelerationIncludingGravityXAvailable;

			case this.ACCELERATION_INCLUDING_GRAVITY_Y:
				return this._accelerationIncludingGravityYAvailable;

			case this.ACCELERATION_INCLUDING_GRAVITY_Z:
				return this._accelerationIncludingGravityZAvailable;

			case this.ROTATION_RATE_ALPHA:
				return this._rotationRateAlphaAvailable;

			case this.ROTATION_RATE_BETA:
				return this._rotationRateBetaAvailable;

			case this.ROTATION_RATE_GAMMA:
				return this._rotationRateGammaAvailable;
		}
	},

	ACCELERATION_X: 'accelerationX',
	ACCELERATION_Y: 'accelerationY',
	ACCELERATION_Z: 'accelerationZ',

	ACCELERATION_INCLUDING_GRAVITY_X: 'accelerationIncludingGravityX',
	ACCELERATION_INCLUDING_GRAVITY_Y: 'accelerationIncludingGravityY',
	ACCELERATION_INCLUDING_GRAVITY_Z: 'accelerationIncludingGravityZ',

	ROTATION_RATE_ALPHA: 'rotationRateAlpha',
	ROTATION_RATE_BETA: 'rotationRateBeta',
	ROTATION_RATE_GAMMA: 'rotationRateGamma'

};

module.exports = FULLTILT;

},{"es6-promise":4}],2:[function(_dereq_,module,exports){
/**
* JavaScript project for accessing and normalizing the accelerometer and gyroscope data on mobile devices
*
* @author Doruk Eker <dorukeker@gmail.com>
* @copyright Doruk Eker <http://dorukeker.com>
* @version 2.0.4
* @license MIT License | http://opensource.org/licenses/MIT
*/

var Promise = _dereq_('es6-promise').Promise;
var FULLTILT = _dereq_("./fulltilt.js");

(function(root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(function() {
      return (root.GyroNorm = factory());
    });
  } else if (typeof module === 'object' && module.exports) {
    module.exports = (root.GyroNorm = factory());
  } else {
    root.GyroNorm = factory();
  }
}(this, function() {
  /* Constants */
  var GAME                            = 'game';
  var WORLD                           = 'world';
  var DEVICE_ORIENTATION              = 'deviceorientation';
  var ACCELERATION                    = 'acceleration';
  var ACCELERATION_INCLUDING_GRAVITY  = 'accelerationinludinggravity';
  var ROTATION_RATE                   = 'rotationrate';

  /*-------------------------------------------------------*/
  /* PRIVATE VARIABLES */

  var _interval           = null;       // Timer to return values
  var _isCalibrating      = false;      // Flag if calibrating
  var _calibrationValue   = 0;          // Alpha offset value
  var _gravityCoefficient = 0;          // Coefficient to normalze gravity related values
  var _isRunning          = false;      // Boolean value if GyroNorm is tracking
  var _isReady            = false;      // Boolean value if GyroNorm is is initialized

  var _do                 = null;       // Object to store the device orientation values
  var _dm                 = null;       // Object to store the device motion values

  /* OPTIONS */
  var _frequency          = 50;         // Frequency for the return data in milliseconds
  var _gravityNormalized  = true;       // Flag if to normalize gravity values
  var _orientationBase    = GAME;       // Can be GyroNorm.GAME or GyroNorm.WORLD. GyroNorm.GAME returns orientation values with respect to the head direction of the device. GyroNorm.WORLD returns the orientation values with respect to the actual north direction of the world.
  var _decimalCount       = 2;          // Number of digits after the decimals point for the return values
  var _logger             = null;       // Function to callback on error. There is no default value. It can only be set by the user on gn.init()
  var _screenAdjusted     = false;      // If set to true it will return screen adjusted values. (e.g. On a horizontal orientation of a mobile device, the head would be one of the sides, instead of  the actual head of the device.)

  var _values = {
    do: {
      alpha: 0,
      beta: 0,
      gamma: 0,
      absolute: false
    },
    dm: {
      x: 0,
      y: 0,
      z: 0,
      gx: 0,
      gy: 0,
      gz: 0,
      alpha: 0,
      beta: 0,
      gamma: 0
    }
  }

  /*-------------------------------------------------------*/
  /* PUBLIC FUNCTIONS */

  /*
  *
  * Constructor function
  *
  */

  var GyroNorm = function(options) {}

  /* Constants */
  GyroNorm.GAME                             = GAME;
  GyroNorm.WORLD                            = WORLD;
  GyroNorm.DEVICE_ORIENTATION               = DEVICE_ORIENTATION;
  GyroNorm.ACCELERATION                     = ACCELERATION;
  GyroNorm.ACCELERATION_INCLUDING_GRAVITY   = ACCELERATION_INCLUDING_GRAVITY;
  GyroNorm.ROTATION_RATE                    = ROTATION_RATE;

  /*
  *
  * Initialize GyroNorm instance function
  *
  * @param object options - values are as follows. If set in the init function they overwrite the default option values
  * @param int options.frequency
  * @param boolean options.gravityNormalized
  * @param boolean options.orientationBase
  * @param boolean options.decimalCount
  * @param function options.logger
  * @param function options.screenAdjusted
  *
  */

  GyroNorm.prototype.init = function(options) {
    // Assign options that are passed with the constructor function
    if (options && options.frequency) _frequency = options.frequency;
    if (options && options.gravityNormalized) _gravityNormalized = options.gravityNormalized;
    if (options && options.orientationBase) _orientationBase = options.orientationBase;
    if (options && typeof options.decimalCount === 'number' && options.decimalCount >= 0) _decimalCount = parseInt(options.decimalCount);
    if (options && options.logger) _logger = options.logger;
    if (options && options.screenAdjusted) _screenAdjusted = options.screenAdjusted;

    var deviceOrientationPromise = new FULLTILT.getDeviceOrientation({ 'type': _orientationBase }).then(function(controller) {
      _do = controller;
    });

    var deviceMotionPromise = new FULLTILT.getDeviceMotion().then(function(controller) {
      _dm = controller;
      // Set gravity coefficient
      _gravityCoefficient = (_dm.getScreenAdjustedAccelerationIncludingGravity().z > 0) ? -1 : 1;
    });

    return Promise.all([deviceOrientationPromise, deviceMotionPromise]).then(function() {
      _isReady = true;
    });
  }

  /*
  *
  * Stops all the tracking and listening on the window objects
  *
  */
  GyroNorm.prototype.end = function() {
    try {
      _isReady = false;
      this.stop();
      _dm.stop();
      _do.stop();
    } catch(err){
      log(err);
    }
  }

  /*
  *
  * Starts tracking the values
  *
  * @param function callback - Callback function to read the values
  *
  */
  GyroNorm.prototype.start = function(callback) {
    if (!_isReady) {
      log({ message: 'GyroNorm is not initialized yet. First call the "init()" function.', code: 1 });
      return;
    }

    _interval = setInterval(function() {
      callback(snapShot());
    }, _frequency);
    _isRunning = true;
  }

  /*
  *
  * Stops tracking the values
  *
  */
  GyroNorm.prototype.stop = function() {
    if (_interval) {
      clearInterval(_interval);
      _isRunning = false;
    }
  }

  /*
  *
  * Toggles if to normalize gravity related values
  *
  * @param boolean flag
  *
  */
  GyroNorm.prototype.normalizeGravity = function(flag) {
    _gravityNormalized = (flag) ? true : false;
  }


  /*
  *
  * Sets the current head direction as alpha = 0
  * Can only be used if device orientation is being tracked, values are not screen adjusted, value type is GyroNorm.EULER and orientation base is GyroNorm.GAME
  *
  * @return: If head direction is set successfully returns true, else false
  *
  */
  GyroNorm.prototype.setHeadDirection = function() {
    if (_screenAdjusted || _orientationBase === WORLD) {
      return false;
    }

    _calibrationValue = _do.getFixedFrameEuler().alpha;
    return true;
  }

  /*
  *
  * Sets the log function
  *
  */
  GyroNorm.prototype.startLogging = function(logger) {
    if (logger) {
      _logger = logger;
    }
  }

  /*
  *
  * Sets the log function to null which stops the logging
  *
  */
  GyroNorm.prototype.stopLogging = function() {
    _logger = null;
  }

  /*
  *
  * Returns if certain type of event is available on the device
  *
  * @param string _eventType - possible values are "deviceorientation" , "devicemotion" , "compassneedscalibration"
  *
  * @return true if event is available false if not
  *
  */
  GyroNorm.prototype.isAvailable = function(_eventType) {

    var doSnapShot = _do.getScreenAdjustedEuler();
    var accSnapShot = _dm.getScreenAdjustedAcceleration();
    var accGraSnapShot = _dm.getScreenAdjustedAccelerationIncludingGravity();
    var rotRateSnapShot = _dm.getScreenAdjustedRotationRate();

    switch (_eventType) {
      case DEVICE_ORIENTATION:
        return ((doSnapShot.alpha && doSnapShot.alpha !== null) && (doSnapShot.beta && doSnapShot.beta !== null) && (doSnapShot.gamma && doSnapShot.gamma !== null));
        break;

      case ACCELERATION:
        return (accSnapShot && accSnapShot.x && accSnapShot.y && accSnapShot.z);
        break;

      case ACCELERATION_INCLUDING_GRAVITY:
        return (accGraSnapShot && accGraSnapShot.x && accGraSnapShot.y && accGraSnapShot.z);
        break;

      case ROTATION_RATE:
        return (rotRateSnapShot && rotRateSnapShot.alpha && rotRateSnapShot.beta && rotRateSnapShot.gamma);
        break;

      default:
        return {
          deviceOrientationAvailable: ((doSnapShot.alpha && doSnapShot.alpha !== null) && (doSnapShot.beta && doSnapShot.beta !== null) && (doSnapShot.gamma && doSnapShot.gamma !== null)),
          accelerationAvailable: (accSnapShot && accSnapShot.x && accSnapShot.y && accSnapShot.z),
          accelerationIncludingGravityAvailable: (accGraSnapShot && accGraSnapShot.x && accGraSnapShot.y && accGraSnapShot.z),
          rotationRateAvailable: (rotRateSnapShot && rotRateSnapShot.alpha && rotRateSnapShot.beta && rotRateSnapShot.gamma)
        }
        break;
    }
  }

  /*
  *
  * Returns boolean value if the GyroNorm is running
  *
  */
  GyroNorm.prototype.isRunning = function() {
    return _isRunning;
  }

  /*-------------------------------------------------------*/
  /* PRIVATE FUNCTIONS */

  /*
  *
  * Utility function to round with digits after the decimal point
  *
  * @param float number - the original number to round
  *
  */
  function rnd(number) {
    return Math.round(number * Math.pow(10, _decimalCount)) / Math.pow(10, _decimalCount);
  }

  /*
  *
  * Starts calibration
  *
  */
  function calibrate() {
    _isCalibrating = true;
    _calibrationValues = new Array();
  }

  /*
  *
  * Takes a snapshot of the current deviceo orientaion and device motion values
  *
  */
  function snapShot() {
    var doSnapShot = {};

    if (_screenAdjusted) {
      doSnapShot = _do.getScreenAdjustedEuler();
    } else {
      doSnapShot = _do.getFixedFrameEuler();
    }

    var accSnapShot = _dm.getScreenAdjustedAcceleration();
    var accGraSnapShot = _dm.getScreenAdjustedAccelerationIncludingGravity();
    var rotRateSnapShot = _dm.getScreenAdjustedRotationRate();

    var alphaToSend = 0;

    if (_orientationBase === GAME) {
      alphaToSend = doSnapShot.alpha - _calibrationValue;
      alphaToSend = (alphaToSend < 0) ? (360 - Math.abs(alphaToSend)) : alphaToSend;
    } else {
      alphaToSend = doSnapShot.alpha;
    }

    var snapShot = {
      do: {
        alpha: rnd(alphaToSend),
        beta: rnd(doSnapShot.beta),
        gamma: rnd(doSnapShot.gamma),
        absolute: _do.isAbsolute()
      },
      dm: {
        x: rnd(accSnapShot.x),
        y: rnd(accSnapShot.y),
        z: rnd(accSnapShot.z),
        gx: rnd(accGraSnapShot.x),
        gy: rnd(accGraSnapShot.y),
        gz: rnd(accGraSnapShot.z),
        alpha: rnd(rotRateSnapShot.alpha),
        beta: rnd(rotRateSnapShot.beta),
        gamma: rnd(rotRateSnapShot.gamma),
        timeStamp: _dm.getLastRawEventData().timeStamp
      }
    };

    // Normalize gravity
    if (_gravityNormalized) {
      snapShot.dm.gx *= _gravityCoefficient;
      snapShot.dm.gy *= _gravityCoefficient;
      snapShot.dm.gz *= _gravityCoefficient;
    }

    return snapShot;
  }


  /*
  *
  * Starts listening to orientation event on the window object
  *
  */
  function log(err) {
    if (_logger) {
      if (typeof(err) == 'string') {
        err = { message: err, code: 0 }
      }
      _logger(err);
    }
  }

  return GyroNorm;
}));
},{"./fulltilt.js":1,"es6-promise":4}],3:[function(_dereq_,module,exports){
// shim for using process in browser

var process = module.exports = {};

// cached from whatever global is present so that test runners that stub it
// don't break things.  But we need to wrap it in a try catch in case it is
// wrapped in strict mode code which doesn't define any globals.  It's inside a
// function because try/catches deoptimize in certain engines.

var cachedSetTimeout;
var cachedClearTimeout;

(function () {
  try {
    cachedSetTimeout = setTimeout;
  } catch (e) {
    cachedSetTimeout = function () {
      throw new Error('setTimeout is not defined');
    }
  }
  try {
    cachedClearTimeout = clearTimeout;
  } catch (e) {
    cachedClearTimeout = function () {
      throw new Error('clearTimeout is not defined');
    }
  }
} ())
var queue = [];
var draining = false;
var currentQueue;
var queueIndex = -1;

function cleanUpNextTick() {
    if (!draining || !currentQueue) {
        return;
    }
    draining = false;
    if (currentQueue.length) {
        queue = currentQueue.concat(queue);
    } else {
        queueIndex = -1;
    }
    if (queue.length) {
        drainQueue();
    }
}

function drainQueue() {
    if (draining) {
        return;
    }
    var timeout = cachedSetTimeout(cleanUpNextTick);
    draining = true;

    var len = queue.length;
    while(len) {
        currentQueue = queue;
        queue = [];
        while (++queueIndex < len) {
            if (currentQueue) {
                currentQueue[queueIndex].run();
            }
        }
        queueIndex = -1;
        len = queue.length;
    }
    currentQueue = null;
    draining = false;
    cachedClearTimeout(timeout);
}

process.nextTick = function (fun) {
    var args = new Array(arguments.length - 1);
    if (arguments.length > 1) {
        for (var i = 1; i < arguments.length; i++) {
            args[i - 1] = arguments[i];
        }
    }
    queue.push(new Item(fun, args));
    if (queue.length === 1 && !draining) {
        cachedSetTimeout(drainQueue, 0);
    }
};

// v8 likes predictible objects
function Item(fun, array) {
    this.fun = fun;
    this.array = array;
}
Item.prototype.run = function () {
    this.fun.apply(null, this.array);
};
process.title = 'browser';
process.browser = true;
process.env = {};
process.argv = [];
process.version = ''; // empty string to avoid regexp issues
process.versions = {};

function noop() {}

process.on = noop;
process.addListener = noop;
process.once = noop;
process.off = noop;
process.removeListener = noop;
process.removeAllListeners = noop;
process.emit = noop;

process.binding = function (name) {
    throw new Error('process.binding is not supported');
};

process.cwd = function () { return '/' };
process.chdir = function (dir) {
    throw new Error('process.chdir is not supported');
};
process.umask = function() { return 0; };

},{}],4:[function(_dereq_,module,exports){
(function (process,global){
/*!
 * @overview es6-promise - a tiny implementation of Promises/A+.
 * @copyright Copyright (c) 2014 Yehuda Katz, Tom Dale, Stefan Penner and contributors (Conversion to ES6 API by Jake Archibald)
 * @license   Licensed under MIT license
 *            See https://raw.githubusercontent.com/jakearchibald/es6-promise/master/LICENSE
 * @version   3.2.1
 */

(function() {
    "use strict";
    function lib$es6$promise$utils$$objectOrFunction(x) {
      return typeof x === 'function' || (typeof x === 'object' && x !== null);
    }

    function lib$es6$promise$utils$$isFunction(x) {
      return typeof x === 'function';
    }

    function lib$es6$promise$utils$$isMaybeThenable(x) {
      return typeof x === 'object' && x !== null;
    }

    var lib$es6$promise$utils$$_isArray;
    if (!Array.isArray) {
      lib$es6$promise$utils$$_isArray = function (x) {
        return Object.prototype.toString.call(x) === '[object Array]';
      };
    } else {
      lib$es6$promise$utils$$_isArray = Array.isArray;
    }

    var lib$es6$promise$utils$$isArray = lib$es6$promise$utils$$_isArray;
    var lib$es6$promise$asap$$len = 0;
    var lib$es6$promise$asap$$vertxNext;
    var lib$es6$promise$asap$$customSchedulerFn;

    var lib$es6$promise$asap$$asap = function asap(callback, arg) {
      lib$es6$promise$asap$$queue[lib$es6$promise$asap$$len] = callback;
      lib$es6$promise$asap$$queue[lib$es6$promise$asap$$len + 1] = arg;
      lib$es6$promise$asap$$len += 2;
      if (lib$es6$promise$asap$$len === 2) {
        // If len is 2, that means that we need to schedule an async flush.
        // If additional callbacks are queued before the queue is flushed, they
        // will be processed by this flush that we are scheduling.
        if (lib$es6$promise$asap$$customSchedulerFn) {
          lib$es6$promise$asap$$customSchedulerFn(lib$es6$promise$asap$$flush);
        } else {
          lib$es6$promise$asap$$scheduleFlush();
        }
      }
    }

    function lib$es6$promise$asap$$setScheduler(scheduleFn) {
      lib$es6$promise$asap$$customSchedulerFn = scheduleFn;
    }

    function lib$es6$promise$asap$$setAsap(asapFn) {
      lib$es6$promise$asap$$asap = asapFn;
    }

    var lib$es6$promise$asap$$browserWindow = (typeof window !== 'undefined') ? window : undefined;
    var lib$es6$promise$asap$$browserGlobal = lib$es6$promise$asap$$browserWindow || {};
    var lib$es6$promise$asap$$BrowserMutationObserver = lib$es6$promise$asap$$browserGlobal.MutationObserver || lib$es6$promise$asap$$browserGlobal.WebKitMutationObserver;
    var lib$es6$promise$asap$$isNode = typeof self === 'undefined' && typeof process !== 'undefined' && {}.toString.call(process) === '[object process]';

    // test for web worker but not in IE10
    var lib$es6$promise$asap$$isWorker = typeof Uint8ClampedArray !== 'undefined' &&
      typeof importScripts !== 'undefined' &&
      typeof MessageChannel !== 'undefined';

    // node
    function lib$es6$promise$asap$$useNextTick() {
      // node version 0.10.x displays a deprecation warning when nextTick is used recursively
      // see https://github.com/cujojs/when/issues/410 for details
      return function() {
        process.nextTick(lib$es6$promise$asap$$flush);
      };
    }

    // vertx
    function lib$es6$promise$asap$$useVertxTimer() {
      return function() {
        lib$es6$promise$asap$$vertxNext(lib$es6$promise$asap$$flush);
      };
    }

    function lib$es6$promise$asap$$useMutationObserver() {
      var iterations = 0;
      var observer = new lib$es6$promise$asap$$BrowserMutationObserver(lib$es6$promise$asap$$flush);
      var node = document.createTextNode('');
      observer.observe(node, { characterData: true });

      return function() {
        node.data = (iterations = ++iterations % 2);
      };
    }

    // web worker
    function lib$es6$promise$asap$$useMessageChannel() {
      var channel = new MessageChannel();
      channel.port1.onmessage = lib$es6$promise$asap$$flush;
      return function () {
        channel.port2.postMessage(0);
      };
    }

    function lib$es6$promise$asap$$useSetTimeout() {
      return function() {
        setTimeout(lib$es6$promise$asap$$flush, 1);
      };
    }

    var lib$es6$promise$asap$$queue = new Array(1000);
    function lib$es6$promise$asap$$flush() {
      for (var i = 0; i < lib$es6$promise$asap$$len; i+=2) {
        var callback = lib$es6$promise$asap$$queue[i];
        var arg = lib$es6$promise$asap$$queue[i+1];

        callback(arg);

        lib$es6$promise$asap$$queue[i] = undefined;
        lib$es6$promise$asap$$queue[i+1] = undefined;
      }

      lib$es6$promise$asap$$len = 0;
    }

    function lib$es6$promise$asap$$attemptVertx() {
      try {
        var r = _dereq_;
        var vertx = r('vertx');
        lib$es6$promise$asap$$vertxNext = vertx.runOnLoop || vertx.runOnContext;
        return lib$es6$promise$asap$$useVertxTimer();
      } catch(e) {
        return lib$es6$promise$asap$$useSetTimeout();
      }
    }

    var lib$es6$promise$asap$$scheduleFlush;
    // Decide what async method to use to triggering processing of queued callbacks:
    if (lib$es6$promise$asap$$isNode) {
      lib$es6$promise$asap$$scheduleFlush = lib$es6$promise$asap$$useNextTick();
    } else if (lib$es6$promise$asap$$BrowserMutationObserver) {
      lib$es6$promise$asap$$scheduleFlush = lib$es6$promise$asap$$useMutationObserver();
    } else if (lib$es6$promise$asap$$isWorker) {
      lib$es6$promise$asap$$scheduleFlush = lib$es6$promise$asap$$useMessageChannel();
    } else if (lib$es6$promise$asap$$browserWindow === undefined && typeof _dereq_ === 'function') {
      lib$es6$promise$asap$$scheduleFlush = lib$es6$promise$asap$$attemptVertx();
    } else {
      lib$es6$promise$asap$$scheduleFlush = lib$es6$promise$asap$$useSetTimeout();
    }
    function lib$es6$promise$then$$then(onFulfillment, onRejection) {
      var parent = this;

      var child = new this.constructor(lib$es6$promise$$internal$$noop);

      if (child[lib$es6$promise$$internal$$PROMISE_ID] === undefined) {
        lib$es6$promise$$internal$$makePromise(child);
      }

      var state = parent._state;

      if (state) {
        var callback = arguments[state - 1];
        lib$es6$promise$asap$$asap(function(){
          lib$es6$promise$$internal$$invokeCallback(state, child, callback, parent._result);
        });
      } else {
        lib$es6$promise$$internal$$subscribe(parent, child, onFulfillment, onRejection);
      }

      return child;
    }
    var lib$es6$promise$then$$default = lib$es6$promise$then$$then;
    function lib$es6$promise$promise$resolve$$resolve(object) {
      /*jshint validthis:true */
      var Constructor = this;

      if (object && typeof object === 'object' && object.constructor === Constructor) {
        return object;
      }

      var promise = new Constructor(lib$es6$promise$$internal$$noop);
      lib$es6$promise$$internal$$resolve(promise, object);
      return promise;
    }
    var lib$es6$promise$promise$resolve$$default = lib$es6$promise$promise$resolve$$resolve;
    var lib$es6$promise$$internal$$PROMISE_ID = Math.random().toString(36).substring(16);

    function lib$es6$promise$$internal$$noop() {}

    var lib$es6$promise$$internal$$PENDING   = void 0;
    var lib$es6$promise$$internal$$FULFILLED = 1;
    var lib$es6$promise$$internal$$REJECTED  = 2;

    var lib$es6$promise$$internal$$GET_THEN_ERROR = new lib$es6$promise$$internal$$ErrorObject();

    function lib$es6$promise$$internal$$selfFulfillment() {
      return new TypeError("You cannot resolve a promise with itself");
    }

    function lib$es6$promise$$internal$$cannotReturnOwn() {
      return new TypeError('A promises callback cannot return that same promise.');
    }

    function lib$es6$promise$$internal$$getThen(promise) {
      try {
        return promise.then;
      } catch(error) {
        lib$es6$promise$$internal$$GET_THEN_ERROR.error = error;
        return lib$es6$promise$$internal$$GET_THEN_ERROR;
      }
    }

    function lib$es6$promise$$internal$$tryThen(then, value, fulfillmentHandler, rejectionHandler) {
      try {
        then.call(value, fulfillmentHandler, rejectionHandler);
      } catch(e) {
        return e;
      }
    }

    function lib$es6$promise$$internal$$handleForeignThenable(promise, thenable, then) {
       lib$es6$promise$asap$$asap(function(promise) {
        var sealed = false;
        var error = lib$es6$promise$$internal$$tryThen(then, thenable, function(value) {
          if (sealed) { return; }
          sealed = true;
          if (thenable !== value) {
            lib$es6$promise$$internal$$resolve(promise, value);
          } else {
            lib$es6$promise$$internal$$fulfill(promise, value);
          }
        }, function(reason) {
          if (sealed) { return; }
          sealed = true;

          lib$es6$promise$$internal$$reject(promise, reason);
        }, 'Settle: ' + (promise._label || ' unknown promise'));

        if (!sealed && error) {
          sealed = true;
          lib$es6$promise$$internal$$reject(promise, error);
        }
      }, promise);
    }

    function lib$es6$promise$$internal$$handleOwnThenable(promise, thenable) {
      if (thenable._state === lib$es6$promise$$internal$$FULFILLED) {
        lib$es6$promise$$internal$$fulfill(promise, thenable._result);
      } else if (thenable._state === lib$es6$promise$$internal$$REJECTED) {
        lib$es6$promise$$internal$$reject(promise, thenable._result);
      } else {
        lib$es6$promise$$internal$$subscribe(thenable, undefined, function(value) {
          lib$es6$promise$$internal$$resolve(promise, value);
        }, function(reason) {
          lib$es6$promise$$internal$$reject(promise, reason);
        });
      }
    }

    function lib$es6$promise$$internal$$handleMaybeThenable(promise, maybeThenable, then) {
      if (maybeThenable.constructor === promise.constructor &&
          then === lib$es6$promise$then$$default &&
          constructor.resolve === lib$es6$promise$promise$resolve$$default) {
        lib$es6$promise$$internal$$handleOwnThenable(promise, maybeThenable);
      } else {
        if (then === lib$es6$promise$$internal$$GET_THEN_ERROR) {
          lib$es6$promise$$internal$$reject(promise, lib$es6$promise$$internal$$GET_THEN_ERROR.error);
        } else if (then === undefined) {
          lib$es6$promise$$internal$$fulfill(promise, maybeThenable);
        } else if (lib$es6$promise$utils$$isFunction(then)) {
          lib$es6$promise$$internal$$handleForeignThenable(promise, maybeThenable, then);
        } else {
          lib$es6$promise$$internal$$fulfill(promise, maybeThenable);
        }
      }
    }

    function lib$es6$promise$$internal$$resolve(promise, value) {
      if (promise === value) {
        lib$es6$promise$$internal$$reject(promise, lib$es6$promise$$internal$$selfFulfillment());
      } else if (lib$es6$promise$utils$$objectOrFunction(value)) {
        lib$es6$promise$$internal$$handleMaybeThenable(promise, value, lib$es6$promise$$internal$$getThen(value));
      } else {
        lib$es6$promise$$internal$$fulfill(promise, value);
      }
    }

    function lib$es6$promise$$internal$$publishRejection(promise) {
      if (promise._onerror) {
        promise._onerror(promise._result);
      }

      lib$es6$promise$$internal$$publish(promise);
    }

    function lib$es6$promise$$internal$$fulfill(promise, value) {
      if (promise._state !== lib$es6$promise$$internal$$PENDING) { return; }

      promise._result = value;
      promise._state = lib$es6$promise$$internal$$FULFILLED;

      if (promise._subscribers.length !== 0) {
        lib$es6$promise$asap$$asap(lib$es6$promise$$internal$$publish, promise);
      }
    }

    function lib$es6$promise$$internal$$reject(promise, reason) {
      if (promise._state !== lib$es6$promise$$internal$$PENDING) { return; }
      promise._state = lib$es6$promise$$internal$$REJECTED;
      promise._result = reason;

      lib$es6$promise$asap$$asap(lib$es6$promise$$internal$$publishRejection, promise);
    }

    function lib$es6$promise$$internal$$subscribe(parent, child, onFulfillment, onRejection) {
      var subscribers = parent._subscribers;
      var length = subscribers.length;

      parent._onerror = null;

      subscribers[length] = child;
      subscribers[length + lib$es6$promise$$internal$$FULFILLED] = onFulfillment;
      subscribers[length + lib$es6$promise$$internal$$REJECTED]  = onRejection;

      if (length === 0 && parent._state) {
        lib$es6$promise$asap$$asap(lib$es6$promise$$internal$$publish, parent);
      }
    }

    function lib$es6$promise$$internal$$publish(promise) {
      var subscribers = promise._subscribers;
      var settled = promise._state;

      if (subscribers.length === 0) { return; }

      var child, callback, detail = promise._result;

      for (var i = 0; i < subscribers.length; i += 3) {
        child = subscribers[i];
        callback = subscribers[i + settled];

        if (child) {
          lib$es6$promise$$internal$$invokeCallback(settled, child, callback, detail);
        } else {
          callback(detail);
        }
      }

      promise._subscribers.length = 0;
    }

    function lib$es6$promise$$internal$$ErrorObject() {
      this.error = null;
    }

    var lib$es6$promise$$internal$$TRY_CATCH_ERROR = new lib$es6$promise$$internal$$ErrorObject();

    function lib$es6$promise$$internal$$tryCatch(callback, detail) {
      try {
        return callback(detail);
      } catch(e) {
        lib$es6$promise$$internal$$TRY_CATCH_ERROR.error = e;
        return lib$es6$promise$$internal$$TRY_CATCH_ERROR;
      }
    }

    function lib$es6$promise$$internal$$invokeCallback(settled, promise, callback, detail) {
      var hasCallback = lib$es6$promise$utils$$isFunction(callback),
          value, error, succeeded, failed;

      if (hasCallback) {
        value = lib$es6$promise$$internal$$tryCatch(callback, detail);

        if (value === lib$es6$promise$$internal$$TRY_CATCH_ERROR) {
          failed = true;
          error = value.error;
          value = null;
        } else {
          succeeded = true;
        }

        if (promise === value) {
          lib$es6$promise$$internal$$reject(promise, lib$es6$promise$$internal$$cannotReturnOwn());
          return;
        }

      } else {
        value = detail;
        succeeded = true;
      }

      if (promise._state !== lib$es6$promise$$internal$$PENDING) {
        // noop
      } else if (hasCallback && succeeded) {
        lib$es6$promise$$internal$$resolve(promise, value);
      } else if (failed) {
        lib$es6$promise$$internal$$reject(promise, error);
      } else if (settled === lib$es6$promise$$internal$$FULFILLED) {
        lib$es6$promise$$internal$$fulfill(promise, value);
      } else if (settled === lib$es6$promise$$internal$$REJECTED) {
        lib$es6$promise$$internal$$reject(promise, value);
      }
    }

    function lib$es6$promise$$internal$$initializePromise(promise, resolver) {
      try {
        resolver(function resolvePromise(value){
          lib$es6$promise$$internal$$resolve(promise, value);
        }, function rejectPromise(reason) {
          lib$es6$promise$$internal$$reject(promise, reason);
        });
      } catch(e) {
        lib$es6$promise$$internal$$reject(promise, e);
      }
    }

    var lib$es6$promise$$internal$$id = 0;
    function lib$es6$promise$$internal$$nextId() {
      return lib$es6$promise$$internal$$id++;
    }

    function lib$es6$promise$$internal$$makePromise(promise) {
      promise[lib$es6$promise$$internal$$PROMISE_ID] = lib$es6$promise$$internal$$id++;
      promise._state = undefined;
      promise._result = undefined;
      promise._subscribers = [];
    }

    function lib$es6$promise$promise$all$$all(entries) {
      return new lib$es6$promise$enumerator$$default(this, entries).promise;
    }
    var lib$es6$promise$promise$all$$default = lib$es6$promise$promise$all$$all;
    function lib$es6$promise$promise$race$$race(entries) {
      /*jshint validthis:true */
      var Constructor = this;

      if (!lib$es6$promise$utils$$isArray(entries)) {
        return new Constructor(function(resolve, reject) {
          reject(new TypeError('You must pass an array to race.'));
        });
      } else {
        return new Constructor(function(resolve, reject) {
          var length = entries.length;
          for (var i = 0; i < length; i++) {
            Constructor.resolve(entries[i]).then(resolve, reject);
          }
        });
      }
    }
    var lib$es6$promise$promise$race$$default = lib$es6$promise$promise$race$$race;
    function lib$es6$promise$promise$reject$$reject(reason) {
      /*jshint validthis:true */
      var Constructor = this;
      var promise = new Constructor(lib$es6$promise$$internal$$noop);
      lib$es6$promise$$internal$$reject(promise, reason);
      return promise;
    }
    var lib$es6$promise$promise$reject$$default = lib$es6$promise$promise$reject$$reject;


    function lib$es6$promise$promise$$needsResolver() {
      throw new TypeError('You must pass a resolver function as the first argument to the promise constructor');
    }

    function lib$es6$promise$promise$$needsNew() {
      throw new TypeError("Failed to construct 'Promise': Please use the 'new' operator, this object constructor cannot be called as a function.");
    }

    var lib$es6$promise$promise$$default = lib$es6$promise$promise$$Promise;
    /**
      Promise objects represent the eventual result of an asynchronous operation. The
      primary way of interacting with a promise is through its `then` method, which
      registers callbacks to receive either a promise's eventual value or the reason
      why the promise cannot be fulfilled.

      Terminology
      -----------

      - `promise` is an object or function with a `then` method whose behavior conforms to this specification.
      - `thenable` is an object or function that defines a `then` method.
      - `value` is any legal JavaScript value (including undefined, a thenable, or a promise).
      - `exception` is a value that is thrown using the throw statement.
      - `reason` is a value that indicates why a promise was rejected.
      - `settled` the final resting state of a promise, fulfilled or rejected.

      A promise can be in one of three states: pending, fulfilled, or rejected.

      Promises that are fulfilled have a fulfillment value and are in the fulfilled
      state.  Promises that are rejected have a rejection reason and are in the
      rejected state.  A fulfillment value is never a thenable.

      Promises can also be said to *resolve* a value.  If this value is also a
      promise, then the original promise's settled state will match the value's
      settled state.  So a promise that *resolves* a promise that rejects will
      itself reject, and a promise that *resolves* a promise that fulfills will
      itself fulfill.


      Basic Usage:
      ------------

      ```js
      var promise = new Promise(function(resolve, reject) {
        // on success
        resolve(value);

        // on failure
        reject(reason);
      });

      promise.then(function(value) {
        // on fulfillment
      }, function(reason) {
        // on rejection
      });
      ```

      Advanced Usage:
      ---------------

      Promises shine when abstracting away asynchronous interactions such as
      `XMLHttpRequest`s.

      ```js
      function getJSON(url) {
        return new Promise(function(resolve, reject){
          var xhr = new XMLHttpRequest();

          xhr.open('GET', url);
          xhr.onreadystatechange = handler;
          xhr.responseType = 'json';
          xhr.setRequestHeader('Accept', 'application/json');
          xhr.send();

          function handler() {
            if (this.readyState === this.DONE) {
              if (this.status === 200) {
                resolve(this.response);
              } else {
                reject(new Error('getJSON: `' + url + '` failed with status: [' + this.status + ']'));
              }
            }
          };
        });
      }

      getJSON('/posts.json').then(function(json) {
        // on fulfillment
      }, function(reason) {
        // on rejection
      });
      ```

      Unlike callbacks, promises are great composable primitives.

      ```js
      Promise.all([
        getJSON('/posts'),
        getJSON('/comments')
      ]).then(function(values){
        values[0] // => postsJSON
        values[1] // => commentsJSON

        return values;
      });
      ```

      @class Promise
      @param {function} resolver
      Useful for tooling.
      @constructor
    */
    function lib$es6$promise$promise$$Promise(resolver) {
      this[lib$es6$promise$$internal$$PROMISE_ID] = lib$es6$promise$$internal$$nextId();
      this._result = this._state = undefined;
      this._subscribers = [];

      if (lib$es6$promise$$internal$$noop !== resolver) {
        typeof resolver !== 'function' && lib$es6$promise$promise$$needsResolver();
        this instanceof lib$es6$promise$promise$$Promise ? lib$es6$promise$$internal$$initializePromise(this, resolver) : lib$es6$promise$promise$$needsNew();
      }
    }

    lib$es6$promise$promise$$Promise.all = lib$es6$promise$promise$all$$default;
    lib$es6$promise$promise$$Promise.race = lib$es6$promise$promise$race$$default;
    lib$es6$promise$promise$$Promise.resolve = lib$es6$promise$promise$resolve$$default;
    lib$es6$promise$promise$$Promise.reject = lib$es6$promise$promise$reject$$default;
    lib$es6$promise$promise$$Promise._setScheduler = lib$es6$promise$asap$$setScheduler;
    lib$es6$promise$promise$$Promise._setAsap = lib$es6$promise$asap$$setAsap;
    lib$es6$promise$promise$$Promise._asap = lib$es6$promise$asap$$asap;

    lib$es6$promise$promise$$Promise.prototype = {
      constructor: lib$es6$promise$promise$$Promise,

    /**
      The primary way of interacting with a promise is through its `then` method,
      which registers callbacks to receive either a promise's eventual value or the
      reason why the promise cannot be fulfilled.

      ```js
      findUser().then(function(user){
        // user is available
      }, function(reason){
        // user is unavailable, and you are given the reason why
      });
      ```

      Chaining
      --------

      The return value of `then` is itself a promise.  This second, 'downstream'
      promise is resolved with the return value of the first promise's fulfillment
      or rejection handler, or rejected if the handler throws an exception.

      ```js
      findUser().then(function (user) {
        return user.name;
      }, function (reason) {
        return 'default name';
      }).then(function (userName) {
        // If `findUser` fulfilled, `userName` will be the user's name, otherwise it
        // will be `'default name'`
      });

      findUser().then(function (user) {
        throw new Error('Found user, but still unhappy');
      }, function (reason) {
        throw new Error('`findUser` rejected and we're unhappy');
      }).then(function (value) {
        // never reached
      }, function (reason) {
        // if `findUser` fulfilled, `reason` will be 'Found user, but still unhappy'.
        // If `findUser` rejected, `reason` will be '`findUser` rejected and we're unhappy'.
      });
      ```
      If the downstream promise does not specify a rejection handler, rejection reasons will be propagated further downstream.

      ```js
      findUser().then(function (user) {
        throw new PedagogicalException('Upstream error');
      }).then(function (value) {
        // never reached
      }).then(function (value) {
        // never reached
      }, function (reason) {
        // The `PedgagocialException` is propagated all the way down to here
      });
      ```

      Assimilation
      ------------

      Sometimes the value you want to propagate to a downstream promise can only be
      retrieved asynchronously. This can be achieved by returning a promise in the
      fulfillment or rejection handler. The downstream promise will then be pending
      until the returned promise is settled. This is called *assimilation*.

      ```js
      findUser().then(function (user) {
        return findCommentsByAuthor(user);
      }).then(function (comments) {
        // The user's comments are now available
      });
      ```

      If the assimliated promise rejects, then the downstream promise will also reject.

      ```js
      findUser().then(function (user) {
        return findCommentsByAuthor(user);
      }).then(function (comments) {
        // If `findCommentsByAuthor` fulfills, we'll have the value here
      }, function (reason) {
        // If `findCommentsByAuthor` rejects, we'll have the reason here
      });
      ```

      Simple Example
      --------------

      Synchronous Example

      ```javascript
      var result;

      try {
        result = findResult();
        // success
      } catch(reason) {
        // failure
      }
      ```

      Errback Example

      ```js
      findResult(function(result, err){
        if (err) {
          // failure
        } else {
          // success
        }
      });
      ```

      Promise Example;

      ```javascript
      findResult().then(function(result){
        // success
      }, function(reason){
        // failure
      });
      ```

      Advanced Example
      --------------

      Synchronous Example

      ```javascript
      var author, books;

      try {
        author = findAuthor();
        books  = findBooksByAuthor(author);
        // success
      } catch(reason) {
        // failure
      }
      ```

      Errback Example

      ```js

      function foundBooks(books) {

      }

      function failure(reason) {

      }

      findAuthor(function(author, err){
        if (err) {
          failure(err);
          // failure
        } else {
          try {
            findBoooksByAuthor(author, function(books, err) {
              if (err) {
                failure(err);
              } else {
                try {
                  foundBooks(books);
                } catch(reason) {
                  failure(reason);
                }
              }
            });
          } catch(error) {
            failure(err);
          }
          // success
        }
      });
      ```

      Promise Example;

      ```javascript
      findAuthor().
        then(findBooksByAuthor).
        then(function(books){
          // found books
      }).catch(function(reason){
        // something went wrong
      });
      ```

      @method then
      @param {Function} onFulfilled
      @param {Function} onRejected
      Useful for tooling.
      @return {Promise}
    */
      then: lib$es6$promise$then$$default,

    /**
      `catch` is simply sugar for `then(undefined, onRejection)` which makes it the same
      as the catch block of a try/catch statement.

      ```js
      function findAuthor(){
        throw new Error('couldn't find that author');
      }

      // synchronous
      try {
        findAuthor();
      } catch(reason) {
        // something went wrong
      }

      // async with promises
      findAuthor().catch(function(reason){
        // something went wrong
      });
      ```

      @method catch
      @param {Function} onRejection
      Useful for tooling.
      @return {Promise}
    */
      'catch': function(onRejection) {
        return this.then(null, onRejection);
      }
    };
    var lib$es6$promise$enumerator$$default = lib$es6$promise$enumerator$$Enumerator;
    function lib$es6$promise$enumerator$$Enumerator(Constructor, input) {
      this._instanceConstructor = Constructor;
      this.promise = new Constructor(lib$es6$promise$$internal$$noop);

      if (!this.promise[lib$es6$promise$$internal$$PROMISE_ID]) {
        lib$es6$promise$$internal$$makePromise(this.promise);
      }

      if (lib$es6$promise$utils$$isArray(input)) {
        this._input     = input;
        this.length     = input.length;
        this._remaining = input.length;

        this._result = new Array(this.length);

        if (this.length === 0) {
          lib$es6$promise$$internal$$fulfill(this.promise, this._result);
        } else {
          this.length = this.length || 0;
          this._enumerate();
          if (this._remaining === 0) {
            lib$es6$promise$$internal$$fulfill(this.promise, this._result);
          }
        }
      } else {
        lib$es6$promise$$internal$$reject(this.promise, lib$es6$promise$enumerator$$validationError());
      }
    }

    function lib$es6$promise$enumerator$$validationError() {
      return new Error('Array Methods must be provided an Array');
    }

    lib$es6$promise$enumerator$$Enumerator.prototype._enumerate = function() {
      var length  = this.length;
      var input   = this._input;

      for (var i = 0; this._state === lib$es6$promise$$internal$$PENDING && i < length; i++) {
        this._eachEntry(input[i], i);
      }
    };

    lib$es6$promise$enumerator$$Enumerator.prototype._eachEntry = function(entry, i) {
      var c = this._instanceConstructor;
      var resolve = c.resolve;

      if (resolve === lib$es6$promise$promise$resolve$$default) {
        var then = lib$es6$promise$$internal$$getThen(entry);

        if (then === lib$es6$promise$then$$default &&
            entry._state !== lib$es6$promise$$internal$$PENDING) {
          this._settledAt(entry._state, i, entry._result);
        } else if (typeof then !== 'function') {
          this._remaining--;
          this._result[i] = entry;
        } else if (c === lib$es6$promise$promise$$default) {
          var promise = new c(lib$es6$promise$$internal$$noop);
          lib$es6$promise$$internal$$handleMaybeThenable(promise, entry, then);
          this._willSettleAt(promise, i);
        } else {
          this._willSettleAt(new c(function(resolve) { resolve(entry); }), i);
        }
      } else {
        this._willSettleAt(resolve(entry), i);
      }
    };

    lib$es6$promise$enumerator$$Enumerator.prototype._settledAt = function(state, i, value) {
      var promise = this.promise;

      if (promise._state === lib$es6$promise$$internal$$PENDING) {
        this._remaining--;

        if (state === lib$es6$promise$$internal$$REJECTED) {
          lib$es6$promise$$internal$$reject(promise, value);
        } else {
          this._result[i] = value;
        }
      }

      if (this._remaining === 0) {
        lib$es6$promise$$internal$$fulfill(promise, this._result);
      }
    };

    lib$es6$promise$enumerator$$Enumerator.prototype._willSettleAt = function(promise, i) {
      var enumerator = this;

      lib$es6$promise$$internal$$subscribe(promise, undefined, function(value) {
        enumerator._settledAt(lib$es6$promise$$internal$$FULFILLED, i, value);
      }, function(reason) {
        enumerator._settledAt(lib$es6$promise$$internal$$REJECTED, i, reason);
      });
    };
    function lib$es6$promise$polyfill$$polyfill() {
      var local;

      if (typeof global !== 'undefined') {
          local = global;
      } else if (typeof self !== 'undefined') {
          local = self;
      } else {
          try {
              local = Function('return this')();
          } catch (e) {
              throw new Error('polyfill failed because global object is unavailable in this environment');
          }
      }

      var P = local.Promise;

      if (P && Object.prototype.toString.call(P.resolve()) === '[object Promise]' && !P.cast) {
        return;
      }

      local.Promise = lib$es6$promise$promise$$default;
    }
    var lib$es6$promise$polyfill$$default = lib$es6$promise$polyfill$$polyfill;

    var lib$es6$promise$umd$$ES6Promise = {
      'Promise': lib$es6$promise$promise$$default,
      'polyfill': lib$es6$promise$polyfill$$default
    };

    /* global define:true module:true window: true */
    if (typeof define === 'function' && define['amd']) {
      define(function() { return lib$es6$promise$umd$$ES6Promise; });
    } else if (typeof module !== 'undefined' && module['exports']) {
      module['exports'] = lib$es6$promise$umd$$ES6Promise;
    } else if (typeof this !== 'undefined') {
      this['ES6Promise'] = lib$es6$promise$umd$$ES6Promise;
    }

    lib$es6$promise$polyfill$$default();
}).call(this);


}).call(this,_dereq_('_process'),typeof global !== "undefined" ? global : typeof self !== "undefined" ? self : typeof window !== "undefined" ? window : {})

},{"_process":3}],5:[function(_dereq_,module,exports){
/* Copyright (c) 2013, Brandon Jones, Colin MacKenzie IV. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/**
 * @class Quaternion
 * @name quat
 */
var quat = {};

// imports...
var vec3 = _dereq_('vec3');
var vec4 = _dereq_('vec4');
var mat3 = _dereq_('mat3');
var GLMAT_ARRAY_TYPE = _dereq_('common').GLMAT_ARRAY_TYPE;

/**
 * Creates a new identity quat
 *
 * @returns {quat} a new quaternion
 */
quat.create = function() {
    var out = new GLMAT_ARRAY_TYPE(4);
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 1;
    return out;
};

/**
 * Sets a quaternion to represent the shortest rotation from one
 * vector to another.
 *
 * Both vectors are assumed to be unit length.
 *
 * @param {quat} out the receiving quaternion.
 * @param {vec3} a the initial vector
 * @param {vec3} b the destination vector
 * @returns {quat} out
 */
quat.rotationTo = (function() {
    var tmpvec3 = vec3.create();
    var xUnitVec3 = vec3.fromValues(1,0,0);
    var yUnitVec3 = vec3.fromValues(0,1,0);

    return function(out, a, b) {
        var dot = vec3.dot(a, b);
        if (dot < -0.999999) {
            vec3.cross(tmpvec3, xUnitVec3, a);
            if (vec3.length(tmpvec3) < 0.000001)
                vec3.cross(tmpvec3, yUnitVec3, a);
            vec3.normalize(tmpvec3, tmpvec3);
            quat.setAxisAngle(out, tmpvec3, Math.PI);
            return out;
        } else if (dot > 0.999999) {
            out[0] = 0;
            out[1] = 0;
            out[2] = 0;
            out[3] = 1;
            return out;
        } else {
            vec3.cross(tmpvec3, a, b);
            out[0] = tmpvec3[0];
            out[1] = tmpvec3[1];
            out[2] = tmpvec3[2];
            out[3] = 1 + dot;
            return quat.normalize(out, out);
        }
    };
})();

/**
 * Sets the specified quaternion with values corresponding to the given
 * axes. Each axis is a vec3 and is expected to be unit length and
 * perpendicular to all other specified axes.
 *
 * @param {vec3} view  the vector representing the viewing direction
 * @param {vec3} right the vector representing the local "right" direction
 * @param {vec3} up    the vector representing the local "up" direction
 * @returns {quat} out
 */
quat.setAxes = (function() {
    var matr = mat3.create();

    return function(out, view, right, up) {
        matr[0] = right[0];
        matr[3] = right[1];
        matr[6] = right[2];

        matr[1] = up[0];
        matr[4] = up[1];
        matr[7] = up[2];

        matr[2] = -view[0];
        matr[5] = -view[1];
        matr[8] = -view[2];

        return quat.normalize(out, quat.fromMat3(out, matr));
    };
})();

/**
 * Creates a new quat initialized with values from an existing quaternion
 *
 * @param {quat} a quaternion to clone
 * @returns {quat} a new quaternion
 * @function
 */
quat.clone = vec4.clone;

/**
 * Creates a new quat initialized with the given values
 *
 * @param {Number} x X component
 * @param {Number} y Y component
 * @param {Number} z Z component
 * @param {Number} w W component
 * @returns {quat} a new quaternion
 * @function
 */
quat.fromValues = vec4.fromValues;

/**
 * Copy the values from one quat to another
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a the source quaternion
 * @returns {quat} out
 * @function
 */
quat.copy = vec4.copy;

/**
 * Set the components of a quat to the given values
 *
 * @param {quat} out the receiving quaternion
 * @param {Number} x X component
 * @param {Number} y Y component
 * @param {Number} z Z component
 * @param {Number} w W component
 * @returns {quat} out
 * @function
 */
quat.set = vec4.set;

/**
 * Set a quat to the identity quaternion
 *
 * @param {quat} out the receiving quaternion
 * @returns {quat} out
 */
quat.identity = function(out) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 1;
    return out;
};

/**
 * Sets a quat from the given angle and rotation axis,
 * then returns it.
 *
 * @param {quat} out the receiving quaternion
 * @param {vec3} axis the axis around which to rotate
 * @param {Number} rad the angle in radians
 * @returns {quat} out
 **/
quat.setAxisAngle = function(out, axis, rad) {
    rad = rad * 0.5;
    var s = Math.sin(rad);
    out[0] = s * axis[0];
    out[1] = s * axis[1];
    out[2] = s * axis[2];
    out[3] = Math.cos(rad);
    return out;
};

/**
 * Adds two quat's
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a the first operand
 * @param {quat} b the second operand
 * @returns {quat} out
 * @function
 */
quat.add = vec4.add;

/**
 * Multiplies two quat's
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a the first operand
 * @param {quat} b the second operand
 * @returns {quat} out
 */
quat.multiply = function(out, a, b) {
    var ax = a[0], ay = a[1], az = a[2], aw = a[3],
        bx = b[0], by = b[1], bz = b[2], bw = b[3];

    out[0] = ax * bw + aw * bx + ay * bz - az * by;
    out[1] = ay * bw + aw * by + az * bx - ax * bz;
    out[2] = az * bw + aw * bz + ax * by - ay * bx;
    out[3] = aw * bw - ax * bx - ay * by - az * bz;
    return out;
};

/**
 * Alias for {@link quat.multiply}
 * @function
 */
quat.mul = quat.multiply;

/**
 * Scales a quat by a scalar number
 *
 * @param {quat} out the receiving vector
 * @param {quat} a the vector to scale
 * @param {Number} b amount to scale the vector by
 * @returns {quat} out
 * @function
 */
quat.scale = vec4.scale;

/**
 * Rotates a quaternion by the given angle about the X axis
 *
 * @param {quat} out quat receiving operation result
 * @param {quat} a quat to rotate
 * @param {number} rad angle (in radians) to rotate
 * @returns {quat} out
 */
quat.rotateX = function (out, a, rad) {
    rad *= 0.5; 

    var ax = a[0], ay = a[1], az = a[2], aw = a[3],
        bx = Math.sin(rad), bw = Math.cos(rad);

    out[0] = ax * bw + aw * bx;
    out[1] = ay * bw + az * bx;
    out[2] = az * bw - ay * bx;
    out[3] = aw * bw - ax * bx;
    return out;
};

/**
 * Rotates a quaternion by the given angle about the Y axis
 *
 * @param {quat} out quat receiving operation result
 * @param {quat} a quat to rotate
 * @param {number} rad angle (in radians) to rotate
 * @returns {quat} out
 */
quat.rotateY = function (out, a, rad) {
    rad *= 0.5; 

    var ax = a[0], ay = a[1], az = a[2], aw = a[3],
        by = Math.sin(rad), bw = Math.cos(rad);

    out[0] = ax * bw - az * by;
    out[1] = ay * bw + aw * by;
    out[2] = az * bw + ax * by;
    out[3] = aw * bw - ay * by;
    return out;
};

/**
 * Rotates a quaternion by the given angle about the Z axis
 *
 * @param {quat} out quat receiving operation result
 * @param {quat} a quat to rotate
 * @param {number} rad angle (in radians) to rotate
 * @returns {quat} out
 */
quat.rotateZ = function (out, a, rad) {
    rad *= 0.5; 

    var ax = a[0], ay = a[1], az = a[2], aw = a[3],
        bz = Math.sin(rad), bw = Math.cos(rad);

    out[0] = ax * bw + ay * bz;
    out[1] = ay * bw - ax * bz;
    out[2] = az * bw + aw * bz;
    out[3] = aw * bw - az * bz;
    return out;
};

/**
 * Calculates the W component of a quat from the X, Y, and Z components.
 * Assumes that quaternion is 1 unit in length.
 * Any existing W component will be ignored.
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a quat to calculate W component of
 * @returns {quat} out
 */
quat.calculateW = function (out, a) {
    var x = a[0], y = a[1], z = a[2];

    out[0] = x;
    out[1] = y;
    out[2] = z;
    out[3] = -Math.sqrt(Math.abs(1.0 - x * x - y * y - z * z));
    return out;
};

/**
 * Calculates the dot product of two quat's
 *
 * @param {quat} a the first operand
 * @param {quat} b the second operand
 * @returns {Number} dot product of a and b
 * @function
 */
quat.dot = vec4.dot;

/**
 * Performs a linear interpolation between two quat's
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a the first operand
 * @param {quat} b the second operand
 * @param {Number} t interpolation amount between the two inputs
 * @returns {quat} out
 * @function
 */
quat.lerp = vec4.lerp;

/**
 * Performs a spherical linear interpolation between two quat
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a the first operand
 * @param {quat} b the second operand
 * @param {Number} t interpolation amount between the two inputs
 * @returns {quat} out
 */
quat.slerp = function (out, a, b, t) {
    // benchmarks:
    //    http://jsperf.com/quaternion-slerp-implementations

    var ax = a[0], ay = a[1], az = a[2], aw = a[3],
        bx = b[0], by = b[1], bz = b[2], bw = b[3];

    var        omega, cosom, sinom, scale0, scale1;

    // calc cosine
    cosom = ax * bx + ay * by + az * bz + aw * bw;
    // adjust signs (if necessary)
    if ( cosom < 0.0 ) {
        cosom = -cosom;
        bx = - bx;
        by = - by;
        bz = - bz;
        bw = - bw;
    }
    // calculate coefficients
    if ( (1.0 - cosom) > 0.000001 ) {
        // standard case (slerp)
        omega  = Math.acos(cosom);
        sinom  = Math.sin(omega);
        scale0 = Math.sin((1.0 - t) * omega) / sinom;
        scale1 = Math.sin(t * omega) / sinom;
    } else {        
        // "from" and "to" quaternions are very close 
        //  ... so we can do a linear interpolation
        scale0 = 1.0 - t;
        scale1 = t;
    }
    // calculate final values
    out[0] = scale0 * ax + scale1 * bx;
    out[1] = scale0 * ay + scale1 * by;
    out[2] = scale0 * az + scale1 * bz;
    out[3] = scale0 * aw + scale1 * bw;
    
    return out;
};

/**
 * Calculates the inverse of a quat
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a quat to calculate inverse of
 * @returns {quat} out
 */
quat.invert = function(out, a) {
    var a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3],
        dot = a0*a0 + a1*a1 + a2*a2 + a3*a3,
        invDot = dot ? 1.0/dot : 0;
    
    // TODO: Would be faster to return [0,0,0,0] immediately if dot == 0

    out[0] = -a0*invDot;
    out[1] = -a1*invDot;
    out[2] = -a2*invDot;
    out[3] = a3*invDot;
    return out;
};

/**
 * Calculates the conjugate of a quat
 * If the quaternion is normalized, this function is faster than quat.inverse and produces the same result.
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a quat to calculate conjugate of
 * @returns {quat} out
 */
quat.conjugate = function (out, a) {
    out[0] = -a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    out[3] = a[3];
    return out;
};

/**
 * Calculates the length of a quat
 *
 * @param {quat} a vector to calculate length of
 * @returns {Number} length of a
 * @function
 */
quat.length = vec4.length;

/**
 * Alias for {@link quat.length}
 * @function
 */
quat.len = quat.length;

/**
 * Calculates the squared length of a quat
 *
 * @param {quat} a vector to calculate squared length of
 * @returns {Number} squared length of a
 * @function
 */
quat.squaredLength = vec4.squaredLength;

/**
 * Alias for {@link quat.squaredLength}
 * @function
 */
quat.sqrLen = quat.squaredLength;

/**
 * Normalize a quat
 *
 * @param {quat} out the receiving quaternion
 * @param {quat} a quaternion to normalize
 * @returns {quat} out
 * @function
 */
quat.normalize = vec4.normalize;

/**
 * Creates a quaternion from the given 3x3 rotation matrix.
 *
 * NOTE: The resultant quaternion is not normalized, so you should be sure
 * to renormalize the quaternion yourself where necessary.
 *
 * @param {quat} out the receiving quaternion
 * @param {mat3} m rotation matrix
 * @returns {quat} out
 * @function
 */
quat.fromMat3 = function(out, m) {
    // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
    // article "Quaternion Calculus and Fast Animation".
    var fTrace = m[0] + m[4] + m[8];
    var fRoot;

    if ( fTrace > 0.0 ) {
        // |w| > 1/2, may as well choose w > 1/2
        fRoot = Math.sqrt(fTrace + 1.0);  // 2w
        out[3] = 0.5 * fRoot;
        fRoot = 0.5/fRoot;  // 1/(4w)
        out[0] = (m[7]-m[5])*fRoot;
        out[1] = (m[2]-m[6])*fRoot;
        out[2] = (m[3]-m[1])*fRoot;
    } else {
        // |w| <= 1/2
        var i = 0;
        if ( m[4] > m[0] )
          i = 1;
        if ( m[8] > m[i*3+i] )
          i = 2;
        var j = (i+1)%3;
        var k = (i+2)%3;
        
        fRoot = Math.sqrt(m[i*3+i]-m[j*3+j]-m[k*3+k] + 1.0);
        out[i] = 0.5 * fRoot;
        fRoot = 0.5 / fRoot;
        out[3] = (m[k*3+j] - m[j*3+k]) * fRoot;
        out[j] = (m[j*3+i] + m[i*3+j]) * fRoot;
        out[k] = (m[k*3+i] + m[i*3+k]) * fRoot;
    }
    
    return out;
};

/**
 * Returns a string representation of a quatenion
 *
 * @param {quat} vec vector to represent as a string
 * @returns {String} string representation of the vector
 */
quat.str = function (a) {
    return 'quat(' + a[0] + ', ' + a[1] + ', ' + a[2] + ', ' + a[3] + ')';
};

module.exports = quat;


},{"common":6,"mat3":7,"vec3":9,"vec4":8}],6:[function(_dereq_,module,exports){
/* Copyright (c) 2013, Brandon Jones, Colin MacKenzie IV. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

if(!GLMAT_EPSILON) {
    var GLMAT_EPSILON = 0.000001;
}

if(!GLMAT_ARRAY_TYPE) {
    var GLMAT_ARRAY_TYPE = (typeof Float32Array !== 'undefined') ? Float32Array : Array;
}

if(!GLMAT_RANDOM) {
    var GLMAT_RANDOM = Math.random;
}

/**
 * @class Common utilities
 * @name glMatrix
 */
var glMatrix = {};

/**
 * Sets the type of array used when creating new vectors and matricies
 *
 * @param {Type} type Array type, such as Float32Array or Array
 */
glMatrix.setMatrixArrayType = function(type) {
    GLMAT_ARRAY_TYPE = type;
}


var degree = Math.PI / 180;

/**
* Convert Degree To Radian
*
* @param {Number} Angle in Degrees
*/
glMatrix.toRadian = function(a){
     return a * degree;
}

module.exports = {
  GLMAT_EPSILON : GLMAT_EPSILON,
  GLMAT_ARRAY_TYPE : GLMAT_ARRAY_TYPE,
  GLMAT_RANDOM : GLMAT_RANDOM,
  glMatrix : glMatrix
};

},{}],7:[function(_dereq_,module,exports){
/* Copyright (c) 2013, Brandon Jones, Colin MacKenzie IV. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/**
 * @class 3x3 Matrix
 * @name mat3
 */
var mat3 = {};

var GLMAT_ARRAY_TYPE = _dereq_('common').GLMAT_ARRAY_TYPE;

/**
 * Creates a new identity mat3
 *
 * @returns {mat3} a new 3x3 matrix
 */
mat3.create = function() {
    var out = new GLMAT_ARRAY_TYPE(9);
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 1;
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
    out[8] = 1;
    return out;
};

/**
 * Copies the upper-left 3x3 values into the given mat3.
 *
 * @param {mat3} out the receiving 3x3 matrix
 * @param {mat4} a   the source 4x4 matrix
 * @returns {mat3} out
 */
mat3.fromMat4 = function(out, a) {
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[4];
    out[4] = a[5];
    out[5] = a[6];
    out[6] = a[8];
    out[7] = a[9];
    out[8] = a[10];
    return out;
};

/**
 * Creates a new mat3 initialized with values from an existing matrix
 *
 * @param {mat3} a matrix to clone
 * @returns {mat3} a new 3x3 matrix
 */
mat3.clone = function(a) {
    var out = new GLMAT_ARRAY_TYPE(9);
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
    out[4] = a[4];
    out[5] = a[5];
    out[6] = a[6];
    out[7] = a[7];
    out[8] = a[8];
    return out;
};

/**
 * Copy the values from one mat3 to another
 *
 * @param {mat3} out the receiving matrix
 * @param {mat3} a the source matrix
 * @returns {mat3} out
 */
mat3.copy = function(out, a) {
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
    out[4] = a[4];
    out[5] = a[5];
    out[6] = a[6];
    out[7] = a[7];
    out[8] = a[8];
    return out;
};

/**
 * Set a mat3 to the identity matrix
 *
 * @param {mat3} out the receiving matrix
 * @returns {mat3} out
 */
mat3.identity = function(out) {
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 1;
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
    out[8] = 1;
    return out;
};

/**
 * Transpose the values of a mat3
 *
 * @param {mat3} out the receiving matrix
 * @param {mat3} a the source matrix
 * @returns {mat3} out
 */
mat3.transpose = function(out, a) {
    // If we are transposing ourselves we can skip a few steps but have to cache some values
    if (out === a) {
        var a01 = a[1], a02 = a[2], a12 = a[5];
        out[1] = a[3];
        out[2] = a[6];
        out[3] = a01;
        out[5] = a[7];
        out[6] = a02;
        out[7] = a12;
    } else {
        out[0] = a[0];
        out[1] = a[3];
        out[2] = a[6];
        out[3] = a[1];
        out[4] = a[4];
        out[5] = a[7];
        out[6] = a[2];
        out[7] = a[5];
        out[8] = a[8];
    }
    
    return out;
};

/**
 * Inverts a mat3
 *
 * @param {mat3} out the receiving matrix
 * @param {mat3} a the source matrix
 * @returns {mat3} out
 */
mat3.invert = function(out, a) {
    var a00 = a[0], a01 = a[1], a02 = a[2],
        a10 = a[3], a11 = a[4], a12 = a[5],
        a20 = a[6], a21 = a[7], a22 = a[8],

        b01 = a22 * a11 - a12 * a21,
        b11 = -a22 * a10 + a12 * a20,
        b21 = a21 * a10 - a11 * a20,

        // Calculate the determinant
        det = a00 * b01 + a01 * b11 + a02 * b21;

    if (!det) { 
        return null; 
    }
    det = 1.0 / det;

    out[0] = b01 * det;
    out[1] = (-a22 * a01 + a02 * a21) * det;
    out[2] = (a12 * a01 - a02 * a11) * det;
    out[3] = b11 * det;
    out[4] = (a22 * a00 - a02 * a20) * det;
    out[5] = (-a12 * a00 + a02 * a10) * det;
    out[6] = b21 * det;
    out[7] = (-a21 * a00 + a01 * a20) * det;
    out[8] = (a11 * a00 - a01 * a10) * det;
    return out;
};

/**
 * Calculates the adjugate of a mat3
 *
 * @param {mat3} out the receiving matrix
 * @param {mat3} a the source matrix
 * @returns {mat3} out
 */
mat3.adjoint = function(out, a) {
    var a00 = a[0], a01 = a[1], a02 = a[2],
        a10 = a[3], a11 = a[4], a12 = a[5],
        a20 = a[6], a21 = a[7], a22 = a[8];

    out[0] = (a11 * a22 - a12 * a21);
    out[1] = (a02 * a21 - a01 * a22);
    out[2] = (a01 * a12 - a02 * a11);
    out[3] = (a12 * a20 - a10 * a22);
    out[4] = (a00 * a22 - a02 * a20);
    out[5] = (a02 * a10 - a00 * a12);
    out[6] = (a10 * a21 - a11 * a20);
    out[7] = (a01 * a20 - a00 * a21);
    out[8] = (a00 * a11 - a01 * a10);
    return out;
};

/**
 * Calculates the determinant of a mat3
 *
 * @param {mat3} a the source matrix
 * @returns {Number} determinant of a
 */
mat3.determinant = function (a) {
    var a00 = a[0], a01 = a[1], a02 = a[2],
        a10 = a[3], a11 = a[4], a12 = a[5],
        a20 = a[6], a21 = a[7], a22 = a[8];

    return a00 * (a22 * a11 - a12 * a21) + a01 * (-a22 * a10 + a12 * a20) + a02 * (a21 * a10 - a11 * a20);
};

/**
 * Multiplies two mat3's
 *
 * @param {mat3} out the receiving matrix
 * @param {mat3} a the first operand
 * @param {mat3} b the second operand
 * @returns {mat3} out
 */
mat3.multiply = function (out, a, b) {
    var a00 = a[0], a01 = a[1], a02 = a[2],
        a10 = a[3], a11 = a[4], a12 = a[5],
        a20 = a[6], a21 = a[7], a22 = a[8],

        b00 = b[0], b01 = b[1], b02 = b[2],
        b10 = b[3], b11 = b[4], b12 = b[5],
        b20 = b[6], b21 = b[7], b22 = b[8];

    out[0] = b00 * a00 + b01 * a10 + b02 * a20;
    out[1] = b00 * a01 + b01 * a11 + b02 * a21;
    out[2] = b00 * a02 + b01 * a12 + b02 * a22;

    out[3] = b10 * a00 + b11 * a10 + b12 * a20;
    out[4] = b10 * a01 + b11 * a11 + b12 * a21;
    out[5] = b10 * a02 + b11 * a12 + b12 * a22;

    out[6] = b20 * a00 + b21 * a10 + b22 * a20;
    out[7] = b20 * a01 + b21 * a11 + b22 * a21;
    out[8] = b20 * a02 + b21 * a12 + b22 * a22;
    return out;
};

/**
 * Alias for {@link mat3.multiply}
 * @function
 */
mat3.mul = mat3.multiply;

/**
 * Translate a mat3 by the given vector
 *
 * @param {mat3} out the receiving matrix
 * @param {mat3} a the matrix to translate
 * @param {vec2} v vector to translate by
 * @returns {mat3} out
 */
mat3.translate = function(out, a, v) {
    var a00 = a[0], a01 = a[1], a02 = a[2],
        a10 = a[3], a11 = a[4], a12 = a[5],
        a20 = a[6], a21 = a[7], a22 = a[8],
        x = v[0], y = v[1];

    out[0] = a00;
    out[1] = a01;
    out[2] = a02;

    out[3] = a10;
    out[4] = a11;
    out[5] = a12;

    out[6] = x * a00 + y * a10 + a20;
    out[7] = x * a01 + y * a11 + a21;
    out[8] = x * a02 + y * a12 + a22;
    return out;
};

/**
 * Rotates a mat3 by the given angle
 *
 * @param {mat3} out the receiving matrix
 * @param {mat3} a the matrix to rotate
 * @param {Number} rad the angle to rotate the matrix by
 * @returns {mat3} out
 */
mat3.rotate = function (out, a, rad) {
    var a00 = a[0], a01 = a[1], a02 = a[2],
        a10 = a[3], a11 = a[4], a12 = a[5],
        a20 = a[6], a21 = a[7], a22 = a[8],

        s = Math.sin(rad),
        c = Math.cos(rad);

    out[0] = c * a00 + s * a10;
    out[1] = c * a01 + s * a11;
    out[2] = c * a02 + s * a12;

    out[3] = c * a10 - s * a00;
    out[4] = c * a11 - s * a01;
    out[5] = c * a12 - s * a02;

    out[6] = a20;
    out[7] = a21;
    out[8] = a22;
    return out;
};

/**
 * Scales the mat3 by the dimensions in the given vec2
 *
 * @param {mat3} out the receiving matrix
 * @param {mat3} a the matrix to rotate
 * @param {vec2} v the vec2 to scale the matrix by
 * @returns {mat3} out
 **/
mat3.scale = function(out, a, v) {
    var x = v[0], y = v[1];

    out[0] = x * a[0];
    out[1] = x * a[1];
    out[2] = x * a[2];

    out[3] = y * a[3];
    out[4] = y * a[4];
    out[5] = y * a[5];

    out[6] = a[6];
    out[7] = a[7];
    out[8] = a[8];
    return out;
};

/**
 * Copies the values from a mat2d into a mat3
 *
 * @param {mat3} out the receiving matrix
 * @param {mat2d} a the matrix to copy
 * @returns {mat3} out
 **/
mat3.fromMat2d = function(out, a) {
    out[0] = a[0];
    out[1] = a[1];
    out[2] = 0;

    out[3] = a[2];
    out[4] = a[3];
    out[5] = 0;

    out[6] = a[4];
    out[7] = a[5];
    out[8] = 1;
    return out;
};

/**
* Calculates a 3x3 matrix from the given quaternion
*
* @param {mat3} out mat3 receiving operation result
* @param {quat} q Quaternion to create matrix from
*
* @returns {mat3} out
*/
mat3.fromQuat = function (out, q) {
    var x = q[0], y = q[1], z = q[2], w = q[3],
        x2 = x + x,
        y2 = y + y,
        z2 = z + z,

        xx = x * x2,
        yx = y * x2,
        yy = y * y2,
        zx = z * x2,
        zy = z * y2,
        zz = z * z2,
        wx = w * x2,
        wy = w * y2,
        wz = w * z2;

    out[0] = 1 - yy - zz;
    out[3] = yx - wz;
    out[6] = zx + wy;

    out[1] = yx + wz;
    out[4] = 1 - xx - zz;
    out[7] = zy - wx;

    out[2] = zx - wy;
    out[5] = zy + wx;
    out[8] = 1 - xx - yy;

    return out;
};

/**
* Calculates a 3x3 normal matrix (transpose inverse) from the 4x4 matrix
*
* @param {mat3} out mat3 receiving operation result
* @param {mat4} a Mat4 to derive the normal matrix from
*
* @returns {mat3} out
*/
mat3.normalFromMat4 = function (out, a) {
    var a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3],
        a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7],
        a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11],
        a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15],

        b00 = a00 * a11 - a01 * a10,
        b01 = a00 * a12 - a02 * a10,
        b02 = a00 * a13 - a03 * a10,
        b03 = a01 * a12 - a02 * a11,
        b04 = a01 * a13 - a03 * a11,
        b05 = a02 * a13 - a03 * a12,
        b06 = a20 * a31 - a21 * a30,
        b07 = a20 * a32 - a22 * a30,
        b08 = a20 * a33 - a23 * a30,
        b09 = a21 * a32 - a22 * a31,
        b10 = a21 * a33 - a23 * a31,
        b11 = a22 * a33 - a23 * a32,

        // Calculate the determinant
        det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;

    if (!det) { 
        return null; 
    }
    det = 1.0 / det;

    out[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
    out[1] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
    out[2] = (a10 * b10 - a11 * b08 + a13 * b06) * det;

    out[3] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
    out[4] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
    out[5] = (a01 * b08 - a00 * b10 - a03 * b06) * det;

    out[6] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
    out[7] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
    out[8] = (a30 * b04 - a31 * b02 + a33 * b00) * det;

    return out;
};

/**
 * Returns a string representation of a mat3
 *
 * @param {mat3} mat matrix to represent as a string
 * @returns {String} string representation of the matrix
 */
mat3.str = function (a) {
    return 'mat3(' + a[0] + ', ' + a[1] + ', ' + a[2] + ', ' + 
                    a[3] + ', ' + a[4] + ', ' + a[5] + ', ' + 
                    a[6] + ', ' + a[7] + ', ' + a[8] + ')';
};

/**
 * Returns Frobenius norm of a mat3
 *
 * @param {mat3} a the matrix to calculate Frobenius norm of
 * @returns {Number} Frobenius norm
 */
mat3.frob = function (a) {
    return(Math.sqrt(Math.pow(a[0], 2) + Math.pow(a[1], 2) + Math.pow(a[2], 2) + Math.pow(a[3], 2) + Math.pow(a[4], 2) + Math.pow(a[5], 2) + Math.pow(a[6], 2) + Math.pow(a[7], 2) + Math.pow(a[8], 2)))
};

module.exports = mat3;
},{"common":6}],8:[function(_dereq_,module,exports){
/* Copyright (c) 2013, Brandon Jones, Colin MacKenzie IV. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/**
 * @class 4 Dimensional Vector
 * @name vec4
 */
var vec4 = {};

var GLMAT_ARRAY_TYPE = _dereq_('common').GLMAT_ARRAY_TYPE;
var GLMAT_RANDOM = _dereq_('common').GLMAT_RANDOM;

/**
 * Creates a new, empty vec4
 *
 * @returns {vec4} a new 4D vector
 */
vec4.create = function() {
    var out = new GLMAT_ARRAY_TYPE(4);
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    return out;
};

/**
 * Creates a new vec4 initialized with values from an existing vector
 *
 * @param {vec4} a vector to clone
 * @returns {vec4} a new 4D vector
 */
vec4.clone = function(a) {
    var out = new GLMAT_ARRAY_TYPE(4);
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
    return out;
};

/**
 * Creates a new vec4 initialized with the given values
 *
 * @param {Number} x X component
 * @param {Number} y Y component
 * @param {Number} z Z component
 * @param {Number} w W component
 * @returns {vec4} a new 4D vector
 */
vec4.fromValues = function(x, y, z, w) {
    var out = new GLMAT_ARRAY_TYPE(4);
    out[0] = x;
    out[1] = y;
    out[2] = z;
    out[3] = w;
    return out;
};

/**
 * Copy the values from one vec4 to another
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the source vector
 * @returns {vec4} out
 */
vec4.copy = function(out, a) {
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
    return out;
};

/**
 * Set the components of a vec4 to the given values
 *
 * @param {vec4} out the receiving vector
 * @param {Number} x X component
 * @param {Number} y Y component
 * @param {Number} z Z component
 * @param {Number} w W component
 * @returns {vec4} out
 */
vec4.set = function(out, x, y, z, w) {
    out[0] = x;
    out[1] = y;
    out[2] = z;
    out[3] = w;
    return out;
};

/**
 * Adds two vec4's
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {vec4} out
 */
vec4.add = function(out, a, b) {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
    out[3] = a[3] + b[3];
    return out;
};

/**
 * Subtracts vector b from vector a
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {vec4} out
 */
vec4.subtract = function(out, a, b) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
    out[3] = a[3] - b[3];
    return out;
};

/**
 * Alias for {@link vec4.subtract}
 * @function
 */
vec4.sub = vec4.subtract;

/**
 * Multiplies two vec4's
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {vec4} out
 */
vec4.multiply = function(out, a, b) {
    out[0] = a[0] * b[0];
    out[1] = a[1] * b[1];
    out[2] = a[2] * b[2];
    out[3] = a[3] * b[3];
    return out;
};

/**
 * Alias for {@link vec4.multiply}
 * @function
 */
vec4.mul = vec4.multiply;

/**
 * Divides two vec4's
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {vec4} out
 */
vec4.divide = function(out, a, b) {
    out[0] = a[0] / b[0];
    out[1] = a[1] / b[1];
    out[2] = a[2] / b[2];
    out[3] = a[3] / b[3];
    return out;
};

/**
 * Alias for {@link vec4.divide}
 * @function
 */
vec4.div = vec4.divide;

/**
 * Returns the minimum of two vec4's
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {vec4} out
 */
vec4.min = function(out, a, b) {
    out[0] = Math.min(a[0], b[0]);
    out[1] = Math.min(a[1], b[1]);
    out[2] = Math.min(a[2], b[2]);
    out[3] = Math.min(a[3], b[3]);
    return out;
};

/**
 * Returns the maximum of two vec4's
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {vec4} out
 */
vec4.max = function(out, a, b) {
    out[0] = Math.max(a[0], b[0]);
    out[1] = Math.max(a[1], b[1]);
    out[2] = Math.max(a[2], b[2]);
    out[3] = Math.max(a[3], b[3]);
    return out;
};

/**
 * Scales a vec4 by a scalar number
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the vector to scale
 * @param {Number} b amount to scale the vector by
 * @returns {vec4} out
 */
vec4.scale = function(out, a, b) {
    out[0] = a[0] * b;
    out[1] = a[1] * b;
    out[2] = a[2] * b;
    out[3] = a[3] * b;
    return out;
};

/**
 * Adds two vec4's after scaling the second operand by a scalar value
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @param {Number} scale the amount to scale b by before adding
 * @returns {vec4} out
 */
vec4.scaleAndAdd = function(out, a, b, scale) {
    out[0] = a[0] + (b[0] * scale);
    out[1] = a[1] + (b[1] * scale);
    out[2] = a[2] + (b[2] * scale);
    out[3] = a[3] + (b[3] * scale);
    return out;
};

/**
 * Calculates the euclidian distance between two vec4's
 *
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {Number} distance between a and b
 */
vec4.distance = function(a, b) {
    var x = b[0] - a[0],
        y = b[1] - a[1],
        z = b[2] - a[2],
        w = b[3] - a[3];
    return Math.sqrt(x*x + y*y + z*z + w*w);
};

/**
 * Alias for {@link vec4.distance}
 * @function
 */
vec4.dist = vec4.distance;

/**
 * Calculates the squared euclidian distance between two vec4's
 *
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {Number} squared distance between a and b
 */
vec4.squaredDistance = function(a, b) {
    var x = b[0] - a[0],
        y = b[1] - a[1],
        z = b[2] - a[2],
        w = b[3] - a[3];
    return x*x + y*y + z*z + w*w;
};

/**
 * Alias for {@link vec4.squaredDistance}
 * @function
 */
vec4.sqrDist = vec4.squaredDistance;

/**
 * Calculates the length of a vec4
 *
 * @param {vec4} a vector to calculate length of
 * @returns {Number} length of a
 */
vec4.length = function (a) {
    var x = a[0],
        y = a[1],
        z = a[2],
        w = a[3];
    return Math.sqrt(x*x + y*y + z*z + w*w);
};

/**
 * Alias for {@link vec4.length}
 * @function
 */
vec4.len = vec4.length;

/**
 * Calculates the squared length of a vec4
 *
 * @param {vec4} a vector to calculate squared length of
 * @returns {Number} squared length of a
 */
vec4.squaredLength = function (a) {
    var x = a[0],
        y = a[1],
        z = a[2],
        w = a[3];
    return x*x + y*y + z*z + w*w;
};

/**
 * Alias for {@link vec4.squaredLength}
 * @function
 */
vec4.sqrLen = vec4.squaredLength;

/**
 * Negates the components of a vec4
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a vector to negate
 * @returns {vec4} out
 */
vec4.negate = function(out, a) {
    out[0] = -a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    out[3] = -a[3];
    return out;
};

/**
 * Normalize a vec4
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a vector to normalize
 * @returns {vec4} out
 */
vec4.normalize = function(out, a) {
    var x = a[0],
        y = a[1],
        z = a[2],
        w = a[3];
    var len = x*x + y*y + z*z + w*w;
    if (len > 0) {
        len = 1 / Math.sqrt(len);
        out[0] = a[0] * len;
        out[1] = a[1] * len;
        out[2] = a[2] * len;
        out[3] = a[3] * len;
    }
    return out;
};

/**
 * Calculates the dot product of two vec4's
 *
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @returns {Number} dot product of a and b
 */
vec4.dot = function (a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
};

/**
 * Performs a linear interpolation between two vec4's
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the first operand
 * @param {vec4} b the second operand
 * @param {Number} t interpolation amount between the two inputs
 * @returns {vec4} out
 */
vec4.lerp = function (out, a, b, t) {
    var ax = a[0],
        ay = a[1],
        az = a[2],
        aw = a[3];
    out[0] = ax + t * (b[0] - ax);
    out[1] = ay + t * (b[1] - ay);
    out[2] = az + t * (b[2] - az);
    out[3] = aw + t * (b[3] - aw);
    return out;
};

/**
 * Generates a random vector with the given scale
 *
 * @param {vec4} out the receiving vector
 * @param {Number} [scale] Length of the resulting vector. If ommitted, a unit vector will be returned
 * @returns {vec4} out
 */
vec4.random = function (out, scale) {
    scale = scale || 1.0;

    //TODO: This is a pretty awful way of doing this. Find something better.
    out[0] = GLMAT_RANDOM();
    out[1] = GLMAT_RANDOM();
    out[2] = GLMAT_RANDOM();
    out[3] = GLMAT_RANDOM();
    vec4.normalize(out, out);
    vec4.scale(out, out, scale);
    return out;
};

/**
 * Transforms the vec4 with a mat4.
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the vector to transform
 * @param {mat4} m matrix to transform with
 * @returns {vec4} out
 */
vec4.transformMat4 = function(out, a, m) {
    var x = a[0], y = a[1], z = a[2], w = a[3];
    out[0] = m[0] * x + m[4] * y + m[8] * z + m[12] * w;
    out[1] = m[1] * x + m[5] * y + m[9] * z + m[13] * w;
    out[2] = m[2] * x + m[6] * y + m[10] * z + m[14] * w;
    out[3] = m[3] * x + m[7] * y + m[11] * z + m[15] * w;
    return out;
};

/**
 * Transforms the vec4 with a quat
 *
 * @param {vec4} out the receiving vector
 * @param {vec4} a the vector to transform
 * @param {quat} q quaternion to transform with
 * @returns {vec4} out
 */
vec4.transformQuat = function(out, a, q) {
    var x = a[0], y = a[1], z = a[2],
        qx = q[0], qy = q[1], qz = q[2], qw = q[3],

        // calculate quat * vec
        ix = qw * x + qy * z - qz * y,
        iy = qw * y + qz * x - qx * z,
        iz = qw * z + qx * y - qy * x,
        iw = -qx * x - qy * y - qz * z;

    // calculate result * inverse quat
    out[0] = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    out[1] = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    out[2] = iz * qw + iw * -qz + ix * -qy - iy * -qx;
    return out;
};

/**
 * Perform some operation over an array of vec4s.
 *
 * @param {Array} a the array of vectors to iterate over
 * @param {Number} stride Number of elements between the start of each vec4. If 0 assumes tightly packed
 * @param {Number} offset Number of elements to skip at the beginning of the array
 * @param {Number} count Number of vec2s to iterate over. If 0 iterates over entire array
 * @param {Function} fn Function to call for each vector in the array
 * @param {Object} [arg] additional argument to pass to fn
 * @returns {Array} a
 * @function
 */
vec4.forEach = (function() {
    var vec = vec4.create();

    return function(a, stride, offset, count, fn, arg) {
        var i, l;
        if(!stride) {
            stride = 4;
        }

        if(!offset) {
            offset = 0;
        }
        
        if(count) {
            l = Math.min((count * stride) + offset, a.length);
        } else {
            l = a.length;
        }

        for(i = offset; i < l; i += stride) {
            vec[0] = a[i]; vec[1] = a[i+1]; vec[2] = a[i+2]; vec[3] = a[i+3];
            fn(vec, vec, arg);
            a[i] = vec[0]; a[i+1] = vec[1]; a[i+2] = vec[2]; a[i+3] = vec[3];
        }
        
        return a;
    };
})();

/**
 * Returns a string representation of a vector
 *
 * @param {vec4} vec vector to represent as a string
 * @returns {String} string representation of the vector
 */
vec4.str = function (a) {
    return 'vec4(' + a[0] + ', ' + a[1] + ', ' + a[2] + ', ' + a[3] + ')';
};

module.exports = vec4;


},{"common":6}],9:[function(_dereq_,module,exports){
/* Copyright (c) 2013, Brandon Jones, Colin MacKenzie IV. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/**
 * @class 3 Dimensional Vector
 * @name vec3
 */
var vec3 = {};

var GLMAT_ARRAY_TYPE = _dereq_('common').GLMAT_ARRAY_TYPE;
var GLMAT_RANDOM = _dereq_('common').GLMAT_RANDOM;

/**
 * Creates a new, empty vec3
 *
 * @returns {vec3} a new 3D vector
 */
vec3.create = function() {
    var out = new GLMAT_ARRAY_TYPE(3);
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    return out;
};

/**
 * Creates a new vec3 initialized with values from an existing vector
 *
 * @param {vec3} a vector to clone
 * @returns {vec3} a new 3D vector
 */
vec3.clone = function(a) {
    var out = new GLMAT_ARRAY_TYPE(3);
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    return out;
};

/**
 * Creates a new vec3 initialized with the given values
 *
 * @param {Number} x X component
 * @param {Number} y Y component
 * @param {Number} z Z component
 * @returns {vec3} a new 3D vector
 */
vec3.fromValues = function(x, y, z) {
    var out = new GLMAT_ARRAY_TYPE(3);
    out[0] = x;
    out[1] = y;
    out[2] = z;
    return out;
};

/**
 * Copy the values from one vec3 to another
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the source vector
 * @returns {vec3} out
 */
vec3.copy = function(out, a) {
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    return out;
};

/**
 * Set the components of a vec3 to the given values
 *
 * @param {vec3} out the receiving vector
 * @param {Number} x X component
 * @param {Number} y Y component
 * @param {Number} z Z component
 * @returns {vec3} out
 */
vec3.set = function(out, x, y, z) {
    out[0] = x;
    out[1] = y;
    out[2] = z;
    return out;
};

/**
 * Adds two vec3's
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {vec3} out
 */
vec3.add = function(out, a, b) {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
    return out;
};

/**
 * Subtracts vector b from vector a
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {vec3} out
 */
vec3.subtract = function(out, a, b) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
    return out;
};

/**
 * Alias for {@link vec3.subtract}
 * @function
 */
vec3.sub = vec3.subtract;

/**
 * Multiplies two vec3's
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {vec3} out
 */
vec3.multiply = function(out, a, b) {
    out[0] = a[0] * b[0];
    out[1] = a[1] * b[1];
    out[2] = a[2] * b[2];
    return out;
};

/**
 * Alias for {@link vec3.multiply}
 * @function
 */
vec3.mul = vec3.multiply;

/**
 * Divides two vec3's
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {vec3} out
 */
vec3.divide = function(out, a, b) {
    out[0] = a[0] / b[0];
    out[1] = a[1] / b[1];
    out[2] = a[2] / b[2];
    return out;
};

/**
 * Alias for {@link vec3.divide}
 * @function
 */
vec3.div = vec3.divide;

/**
 * Returns the minimum of two vec3's
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {vec3} out
 */
vec3.min = function(out, a, b) {
    out[0] = Math.min(a[0], b[0]);
    out[1] = Math.min(a[1], b[1]);
    out[2] = Math.min(a[2], b[2]);
    return out;
};

/**
 * Returns the maximum of two vec3's
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {vec3} out
 */
vec3.max = function(out, a, b) {
    out[0] = Math.max(a[0], b[0]);
    out[1] = Math.max(a[1], b[1]);
    out[2] = Math.max(a[2], b[2]);
    return out;
};

/**
 * Scales a vec3 by a scalar number
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the vector to scale
 * @param {Number} b amount to scale the vector by
 * @returns {vec3} out
 */
vec3.scale = function(out, a, b) {
    out[0] = a[0] * b;
    out[1] = a[1] * b;
    out[2] = a[2] * b;
    return out;
};

/**
 * Adds two vec3's after scaling the second operand by a scalar value
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @param {Number} scale the amount to scale b by before adding
 * @returns {vec3} out
 */
vec3.scaleAndAdd = function(out, a, b, scale) {
    out[0] = a[0] + (b[0] * scale);
    out[1] = a[1] + (b[1] * scale);
    out[2] = a[2] + (b[2] * scale);
    return out;
};

/**
 * Calculates the euclidian distance between two vec3's
 *
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {Number} distance between a and b
 */
vec3.distance = function(a, b) {
    var x = b[0] - a[0],
        y = b[1] - a[1],
        z = b[2] - a[2];
    return Math.sqrt(x*x + y*y + z*z);
};

/**
 * Alias for {@link vec3.distance}
 * @function
 */
vec3.dist = vec3.distance;

/**
 * Calculates the squared euclidian distance between two vec3's
 *
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {Number} squared distance between a and b
 */
vec3.squaredDistance = function(a, b) {
    var x = b[0] - a[0],
        y = b[1] - a[1],
        z = b[2] - a[2];
    return x*x + y*y + z*z;
};

/**
 * Alias for {@link vec3.squaredDistance}
 * @function
 */
vec3.sqrDist = vec3.squaredDistance;

/**
 * Calculates the length of a vec3
 *
 * @param {vec3} a vector to calculate length of
 * @returns {Number} length of a
 */
vec3.length = function (a) {
    var x = a[0],
        y = a[1],
        z = a[2];
    return Math.sqrt(x*x + y*y + z*z);
};

/**
 * Alias for {@link vec3.length}
 * @function
 */
vec3.len = vec3.length;

/**
 * Calculates the squared length of a vec3
 *
 * @param {vec3} a vector to calculate squared length of
 * @returns {Number} squared length of a
 */
vec3.squaredLength = function (a) {
    var x = a[0],
        y = a[1],
        z = a[2];
    return x*x + y*y + z*z;
};

/**
 * Alias for {@link vec3.squaredLength}
 * @function
 */
vec3.sqrLen = vec3.squaredLength;

/**
 * Negates the components of a vec3
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a vector to negate
 * @returns {vec3} out
 */
vec3.negate = function(out, a) {
    out[0] = -a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    return out;
};

/**
 * Normalize a vec3
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a vector to normalize
 * @returns {vec3} out
 */
vec3.normalize = function(out, a) {
    var x = a[0],
        y = a[1],
        z = a[2];
    var len = x*x + y*y + z*z;
    if (len > 0) {
        //TODO: evaluate use of glm_invsqrt here?
        len = 1 / Math.sqrt(len);
        out[0] = a[0] * len;
        out[1] = a[1] * len;
        out[2] = a[2] * len;
    }
    return out;
};

/**
 * Calculates the dot product of two vec3's
 *
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {Number} dot product of a and b
 */
vec3.dot = function (a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
};

/**
 * Computes the cross product of two vec3's
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {vec3} out
 */
vec3.cross = function(out, a, b) {
    var ax = a[0], ay = a[1], az = a[2],
        bx = b[0], by = b[1], bz = b[2];

    out[0] = ay * bz - az * by;
    out[1] = az * bx - ax * bz;
    out[2] = ax * by - ay * bx;
    return out;
};

/**
 * Performs a linear interpolation between two vec3's
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @param {Number} t interpolation amount between the two inputs
 * @returns {vec3} out
 */
vec3.lerp = function (out, a, b, t) {
    var ax = a[0],
        ay = a[1],
        az = a[2];
    out[0] = ax + t * (b[0] - ax);
    out[1] = ay + t * (b[1] - ay);
    out[2] = az + t * (b[2] - az);
    return out;
};

/**
 * Generates a random vector with the given scale
 *
 * @param {vec3} out the receiving vector
 * @param {Number} [scale] Length of the resulting vector. If ommitted, a unit vector will be returned
 * @returns {vec3} out
 */
vec3.random = function (out, scale) {
    scale = scale || 1.0;

    var r = GLMAT_RANDOM() * 2.0 * Math.PI;
    var z = (GLMAT_RANDOM() * 2.0) - 1.0;
    var zScale = Math.sqrt(1.0-z*z) * scale;

    out[0] = Math.cos(r) * zScale;
    out[1] = Math.sin(r) * zScale;
    out[2] = z * scale;
    return out;
};

/**
 * Transforms the vec3 with a mat4.
 * 4th vector component is implicitly '1'
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the vector to transform
 * @param {mat4} m matrix to transform with
 * @returns {vec3} out
 */
vec3.transformMat4 = function(out, a, m) {
    var x = a[0], y = a[1], z = a[2];
    out[0] = m[0] * x + m[4] * y + m[8] * z + m[12];
    out[1] = m[1] * x + m[5] * y + m[9] * z + m[13];
    out[2] = m[2] * x + m[6] * y + m[10] * z + m[14];
    return out;
};

/**
 * Transforms the vec3 with a mat3.
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the vector to transform
 * @param {mat4} m the 3x3 matrix to transform with
 * @returns {vec3} out
 */
vec3.transformMat3 = function(out, a, m) {
    var x = a[0], y = a[1], z = a[2];
    out[0] = x * m[0] + y * m[3] + z * m[6];
    out[1] = x * m[1] + y * m[4] + z * m[7];
    out[2] = x * m[2] + y * m[5] + z * m[8];
    return out;
};

/**
 * Transforms the vec3 with a quat
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the vector to transform
 * @param {quat} q quaternion to transform with
 * @returns {vec3} out
 */
vec3.transformQuat = function(out, a, q) {
    // benchmarks: http://jsperf.com/quaternion-transform-vec3-implementations

    var x = a[0], y = a[1], z = a[2],
        qx = q[0], qy = q[1], qz = q[2], qw = q[3],

        // calculate quat * vec
        ix = qw * x + qy * z - qz * y,
        iy = qw * y + qz * x - qx * z,
        iz = qw * z + qx * y - qy * x,
        iw = -qx * x - qy * y - qz * z;

    // calculate result * inverse quat
    out[0] = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    out[1] = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    out[2] = iz * qw + iw * -qz + ix * -qy - iy * -qx;
    return out;
};

/*
* Rotate a 3D vector around the x-axis
* @param {vec3} out The receiving vec3
* @param {vec3} a The vec3 point to rotate
* @param {vec3} b The origin of the rotation
* @param {Number} c The angle of rotation
* @returns {vec3} out
*/
vec3.rotateX = function(out, a, b, c){
   var p = [], r=[];
	  //Translate point to the origin
	  p[0] = a[0] - b[0];
	  p[1] = a[1] - b[1];
  	p[2] = a[2] - b[2];

	  //perform rotation
	  r[0] = p[0];
	  r[1] = p[1]*Math.cos(c) - p[2]*Math.sin(c);
	  r[2] = p[1]*Math.sin(c) + p[2]*Math.cos(c);

	  //translate to correct position
	  out[0] = r[0] + b[0];
	  out[1] = r[1] + b[1];
	  out[2] = r[2] + b[2];

  	return out;
};

/*
* Rotate a 3D vector around the y-axis
* @param {vec3} out The receiving vec3
* @param {vec3} a The vec3 point to rotate
* @param {vec3} b The origin of the rotation
* @param {Number} c The angle of rotation
* @returns {vec3} out
*/
vec3.rotateY = function(out, a, b, c){
  	var p = [], r=[];
  	//Translate point to the origin
  	p[0] = a[0] - b[0];
  	p[1] = a[1] - b[1];
  	p[2] = a[2] - b[2];
  
  	//perform rotation
  	r[0] = p[2]*Math.sin(c) + p[0]*Math.cos(c);
  	r[1] = p[1];
  	r[2] = p[2]*Math.cos(c) - p[0]*Math.sin(c);
  
  	//translate to correct position
  	out[0] = r[0] + b[0];
  	out[1] = r[1] + b[1];
  	out[2] = r[2] + b[2];
  
  	return out;
};

/*
* Rotate a 3D vector around the z-axis
* @param {vec3} out The receiving vec3
* @param {vec3} a The vec3 point to rotate
* @param {vec3} b The origin of the rotation
* @param {Number} c The angle of rotation
* @returns {vec3} out
*/
vec3.rotateZ = function(out, a, b, c){
  	var p = [], r=[];
  	//Translate point to the origin
  	p[0] = a[0] - b[0];
  	p[1] = a[1] - b[1];
  	p[2] = a[2] - b[2];
  
  	//perform rotation
  	r[0] = p[0]*Math.cos(c) - p[1]*Math.sin(c);
  	r[1] = p[0]*Math.sin(c) + p[1]*Math.cos(c);
  	r[2] = p[2];
  
  	//translate to correct position
  	out[0] = r[0] + b[0];
  	out[1] = r[1] + b[1];
  	out[2] = r[2] + b[2];
  
  	return out;
};

/**
 * Perform some operation over an array of vec3s.
 *
 * @param {Array} a the array of vectors to iterate over
 * @param {Number} stride Number of elements between the start of each vec3. If 0 assumes tightly packed
 * @param {Number} offset Number of elements to skip at the beginning of the array
 * @param {Number} count Number of vec3s to iterate over. If 0 iterates over entire array
 * @param {Function} fn Function to call for each vector in the array
 * @param {Object} [arg] additional argument to pass to fn
 * @returns {Array} a
 * @function
 */
vec3.forEach = (function() {
    var vec = vec3.create();

    return function(a, stride, offset, count, fn, arg) {
        var i, l;
        if(!stride) {
            stride = 3;
        }

        if(!offset) {
            offset = 0;
        }
        
        if(count) {
            l = Math.min((count * stride) + offset, a.length);
        } else {
            l = a.length;
        }

        for(i = offset; i < l; i += stride) {
            vec[0] = a[i]; vec[1] = a[i+1]; vec[2] = a[i+2];
            fn(vec, vec, arg);
            a[i] = vec[0]; a[i+1] = vec[1]; a[i+2] = vec[2];
        }
        
        return a;
    };
})();

/**
 * Returns a string representation of a vector
 *
 * @param {vec3} vec vector to represent as a string
 * @returns {String} string representation of the vector
 */
vec3.str = function (a) {
    return 'vec3(' + a[0] + ', ' + a[1] + ', ' + a[2] + ')';
};

module.exports = vec3;


},{"common":10}],10:[function(_dereq_,module,exports){
arguments[4][6][0].apply(exports,arguments)
},{"dup":6}],11:[function(_dereq_,module,exports){
/*
 * Copyright 2015 Google Inc. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * TODO: Fix up all "new THREE" instantiations to improve performance.
 */

// should require
// Util SensorSample 
// quat, vec3 
var vec3 = _dereq_('gl-matrix-vec3');
var quat = _dereq_('gl-matrix-quat');

//var Util = require('./util.js');
var SensorSample = _dereq_('./sensor-sample.js');

var DEBUG = false;

var MIN_TIMESTEP = 0.001;
var MAX_TIMESTEP = 1;

// Helper method to validate the time steps of sensor timestamps.
function isTimestampDeltaValid(timestampDeltaS) {
  if (isNaN(timestampDeltaS)) {
    return false;
  }
  if (timestampDeltaS <= MIN_TIMESTEP) {
    return false;
  }
  if (timestampDeltaS > MAX_TIMESTEP) {
    return false;
  }
  return true;
}


/**
 * An implementation of a simple complementary filter, which fuses gyroscope and
 * accelerometer data from the 'devicemotion' event.
 *
 * Accelerometer data is very noisy, but stable over the long term.
 * Gyroscope data is smooth, but tends to drift over the long term.
 *
 * This fusion is relatively simple:
 * 1. Get orientation estimates from accelerometer by applying a low-pass filter
 *    on that data.
 * 2. Get orientation estimates from gyroscope by integrating over time.
 * 3. Combine the two estimates, weighing (1) in the long term, but (2) for the
 *    short term.
 */
function ComplementaryFilter(kFilter) {
  this.kFilter = kFilter;

  // Raw sensor measurements.
  this.currentAccelMeasurement = new SensorSample();
  this.currentGyroMeasurement = new SensorSample();
  this.previousGyroMeasurement = new SensorSample();

  // Current filter orientation
/*
  this.filterQ = new THREE.Quaternion();
  this.previousFilterQ = new THREE.Quaternion();
*/
  this.filterQ = quat.create();
  this.previousFilterQ = quat.create();
  
  // Orientation based on the accelerometer.
//  this.accelQ = new THREE.Quaternion();
  this.accelQ = quat.create();

  // Whether or not the orientation has been initialized.
  this.isOrientationInitialized = false;
  // Running estimate of gravity based on the current orientation.
//  this.estimatedGravity = new THREE.Vector3();
  this.estimatedGravity = vec3.create();

  // Measured gravity based on accelerometer.
//  this.measuredGravity = new THREE.Vector3();
  this.measuredGravity = vec3.create();

  // Debug only quaternion of gyro-based orientation.
//  this.gyroIntegralQ = new THREE.Quaternion();
  this.gyroIntegralQ = quat.create();

}

ComplementaryFilter.prototype.addAccelMeasurement = function(vector, timestampS) {
  this.currentAccelMeasurement.set(vector, timestampS);
};

ComplementaryFilter.prototype.addGyroMeasurement = function(vector, timestampS) {
  this.currentGyroMeasurement.set(vector, timestampS);

  var deltaT = timestampS - this.previousGyroMeasurement.timestampS;
  if (isTimestampDeltaValid(deltaT)) {
    this.run_();
  }
  
  this.previousGyroMeasurement.copy(this.currentGyroMeasurement);
};

ComplementaryFilter.prototype.run_ = function() {
  this.accelQ = this.accelToQuaternion_(this.currentAccelMeasurement.sample);

  if (!this.isOrientationInitialized) {
//    this.previousFilterQ.copy(this.accelQ);
    quat.copy(this.previousFilterQ, this.accelQ);

    this.isOrientationInitialized = true;
    return;
  }

  var deltaT = this.currentGyroMeasurement.timestampS -
      this.previousGyroMeasurement.timestampS;

  // Convert gyro rotation vector to a quaternion delta.
  var gyroDeltaQ = this.gyroToQuaternionDelta_(this.currentGyroMeasurement.sample, deltaT);
//  this.gyroIntegralQ.multiply(gyroDeltaQ);
  quat.multiply(this.gyroIntegralQ, this.gyroIntegralQ, gyroDeltaQ);


  // filter_1 = K * (filter_0 + gyro * dT) + (1 - K) * accel.
/*
  this.filterQ.copy(this.previousFilterQ);
  this.filterQ.multiply(gyroDeltaQ);
*/
  quat.copy(this.filterQ, this.previousFilterQ);
  quat.multiply(this.filterQ, this.filterQ, gyroDeltaQ);
  
  // Calculate the delta between the current estimated gravity and the real
  // gravity vector from accelerometer.
/*
  var invFilterQ = new THREE.Quaternion();
  invFilterQ.copy(this.filterQ);
  invFilterQ.inverse();
*/
  var invFilterQ = quat.create();
  quat.copy(invFilterQ, this.filterQ);
  quat.conjugate(invFilterQ, invFilterQ)
  
/*
  this.estimatedGravity.set(0, 0, -1);
  this.estimatedGravity.applyQuaternion(invFilterQ);
  this.estimatedGravity.normalize();

  this.measuredGravity.copy(this.currentAccelMeasurement.sample);
  this.measuredGravity.normalize();
*/
  vec3.set(this.estimatedGravity, 0, 0, -1);
  vec3.transformQuat(this.estimatedGravity, this.estimatedGravity, invFilterQ);
  vec3.normalize(this.estimatedGravity, this.estimatedGravity);

  vec3.copy(this.measuredGravity, this.currentAccelMeasurement.sample);
  vec3.normalize(this.measuredGravity, this.measuredGravity);
  
  // Compare estimated gravity with measured gravity, get the delta quaternion
  // between the two.
/*
  var deltaQ = new THREE.Quaternion();
  deltaQ.setFromUnitVectors(this.estimatedGravity, this.measuredGravity);
  deltaQ.inverse();
*/

  var deltaQ = setFromUnitVectors(this.estimatedGravity, this.measuredGravity);
  quat.conjugate(deltaQ, deltaQ)
  
  if (DEBUG) {
/*
    console.log('Delta: NaN rad, G_est: (%s, %s, %s), G_meas: (%s, %s, %s)',
                //Util.getQuaternionAngle(deltaQ),
                (this.estimatedGravity.x).toFixed(1),
                (this.estimatedGravity.y).toFixed(1),
                (this.estimatedGravity.z).toFixed(1),
                (this.measuredGravity.x).toFixed(1),
                (this.measuredGravity.y).toFixed(1),
                (this.measuredGravity.z).toFixed(1));
*/
  }

  // Calculate the SLERP target: current orientation plus the measured-estimated
  // quaternion delta.
/*
  var targetQ = new THREE.Quaternion();
  targetQ.copy(this.filterQ);
  targetQ.multiply(deltaQ);
*/
  var targetQ = quat.create();
  quat.copy(targetQ, this.filterQ);
  quat.multiply(targetQ, targetQ, deltaQ);
  
  // SLERP factor: 0 is pure gyro, 1 is pure accel.
//  this.filterQ.slerp(targetQ, 1 - this.kFilter);
  quat.slerp(this.filterQ, this.filterQ, targetQ,  1 - this.kFilter);

//  this.previousFilterQ.copy(this.filterQ);
  quat.copy(this.previousFilterQ, this.filterQ);
};

ComplementaryFilter.prototype.getOrientation = function() {
  return this.filterQ;
};

ComplementaryFilter.prototype.accelToQuaternion_ = function(accel) {
/*
  var normAccel = new THREE.Vector3();
  normAccel.copy(accel);
  normAccel.normalize();
  var quat = new THREE.Quaternion();
  quat.setFromUnitVectors(new THREE.Vector3(0, 0, -1), normAccel);
  return quat;
*/
  var normAccel = vec3.create();
  vec3.copy(normAccel, accel);
  vec3.normalize(normAccel, normAccel);
  var quat = setFromUnitVectors( vec3.fromValues(0, 0, -1), normAccel);
  return quat;
};

ComplementaryFilter.prototype.gyroToQuaternionDelta_ = function(gyro, dt) {
  // Extract axis and angle from the gyroscope data.
/*
  var quat = new THREE.Quaternion();
  var axis = new THREE.Vector3();
*/
  var out = quat.create();
  var axis = vec3.create();
  
/*
  axis.copy(gyro);
  axis.normalize();
  quat.setFromAxisAngle(axis, gyro.length() * dt);
*/
  vec3.copy(axis, gyro);
  vec3.normalize(axis, axis);
  quat.setAxisAngle(out, axis, vec3.length(gyro) * dt);
  
  return out;
};

// gl-matrix ver.
var setFromUnitVectors = function() {

	// http://lolengine.net/blog/2014/02/24/quaternion-from-two-vectors-final

	// assumes direction vectors vFrom and vTo are normalized

	var v1, r;

	var EPS = 0.000001;

	return function setFromUnitVectors( vFrom, vTo ) {

//		if ( v1 === undefined ) v1 = new THREE.Vector3();
		if ( v1 === undefined ) v1 = vec3.create();
		

//		r = vFrom.dot( vTo ) + 1;
		r = vec3.dot( vFrom, vTo ) + 1;

		if ( r < EPS ) {

			r = 0;

//			if ( Math.abs( vFrom.x ) > Math.abs( vFrom.z ) ) {
			if ( Math.abs( vFrom[0] ) > Math.abs( vFrom[2] ) ) {
//				v1.set( - vFrom.y, vFrom.x, 0 );
				vec3.set(v1,  - vFrom[1], vFrom[0], 0 );

			} else {

//				v1.set( 0, - vFrom.z, vFrom.y );
				vec3.set(v1, 0, - vFrom[2], vFrom[1] );

			}

		} else {

//			v1.crossVectors( vFrom, vTo );
			vec3.cross( v1, vFrom, vTo );

		}

		// outQ
		var out = quat.fromValues(v1[0], v1[1], v1[2], r);

		return quat.normalize(out, out);

	};

}()
	
module.exports = ComplementaryFilter;
},{"./sensor-sample.js":14,"gl-matrix-quat":5,"gl-matrix-vec3":9}],12:[function(_dereq_,module,exports){
// vec3
// quat
var vec3 = _dereq_('gl-matrix-vec3');
var quat = _dereq_('gl-matrix-quat');

var GyroNorm = _dereq_('../lib/gyronorm/gyronorm.js');

var ComplementaryFilter = _dereq_('./complementary-filter.js');
var PosePredictor = _dereq_('./pose-predictor.js');


var MIN_TIMESTEP = 0.001;
var MAX_TIMESTEP = 1;

var flag = true;
function ComplementaryOrientation() {
/*
  this.accelerometer = new THREE.Vector3();
  this.gyroscope = new THREE.Vector3();
*/
  this.accelerometer = vec3.create();
  this.gyroscope = vec3.create();


			

  var scope = this;
  this.gn = new GyroNorm();

  this.gn.init({
    frequency: 1000/60,                   // ( How often the object sends the values - milliseconds )
    gravityNormalized:true,         // ( If the garvity related values to be normalized )
    orientationBase:GyroNorm.GAME,      // ( Can be GyroNorm.GAME or GyroNorm.WORLD. gn.GAME returns orientation values with respect to the head direction of the device. gn.WORLD returns the orientation values with respect to the actual north direction of the world. )
    decimalCount:2,                 // ( How many digits after the decimal point will there be in the return values )
    logger:null,                    // ( Function to be called to log messages from gyronorm.js )
    screenAdjusted:false            // ( If set to true it will return screen adjusted values. )
})
.then(function(){

		
	    scope.gn.start(function(data){
		    
			
			scope.onDeviceMotionChange_({
				accelerationIncludingGravity: {
					x: data.dm.gx,
					y: data.dm.gy,
					z: data.dm.gz,
				},
				timeStamp: data.dm.timeStamp,
				rotationRate: {
					alpha: data.dm.alpha,
					beta: data.dm.beta,
					gamma: data.dm.gamma
				}
			});

	    });
	});


//  window.addEventListener('devicemotion', this.onDeviceMotionChange_.bind(this));
  window.addEventListener('orientationchange', this.onScreenOrientationChange_.bind(this));

  // 아래 두 객체의 인수를 조정해서 감도를 제어한다.
  this.filter = new ComplementaryFilter(0.7); // kFilter 의 값이 1에 가까울 수록 민감도가 떨어진다.
  this.posePredictor = new PosePredictor(0.050);

//  this.filterToWorldQ = new THREE.Quaternion();
  this.filterToWorldQ = quat.create();

  // Set the filter to world transform, but only for Android.
  if (isIOS()) {
//    this.filterToWorldQ.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI/2);
    quat.setAxisAngle(this.filterToWorldQ, vec3.fromValues(1, 0, 0), Math.PI/2);
  } else {
//    this.filterToWorldQ.setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI/2);
    quat.setAxisAngle(this.filterToWorldQ, vec3.fromValues(1, 0, 0), -Math.PI/2);
  }

//  this.worldToScreenQ = new THREE.Quaternion();
  this.worldToScreenQ = quat.create();

  this.setScreenTransform_();
}

ComplementaryOrientation.prototype.onDeviceMotionChange_ = function(deviceMotion) {
  var accGravity = deviceMotion.accelerationIncludingGravity;
  var rotRate = deviceMotion.rotationRate;
  var timestampS = deviceMotion.timeStamp / 1000;

  var deltaS = timestampS - this.previousTimestampS;
  if (deltaS <= MIN_TIMESTEP || deltaS > MAX_TIMESTEP) {
    console.warn('Invalid timestamps detected. Time step between successive ' +
                 'gyroscope sensor samples is very small or not monotonic');
    this.previousTimestampS = timestampS;
    return;
  }
/*
  this.accelerometer.set(-accGravity.x, -accGravity.y, -accGravity.z);
  this.gyroscope.set(rotRate.alpha, rotRate.beta, rotRate.gamma);
*/
  vec3.set(this.accelerometer, -accGravity.x, -accGravity.y, -accGravity.z);
  vec3.set(this.gyroscope, rotRate.alpha, rotRate.beta, rotRate.gamma);
  
  // In iOS, rotationRate is reported in degrees, so we first convert to
  // radians.
  if (isIOS()) {
//     this.gyroscope.multiplyScalar(Math.PI / 180);
	vec3.scale(this.gyroscope, this.gyroscope, Math.PI / 180)
  }

  this.filter.addAccelMeasurement(this.accelerometer, timestampS);
  this.filter.addGyroMeasurement(this.gyroscope, timestampS);

  this.previousTimestampS = timestampS;
};

ComplementaryOrientation.prototype.onScreenOrientationChange_ =
    function(screenOrientation) {
  this.setScreenTransform_();
};

ComplementaryOrientation.prototype.setScreenTransform_ = function() {
  quat.set(this.worldToScreenQ, 0, 0, 0, 1);
  switch (window.orientation) {
    case 0:
      break;
    case 90:
//      this.worldToScreenQ.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI/2);
	    quat.setAxisAngle(this.worldToScreenQ, vec3.fromValues(0, 0, 1), -Math.PI/2);

      break;
    case -90: 
//      this.worldToScreenQ.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI/2);
	    quat.setAxisAngle(this.worldToScreenQ, vec3.fromValues(0, 0, 1), Math.PI/2);
      break;
    case 180:
      break;
  }
};

ComplementaryOrientation.prototype.getOrientation = function() {
  // Convert from filter space to the the same system used by the
  // deviceorientation event.
  var orientation = this.filter.getOrientation();

  // Predict orientation.
  this.predictedQ = this.posePredictor.getPrediction(orientation, this.gyroscope, this.previousTimestampS);

  // Convert to THREE coordinate system: -Z forward, Y up, X right.
//  var out = new THREE.Quaternion();
  var out = quat.create();

/*
  out.copy(this.filterToWorldQ);
  out.multiply(this.predictedQ);
  out.multiply(this.worldToScreenQ);
*/
  quat.copy(out, this.filterToWorldQ);
  quat.multiply(out, out, this.predictedQ);
  quat.multiply(out, out, this.worldToScreenQ)

  return out;
};

function isIOS() {
	return /iPad|iPhone|iPod/.test(navigator.platform);
}

window.ComplementaryOrientation = module.exports = ComplementaryOrientation;
},{"../lib/gyronorm/gyronorm.js":2,"./complementary-filter.js":11,"./pose-predictor.js":13,"gl-matrix-quat":5,"gl-matrix-vec3":9}],13:[function(_dereq_,module,exports){
/*
 * Copyright 2015 Google Inc. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
var DEBUG = false;

/**
 * Given an orientation and the gyroscope data, predicts the future orientation
 * of the head. This makes rendering appear faster.
 *
 * Also see: http://msl.cs.uiuc.edu/~lavalle/papers/LavYerKatAnt14.pdf
 *
 * @param {Number} predictionTimeS time from head movement to the appearance of
 * the corresponding image.
 */
 
// quat
var quat = _dereq_('gl-matrix-quat');
var vec3 = _dereq_('gl-matrix-vec3');


function PosePredictor(predictionTimeS) {
  this.predictionTimeS = predictionTimeS;

  // The quaternion corresponding to the previous state.
//  this.previousQ = new THREE.Quaternion();
  this.previousQ = quat.create();

  // Previous time a prediction occurred.
  this.previousTimestampS = null;

  // The delta quaternion that adjusts the current pose.
//  this.deltaQ = new THREE.Quaternion();
  this.deltaQ = quat.create();

  // The output quaternion.
//  this.outQ = new THREE.Quaternion();
  this.outQ = quat.create();

}

PosePredictor.prototype.getPrediction = function(currentQ, gyro, timestampS) {
  if (!this.previousTimestampS) {
//    this.previousQ.copy(currentQ);
  	quat.copy(this.previousQ, currentQ)
    this.previousTimestampS = timestampS;
    return currentQ;
  }

  // Calculate axis and angle based on gyroscope rotation rate data.
  var axis = vec3.create();
/*
  axis.copy(gyro);
  axis.normalize();
*/
  vec3.copy(axis, gyro);
  vec3.normalize(axis, axis);
  
  var angularSpeed = vec3.length(gyro);

  // If we're rotating slowly, don't do prediction.
  if (angularSpeed < 20 * ( Math.PI / 180 ) ) {
    if (DEBUG) {
      console.log('Moving slowly, at %s deg/s: no prediction',
                  (angularSpeed * (180 / Math.PI)).toFixed(1));
    }
/*
    this.outQ.copy(currentQ);
    this.previousQ.copy(currentQ);
*/
    quat.copy(this.outQ, currentQ);
    quat.copy(this.previousQ, currentQ);
    return this.outQ;
  }

  // Get the predicted angle based on the time delta and latency.
  var deltaT = timestampS - this.previousTimestampS;
  var predictAngle = angularSpeed * this.predictionTimeS;

/*
  this.deltaQ.setFromAxisAngle(axis, predictAngle);
  this.outQ.copy(this.previousQ);
  this.outQ.multiply(this.deltaQ);

  this.previousQ.copy(currentQ);
*/
  quat.setAxisAngle(this.deltaQ, axis, predictAngle);
  quat.copy(this.outQ, this.previousQ);
  quat.multiply(this.outQ, this.outQ, this.deltaQ);

  quat.copy(this.previousQ, currentQ);

  return this.outQ;
};

module.exports = PosePredictor;
},{"gl-matrix-quat":5,"gl-matrix-vec3":9}],14:[function(_dereq_,module,exports){
function SensorSample(sample, timestampS) {
  this.set(sample, timestampS);
};

SensorSample.prototype.set = function(sample, timestampS) {
  this.sample = sample;
  this.timestampS = timestampS;
};

SensorSample.prototype.copy = function(sensorSample) {
  this.set(sensorSample.sample, sensorSample.timestampS);
};

module.exports = SensorSample;
},{}]},{},[12])
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyaWZ5L25vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJsaWIvZ3lyb25vcm0vZnVsbHRpbHQuanMiLCJsaWIvZ3lyb25vcm0vZ3lyb25vcm0uanMiLCJub2RlX21vZHVsZXMvYnJvd3NlcmlmeS9ub2RlX21vZHVsZXMvcHJvY2Vzcy9icm93c2VyLmpzIiwibm9kZV9tb2R1bGVzL2VzNi1wcm9taXNlL2Rpc3QvZXM2LXByb21pc2UuanMiLCJub2RlX21vZHVsZXMvZ2wtbWF0cml4LXF1YXQvaW5kZXguanMiLCJub2RlX21vZHVsZXMvZ2wtbWF0cml4LXF1YXQvbm9kZV9tb2R1bGVzL2dsLW1hdHJpeC1jb21tb24vaW5kZXguanMiLCJub2RlX21vZHVsZXMvZ2wtbWF0cml4LXF1YXQvbm9kZV9tb2R1bGVzL2dsLW1hdHJpeC1tYXQzL2luZGV4LmpzIiwibm9kZV9tb2R1bGVzL2dsLW1hdHJpeC1xdWF0L25vZGVfbW9kdWxlcy9nbC1tYXRyaXgtdmVjNC9pbmRleC5qcyIsIm5vZGVfbW9kdWxlcy9nbC1tYXRyaXgtdmVjMy9pbmRleC5qcyIsInNyYy9jb21wbGVtZW50YXJ5LWZpbHRlci5qcyIsInNyYy9jb21wbGVtZW50YXJ5LW9yaWVudGF0aW9uLmpzIiwic3JjL3Bvc2UtcHJlZGljdG9yLmpzIiwic3JjL3NlbnNvci1zYW1wbGUuanMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7QUNBQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNsN0NBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDeFhBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7O0FDdkhBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7OztBQy83QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3JoQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNuRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN2ZUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDN2dCQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOzs7O0FDOW1CQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMvU0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzFLQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN6R0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSIsImZpbGUiOiJnZW5lcmF0ZWQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlc0NvbnRlbnQiOlsiKGZ1bmN0aW9uIGUodCxuLHIpe2Z1bmN0aW9uIHMobyx1KXtpZighbltvXSl7aWYoIXRbb10pe3ZhciBhPXR5cGVvZiByZXF1aXJlPT1cImZ1bmN0aW9uXCImJnJlcXVpcmU7aWYoIXUmJmEpcmV0dXJuIGEobywhMCk7aWYoaSlyZXR1cm4gaShvLCEwKTt2YXIgZj1uZXcgRXJyb3IoXCJDYW5ub3QgZmluZCBtb2R1bGUgJ1wiK28rXCInXCIpO3Rocm93IGYuY29kZT1cIk1PRFVMRV9OT1RfRk9VTkRcIixmfXZhciBsPW5bb109e2V4cG9ydHM6e319O3Rbb11bMF0uY2FsbChsLmV4cG9ydHMsZnVuY3Rpb24oZSl7dmFyIG49dFtvXVsxXVtlXTtyZXR1cm4gcyhuP246ZSl9LGwsbC5leHBvcnRzLGUsdCxuLHIpfXJldHVybiBuW29dLmV4cG9ydHN9dmFyIGk9dHlwZW9mIHJlcXVpcmU9PVwiZnVuY3Rpb25cIiYmcmVxdWlyZTtmb3IodmFyIG89MDtvPHIubGVuZ3RoO28rKylzKHJbb10pO3JldHVybiBzfSkiLCIvKipcbiAqXG4gKiBGVUxMIFRJTFRcbiAqIGh0dHA6Ly9naXRodWIuY29tL3JpY2h0ci9GdWxsLVRpbHRcbiAqXG4gKiBBIHN0YW5kYWxvbmUgRGV2aWNlT3JpZW50YXRpb24gKyBEZXZpY2VNb3Rpb24gSmF2YVNjcmlwdCBsaWJyYXJ5IHRoYXRcbiAqIG5vcm1hbGlzZXMgb3JpZW50YXRpb24gc2Vuc29yIGlucHV0LCBhcHBsaWVzIHJlbGV2YW50IHNjcmVlbiBvcmllbnRhdGlvblxuICogdHJhbnNmb3JtcywgcmV0dXJucyBFdWxlciBBbmdsZSwgUXVhdGVybmlvbiBhbmQgUm90YXRpb25cbiAqIE1hdHJpeCByZXByZXNlbnRhdGlvbnMgYmFjayB0byB3ZWIgZGV2ZWxvcGVycyBhbmQgcHJvdmlkZXMgY29udmVyc2lvblxuICogYmV0d2VlbiBhbGwgc3VwcG9ydGVkIG9yaWVudGF0aW9uIHJlcHJlc2VudGF0aW9uIHR5cGVzLlxuICpcbiAqIENvcHlyaWdodDogMjAxNCBSaWNoIFRpYmJldHRcbiAqIExpY2Vuc2U6ICAgTUlUXG4gKlxuICovXG5cbnZhciBQcm9taXNlID0gcmVxdWlyZSgnZXM2LXByb21pc2UnKS5Qcm9taXNlO1xuXG52YXIgTV9QSSAgID0gTWF0aC5QSTtcbnZhciBNX1BJXzIgPSBNX1BJIC8gMjtcbnZhciBNXzJfUEkgPSAyICogTV9QSTtcblxuLy8gRGVncmVlIHRvIFJhZGlhbiBjb252ZXJzaW9uXG52YXIgZGVnVG9SYWQgPSBNX1BJIC8gMTgwO1xudmFyIHJhZFRvRGVnID0gMTgwIC8gTV9QSTtcblxuLy8gSW50ZXJuYWwgZGV2aWNlIG9yaWVudGF0aW9uICsgbW90aW9uIHZhcmlhYmxlc1xudmFyIHNlbnNvcnMgPSB7XG5cdFwib3JpZW50YXRpb25cIjoge1xuXHRcdGFjdGl2ZTogICAgZmFsc2UsXG5cdFx0Y2FsbGJhY2tzOiBbXSxcblx0XHRkYXRhOiAgICAgIHVuZGVmaW5lZFxuXHR9LFxuXHRcIm1vdGlvblwiOiB7XG5cdFx0YWN0aXZlOiAgICBmYWxzZSxcblx0XHRjYWxsYmFja3M6IFtdLFxuXHRcdGRhdGE6ICAgICAgdW5kZWZpbmVkXG5cdH1cbn07XG52YXIgc2NyZWVuQWN0aXZlID0gZmFsc2U7XG5cbi8vIEludGVybmFsIHNjcmVlbiBvcmllbnRhdGlvbiB2YXJpYWJsZXNcbnZhciBoYXNTY3JlZW5PcmllbnRhdGlvbkFQSSA9IHdpbmRvdy5zY3JlZW4gJiYgd2luZG93LnNjcmVlbi5vcmllbnRhdGlvbiAmJiB3aW5kb3cuc2NyZWVuLm9yaWVudGF0aW9uLmFuZ2xlICE9PSB1bmRlZmluZWQgJiYgd2luZG93LnNjcmVlbi5vcmllbnRhdGlvbi5hbmdsZSAhPT0gbnVsbCA/IHRydWUgOiBmYWxzZTtcbnZhciBzY3JlZW5PcmllbnRhdGlvbkFuZ2xlID0gKCBoYXNTY3JlZW5PcmllbnRhdGlvbkFQSSA/IHdpbmRvdy5zY3JlZW4ub3JpZW50YXRpb24uYW5nbGUgOiAoIHdpbmRvdy5vcmllbnRhdGlvbiB8fCAwICkgKSAqIGRlZ1RvUmFkO1xuXG52YXIgU0NSRUVOX1JPVEFUSU9OXzAgICAgICAgID0gMCxcbiAgICBTQ1JFRU5fUk9UQVRJT05fOTAgICAgICAgPSBNX1BJXzIsXG4gICAgU0NSRUVOX1JPVEFUSU9OXzE4MCAgICAgID0gTV9QSSxcbiAgICBTQ1JFRU5fUk9UQVRJT05fMjcwICAgICAgPSBNXzJfUEkgLyAzLFxuICAgIFNDUkVFTl9ST1RBVElPTl9NSU5VU185MCA9IC0gTV9QSV8yO1xuXG4vLyBNYXRoLnNpZ24gcG9seWZpbGxcbmZ1bmN0aW9uIHNpZ24oeCkge1xuXHR4ID0gK3g7IC8vIGNvbnZlcnQgdG8gYSBudW1iZXJcblx0aWYgKHggPT09IDAgfHwgaXNOYU4oeCkpXG5cdFx0cmV0dXJuIHg7XG5cdHJldHVybiB4ID4gMCA/IDEgOiAtMTtcbn1cblxuLy8vLy8gUHJvbWlzZS1iYXNlZCBTZW5zb3IgRGF0YSBjaGVja2VyIC8vLy8vL1xuXG5mdW5jdGlvbiBTZW5zb3JDaGVjayhzZW5zb3JSb290T2JqKSB7XG5cblx0dmFyIHByb21pc2UgPSBuZXcgUHJvbWlzZShmdW5jdGlvbihyZXNvbHZlLCByZWplY3QpIHtcblxuXHRcdHZhciBydW5DaGVjayA9IGZ1bmN0aW9uICh0cmllcykge1xuXG5cdFx0XHRzZXRUaW1lb3V0KGZ1bmN0aW9uKCkge1xuXG5cdFx0XHRcdGlmIChzZW5zb3JSb290T2JqICYmIHNlbnNvclJvb3RPYmouZGF0YSkge1xuXG5cdFx0XHRcdFx0cmVzb2x2ZSgpO1xuXG5cdFx0XHRcdH0gZWxzZSBpZiAodHJpZXMgPj0gNSkge1xuXG5cdFx0XHRcdFx0cmVqZWN0KCk7XG5cblx0XHRcdFx0fSBlbHNlIHtcblxuXHRcdFx0XHRcdHJ1bkNoZWNrKCsrdHJpZXMpO1xuXG5cdFx0XHRcdH1cblxuXHRcdFx0fSwgNTApO1xuXG5cdFx0fTtcblxuXHRcdHJ1bkNoZWNrKDApO1xuXG5cdH0pO1xuXG5cdHJldHVybiBwcm9taXNlO1xuXG59XG5cbi8vLy8vLyBJbnRlcm5hbCBFdmVudCBIYW5kbGVycyAvLy8vLy9cblxuZnVuY3Rpb24gaGFuZGxlU2NyZWVuT3JpZW50YXRpb25DaGFuZ2UgKCkge1xuXG5cdGlmICggaGFzU2NyZWVuT3JpZW50YXRpb25BUEkgKSB7XG5cblx0XHRzY3JlZW5PcmllbnRhdGlvbkFuZ2xlID0gKCB3aW5kb3cuc2NyZWVuLm9yaWVudGF0aW9uLmFuZ2xlIHx8IDAgKSAqIGRlZ1RvUmFkO1xuXG5cdH0gZWxzZSB7XG5cblx0XHRzY3JlZW5PcmllbnRhdGlvbkFuZ2xlID0gKCB3aW5kb3cub3JpZW50YXRpb24gfHwgMCApICogZGVnVG9SYWQ7XG5cblx0fVxuXG59XG5cbmZ1bmN0aW9uIGhhbmRsZURldmljZU9yaWVudGF0aW9uQ2hhbmdlICggZXZlbnQgKSB7XG5cblx0c2Vuc29ycy5vcmllbnRhdGlvbi5kYXRhID0gZXZlbnQ7XG5cblx0Ly8gRmlyZSBldmVyeSBjYWxsYmFjayBmdW5jdGlvbiBlYWNoIHRpbWUgZGV2aWNlb3JpZW50YXRpb24gaXMgdXBkYXRlZFxuXHRmb3IgKCB2YXIgaSBpbiBzZW5zb3JzLm9yaWVudGF0aW9uLmNhbGxiYWNrcyApIHtcblxuXHRcdHNlbnNvcnMub3JpZW50YXRpb24uY2FsbGJhY2tzWyBpIF0uY2FsbCggdGhpcyApO1xuXG5cdH1cblxufVxuXG5mdW5jdGlvbiBoYW5kbGVEZXZpY2VNb3Rpb25DaGFuZ2UgKCBldmVudCApIHtcblxuXHRzZW5zb3JzLm1vdGlvbi5kYXRhID0gZXZlbnQ7XG5cblx0Ly8gRmlyZSBldmVyeSBjYWxsYmFjayBmdW5jdGlvbiBlYWNoIHRpbWUgZGV2aWNlbW90aW9uIGlzIHVwZGF0ZWRcblx0Zm9yICggdmFyIGkgaW4gc2Vuc29ycy5tb3Rpb24uY2FsbGJhY2tzICkge1xuXG5cdFx0c2Vuc29ycy5tb3Rpb24uY2FsbGJhY2tzWyBpIF0uY2FsbCggdGhpcyApO1xuXG5cdH1cblxufVxuXG4vLy8vLyBGVUxMVElMVCBBUEkgUm9vdCBPYmplY3QgLy8vLy9cblxudmFyIEZVTExUSUxUID0ge307XG5cbkZVTExUSUxULnZlcnNpb24gPSBcIjAuNS4xXCI7XG5cbi8vLy8vIEZVTExUSUxUIEFQSSBSb290IE1ldGhvZHMgLy8vLy9cblxuRlVMTFRJTFQuZ2V0RGV2aWNlT3JpZW50YXRpb24gPSBmdW5jdGlvbihvcHRpb25zKSB7XG5cblx0dmFyIHByb21pc2UgPSBuZXcgUHJvbWlzZShmdW5jdGlvbihyZXNvbHZlLCByZWplY3QpIHtcblxuXHRcdHZhciBjb250cm9sID0gbmV3IEZVTExUSUxULkRldmljZU9yaWVudGF0aW9uKG9wdGlvbnMpO1xuXG5cdFx0Y29udHJvbC5zdGFydCgpO1xuXG5cdFx0dmFyIG9yaWVudGF0aW9uU2Vuc29yQ2hlY2sgPSBuZXcgU2Vuc29yQ2hlY2soc2Vuc29ycy5vcmllbnRhdGlvbik7XG5cblx0XHRvcmllbnRhdGlvblNlbnNvckNoZWNrLnRoZW4oZnVuY3Rpb24oKSB7XG5cblx0XHRcdGNvbnRyb2wuX2FscGhhQXZhaWxhYmxlID0gKHNlbnNvcnMub3JpZW50YXRpb24uZGF0YS5hbHBoYSAmJiBzZW5zb3JzLm9yaWVudGF0aW9uLmRhdGEuYWxwaGEgIT09IG51bGwpO1xuXHRcdFx0Y29udHJvbC5fYmV0YUF2YWlsYWJsZSA9IChzZW5zb3JzLm9yaWVudGF0aW9uLmRhdGEuYmV0YSAmJiBzZW5zb3JzLm9yaWVudGF0aW9uLmRhdGEuYmV0YSAhPT0gbnVsbCk7XG5cdFx0XHRjb250cm9sLl9nYW1tYUF2YWlsYWJsZSA9IChzZW5zb3JzLm9yaWVudGF0aW9uLmRhdGEuZ2FtbWEgJiYgc2Vuc29ycy5vcmllbnRhdGlvbi5kYXRhLmdhbW1hICE9PSBudWxsKTtcblxuXHRcdFx0cmVzb2x2ZShjb250cm9sKTtcblxuXHRcdH0pLmNhdGNoKGZ1bmN0aW9uKCkge1xuXG5cdFx0XHRjb250cm9sLnN0b3AoKTtcblx0XHRcdHJlamVjdCgnRGV2aWNlT3JpZW50YXRpb24gaXMgbm90IHN1cHBvcnRlZCcpO1xuXG5cdFx0fSk7XG5cblx0fSk7XG5cblx0cmV0dXJuIHByb21pc2U7XG5cbn07XG5cbkZVTExUSUxULmdldERldmljZU1vdGlvbiA9IGZ1bmN0aW9uKG9wdGlvbnMpIHtcblxuXHR2YXIgcHJvbWlzZSA9IG5ldyBQcm9taXNlKGZ1bmN0aW9uKHJlc29sdmUsIHJlamVjdCkge1xuXG5cdFx0dmFyIGNvbnRyb2wgPSBuZXcgRlVMTFRJTFQuRGV2aWNlTW90aW9uKG9wdGlvbnMpO1xuXG5cdFx0Y29udHJvbC5zdGFydCgpO1xuXG5cdFx0dmFyIG1vdGlvblNlbnNvckNoZWNrID0gbmV3IFNlbnNvckNoZWNrKHNlbnNvcnMubW90aW9uKTtcblxuXHRcdG1vdGlvblNlbnNvckNoZWNrLnRoZW4oZnVuY3Rpb24oKSB7XG5cblx0XHRcdGNvbnRyb2wuX2FjY2VsZXJhdGlvblhBdmFpbGFibGUgPSAoc2Vuc29ycy5tb3Rpb24uZGF0YS5hY2NlbGVyYXRpb24gJiYgc2Vuc29ycy5tb3Rpb24uZGF0YS5hY2NlbGVyYXRpb24ueCk7XG5cdFx0XHRjb250cm9sLl9hY2NlbGVyYXRpb25ZQXZhaWxhYmxlID0gKHNlbnNvcnMubW90aW9uLmRhdGEuYWNjZWxlcmF0aW9uICYmIHNlbnNvcnMubW90aW9uLmRhdGEuYWNjZWxlcmF0aW9uLnkpO1xuXHRcdFx0Y29udHJvbC5fYWNjZWxlcmF0aW9uWkF2YWlsYWJsZSA9IChzZW5zb3JzLm1vdGlvbi5kYXRhLmFjY2VsZXJhdGlvbiAmJiBzZW5zb3JzLm1vdGlvbi5kYXRhLmFjY2VsZXJhdGlvbi56KTtcblxuXHRcdFx0Y29udHJvbC5fYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eVhBdmFpbGFibGUgPSAoc2Vuc29ycy5tb3Rpb24uZGF0YS5hY2NlbGVyYXRpb25JbmNsdWRpbmdHcmF2aXR5ICYmIHNlbnNvcnMubW90aW9uLmRhdGEuYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eS54KTtcblx0XHRcdGNvbnRyb2wuX2FjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHlZQXZhaWxhYmxlID0gKHNlbnNvcnMubW90aW9uLmRhdGEuYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eSAmJiBzZW5zb3JzLm1vdGlvbi5kYXRhLmFjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHkueSk7XG5cdFx0XHRjb250cm9sLl9hY2NlbGVyYXRpb25JbmNsdWRpbmdHcmF2aXR5WkF2YWlsYWJsZSA9IChzZW5zb3JzLm1vdGlvbi5kYXRhLmFjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHkgJiYgc2Vuc29ycy5tb3Rpb24uZGF0YS5hY2NlbGVyYXRpb25JbmNsdWRpbmdHcmF2aXR5LnopO1xuXG5cdFx0XHRjb250cm9sLl9yb3RhdGlvblJhdGVBbHBoYUF2YWlsYWJsZSA9IChzZW5zb3JzLm1vdGlvbi5kYXRhLnJvdGF0aW9uUmF0ZSAmJiBzZW5zb3JzLm1vdGlvbi5kYXRhLnJvdGF0aW9uUmF0ZS5hbHBoYSk7XG5cdFx0XHRjb250cm9sLl9yb3RhdGlvblJhdGVCZXRhQXZhaWxhYmxlID0gKHNlbnNvcnMubW90aW9uLmRhdGEucm90YXRpb25SYXRlICYmIHNlbnNvcnMubW90aW9uLmRhdGEucm90YXRpb25SYXRlLmJldGEpO1xuXHRcdFx0Y29udHJvbC5fcm90YXRpb25SYXRlR2FtbWFBdmFpbGFibGUgPSAoc2Vuc29ycy5tb3Rpb24uZGF0YS5yb3RhdGlvblJhdGUgJiYgc2Vuc29ycy5tb3Rpb24uZGF0YS5yb3RhdGlvblJhdGUuZ2FtbWEpO1xuXG5cdFx0XHRyZXNvbHZlKGNvbnRyb2wpO1xuXG5cdFx0fSkuY2F0Y2goZnVuY3Rpb24oKSB7XG5cblx0XHRcdGNvbnRyb2wuc3RvcCgpO1xuXHRcdFx0cmVqZWN0KCdEZXZpY2VNb3Rpb24gaXMgbm90IHN1cHBvcnRlZCcpO1xuXG5cdFx0fSk7XG5cblx0fSk7XG5cblx0cmV0dXJuIHByb21pc2U7XG5cbn07XG5cbi8vLy8vLyBGVUxMVElMVC5RdWF0ZXJuaW9uIC8vLy8vL1xuXG5GVUxMVElMVC5RdWF0ZXJuaW9uID0gZnVuY3Rpb24gKCB4LCB5LCB6LCB3ICkge1xuXG5cdHZhciBxdWF0LCBvdXRRdWF0O1xuXG5cdHRoaXMuc2V0ID0gZnVuY3Rpb24gKCB4LCB5LCB6LCB3ICkge1xuXG5cdFx0dGhpcy54ID0geCB8fCAwO1xuXHRcdHRoaXMueSA9IHkgfHwgMDtcblx0XHR0aGlzLnogPSB6IHx8IDA7XG5cdFx0dGhpcy53ID0gdyB8fCAxO1xuXG5cdH07XG5cblx0dGhpcy5jb3B5ID0gZnVuY3Rpb24gKCBxdWF0ZXJuaW9uICkge1xuXG5cdFx0dGhpcy54ID0gcXVhdGVybmlvbi54O1xuXHRcdHRoaXMueSA9IHF1YXRlcm5pb24ueTtcblx0XHR0aGlzLnogPSBxdWF0ZXJuaW9uLno7XG5cdFx0dGhpcy53ID0gcXVhdGVybmlvbi53O1xuXG5cdH07XG5cblx0dGhpcy5zZXRGcm9tRXVsZXIgPSAoZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIF94LCBfeSwgX3o7XG5cdFx0dmFyIF94XzIsIF95XzIsIF96XzI7XG5cdFx0dmFyIGNYLCBjWSwgY1osIHNYLCBzWSwgc1o7XG5cblx0XHRyZXR1cm4gZnVuY3Rpb24gKCBldWxlciApIHtcblxuXHRcdFx0ZXVsZXIgPSBldWxlciB8fCB7fTtcblxuXHRcdFx0X3ogPSAoIGV1bGVyLmFscGhhIHx8IDAgKSAqIGRlZ1RvUmFkO1xuXHRcdFx0X3ggPSAoIGV1bGVyLmJldGEgfHwgMCApICogZGVnVG9SYWQ7XG5cdFx0XHRfeSA9ICggZXVsZXIuZ2FtbWEgfHwgMCApICogZGVnVG9SYWQ7XG5cblx0XHRcdF96XzIgPSBfeiAvIDI7XG5cdFx0XHRfeF8yID0gX3ggLyAyO1xuXHRcdFx0X3lfMiA9IF95IC8gMjtcblxuXHRcdFx0Y1ggPSBNYXRoLmNvcyggX3hfMiApO1xuXHRcdFx0Y1kgPSBNYXRoLmNvcyggX3lfMiApO1xuXHRcdFx0Y1ogPSBNYXRoLmNvcyggX3pfMiApO1xuXHRcdFx0c1ggPSBNYXRoLnNpbiggX3hfMiApO1xuXHRcdFx0c1kgPSBNYXRoLnNpbiggX3lfMiApO1xuXHRcdFx0c1ogPSBNYXRoLnNpbiggX3pfMiApO1xuXG5cdFx0XHR0aGlzLnNldChcblx0XHRcdFx0c1ggKiBjWSAqIGNaIC0gY1ggKiBzWSAqIHNaLCAvLyB4XG5cdFx0XHRcdGNYICogc1kgKiBjWiArIHNYICogY1kgKiBzWiwgLy8geVxuXHRcdFx0XHRjWCAqIGNZICogc1ogKyBzWCAqIHNZICogY1osIC8vIHpcblx0XHRcdFx0Y1ggKiBjWSAqIGNaIC0gc1ggKiBzWSAqIHNaICAvLyB3XG5cdFx0XHQpO1xuXG5cdFx0XHR0aGlzLm5vcm1hbGl6ZSgpO1xuXG5cdFx0XHRyZXR1cm4gdGhpcztcblxuXHRcdH07XG5cblx0fSkoKTtcblxuXHR0aGlzLnNldEZyb21Sb3RhdGlvbk1hdHJpeCA9IChmdW5jdGlvbiAoKSB7XG5cblx0XHR2YXIgUjtcblxuXHRcdHJldHVybiBmdW5jdGlvbiggbWF0cml4ICkge1xuXG5cdFx0XHRSID0gbWF0cml4LmVsZW1lbnRzO1xuXG5cdFx0XHR0aGlzLnNldChcblx0XHRcdFx0MC41ICogTWF0aC5zcXJ0KCAxICsgUlswXSAtIFJbNF0gLSBSWzhdICkgKiBzaWduKCBSWzddIC0gUls1XSApLCAvLyB4XG5cdFx0XHRcdDAuNSAqIE1hdGguc3FydCggMSAtIFJbMF0gKyBSWzRdIC0gUls4XSApICogc2lnbiggUlsyXSAtIFJbNl0gKSwgLy8geVxuXHRcdFx0XHQwLjUgKiBNYXRoLnNxcnQoIDEgLSBSWzBdIC0gUls0XSArIFJbOF0gKSAqIHNpZ24oIFJbM10gLSBSWzFdICksIC8vIHpcblx0XHRcdFx0MC41ICogTWF0aC5zcXJ0KCAxICsgUlswXSArIFJbNF0gKyBSWzhdICkgICAgICAgICAgICAgICAgICAgICAgICAvLyB3XG5cdFx0XHQpO1xuXG5cdFx0XHRyZXR1cm4gdGhpcztcblxuXHRcdH07XG5cblx0fSkoKTtcblxuXHR0aGlzLm11bHRpcGx5ID0gZnVuY3Rpb24gKCBxdWF0ZXJuaW9uICkge1xuXG5cdFx0b3V0UXVhdCA9IEZVTExUSUxULlF1YXRlcm5pb24ucHJvdG90eXBlLm11bHRpcGx5UXVhdGVybmlvbnMoIHRoaXMsIHF1YXRlcm5pb24gKTtcblx0XHR0aGlzLmNvcHkoIG91dFF1YXQgKTtcblxuXHRcdHJldHVybiB0aGlzO1xuXG5cdH07XG5cblx0dGhpcy5yb3RhdGVYID0gZnVuY3Rpb24gKCBhbmdsZSApIHtcblxuXHRcdG91dFF1YXQgPSBGVUxMVElMVC5RdWF0ZXJuaW9uLnByb3RvdHlwZS5yb3RhdGVCeUF4aXNBbmdsZSggdGhpcywgWyAxLCAwLCAwIF0sIGFuZ2xlICk7XG5cdFx0dGhpcy5jb3B5KCBvdXRRdWF0ICk7XG5cblx0XHRyZXR1cm4gdGhpcztcblxuXHR9O1xuXG5cdHRoaXMucm90YXRlWSA9IGZ1bmN0aW9uICggYW5nbGUgKSB7XG5cblx0XHRvdXRRdWF0ID0gRlVMTFRJTFQuUXVhdGVybmlvbi5wcm90b3R5cGUucm90YXRlQnlBeGlzQW5nbGUoIHRoaXMsIFsgMCwgMSwgMCBdLCBhbmdsZSApO1xuXHRcdHRoaXMuY29weSggb3V0UXVhdCApO1xuXG5cdFx0cmV0dXJuIHRoaXM7XG5cblx0fTtcblxuXHR0aGlzLnJvdGF0ZVogPSBmdW5jdGlvbiAoIGFuZ2xlICkge1xuXG5cdFx0b3V0UXVhdCA9IEZVTExUSUxULlF1YXRlcm5pb24ucHJvdG90eXBlLnJvdGF0ZUJ5QXhpc0FuZ2xlKCB0aGlzLCBbIDAsIDAsIDEgXSwgYW5nbGUgKTtcblx0XHR0aGlzLmNvcHkoIG91dFF1YXQgKTtcblxuXHRcdHJldHVybiB0aGlzO1xuXG5cdH07XG5cblx0dGhpcy5ub3JtYWxpemUgPSBmdW5jdGlvbiAoKSB7XG5cblx0XHRyZXR1cm4gRlVMTFRJTFQuUXVhdGVybmlvbi5wcm90b3R5cGUubm9ybWFsaXplKCB0aGlzICk7XG5cblx0fTtcblxuXHQvLyBJbml0aWFsaXplIG9iamVjdCB2YWx1ZXNcblx0dGhpcy5zZXQoIHgsIHksIHosIHcgKTtcblxufTtcblxuRlVMTFRJTFQuUXVhdGVybmlvbi5wcm90b3R5cGUgPSB7XG5cblx0Y29uc3RydWN0b3I6IEZVTExUSUxULlF1YXRlcm5pb24sXG5cblx0bXVsdGlwbHlRdWF0ZXJuaW9uczogZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIG11bHRpcGxpZWRRdWF0ID0gbmV3IEZVTExUSUxULlF1YXRlcm5pb24oKTtcblxuXHRcdHJldHVybiBmdW5jdGlvbiAoIGEsIGIgKSB7XG5cblx0XHRcdHZhciBxYXggPSBhLngsIHFheSA9IGEueSwgcWF6ID0gYS56LCBxYXcgPSBhLnc7XG5cdFx0XHR2YXIgcWJ4ID0gYi54LCBxYnkgPSBiLnksIHFieiA9IGIueiwgcWJ3ID0gYi53O1xuXG5cdFx0XHRtdWx0aXBsaWVkUXVhdC5zZXQoXG5cdFx0XHRcdHFheCAqIHFidyArIHFhdyAqIHFieCArIHFheSAqIHFieiAtIHFheiAqIHFieSwgLy8geFxuXHRcdFx0XHRxYXkgKiBxYncgKyBxYXcgKiBxYnkgKyBxYXogKiBxYnggLSBxYXggKiBxYnosIC8vIHlcblx0XHRcdFx0cWF6ICogcWJ3ICsgcWF3ICogcWJ6ICsgcWF4ICogcWJ5IC0gcWF5ICogcWJ4LCAvLyB6XG5cdFx0XHRcdHFhdyAqIHFidyAtIHFheCAqIHFieCAtIHFheSAqIHFieSAtIHFheiAqIHFieiAgLy8gd1xuXHRcdFx0KTtcblxuXHRcdFx0cmV0dXJuIG11bHRpcGxpZWRRdWF0O1xuXG5cdFx0fTtcblxuXHR9KCksXG5cblx0bm9ybWFsaXplOiBmdW5jdGlvbiggcSApIHtcblxuXHRcdHZhciBsZW4gPSBNYXRoLnNxcnQoIHEueCAqIHEueCArIHEueSAqIHEueSArIHEueiAqIHEueiArIHEudyAqIHEudyApO1xuXG5cdFx0aWYgKCBsZW4gPT09IDAgKSB7XG5cblx0XHRcdHEueCA9IDA7XG5cdFx0XHRxLnkgPSAwO1xuXHRcdFx0cS56ID0gMDtcblx0XHRcdHEudyA9IDE7XG5cblx0XHR9IGVsc2Uge1xuXG5cdFx0XHRsZW4gPSAxIC8gbGVuO1xuXG5cdFx0XHRxLnggKj0gbGVuO1xuXHRcdFx0cS55ICo9IGxlbjtcblx0XHRcdHEueiAqPSBsZW47XG5cdFx0XHRxLncgKj0gbGVuO1xuXG5cdFx0fVxuXG5cdFx0cmV0dXJuIHE7XG5cblx0fSxcblxuXHRyb3RhdGVCeUF4aXNBbmdsZTogZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIG91dHB1dFF1YXRlcm5pb24gPSBuZXcgRlVMTFRJTFQuUXVhdGVybmlvbigpO1xuXHRcdHZhciB0cmFuc2Zvcm1RdWF0ZXJuaW9uID0gbmV3IEZVTExUSUxULlF1YXRlcm5pb24oKTtcblxuXHRcdHZhciBoYWxmQW5nbGUsIHNBO1xuXG5cdFx0cmV0dXJuIGZ1bmN0aW9uICggdGFyZ2V0UXVhdGVybmlvbiwgYXhpcywgYW5nbGUgKSB7XG5cblx0XHRcdGhhbGZBbmdsZSA9ICggYW5nbGUgfHwgMCApIC8gMjtcblx0XHRcdHNBID0gTWF0aC5zaW4oIGhhbGZBbmdsZSApO1xuXG5cdFx0XHR0cmFuc2Zvcm1RdWF0ZXJuaW9uLnNldChcblx0XHRcdFx0KCBheGlzWyAwIF0gfHwgMCApICogc0EsIC8vIHhcblx0XHRcdFx0KCBheGlzWyAxIF0gfHwgMCApICogc0EsIC8vIHlcblx0XHRcdFx0KCBheGlzWyAyIF0gfHwgMCApICogc0EsIC8vIHpcblx0XHRcdFx0TWF0aC5jb3MoIGhhbGZBbmdsZSApICAgIC8vIHdcblx0XHRcdCk7XG5cblx0XHRcdC8vIE11bHRpcGx5IHF1YXRlcm5pb24gYnkgcVxuXHRcdFx0b3V0cHV0UXVhdGVybmlvbiA9IEZVTExUSUxULlF1YXRlcm5pb24ucHJvdG90eXBlLm11bHRpcGx5UXVhdGVybmlvbnMoIHRhcmdldFF1YXRlcm5pb24sIHRyYW5zZm9ybVF1YXRlcm5pb24gKTtcblxuXHRcdFx0cmV0dXJuIEZVTExUSUxULlF1YXRlcm5pb24ucHJvdG90eXBlLm5vcm1hbGl6ZSggb3V0cHV0UXVhdGVybmlvbiApO1xuXG5cdFx0fTtcblxuXHR9KClcblxufTtcblxuLy8vLy8vIEZVTExUSUxULlJvdGF0aW9uTWF0cml4IC8vLy8vL1xuXG5GVUxMVElMVC5Sb3RhdGlvbk1hdHJpeCA9IGZ1bmN0aW9uICggbTExLCBtMTIsIG0xMywgbTIxLCBtMjIsIG0yMywgbTMxLCBtMzIsIG0zMyApIHtcblxuXHR2YXIgb3V0TWF0cml4O1xuXG5cdHRoaXMuZWxlbWVudHMgPSBuZXcgRmxvYXQzMkFycmF5KCA5ICk7XG5cblx0dGhpcy5pZGVudGl0eSA9IGZ1bmN0aW9uICgpIHtcblxuXHRcdHRoaXMuc2V0KFxuXHRcdFx0MSwgMCwgMCxcblx0XHRcdDAsIDEsIDAsXG5cdFx0XHQwLCAwLCAxXG5cdFx0KTtcblxuXHRcdHJldHVybiB0aGlzO1xuXG5cdH07XG5cblx0dGhpcy5zZXQgPSBmdW5jdGlvbiAoIG0xMSwgbTEyLCBtMTMsIG0yMSwgbTIyLCBtMjMsIG0zMSwgbTMyLCBtMzMgKSB7XG5cblx0XHR0aGlzLmVsZW1lbnRzWyAwIF0gPSBtMTEgfHwgMTtcblx0XHR0aGlzLmVsZW1lbnRzWyAxIF0gPSBtMTIgfHwgMDtcblx0XHR0aGlzLmVsZW1lbnRzWyAyIF0gPSBtMTMgfHwgMDtcblx0XHR0aGlzLmVsZW1lbnRzWyAzIF0gPSBtMjEgfHwgMDtcblx0XHR0aGlzLmVsZW1lbnRzWyA0IF0gPSBtMjIgfHwgMTtcblx0XHR0aGlzLmVsZW1lbnRzWyA1IF0gPSBtMjMgfHwgMDtcblx0XHR0aGlzLmVsZW1lbnRzWyA2IF0gPSBtMzEgfHwgMDtcblx0XHR0aGlzLmVsZW1lbnRzWyA3IF0gPSBtMzIgfHwgMDtcblx0XHR0aGlzLmVsZW1lbnRzWyA4IF0gPSBtMzMgfHwgMTtcblxuXHR9O1xuXG5cdHRoaXMuY29weSA9IGZ1bmN0aW9uICggbWF0cml4ICkge1xuXG5cdFx0dGhpcy5lbGVtZW50c1sgMCBdID0gbWF0cml4LmVsZW1lbnRzWyAwIF07XG5cdFx0dGhpcy5lbGVtZW50c1sgMSBdID0gbWF0cml4LmVsZW1lbnRzWyAxIF07XG5cdFx0dGhpcy5lbGVtZW50c1sgMiBdID0gbWF0cml4LmVsZW1lbnRzWyAyIF07XG5cdFx0dGhpcy5lbGVtZW50c1sgMyBdID0gbWF0cml4LmVsZW1lbnRzWyAzIF07XG5cdFx0dGhpcy5lbGVtZW50c1sgNCBdID0gbWF0cml4LmVsZW1lbnRzWyA0IF07XG5cdFx0dGhpcy5lbGVtZW50c1sgNSBdID0gbWF0cml4LmVsZW1lbnRzWyA1IF07XG5cdFx0dGhpcy5lbGVtZW50c1sgNiBdID0gbWF0cml4LmVsZW1lbnRzWyA2IF07XG5cdFx0dGhpcy5lbGVtZW50c1sgNyBdID0gbWF0cml4LmVsZW1lbnRzWyA3IF07XG5cdFx0dGhpcy5lbGVtZW50c1sgOCBdID0gbWF0cml4LmVsZW1lbnRzWyA4IF07XG5cblx0fTtcblxuXHR0aGlzLnNldEZyb21FdWxlciA9IChmdW5jdGlvbigpIHtcblxuXHRcdHZhciBfeCwgX3ksIF96O1xuXHRcdHZhciBjWCwgY1ksIGNaLCBzWCwgc1ksIHNaO1xuXG5cdFx0cmV0dXJuIGZ1bmN0aW9uICggZXVsZXIgKSB7XG5cblx0XHRcdGV1bGVyID0gZXVsZXIgfHwge307XG5cblx0XHRcdF96ID0gKCBldWxlci5hbHBoYSB8fCAwICkgKiBkZWdUb1JhZDtcblx0XHRcdF94ID0gKCBldWxlci5iZXRhIHx8IDAgKSAqIGRlZ1RvUmFkO1xuXHRcdFx0X3kgPSAoIGV1bGVyLmdhbW1hIHx8IDAgKSAqIGRlZ1RvUmFkO1xuXG5cdFx0XHRjWCA9IE1hdGguY29zKCBfeCApO1xuXHRcdFx0Y1kgPSBNYXRoLmNvcyggX3kgKTtcblx0XHRcdGNaID0gTWF0aC5jb3MoIF96ICk7XG5cdFx0XHRzWCA9IE1hdGguc2luKCBfeCApO1xuXHRcdFx0c1kgPSBNYXRoLnNpbiggX3kgKTtcblx0XHRcdHNaID0gTWF0aC5zaW4oIF96ICk7XG5cblx0XHRcdC8vXG5cdFx0XHQvLyBaWFktb3JkZXJlZCByb3RhdGlvbiBtYXRyaXggY29uc3RydWN0aW9uLlxuXHRcdFx0Ly9cblxuXHRcdFx0dGhpcy5zZXQoXG5cdFx0XHRcdGNaICogY1kgLSBzWiAqIHNYICogc1ksIC8vIDEsMVxuXHRcdFx0XHQtIGNYICogc1osICAgICAgICAgICAgICAvLyAxLDJcblx0XHRcdFx0Y1kgKiBzWiAqIHNYICsgY1ogKiBzWSwgLy8gMSwzXG5cblx0XHRcdFx0Y1kgKiBzWiArIGNaICogc1ggKiBzWSwgLy8gMiwxXG5cdFx0XHRcdGNaICogY1gsICAgICAgICAgICAgICAgIC8vIDIsMlxuXHRcdFx0XHRzWiAqIHNZIC0gY1ogKiBjWSAqIHNYLCAvLyAyLDNcblxuXHRcdFx0XHQtIGNYICogc1ksICAgICAgICAgICAgICAvLyAzLDFcblx0XHRcdFx0c1gsICAgICAgICAgICAgICAgICAgICAgLy8gMywyXG5cdFx0XHRcdGNYICogY1kgICAgICAgICAgICAgICAgIC8vIDMsM1xuXHRcdFx0KTtcblxuXHRcdFx0dGhpcy5ub3JtYWxpemUoKTtcblxuXHRcdFx0cmV0dXJuIHRoaXM7XG5cblx0XHR9O1xuXG5cdH0pKCk7XG5cblx0dGhpcy5zZXRGcm9tUXVhdGVybmlvbiA9IChmdW5jdGlvbigpIHtcblxuXHRcdHZhciBzcXcsIHNxeCwgc3F5LCBzcXo7XG5cblx0XHRyZXR1cm4gZnVuY3Rpb24oIHEgKSB7XG5cblx0XHRcdHNxdyA9IHEudyAqIHEudztcblx0XHRcdHNxeCA9IHEueCAqIHEueDtcblx0XHRcdHNxeSA9IHEueSAqIHEueTtcblx0XHRcdHNxeiA9IHEueiAqIHEuejtcblxuXHRcdFx0dGhpcy5zZXQoXG5cdFx0XHRcdHNxdyArIHNxeCAtIHNxeSAtIHNxeiwgICAgICAgLy8gMSwxXG5cdFx0XHRcdDIgKiAocS54ICogcS55IC0gcS53ICogcS56KSwgLy8gMSwyXG5cdFx0XHRcdDIgKiAocS54ICogcS56ICsgcS53ICogcS55KSwgLy8gMSwzXG5cblx0XHRcdFx0MiAqIChxLnggKiBxLnkgKyBxLncgKiBxLnopLCAvLyAyLDFcblx0XHRcdFx0c3F3IC0gc3F4ICsgc3F5IC0gc3F6LCAgICAgICAvLyAyLDJcblx0XHRcdFx0MiAqIChxLnkgKiBxLnogLSBxLncgKiBxLngpLCAvLyAyLDNcblxuXHRcdFx0XHQyICogKHEueCAqIHEueiAtIHEudyAqIHEueSksIC8vIDMsMVxuXHRcdFx0XHQyICogKHEueSAqIHEueiArIHEudyAqIHEueCksIC8vIDMsMlxuXHRcdFx0XHRzcXcgLSBzcXggLSBzcXkgKyBzcXogICAgICAgIC8vIDMsM1xuXHRcdFx0KTtcblxuXHRcdFx0cmV0dXJuIHRoaXM7XG5cblx0XHR9O1xuXG5cdH0pKCk7XG5cblx0dGhpcy5tdWx0aXBseSA9IGZ1bmN0aW9uICggbSApIHtcblxuXHRcdG91dE1hdHJpeCA9IEZVTExUSUxULlJvdGF0aW9uTWF0cml4LnByb3RvdHlwZS5tdWx0aXBseU1hdHJpY2VzKCB0aGlzLCBtICk7XG5cdFx0dGhpcy5jb3B5KCBvdXRNYXRyaXggKTtcblxuXHRcdHJldHVybiB0aGlzO1xuXG5cdH07XG5cblx0dGhpcy5yb3RhdGVYID0gZnVuY3Rpb24gKCBhbmdsZSApIHtcblxuXHRcdG91dE1hdHJpeCA9IEZVTExUSUxULlJvdGF0aW9uTWF0cml4LnByb3RvdHlwZS5yb3RhdGVCeUF4aXNBbmdsZSggdGhpcywgWyAxLCAwLCAwIF0sIGFuZ2xlICk7XG5cdFx0dGhpcy5jb3B5KCBvdXRNYXRyaXggKTtcblxuXHRcdHJldHVybiB0aGlzO1xuXG5cdH07XG5cblx0dGhpcy5yb3RhdGVZID0gZnVuY3Rpb24gKCBhbmdsZSApIHtcblxuXHRcdG91dE1hdHJpeCA9IEZVTExUSUxULlJvdGF0aW9uTWF0cml4LnByb3RvdHlwZS5yb3RhdGVCeUF4aXNBbmdsZSggdGhpcywgWyAwLCAxLCAwIF0sIGFuZ2xlICk7XG5cdFx0dGhpcy5jb3B5KCBvdXRNYXRyaXggKTtcblxuXHRcdHJldHVybiB0aGlzO1xuXG5cdH07XG5cblx0dGhpcy5yb3RhdGVaID0gZnVuY3Rpb24gKCBhbmdsZSApIHtcblxuXHRcdG91dE1hdHJpeCA9IEZVTExUSUxULlJvdGF0aW9uTWF0cml4LnByb3RvdHlwZS5yb3RhdGVCeUF4aXNBbmdsZSggdGhpcywgWyAwLCAwLCAxIF0sIGFuZ2xlICk7XG5cdFx0dGhpcy5jb3B5KCBvdXRNYXRyaXggKTtcblxuXHRcdHJldHVybiB0aGlzO1xuXG5cdH07XG5cblx0dGhpcy5ub3JtYWxpemUgPSBmdW5jdGlvbiAoKSB7XG5cblx0XHRyZXR1cm4gRlVMTFRJTFQuUm90YXRpb25NYXRyaXgucHJvdG90eXBlLm5vcm1hbGl6ZSggdGhpcyApO1xuXG5cdH07XG5cblx0Ly8gSW5pdGlhbGl6ZSBvYmplY3QgdmFsdWVzXG5cdHRoaXMuc2V0KCBtMTEsIG0xMiwgbTEzLCBtMjEsIG0yMiwgbTIzLCBtMzEsIG0zMiwgbTMzICk7XG5cbn07XG5cbkZVTExUSUxULlJvdGF0aW9uTWF0cml4LnByb3RvdHlwZSA9IHtcblxuXHRjb25zdHJ1Y3RvcjogRlVMTFRJTFQuUm90YXRpb25NYXRyaXgsXG5cblx0bXVsdGlwbHlNYXRyaWNlczogZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIG1hdHJpeCA9IG5ldyBGVUxMVElMVC5Sb3RhdGlvbk1hdHJpeCgpO1xuXG5cdFx0dmFyIGFFLCBiRTtcblxuXHRcdHJldHVybiBmdW5jdGlvbiAoIGEsIGIgKSB7XG5cblx0XHRcdGFFID0gYS5lbGVtZW50cztcblx0XHRcdGJFID0gYi5lbGVtZW50cztcblxuXHRcdFx0bWF0cml4LnNldChcblx0XHRcdFx0YUVbMF0gKiBiRVswXSArIGFFWzFdICogYkVbM10gKyBhRVsyXSAqIGJFWzZdLFxuXHRcdFx0XHRhRVswXSAqIGJFWzFdICsgYUVbMV0gKiBiRVs0XSArIGFFWzJdICogYkVbN10sXG5cdFx0XHRcdGFFWzBdICogYkVbMl0gKyBhRVsxXSAqIGJFWzVdICsgYUVbMl0gKiBiRVs4XSxcblxuXHRcdFx0XHRhRVszXSAqIGJFWzBdICsgYUVbNF0gKiBiRVszXSArIGFFWzVdICogYkVbNl0sXG5cdFx0XHRcdGFFWzNdICogYkVbMV0gKyBhRVs0XSAqIGJFWzRdICsgYUVbNV0gKiBiRVs3XSxcblx0XHRcdFx0YUVbM10gKiBiRVsyXSArIGFFWzRdICogYkVbNV0gKyBhRVs1XSAqIGJFWzhdLFxuXG5cdFx0XHRcdGFFWzZdICogYkVbMF0gKyBhRVs3XSAqIGJFWzNdICsgYUVbOF0gKiBiRVs2XSxcblx0XHRcdFx0YUVbNl0gKiBiRVsxXSArIGFFWzddICogYkVbNF0gKyBhRVs4XSAqIGJFWzddLFxuXHRcdFx0XHRhRVs2XSAqIGJFWzJdICsgYUVbN10gKiBiRVs1XSArIGFFWzhdICogYkVbOF1cblx0XHRcdCk7XG5cblx0XHRcdHJldHVybiBtYXRyaXg7XG5cblx0XHR9O1xuXG5cdH0oKSxcblxuXHRub3JtYWxpemU6IGZ1bmN0aW9uKCBtYXRyaXggKSB7XG5cblx0XHR2YXIgUiA9IG1hdHJpeC5lbGVtZW50cztcblxuXHRcdC8vIENhbGN1bGF0ZSBtYXRyaXggZGV0ZXJtaW5hbnRcblx0XHR2YXIgZGV0ZXJtaW5hbnQgPSBSWzBdICogUls0XSAqIFJbOF0gLSBSWzBdICogUls1XSAqIFJbN10gLSBSWzFdICogUlszXSAqIFJbOF0gKyBSWzFdICogUls1XSAqIFJbNl0gKyBSWzJdICogUlszXSAqIFJbN10gLSBSWzJdICogUls0XSAqIFJbNl07XG5cblx0XHQvLyBOb3JtYWxpemUgbWF0cml4IHZhbHVlc1xuXHRcdFJbMF0gLz0gZGV0ZXJtaW5hbnQ7XG5cdFx0UlsxXSAvPSBkZXRlcm1pbmFudDtcblx0XHRSWzJdIC89IGRldGVybWluYW50O1xuXHRcdFJbM10gLz0gZGV0ZXJtaW5hbnQ7XG5cdFx0Uls0XSAvPSBkZXRlcm1pbmFudDtcblx0XHRSWzVdIC89IGRldGVybWluYW50O1xuXHRcdFJbNl0gLz0gZGV0ZXJtaW5hbnQ7XG5cdFx0Uls3XSAvPSBkZXRlcm1pbmFudDtcblx0XHRSWzhdIC89IGRldGVybWluYW50O1xuXG5cdFx0bWF0cml4LmVsZW1lbnRzID0gUjtcblxuXHRcdHJldHVybiBtYXRyaXg7XG5cblx0fSxcblxuXHRyb3RhdGVCeUF4aXNBbmdsZTogZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIG91dHB1dE1hdHJpeCA9IG5ldyBGVUxMVElMVC5Sb3RhdGlvbk1hdHJpeCgpO1xuXHRcdHZhciB0cmFuc2Zvcm1NYXRyaXggPSBuZXcgRlVMTFRJTFQuUm90YXRpb25NYXRyaXgoKTtcblxuXHRcdHZhciBzQSwgY0E7XG5cdFx0dmFyIHZhbGlkQXhpcyA9IGZhbHNlO1xuXG5cdFx0cmV0dXJuIGZ1bmN0aW9uICggdGFyZ2V0Um90YXRpb25NYXRyaXgsIGF4aXMsIGFuZ2xlICkge1xuXG5cdFx0XHR0cmFuc2Zvcm1NYXRyaXguaWRlbnRpdHkoKTsgLy8gcmVzZXQgdHJhbnNmb3JtIG1hdHJpeFxuXG5cdFx0XHR2YWxpZEF4aXMgPSBmYWxzZTtcblxuXHRcdFx0c0EgPSBNYXRoLnNpbiggYW5nbGUgKTtcblx0XHRcdGNBID0gTWF0aC5jb3MoIGFuZ2xlICk7XG5cblx0XHRcdGlmICggYXhpc1sgMCBdID09PSAxICYmIGF4aXNbIDEgXSA9PT0gMCAmJiBheGlzWyAyIF0gPT09IDAgKSB7IC8vIHhcblxuXHRcdFx0XHR2YWxpZEF4aXMgPSB0cnVlO1xuXG5cdFx0XHRcdHRyYW5zZm9ybU1hdHJpeC5lbGVtZW50c1s0XSA9IGNBO1xuXHRcdFx0XHR0cmFuc2Zvcm1NYXRyaXguZWxlbWVudHNbNV0gPSAtc0E7XG5cdFx0XHRcdHRyYW5zZm9ybU1hdHJpeC5lbGVtZW50c1s3XSA9IHNBO1xuXHRcdFx0XHR0cmFuc2Zvcm1NYXRyaXguZWxlbWVudHNbOF0gPSBjQTtcblxuXHQgXHRcdH0gZWxzZSBpZiAoIGF4aXNbIDEgXSA9PT0gMSAmJiBheGlzWyAwIF0gPT09IDAgJiYgYXhpc1sgMiBdID09PSAwICkgeyAvLyB5XG5cblx0XHRcdFx0dmFsaWRBeGlzID0gdHJ1ZTtcblxuXHRcdFx0XHR0cmFuc2Zvcm1NYXRyaXguZWxlbWVudHNbMF0gPSBjQTtcblx0XHRcdFx0dHJhbnNmb3JtTWF0cml4LmVsZW1lbnRzWzJdID0gc0E7XG5cdFx0XHRcdHRyYW5zZm9ybU1hdHJpeC5lbGVtZW50c1s2XSA9IC1zQTtcblx0XHRcdFx0dHJhbnNmb3JtTWF0cml4LmVsZW1lbnRzWzhdID0gY0E7XG5cblx0IFx0XHR9IGVsc2UgaWYgKCBheGlzWyAyIF0gPT09IDEgJiYgYXhpc1sgMCBdID09PSAwICYmIGF4aXNbIDEgXSA9PT0gMCApIHsgLy8gelxuXG5cdFx0XHRcdHZhbGlkQXhpcyA9IHRydWU7XG5cblx0XHRcdFx0dHJhbnNmb3JtTWF0cml4LmVsZW1lbnRzWzBdID0gY0E7XG5cdFx0XHRcdHRyYW5zZm9ybU1hdHJpeC5lbGVtZW50c1sxXSA9IC1zQTtcblx0XHRcdFx0dHJhbnNmb3JtTWF0cml4LmVsZW1lbnRzWzNdID0gc0E7XG5cdFx0XHRcdHRyYW5zZm9ybU1hdHJpeC5lbGVtZW50c1s0XSA9IGNBO1xuXG5cdCBcdFx0fVxuXG5cdFx0XHRpZiAoIHZhbGlkQXhpcyApIHtcblxuXHRcdFx0XHRvdXRwdXRNYXRyaXggPSBGVUxMVElMVC5Sb3RhdGlvbk1hdHJpeC5wcm90b3R5cGUubXVsdGlwbHlNYXRyaWNlcyggdGFyZ2V0Um90YXRpb25NYXRyaXgsIHRyYW5zZm9ybU1hdHJpeCApO1xuXG5cdFx0XHRcdG91dHB1dE1hdHJpeCA9IEZVTExUSUxULlJvdGF0aW9uTWF0cml4LnByb3RvdHlwZS5ub3JtYWxpemUoIG91dHB1dE1hdHJpeCApO1xuXG5cdFx0XHR9IGVsc2Uge1xuXG5cdFx0XHRcdG91dHB1dE1hdHJpeCA9IHRhcmdldFJvdGF0aW9uTWF0cml4O1xuXG5cdFx0XHR9XG5cblx0XHRcdHJldHVybiBvdXRwdXRNYXRyaXg7XG5cblx0XHR9O1xuXG5cdH0oKVxuXG59O1xuXG4vLy8vLy8gRlVMTFRJTFQuRXVsZXIgLy8vLy8vXG5cbkZVTExUSUxULkV1bGVyID0gZnVuY3Rpb24gKCBhbHBoYSwgYmV0YSwgZ2FtbWEgKSB7XG5cblx0dGhpcy5zZXQgPSBmdW5jdGlvbiAoIGFscGhhLCBiZXRhLCBnYW1tYSApIHtcblxuXHRcdHRoaXMuYWxwaGEgPSBhbHBoYSB8fCAwO1xuXHRcdHRoaXMuYmV0YSAgPSBiZXRhICB8fCAwO1xuXHRcdHRoaXMuZ2FtbWEgPSBnYW1tYSB8fCAwO1xuXG5cdH07XG5cblx0dGhpcy5jb3B5ID0gZnVuY3Rpb24gKCBpbkV1bGVyICkge1xuXG5cdFx0dGhpcy5hbHBoYSA9IGluRXVsZXIuYWxwaGE7XG5cdFx0dGhpcy5iZXRhICA9IGluRXVsZXIuYmV0YTtcblx0XHR0aGlzLmdhbW1hID0gaW5FdWxlci5nYW1tYTtcblxuXHR9O1xuXG5cdHRoaXMuc2V0RnJvbVJvdGF0aW9uTWF0cml4ID0gKGZ1bmN0aW9uICgpIHtcblxuXHRcdHZhciBSLCBfYWxwaGEsIF9iZXRhLCBfZ2FtbWE7XG5cblx0XHRyZXR1cm4gZnVuY3Rpb24gKCBtYXRyaXggKSB7XG5cblx0XHRcdFIgPSBtYXRyaXguZWxlbWVudHM7XG5cblx0XHRcdGlmIChSWzhdID4gMCkgeyAvLyBjb3MoYmV0YSkgPiAwXG5cblx0XHRcdFx0X2FscGhhID0gTWF0aC5hdGFuMigtUlsxXSwgUls0XSk7XG5cdFx0XHRcdF9iZXRhICA9IE1hdGguYXNpbihSWzddKTsgLy8gYmV0YSAoLXBpLzIsIHBpLzIpXG5cdFx0XHRcdF9nYW1tYSA9IE1hdGguYXRhbjIoLVJbNl0sIFJbOF0pOyAvLyBnYW1tYSAoLXBpLzIsIHBpLzIpXG5cblx0XHRcdH0gZWxzZSBpZiAoUls4XSA8IDApIHsgIC8vIGNvcyhiZXRhKSA8IDBcblxuXHRcdFx0XHRfYWxwaGEgPSBNYXRoLmF0YW4yKFJbMV0sIC1SWzRdKTtcblx0XHRcdFx0X2JldGEgID0gLU1hdGguYXNpbihSWzddKTtcblx0XHRcdFx0X2JldGEgICs9IChfYmV0YSA+PSAwKSA/IC0gTV9QSSA6IE1fUEk7IC8vIGJldGEgWy1waSwtcGkvMikgVSAocGkvMixwaSlcblx0XHRcdFx0X2dhbW1hID0gTWF0aC5hdGFuMihSWzZdLCAtUls4XSk7IC8vIGdhbW1hICgtcGkvMiwgcGkvMilcblxuXHRcdFx0fSBlbHNlIHsgLy8gUls4XSA9PSAwXG5cblx0XHRcdFx0aWYgKFJbNl0gPiAwKSB7ICAvLyBjb3MoZ2FtbWEpID09IDAsIGNvcyhiZXRhKSA+IDBcblxuXHRcdFx0XHRcdF9hbHBoYSA9IE1hdGguYXRhbjIoLVJbMV0sIFJbNF0pO1xuXHRcdFx0XHRcdF9iZXRhICA9IE1hdGguYXNpbihSWzddKTsgLy8gYmV0YSBbLXBpLzIsIHBpLzJdXG5cdFx0XHRcdFx0X2dhbW1hID0gLSBNX1BJXzI7IC8vIGdhbW1hID0gLXBpLzJcblxuXHRcdFx0XHR9IGVsc2UgaWYgKFJbNl0gPCAwKSB7IC8vIGNvcyhnYW1tYSkgPT0gMCwgY29zKGJldGEpIDwgMFxuXG5cdFx0XHRcdFx0X2FscGhhID0gTWF0aC5hdGFuMihSWzFdLCAtUls0XSk7XG5cdFx0XHRcdFx0X2JldGEgID0gLU1hdGguYXNpbihSWzddKTtcblx0XHRcdFx0XHRfYmV0YSAgKz0gKF9iZXRhID49IDApID8gLSBNX1BJIDogTV9QSTsgLy8gYmV0YSBbLXBpLC1waS8yKSBVIChwaS8yLHBpKVxuXHRcdFx0XHRcdF9nYW1tYSA9IC0gTV9QSV8yOyAvLyBnYW1tYSA9IC1waS8yXG5cblx0XHRcdFx0fSBlbHNlIHsgLy8gUls2XSA9PSAwLCBjb3MoYmV0YSkgPT0gMFxuXG5cdFx0XHRcdFx0Ly8gZ2ltYmFsIGxvY2sgZGlzY29udGludWl0eVxuXHRcdFx0XHRcdF9hbHBoYSA9IE1hdGguYXRhbjIoUlszXSwgUlswXSk7XG5cdFx0XHRcdFx0X2JldGEgID0gKFJbN10gPiAwKSA/IE1fUElfMiA6IC0gTV9QSV8yOyAvLyBiZXRhID0gKy1waS8yXG5cdFx0XHRcdFx0X2dhbW1hID0gMDsgLy8gZ2FtbWEgPSAwXG5cblx0XHRcdFx0fVxuXG5cdFx0XHR9XG5cblx0XHRcdC8vIGFscGhhIGlzIGluIFstcGksIHBpXSwgbWFrZSBzdXJlIGl0IGlzIGluIFswLCAyKnBpKS5cblx0XHRcdGlmIChfYWxwaGEgPCAwKSB7XG5cdFx0XHRcdF9hbHBoYSArPSBNXzJfUEk7IC8vIGFscGhhIFswLCAyKnBpKVxuXHRcdFx0fVxuXG5cdFx0XHQvLyBDb252ZXJ0IHRvIGRlZ3JlZXNcblx0XHRcdF9hbHBoYSAqPSByYWRUb0RlZztcblx0XHRcdF9iZXRhICAqPSByYWRUb0RlZztcblx0XHRcdF9nYW1tYSAqPSByYWRUb0RlZztcblxuXHRcdFx0Ly8gYXBwbHkgZGVyaXZlZCBldWxlciBhbmdsZXMgdG8gY3VycmVudCBvYmplY3Rcblx0XHRcdHRoaXMuc2V0KCBfYWxwaGEsIF9iZXRhLCBfZ2FtbWEgKTtcblxuXHRcdH07XG5cblx0fSkoKTtcblxuXHR0aGlzLnNldEZyb21RdWF0ZXJuaW9uID0gKGZ1bmN0aW9uICgpIHtcblxuXHRcdHZhciBfYWxwaGEsIF9iZXRhLCBfZ2FtbWE7XG5cblx0XHRyZXR1cm4gZnVuY3Rpb24gKCBxICkge1xuXG5cdFx0XHR2YXIgc3F3ID0gcS53ICogcS53O1xuXHRcdFx0dmFyIHNxeCA9IHEueCAqIHEueDtcblx0XHRcdHZhciBzcXkgPSBxLnkgKiBxLnk7XG5cdFx0XHR2YXIgc3F6ID0gcS56ICogcS56O1xuXG5cdFx0XHR2YXIgdW5pdExlbmd0aCA9IHNxdyArIHNxeCArIHNxeSArIHNxejsgLy8gTm9ybWFsaXNlZCA9PSAxLCBvdGhlcndpc2UgY29ycmVjdGlvbiBkaXZpc29yLlxuXHRcdFx0dmFyIHd4eXogPSBxLncgKiBxLnggKyBxLnkgKiBxLno7XG5cdFx0XHR2YXIgZXBzaWxvbiA9IDFlLTY7IC8vIHJvdW5kaW5nIGZhY3RvclxuXG5cdFx0XHRpZiAod3h5eiA+ICgwLjUgLSBlcHNpbG9uKSAqIHVuaXRMZW5ndGgpIHtcblxuXHRcdFx0XHRfYWxwaGEgPSAyICogTWF0aC5hdGFuMihxLnksIHEudyk7XG5cdFx0XHRcdF9iZXRhID0gTV9QSV8yO1xuXHRcdFx0XHRfZ2FtbWEgPSAwO1xuXG5cdFx0XHR9IGVsc2UgaWYgKHd4eXogPCAoLTAuNSArIGVwc2lsb24pICogdW5pdExlbmd0aCkge1xuXG5cdFx0XHRcdF9hbHBoYSA9IC0yICogTWF0aC5hdGFuMihxLnksIHEudyk7XG5cdFx0XHRcdF9iZXRhID0gLU1fUElfMjtcblx0XHRcdFx0X2dhbW1hID0gMDtcblxuXHRcdFx0fSBlbHNlIHtcblxuXHRcdFx0XHR2YXIgYVggPSBzcXcgLSBzcXggKyBzcXkgLSBzcXo7XG5cdFx0XHRcdHZhciBhWSA9IDIgKiAocS53ICogcS56IC0gcS54ICogcS55KTtcblxuXHRcdFx0XHR2YXIgZ1ggPSBzcXcgLSBzcXggLSBzcXkgKyBzcXo7XG5cdFx0XHRcdHZhciBnWSA9IDIgKiAocS53ICogcS55IC0gcS54ICogcS56KTtcblxuXHRcdFx0XHRpZiAoZ1ggPiAwKSB7XG5cblx0XHRcdFx0XHRfYWxwaGEgPSBNYXRoLmF0YW4yKGFZLCBhWCk7XG5cdFx0XHRcdFx0X2JldGEgID0gTWF0aC5hc2luKDIgKiB3eHl6IC8gdW5pdExlbmd0aCk7XG5cdFx0XHRcdFx0X2dhbW1hID0gTWF0aC5hdGFuMihnWSwgZ1gpO1xuXG5cdFx0XHRcdH0gZWxzZSB7XG5cblx0XHRcdFx0XHRfYWxwaGEgPSBNYXRoLmF0YW4yKC1hWSwgLWFYKTtcblx0XHRcdFx0XHRfYmV0YSAgPSAtTWF0aC5hc2luKDIgKiB3eHl6IC8gdW5pdExlbmd0aCk7XG5cdFx0XHRcdFx0X2JldGEgICs9IF9iZXRhIDwgMCA/IE1fUEkgOiAtIE1fUEk7XG5cdFx0XHRcdFx0X2dhbW1hID0gTWF0aC5hdGFuMigtZ1ksIC1nWCk7XG5cblx0XHRcdFx0fVxuXG5cdFx0XHR9XG5cblx0XHRcdC8vIGFscGhhIGlzIGluIFstcGksIHBpXSwgbWFrZSBzdXJlIGl0IGlzIGluIFswLCAyKnBpKS5cblx0XHRcdGlmIChfYWxwaGEgPCAwKSB7XG5cdFx0XHRcdF9hbHBoYSArPSBNXzJfUEk7IC8vIGFscGhhIFswLCAyKnBpKVxuXHRcdFx0fVxuXG5cdFx0XHQvLyBDb252ZXJ0IHRvIGRlZ3JlZXNcblx0XHRcdF9hbHBoYSAqPSByYWRUb0RlZztcblx0XHRcdF9iZXRhICAqPSByYWRUb0RlZztcblx0XHRcdF9nYW1tYSAqPSByYWRUb0RlZztcblxuXHRcdFx0Ly8gYXBwbHkgZGVyaXZlZCBldWxlciBhbmdsZXMgdG8gY3VycmVudCBvYmplY3Rcblx0XHRcdHRoaXMuc2V0KCBfYWxwaGEsIF9iZXRhLCBfZ2FtbWEgKTtcblxuXHRcdH07XG5cblx0fSkoKTtcblxuXHR0aGlzLnJvdGF0ZVggPSBmdW5jdGlvbiAoIGFuZ2xlICkge1xuXG5cdFx0RlVMTFRJTFQuRXVsZXIucHJvdG90eXBlLnJvdGF0ZUJ5QXhpc0FuZ2xlKCB0aGlzLCBbIDEsIDAsIDAgXSwgYW5nbGUgKTtcblxuXHRcdHJldHVybiB0aGlzO1xuXG5cdH07XG5cblx0dGhpcy5yb3RhdGVZID0gZnVuY3Rpb24gKCBhbmdsZSApIHtcblxuXHRcdEZVTExUSUxULkV1bGVyLnByb3RvdHlwZS5yb3RhdGVCeUF4aXNBbmdsZSggdGhpcywgWyAwLCAxLCAwIF0sIGFuZ2xlICk7XG5cblx0XHRyZXR1cm4gdGhpcztcblxuXHR9O1xuXG5cdHRoaXMucm90YXRlWiA9IGZ1bmN0aW9uICggYW5nbGUgKSB7XG5cblx0XHRGVUxMVElMVC5FdWxlci5wcm90b3R5cGUucm90YXRlQnlBeGlzQW5nbGUoIHRoaXMsIFsgMCwgMCwgMSBdLCBhbmdsZSApO1xuXG5cdFx0cmV0dXJuIHRoaXM7XG5cblx0fTtcblxuXHQvLyBJbml0aWFsaXplIG9iamVjdCB2YWx1ZXNcblx0dGhpcy5zZXQoIGFscGhhLCBiZXRhLCBnYW1tYSApO1xuXG59O1xuXG5GVUxMVElMVC5FdWxlci5wcm90b3R5cGUgPSB7XG5cblx0Y29uc3RydWN0b3I6IEZVTExUSUxULkV1bGVyLFxuXG5cdHJvdGF0ZUJ5QXhpc0FuZ2xlOiBmdW5jdGlvbiAoKSB7XG5cblx0XHR2YXIgX21hdHJpeCA9IG5ldyBGVUxMVElMVC5Sb3RhdGlvbk1hdHJpeCgpO1xuXHRcdHZhciBvdXRFdWxlcjtcblxuXHRcdHJldHVybiBmdW5jdGlvbiAoIHRhcmdldEV1bGVyLCBheGlzLCBhbmdsZSApIHtcblxuXHRcdFx0X21hdHJpeC5zZXRGcm9tRXVsZXIoIHRhcmdldEV1bGVyICk7XG5cblx0XHRcdF9tYXRyaXggPSBGVUxMVElMVC5Sb3RhdGlvbk1hdHJpeC5wcm90b3R5cGUucm90YXRlQnlBeGlzQW5nbGUoIF9tYXRyaXgsIGF4aXMsIGFuZ2xlICk7XG5cblx0XHRcdHRhcmdldEV1bGVyLnNldEZyb21Sb3RhdGlvbk1hdHJpeCggX21hdHJpeCApO1xuXG5cdFx0XHRyZXR1cm4gdGFyZ2V0RXVsZXI7XG5cblx0XHR9O1xuXG5cdH0oKVxuXG59O1xuXG4vLy8vLyBGVUxMVElMVC5EZXZpY2VPcmllbnRhdGlvbiAvLy8vLy9cblxuRlVMTFRJTFQuRGV2aWNlT3JpZW50YXRpb24gPSBmdW5jdGlvbiAob3B0aW9ucykge1xuXG5cdHRoaXMub3B0aW9ucyA9IG9wdGlvbnMgfHwge307IC8vIGJ5IGRlZmF1bHQgdXNlIFVBIGRldmljZW9yaWVudGF0aW9uICd0eXBlJyAoXCJnYW1lXCIgb24gaU9TLCBcIndvcmxkXCIgb24gQW5kcm9pZClcblxuXHR2YXIgdHJpZXMgPSAwO1xuXHR2YXIgbWF4VHJpZXMgPSAyMDA7XG5cdHZhciBzdWNjZXNzQ291bnQgPSAwO1xuXHR2YXIgc3VjY2Vzc1RocmVzaG9sZCA9IDEwO1xuXG5cdHRoaXMuYWxwaGFPZmZzZXRTY3JlZW4gPSAwO1xuXHR0aGlzLmFscGhhT2Zmc2V0RGV2aWNlID0gdW5kZWZpbmVkO1xuXG5cdC8vIENyZWF0ZSBhIGdhbWUtYmFzZWQgZGV2aWNlb3JpZW50YXRpb24gb2JqZWN0IChpbml0aWFsIGFscGhhID09PSAwIGRlZ3JlZXMpXG5cdGlmICh0aGlzLm9wdGlvbnMudHlwZSA9PT0gXCJnYW1lXCIpIHtcblxuXHRcdHZhciBzZXRHYW1lQWxwaGFPZmZzZXQgPSBmdW5jdGlvbihldnQpIHtcblxuXHRcdFx0aWYgKGV2dC5hbHBoYSAhPT0gbnVsbCkgeyAvLyBkbyByZWdhcmRsZXNzIG9mIHdoZXRoZXIgJ2V2dC5hYnNvbHV0ZScgaXMgYWxzbyB0cnVlXG5cdFx0XHRcdHRoaXMuYWxwaGFPZmZzZXREZXZpY2UgPSBuZXcgRlVMTFRJTFQuRXVsZXIoZXZ0LmFscGhhLCAwLCAwKTtcblx0XHRcdFx0dGhpcy5hbHBoYU9mZnNldERldmljZS5yb3RhdGVaKCAtc2NyZWVuT3JpZW50YXRpb25BbmdsZSApO1xuXG5cdFx0XHRcdC8vIERpc2NhcmQgZmlyc3Qge3N1Y2Nlc3NUaHJlc2hvbGR9IHJlc3BvbnNlcyB3aGlsZSBhIGJldHRlciBjb21wYXNzIGxvY2sgaXMgZm91bmQgYnkgVUFcblx0XHRcdFx0aWYoKytzdWNjZXNzQ291bnQgPj0gc3VjY2Vzc1RocmVzaG9sZCkge1xuXHRcdFx0XHRcdHdpbmRvdy5yZW1vdmVFdmVudExpc3RlbmVyKCAnZGV2aWNlb3JpZW50YXRpb24nLCBzZXRHYW1lQWxwaGFPZmZzZXQsIGZhbHNlICk7XG5cdFx0XHRcdFx0cmV0dXJuO1xuXHRcdFx0XHR9XG5cdFx0XHR9XG5cblx0XHRcdGlmKCsrdHJpZXMgPj0gbWF4VHJpZXMpIHtcblx0XHRcdFx0d2luZG93LnJlbW92ZUV2ZW50TGlzdGVuZXIoICdkZXZpY2VvcmllbnRhdGlvbicsIHNldEdhbWVBbHBoYU9mZnNldCwgZmFsc2UgKTtcblx0XHRcdH1cblxuXHRcdH0uYmluZCh0aGlzKTtcblxuXHRcdHdpbmRvdy5hZGRFdmVudExpc3RlbmVyKCAnZGV2aWNlb3JpZW50YXRpb24nLCBzZXRHYW1lQWxwaGFPZmZzZXQsIGZhbHNlICk7XG5cblx0Ly8gQ3JlYXRlIGEgY29tcGFzcy1iYXNlZCBkZXZpY2VvcmllbnRhdGlvbiBvYmplY3QgKGluaXRpYWwgYWxwaGEgPT09IGNvbXBhc3MgZGVncmVlcylcblx0fSBlbHNlIGlmICh0aGlzLm9wdGlvbnMudHlwZSA9PT0gXCJ3b3JsZFwiKSB7XG5cblx0XHR2YXIgc2V0Q29tcGFzc0FscGhhT2Zmc2V0ID0gZnVuY3Rpb24oZXZ0KSB7XG5cblx0XHRcdGlmIChldnQuYWJzb2x1dGUgIT09IHRydWUgJiYgZXZ0LndlYmtpdENvbXBhc3NBY2N1cmFjeSAhPT0gdW5kZWZpbmVkICYmIGV2dC53ZWJraXRDb21wYXNzQWNjdXJhY3kgIT09IG51bGwgJiYgK2V2dC53ZWJraXRDb21wYXNzQWNjdXJhY3kgPj0gMCAmJiArZXZ0LndlYmtpdENvbXBhc3NBY2N1cmFjeSA8IDUwKSB7XG5cdFx0XHRcdHRoaXMuYWxwaGFPZmZzZXREZXZpY2UgPSBuZXcgRlVMTFRJTFQuRXVsZXIoZXZ0LndlYmtpdENvbXBhc3NIZWFkaW5nLCAwLCAwKTtcblx0XHRcdFx0dGhpcy5hbHBoYU9mZnNldERldmljZS5yb3RhdGVaKCBzY3JlZW5PcmllbnRhdGlvbkFuZ2xlICk7XG5cdFx0XHRcdHRoaXMuYWxwaGFPZmZzZXRTY3JlZW4gPSBzY3JlZW5PcmllbnRhdGlvbkFuZ2xlO1xuXG5cdFx0XHRcdC8vIERpc2NhcmQgZmlyc3Qge3N1Y2Nlc3NUaHJlc2hvbGR9IHJlc3BvbnNlcyB3aGlsZSBhIGJldHRlciBjb21wYXNzIGxvY2sgaXMgZm91bmQgYnkgVUFcblx0XHRcdFx0aWYoKytzdWNjZXNzQ291bnQgPj0gc3VjY2Vzc1RocmVzaG9sZCkge1xuXHRcdFx0XHRcdHdpbmRvdy5yZW1vdmVFdmVudExpc3RlbmVyKCAnZGV2aWNlb3JpZW50YXRpb24nLCBzZXRDb21wYXNzQWxwaGFPZmZzZXQsIGZhbHNlICk7XG5cdFx0XHRcdFx0cmV0dXJuO1xuXHRcdFx0XHR9XG5cdFx0XHR9XG5cblx0XHRcdGlmKCsrdHJpZXMgPj0gbWF4VHJpZXMpIHtcblx0XHRcdFx0d2luZG93LnJlbW92ZUV2ZW50TGlzdGVuZXIoICdkZXZpY2VvcmllbnRhdGlvbicsIHNldENvbXBhc3NBbHBoYU9mZnNldCwgZmFsc2UgKTtcblx0XHRcdH1cblxuXHRcdH0uYmluZCh0aGlzKTtcblxuXHRcdHdpbmRvdy5hZGRFdmVudExpc3RlbmVyKCAnZGV2aWNlb3JpZW50YXRpb24nLCBzZXRDb21wYXNzQWxwaGFPZmZzZXQsIGZhbHNlICk7XG5cblx0fSAvLyBlbHNlLi4uIHVzZSB3aGF0ZXZlciBvcmllbnRhdGlvbiBzeXN0ZW0gdGhlIFVBIHByb3ZpZGVzIChcImdhbWVcIiBvbiBpT1MsIFwid29ybGRcIiBvbiBBbmRyb2lkKVxuXG59O1xuXG5GVUxMVElMVC5EZXZpY2VPcmllbnRhdGlvbi5wcm90b3R5cGUgPSB7XG5cblx0Y29uc3RydWN0b3I6IEZVTExUSUxULkRldmljZU9yaWVudGF0aW9uLFxuXG5cdHN0YXJ0OiBmdW5jdGlvbiAoIGNhbGxiYWNrICkge1xuXG5cdFx0aWYgKCBjYWxsYmFjayAmJiBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwoIGNhbGxiYWNrICkgPT0gJ1tvYmplY3QgRnVuY3Rpb25dJyApIHtcblxuXHRcdFx0c2Vuc29ycy5vcmllbnRhdGlvbi5jYWxsYmFja3MucHVzaCggY2FsbGJhY2sgKTtcblxuXHRcdH1cblxuXHRcdGlmKCAhc2NyZWVuQWN0aXZlICkge1xuXG5cdFx0XHRpZiAoIGhhc1NjcmVlbk9yaWVudGF0aW9uQVBJICkge1xuXG5cdFx0XHR3aW5kb3cuc2NyZWVuLm9yaWVudGF0aW9uLmFkZEV2ZW50TGlzdGVuZXIoICdjaGFuZ2UnLCBoYW5kbGVTY3JlZW5PcmllbnRhdGlvbkNoYW5nZSwgZmFsc2UgKTtcblxuXHRcdFx0fSBlbHNlIHtcblxuXHRcdFx0XHR3aW5kb3cuYWRkRXZlbnRMaXN0ZW5lciggJ29yaWVudGF0aW9uY2hhbmdlJywgaGFuZGxlU2NyZWVuT3JpZW50YXRpb25DaGFuZ2UsIGZhbHNlICk7XG5cblx0XHRcdH1cblxuXHRcdH1cblxuXHRcdGlmICggIXNlbnNvcnMub3JpZW50YXRpb24uYWN0aXZlICkge1xuXG5cdFx0XHR3aW5kb3cuYWRkRXZlbnRMaXN0ZW5lciggJ2RldmljZW9yaWVudGF0aW9uJywgaGFuZGxlRGV2aWNlT3JpZW50YXRpb25DaGFuZ2UsIGZhbHNlICk7XG5cblx0XHRcdHNlbnNvcnMub3JpZW50YXRpb24uYWN0aXZlID0gdHJ1ZTtcblxuXHRcdH1cblxuXHR9LFxuXG5cdHN0b3A6IGZ1bmN0aW9uICgpIHtcblxuXHRcdGlmICggc2Vuc29ycy5vcmllbnRhdGlvbi5hY3RpdmUgKSB7XG5cblx0XHRcdHdpbmRvdy5yZW1vdmVFdmVudExpc3RlbmVyKCAnZGV2aWNlb3JpZW50YXRpb24nLCBoYW5kbGVEZXZpY2VPcmllbnRhdGlvbkNoYW5nZSwgZmFsc2UgKTtcblxuXHRcdFx0c2Vuc29ycy5vcmllbnRhdGlvbi5hY3RpdmUgPSBmYWxzZTtcblxuXHRcdH1cblxuXHR9LFxuXG5cdGxpc3RlbjogZnVuY3Rpb24oIGNhbGxiYWNrICkge1xuXG5cdFx0dGhpcy5zdGFydCggY2FsbGJhY2sgKTtcblxuXHR9LFxuXG5cdGdldEZpeGVkRnJhbWVRdWF0ZXJuaW9uOiAoZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIGV1bGVyID0gbmV3IEZVTExUSUxULkV1bGVyKCk7XG5cdFx0dmFyIG1hdHJpeCA9IG5ldyBGVUxMVElMVC5Sb3RhdGlvbk1hdHJpeCgpO1xuXHRcdHZhciBxdWF0ZXJuaW9uID0gbmV3IEZVTExUSUxULlF1YXRlcm5pb24oKTtcblxuXHRcdHJldHVybiBmdW5jdGlvbigpIHtcblxuXHRcdFx0dmFyIG9yaWVudGF0aW9uRGF0YSA9IHNlbnNvcnMub3JpZW50YXRpb24uZGF0YSB8fCB7IGFscGhhOiAwLCBiZXRhOiAwLCBnYW1tYTogMCB9O1xuXG5cdFx0XHR2YXIgYWRqdXN0ZWRBbHBoYSA9IG9yaWVudGF0aW9uRGF0YS5hbHBoYTtcblxuXHRcdFx0aWYgKHRoaXMuYWxwaGFPZmZzZXREZXZpY2UpIHtcblx0XHRcdFx0bWF0cml4LnNldEZyb21FdWxlciggdGhpcy5hbHBoYU9mZnNldERldmljZSApO1xuXHRcdFx0XHRtYXRyaXgucm90YXRlWiggLSB0aGlzLmFscGhhT2Zmc2V0U2NyZWVuICk7XG5cdFx0XHRcdGV1bGVyLnNldEZyb21Sb3RhdGlvbk1hdHJpeCggbWF0cml4ICk7XG5cblx0XHRcdFx0aWYgKGV1bGVyLmFscGhhIDwgMCkge1xuXHRcdFx0XHRcdGV1bGVyLmFscGhhICs9IDM2MDtcblx0XHRcdFx0fVxuXG5cdFx0XHRcdGFkanVzdGVkQWxwaGEgLT0gZXVsZXIuYWxwaGE7XG5cdFx0XHR9XG5cblx0XHRcdGV1bGVyLnNldChcblx0XHRcdFx0YWRqdXN0ZWRBbHBoYSxcblx0XHRcdFx0b3JpZW50YXRpb25EYXRhLmJldGEsXG5cdFx0XHRcdG9yaWVudGF0aW9uRGF0YS5nYW1tYVxuXHRcdFx0KTtcblxuXHRcdFx0cXVhdGVybmlvbi5zZXRGcm9tRXVsZXIoIGV1bGVyICk7XG5cblx0XHRcdHJldHVybiBxdWF0ZXJuaW9uO1xuXG5cdFx0fTtcblxuXHR9KSgpLFxuXG5cdGdldFNjcmVlbkFkanVzdGVkUXVhdGVybmlvbjogKGZ1bmN0aW9uICgpIHtcblxuXHRcdHZhciBxdWF0ZXJuaW9uO1xuXG5cdFx0cmV0dXJuIGZ1bmN0aW9uKCkge1xuXG5cdFx0XHRxdWF0ZXJuaW9uID0gdGhpcy5nZXRGaXhlZEZyYW1lUXVhdGVybmlvbigpO1xuXG5cdFx0XHQvLyBBdXRvbWF0aWNhbGx5IGFwcGx5IHNjcmVlbiBvcmllbnRhdGlvbiB0cmFuc2Zvcm1cblx0XHRcdHF1YXRlcm5pb24ucm90YXRlWiggLSBzY3JlZW5PcmllbnRhdGlvbkFuZ2xlICk7XG5cblx0XHRcdHJldHVybiBxdWF0ZXJuaW9uO1xuXG5cdFx0fTtcblxuXHR9KSgpLFxuXG5cdGdldEZpeGVkRnJhbWVNYXRyaXg6IChmdW5jdGlvbiAoKSB7XG5cblx0XHR2YXIgZXVsZXIgPSBuZXcgRlVMTFRJTFQuRXVsZXIoKTtcblx0XHR2YXIgbWF0cml4ID0gbmV3IEZVTExUSUxULlJvdGF0aW9uTWF0cml4KCk7XG5cblx0XHRyZXR1cm4gZnVuY3Rpb24gKCkge1xuXG5cdFx0XHR2YXIgb3JpZW50YXRpb25EYXRhID0gc2Vuc29ycy5vcmllbnRhdGlvbi5kYXRhIHx8IHsgYWxwaGE6IDAsIGJldGE6IDAsIGdhbW1hOiAwIH07XG5cblx0XHRcdHZhciBhZGp1c3RlZEFscGhhID0gb3JpZW50YXRpb25EYXRhLmFscGhhO1xuXG5cdFx0XHRpZiAodGhpcy5hbHBoYU9mZnNldERldmljZSkge1xuXHRcdFx0XHRtYXRyaXguc2V0RnJvbUV1bGVyKCB0aGlzLmFscGhhT2Zmc2V0RGV2aWNlICk7XG5cdFx0XHRcdG1hdHJpeC5yb3RhdGVaKCAtIHRoaXMuYWxwaGFPZmZzZXRTY3JlZW4gKTtcblx0XHRcdFx0ZXVsZXIuc2V0RnJvbVJvdGF0aW9uTWF0cml4KCBtYXRyaXggKTtcblxuXHRcdFx0XHRpZiAoZXVsZXIuYWxwaGEgPCAwKSB7XG5cdFx0XHRcdFx0ZXVsZXIuYWxwaGEgKz0gMzYwO1xuXHRcdFx0XHR9XG5cblx0XHRcdFx0YWRqdXN0ZWRBbHBoYSAtPSBldWxlci5hbHBoYTtcblx0XHRcdH1cblxuXHRcdFx0ZXVsZXIuc2V0KFxuXHRcdFx0XHRhZGp1c3RlZEFscGhhLFxuXHRcdFx0XHRvcmllbnRhdGlvbkRhdGEuYmV0YSxcblx0XHRcdFx0b3JpZW50YXRpb25EYXRhLmdhbW1hXG5cdFx0XHQpO1xuXG5cdFx0XHRtYXRyaXguc2V0RnJvbUV1bGVyKCBldWxlciApO1xuXG5cdFx0XHRyZXR1cm4gbWF0cml4O1xuXG5cdFx0fTtcblxuXHR9KSgpLFxuXG5cdGdldFNjcmVlbkFkanVzdGVkTWF0cml4OiAoZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIG1hdHJpeDtcblxuXHRcdHJldHVybiBmdW5jdGlvbiAoKSB7XG5cblx0XHRcdG1hdHJpeCA9IHRoaXMuZ2V0Rml4ZWRGcmFtZU1hdHJpeCgpO1xuXG5cdFx0XHQvLyBBdXRvbWF0aWNhbGx5IGFwcGx5IHNjcmVlbiBvcmllbnRhdGlvbiB0cmFuc2Zvcm1cblx0XHRcdG1hdHJpeC5yb3RhdGVaKCAtIHNjcmVlbk9yaWVudGF0aW9uQW5nbGUgKTtcblxuXHRcdFx0cmV0dXJuIG1hdHJpeDtcblxuXHRcdH07XG5cblx0fSkoKSxcblxuXHRnZXRGaXhlZEZyYW1lRXVsZXI6IChmdW5jdGlvbiAoKSB7XG5cblx0XHR2YXIgZXVsZXIgPSBuZXcgRlVMTFRJTFQuRXVsZXIoKTtcblx0XHR2YXIgbWF0cml4O1xuXG5cdFx0cmV0dXJuIGZ1bmN0aW9uICgpIHtcblxuXHRcdFx0bWF0cml4ID0gdGhpcy5nZXRGaXhlZEZyYW1lTWF0cml4KCk7XG5cblx0XHRcdGV1bGVyLnNldEZyb21Sb3RhdGlvbk1hdHJpeCggbWF0cml4ICk7XG5cblx0XHRcdHJldHVybiBldWxlcjtcblxuXHRcdH07XG5cblx0fSkoKSxcblxuXHRnZXRTY3JlZW5BZGp1c3RlZEV1bGVyOiAoZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIGV1bGVyID0gbmV3IEZVTExUSUxULkV1bGVyKCk7XG5cdFx0dmFyIG1hdHJpeDtcblxuXHRcdHJldHVybiBmdW5jdGlvbiAoKSB7XG5cblx0XHRcdG1hdHJpeCA9IHRoaXMuZ2V0U2NyZWVuQWRqdXN0ZWRNYXRyaXgoKTtcblxuXHRcdFx0ZXVsZXIuc2V0RnJvbVJvdGF0aW9uTWF0cml4KCBtYXRyaXggKTtcblxuXHRcdFx0cmV0dXJuIGV1bGVyO1xuXG5cdFx0fTtcblxuXHR9KSgpLFxuXG5cdGlzQWJzb2x1dGU6IGZ1bmN0aW9uICgpIHtcblxuXHRcdGlmICggc2Vuc29ycy5vcmllbnRhdGlvbi5kYXRhICYmIHNlbnNvcnMub3JpZW50YXRpb24uZGF0YS5hYnNvbHV0ZSA9PT0gdHJ1ZSApIHtcblx0XHRcdHJldHVybiB0cnVlO1xuXHRcdH1cblxuXHRcdHJldHVybiBmYWxzZTtcblxuXHR9LFxuXG5cdGdldExhc3RSYXdFdmVudERhdGE6IGZ1bmN0aW9uICgpIHtcblxuXHRcdHJldHVybiBzZW5zb3JzLm9yaWVudGF0aW9uLmRhdGEgfHwge307XG5cblx0fSxcblxuXHRfYWxwaGFBdmFpbGFibGU6IGZhbHNlLFxuXHRfYmV0YUF2YWlsYWJsZTogZmFsc2UsXG5cdF9nYW1tYUF2YWlsYWJsZTogZmFsc2UsIFxuXG5cdGlzQXZhaWxhYmxlOiBmdW5jdGlvbihfdmFsdWVUeXBlKXtcblxuXHRcdHN3aXRjaChfdmFsdWVUeXBlKXtcblx0XHRcdGNhc2UgdGhpcy5BTFBIQTpcblx0XHRcdFx0cmV0dXJuIHRoaXMuX2FscGhhQXZhaWxhYmxlO1xuXG5cdFx0XHRjYXNlIHRoaXMuQkVUQTpcblx0XHRcdFx0cmV0dXJuIHRoaXMuX2JldGFBdmFpbGFibGU7XG5cblx0XHRcdGNhc2UgdGhpcy5HQU1NQTpcblx0XHRcdFx0cmV0dXJuIHRoaXMuX2dhbW1hQXZhaWxhYmxlO1xuXHRcdH1cblx0XHRcblx0fSxcblxuXHRBTFBIQTogJ2FscGhhJyxcblx0QkVUQTogJ2JldGEnLFxuXHRHQU1NQTogJ2dhbW1hJ1xuXG59O1xuXG4vLy8vLyBGVUxMVElMVC5EZXZpY2VNb3Rpb24gLy8vLy8vXG5cbkZVTExUSUxULkRldmljZU1vdGlvbiA9IGZ1bmN0aW9uIChvcHRpb25zKSB7XG5cblx0dGhpcy5vcHRpb25zID0gb3B0aW9ucyB8fCB7fTsgLy8gcGxhY2Vob2xkZXIgb2JqZWN0IHNpbmNlIG5vIG9wdGlvbnMgYXJlIGN1cnJlbnRseSBzdXBwb3J0ZWRcblxufTtcblxuRlVMTFRJTFQuRGV2aWNlTW90aW9uLnByb3RvdHlwZSA9IHtcblxuXHRjb25zdHJ1Y3RvcjogRlVMTFRJTFQuRGV2aWNlTW90aW9uLFxuXG5cdHN0YXJ0OiBmdW5jdGlvbiAoIGNhbGxiYWNrICkge1xuXG5cdFx0aWYgKCBjYWxsYmFjayAmJiBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwoIGNhbGxiYWNrICkgPT0gJ1tvYmplY3QgRnVuY3Rpb25dJyApIHtcblxuXHRcdFx0c2Vuc29ycy5tb3Rpb24uY2FsbGJhY2tzLnB1c2goIGNhbGxiYWNrICk7XG5cblx0XHR9XG5cblx0XHRpZiggIXNjcmVlbkFjdGl2ZSApIHtcblxuXHRcdFx0aWYgKCBoYXNTY3JlZW5PcmllbnRhdGlvbkFQSSApIHtcblxuXHRcdFx0XHR3aW5kb3cuc2NyZWVuLm9yaWVudGF0aW9uLmFkZEV2ZW50TGlzdGVuZXIoICdjaGFuZ2UnLCBoYW5kbGVTY3JlZW5PcmllbnRhdGlvbkNoYW5nZSwgZmFsc2UgKTtcblxuXHRcdFx0fSBlbHNlIHtcblxuXHRcdFx0XHR3aW5kb3cuYWRkRXZlbnRMaXN0ZW5lciggJ29yaWVudGF0aW9uY2hhbmdlJywgaGFuZGxlU2NyZWVuT3JpZW50YXRpb25DaGFuZ2UsIGZhbHNlICk7XG5cblx0XHRcdH1cblxuXHRcdH1cblxuXHRcdGlmICggIXNlbnNvcnMubW90aW9uLmFjdGl2ZSApIHtcblxuXHRcdFx0d2luZG93LmFkZEV2ZW50TGlzdGVuZXIoICdkZXZpY2Vtb3Rpb24nLCBoYW5kbGVEZXZpY2VNb3Rpb25DaGFuZ2UsIGZhbHNlICk7XG5cblx0XHRcdHNlbnNvcnMubW90aW9uLmFjdGl2ZSA9IHRydWU7XG5cblx0XHR9XG5cblx0fSxcblxuXHRzdG9wOiBmdW5jdGlvbiAoKSB7XG5cblx0XHRpZiAoIHNlbnNvcnMubW90aW9uLmFjdGl2ZSApIHtcblxuXHRcdFx0d2luZG93LnJlbW92ZUV2ZW50TGlzdGVuZXIoICdkZXZpY2Vtb3Rpb24nLCBoYW5kbGVEZXZpY2VNb3Rpb25DaGFuZ2UsIGZhbHNlICk7XG5cblx0XHRcdHNlbnNvcnMubW90aW9uLmFjdGl2ZSA9IGZhbHNlO1xuXG5cdFx0fVxuXG5cdH0sXG5cblx0bGlzdGVuOiBmdW5jdGlvbiggY2FsbGJhY2sgKSB7XG5cblx0XHR0aGlzLnN0YXJ0KCBjYWxsYmFjayApO1xuXG5cdH0sXG5cblx0Z2V0U2NyZWVuQWRqdXN0ZWRBY2NlbGVyYXRpb246IGZ1bmN0aW9uICgpIHtcblxuXHRcdHZhciBhY2NEYXRhID0gc2Vuc29ycy5tb3Rpb24uZGF0YSAmJiBzZW5zb3JzLm1vdGlvbi5kYXRhLmFjY2VsZXJhdGlvbiA/IHNlbnNvcnMubW90aW9uLmRhdGEuYWNjZWxlcmF0aW9uIDogeyB4OiAwLCB5OiAwLCB6OiAwIH07XG5cdFx0dmFyIHNjcmVlbkFjY0RhdGEgPSB7fTtcblxuXHRcdHN3aXRjaCAoIHNjcmVlbk9yaWVudGF0aW9uQW5nbGUgKSB7XG5cdFx0XHRjYXNlIFNDUkVFTl9ST1RBVElPTl85MDpcblx0XHRcdFx0c2NyZWVuQWNjRGF0YS54ID0gLSBhY2NEYXRhLnk7XG5cdFx0XHRcdHNjcmVlbkFjY0RhdGEueSA9ICAgYWNjRGF0YS54O1xuXHRcdFx0XHRicmVhaztcblx0XHRcdGNhc2UgU0NSRUVOX1JPVEFUSU9OXzE4MDpcblx0XHRcdFx0c2NyZWVuQWNjRGF0YS54ID0gLSBhY2NEYXRhLng7XG5cdFx0XHRcdHNjcmVlbkFjY0RhdGEueSA9IC0gYWNjRGF0YS55O1xuXHRcdFx0XHRicmVhaztcblx0XHRcdGNhc2UgU0NSRUVOX1JPVEFUSU9OXzI3MDpcblx0XHRcdGNhc2UgU0NSRUVOX1JPVEFUSU9OX01JTlVTXzkwOlxuXHRcdFx0XHRzY3JlZW5BY2NEYXRhLnggPSAgIGFjY0RhdGEueTtcblx0XHRcdFx0c2NyZWVuQWNjRGF0YS55ID0gLSBhY2NEYXRhLng7XG5cdFx0XHRcdGJyZWFrO1xuXHRcdFx0ZGVmYXVsdDogLy8gU0NSRUVOX1JPVEFUSU9OXzBcblx0XHRcdFx0c2NyZWVuQWNjRGF0YS54ID0gICBhY2NEYXRhLng7XG5cdFx0XHRcdHNjcmVlbkFjY0RhdGEueSA9ICAgYWNjRGF0YS55O1xuXHRcdFx0XHRicmVhaztcblx0XHR9XG5cblx0XHRzY3JlZW5BY2NEYXRhLnogPSBhY2NEYXRhLno7XG5cblx0XHRyZXR1cm4gc2NyZWVuQWNjRGF0YTtcblxuXHR9LFxuXG5cdGdldFNjcmVlbkFkanVzdGVkQWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eTogZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIGFjY0dEYXRhID0gc2Vuc29ycy5tb3Rpb24uZGF0YSAmJiBzZW5zb3JzLm1vdGlvbi5kYXRhLmFjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHkgPyBzZW5zb3JzLm1vdGlvbi5kYXRhLmFjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHkgOiB7IHg6IDAsIHk6IDAsIHo6IDAgfTtcblx0XHR2YXIgc2NyZWVuQWNjR0RhdGEgPSB7fTtcblxuXHRcdHN3aXRjaCAoIHNjcmVlbk9yaWVudGF0aW9uQW5nbGUgKSB7XG5cdFx0XHRjYXNlIFNDUkVFTl9ST1RBVElPTl85MDpcblx0XHRcdFx0c2NyZWVuQWNjR0RhdGEueCA9IC0gYWNjR0RhdGEueTtcblx0XHRcdFx0c2NyZWVuQWNjR0RhdGEueSA9ICAgYWNjR0RhdGEueDtcblx0XHRcdFx0YnJlYWs7XG5cdFx0XHRjYXNlIFNDUkVFTl9ST1RBVElPTl8xODA6XG5cdFx0XHRcdHNjcmVlbkFjY0dEYXRhLnggPSAtIGFjY0dEYXRhLng7XG5cdFx0XHRcdHNjcmVlbkFjY0dEYXRhLnkgPSAtIGFjY0dEYXRhLnk7XG5cdFx0XHRcdGJyZWFrO1xuXHRcdFx0Y2FzZSBTQ1JFRU5fUk9UQVRJT05fMjcwOlxuXHRcdFx0Y2FzZSBTQ1JFRU5fUk9UQVRJT05fTUlOVVNfOTA6XG5cdFx0XHRcdHNjcmVlbkFjY0dEYXRhLnggPSAgIGFjY0dEYXRhLnk7XG5cdFx0XHRcdHNjcmVlbkFjY0dEYXRhLnkgPSAtIGFjY0dEYXRhLng7XG5cdFx0XHRcdGJyZWFrO1xuXHRcdFx0ZGVmYXVsdDogLy8gU0NSRUVOX1JPVEFUSU9OXzBcblx0XHRcdFx0c2NyZWVuQWNjR0RhdGEueCA9ICAgYWNjR0RhdGEueDtcblx0XHRcdFx0c2NyZWVuQWNjR0RhdGEueSA9ICAgYWNjR0RhdGEueTtcblx0XHRcdFx0YnJlYWs7XG5cdFx0fVxuXG5cdFx0c2NyZWVuQWNjR0RhdGEueiA9IGFjY0dEYXRhLno7XG5cblx0XHRyZXR1cm4gc2NyZWVuQWNjR0RhdGE7XG5cblx0fSxcblxuXHRnZXRTY3JlZW5BZGp1c3RlZFJvdGF0aW9uUmF0ZTogZnVuY3Rpb24gKCkge1xuXG5cdFx0dmFyIHJvdFJhdGVEYXRhID0gc2Vuc29ycy5tb3Rpb24uZGF0YSAmJiBzZW5zb3JzLm1vdGlvbi5kYXRhLnJvdGF0aW9uUmF0ZSA/IHNlbnNvcnMubW90aW9uLmRhdGEucm90YXRpb25SYXRlIDogeyBhbHBoYTogMCwgYmV0YTogMCwgZ2FtbWE6IDAgfTtcblx0XHR2YXIgc2NyZWVuUm90UmF0ZURhdGEgPSB7fTtcblxuXHRcdHN3aXRjaCAoIHNjcmVlbk9yaWVudGF0aW9uQW5nbGUgKSB7XG5cdFx0XHRjYXNlIFNDUkVFTl9ST1RBVElPTl85MDpcblx0XHRcdFx0c2NyZWVuUm90UmF0ZURhdGEuYmV0YSAgPSAtIHJvdFJhdGVEYXRhLmdhbW1hO1xuXHRcdFx0XHRzY3JlZW5Sb3RSYXRlRGF0YS5nYW1tYSA9ICAgcm90UmF0ZURhdGEuYmV0YTtcblx0XHRcdFx0YnJlYWs7XG5cdFx0XHRjYXNlIFNDUkVFTl9ST1RBVElPTl8xODA6XG5cdFx0XHRcdHNjcmVlblJvdFJhdGVEYXRhLmJldGEgID0gLSByb3RSYXRlRGF0YS5iZXRhO1xuXHRcdFx0XHRzY3JlZW5Sb3RSYXRlRGF0YS5nYW1tYSA9IC0gcm90UmF0ZURhdGEuZ2FtbWE7XG5cdFx0XHRcdGJyZWFrO1xuXHRcdFx0Y2FzZSBTQ1JFRU5fUk9UQVRJT05fMjcwOlxuXHRcdFx0Y2FzZSBTQ1JFRU5fUk9UQVRJT05fTUlOVVNfOTA6XG5cdFx0XHRcdHNjcmVlblJvdFJhdGVEYXRhLmJldGEgID0gICByb3RSYXRlRGF0YS5nYW1tYTtcblx0XHRcdFx0c2NyZWVuUm90UmF0ZURhdGEuZ2FtbWEgPSAtIHJvdFJhdGVEYXRhLmJldGE7XG5cdFx0XHRcdGJyZWFrO1xuXHRcdFx0ZGVmYXVsdDogLy8gU0NSRUVOX1JPVEFUSU9OXzBcblx0XHRcdFx0c2NyZWVuUm90UmF0ZURhdGEuYmV0YSAgPSAgIHJvdFJhdGVEYXRhLmJldGE7XG5cdFx0XHRcdHNjcmVlblJvdFJhdGVEYXRhLmdhbW1hID0gICByb3RSYXRlRGF0YS5nYW1tYTtcblx0XHRcdFx0YnJlYWs7XG5cdFx0fVxuXG5cdFx0c2NyZWVuUm90UmF0ZURhdGEuYWxwaGEgPSByb3RSYXRlRGF0YS5hbHBoYTtcblxuXHRcdHJldHVybiBzY3JlZW5Sb3RSYXRlRGF0YTtcblxuXHR9LFxuXG5cdGdldExhc3RSYXdFdmVudERhdGE6IGZ1bmN0aW9uICgpIHtcblxuXHRcdHJldHVybiBzZW5zb3JzLm1vdGlvbi5kYXRhIHx8IHt9O1xuXG5cdH0sXG5cblx0X2FjY2VsZXJhdGlvblhBdmFpbGFibGU6IGZhbHNlLFxuXHRfYWNjZWxlcmF0aW9uWUF2YWlsYWJsZTogZmFsc2UsXG5cdF9hY2NlbGVyYXRpb25aQXZhaWxhYmxlOiBmYWxzZSxcblxuXHRfYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eVhBdmFpbGFibGU6IGZhbHNlLFxuXHRfYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eVlBdmFpbGFibGU6IGZhbHNlLFxuXHRfYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eVpBdmFpbGFibGU6IGZhbHNlLFxuXG5cdF9yb3RhdGlvblJhdGVBbHBoYUF2YWlsYWJsZTogZmFsc2UsXG5cdF9yb3RhdGlvblJhdGVCZXRhQXZhaWxhYmxlOiBmYWxzZSxcblx0X3JvdGF0aW9uUmF0ZUdhbW1hQXZhaWxhYmxlOiBmYWxzZSxcblxuXHRpc0F2YWlsYWJsZTogZnVuY3Rpb24oX3ZhbHVlVHlwZSl7XG5cblx0XHRzd2l0Y2goX3ZhbHVlVHlwZSl7XG5cdFx0XHRjYXNlIHRoaXMuQUNDRUxFUkFUSU9OX1g6XG5cdFx0XHRcdHJldHVybiB0aGlzLl9hY2NlbGVyYXRpb25YQXZhaWxhYmxlO1xuXG5cdFx0XHRjYXNlIHRoaXMuQUNDRUxFUkFUSU9OX1k6XG5cdFx0XHRcdHJldHVybiB0aGlzLl9hY2NlbGVyYXRpb25ZQXZhaWxhYmxlO1xuXG5cdFx0XHRjYXNlIHRoaXMuQUNDRUxFUkFUSU9OX1o6XG5cdFx0XHRcdHJldHVybiB0aGlzLl9hY2NlbGVyYXRpb25aQXZhaWxhYmxlO1xuXG5cdFx0XHRjYXNlIHRoaXMuQUNDRUxFUkFUSU9OX0lOQ0xVRElOR19HUkFWSVRZX1g6XG5cdFx0XHRcdHJldHVybiB0aGlzLl9hY2NlbGVyYXRpb25JbmNsdWRpbmdHcmF2aXR5WEF2YWlsYWJsZTtcblxuXHRcdFx0Y2FzZSB0aGlzLkFDQ0VMRVJBVElPTl9JTkNMVURJTkdfR1JBVklUWV9ZOlxuXHRcdFx0XHRyZXR1cm4gdGhpcy5fYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eVlBdmFpbGFibGU7XG5cblx0XHRcdGNhc2UgdGhpcy5BQ0NFTEVSQVRJT05fSU5DTFVESU5HX0dSQVZJVFlfWjpcblx0XHRcdFx0cmV0dXJuIHRoaXMuX2FjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHlaQXZhaWxhYmxlO1xuXG5cdFx0XHRjYXNlIHRoaXMuUk9UQVRJT05fUkFURV9BTFBIQTpcblx0XHRcdFx0cmV0dXJuIHRoaXMuX3JvdGF0aW9uUmF0ZUFscGhhQXZhaWxhYmxlO1xuXG5cdFx0XHRjYXNlIHRoaXMuUk9UQVRJT05fUkFURV9CRVRBOlxuXHRcdFx0XHRyZXR1cm4gdGhpcy5fcm90YXRpb25SYXRlQmV0YUF2YWlsYWJsZTtcblxuXHRcdFx0Y2FzZSB0aGlzLlJPVEFUSU9OX1JBVEVfR0FNTUE6XG5cdFx0XHRcdHJldHVybiB0aGlzLl9yb3RhdGlvblJhdGVHYW1tYUF2YWlsYWJsZTtcblx0XHR9XG5cdH0sXG5cblx0QUNDRUxFUkFUSU9OX1g6ICdhY2NlbGVyYXRpb25YJyxcblx0QUNDRUxFUkFUSU9OX1k6ICdhY2NlbGVyYXRpb25ZJyxcblx0QUNDRUxFUkFUSU9OX1o6ICdhY2NlbGVyYXRpb25aJyxcblxuXHRBQ0NFTEVSQVRJT05fSU5DTFVESU5HX0dSQVZJVFlfWDogJ2FjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHlYJyxcblx0QUNDRUxFUkFUSU9OX0lOQ0xVRElOR19HUkFWSVRZX1k6ICdhY2NlbGVyYXRpb25JbmNsdWRpbmdHcmF2aXR5WScsXG5cdEFDQ0VMRVJBVElPTl9JTkNMVURJTkdfR1JBVklUWV9aOiAnYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eVonLFxuXG5cdFJPVEFUSU9OX1JBVEVfQUxQSEE6ICdyb3RhdGlvblJhdGVBbHBoYScsXG5cdFJPVEFUSU9OX1JBVEVfQkVUQTogJ3JvdGF0aW9uUmF0ZUJldGEnLFxuXHRST1RBVElPTl9SQVRFX0dBTU1BOiAncm90YXRpb25SYXRlR2FtbWEnXG5cbn07XG5cbm1vZHVsZS5leHBvcnRzID0gRlVMTFRJTFQ7XG4iLCIvKipcbiogSmF2YVNjcmlwdCBwcm9qZWN0IGZvciBhY2Nlc3NpbmcgYW5kIG5vcm1hbGl6aW5nIHRoZSBhY2NlbGVyb21ldGVyIGFuZCBneXJvc2NvcGUgZGF0YSBvbiBtb2JpbGUgZGV2aWNlc1xuKlxuKiBAYXV0aG9yIERvcnVrIEVrZXIgPGRvcnVrZWtlckBnbWFpbC5jb20+XG4qIEBjb3B5cmlnaHQgRG9ydWsgRWtlciA8aHR0cDovL2RvcnVrZWtlci5jb20+XG4qIEB2ZXJzaW9uIDIuMC40XG4qIEBsaWNlbnNlIE1JVCBMaWNlbnNlIHwgaHR0cDovL29wZW5zb3VyY2Uub3JnL2xpY2Vuc2VzL01JVFxuKi9cblxudmFyIFByb21pc2UgPSByZXF1aXJlKCdlczYtcHJvbWlzZScpLlByb21pc2U7XG52YXIgRlVMTFRJTFQgPSByZXF1aXJlKFwiLi9mdWxsdGlsdC5qc1wiKTtcblxuKGZ1bmN0aW9uKHJvb3QsIGZhY3RvcnkpIHtcbiAgaWYgKHR5cGVvZiBkZWZpbmUgPT09ICdmdW5jdGlvbicgJiYgZGVmaW5lLmFtZCkge1xuICAgIGRlZmluZShmdW5jdGlvbigpIHtcbiAgICAgIHJldHVybiAocm9vdC5HeXJvTm9ybSA9IGZhY3RvcnkoKSk7XG4gICAgfSk7XG4gIH0gZWxzZSBpZiAodHlwZW9mIG1vZHVsZSA9PT0gJ29iamVjdCcgJiYgbW9kdWxlLmV4cG9ydHMpIHtcbiAgICBtb2R1bGUuZXhwb3J0cyA9IChyb290Lkd5cm9Ob3JtID0gZmFjdG9yeSgpKTtcbiAgfSBlbHNlIHtcbiAgICByb290Lkd5cm9Ob3JtID0gZmFjdG9yeSgpO1xuICB9XG59KHRoaXMsIGZ1bmN0aW9uKCkge1xuICAvKiBDb25zdGFudHMgKi9cbiAgdmFyIEdBTUUgICAgICAgICAgICAgICAgICAgICAgICAgICAgPSAnZ2FtZSc7XG4gIHZhciBXT1JMRCAgICAgICAgICAgICAgICAgICAgICAgICAgID0gJ3dvcmxkJztcbiAgdmFyIERFVklDRV9PUklFTlRBVElPTiAgICAgICAgICAgICAgPSAnZGV2aWNlb3JpZW50YXRpb24nO1xuICB2YXIgQUNDRUxFUkFUSU9OICAgICAgICAgICAgICAgICAgICA9ICdhY2NlbGVyYXRpb24nO1xuICB2YXIgQUNDRUxFUkFUSU9OX0lOQ0xVRElOR19HUkFWSVRZICA9ICdhY2NlbGVyYXRpb25pbmx1ZGluZ2dyYXZpdHknO1xuICB2YXIgUk9UQVRJT05fUkFURSAgICAgICAgICAgICAgICAgICA9ICdyb3RhdGlvbnJhdGUnO1xuXG4gIC8qLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLSovXG4gIC8qIFBSSVZBVEUgVkFSSUFCTEVTICovXG5cbiAgdmFyIF9pbnRlcnZhbCAgICAgICAgICAgPSBudWxsOyAgICAgICAvLyBUaW1lciB0byByZXR1cm4gdmFsdWVzXG4gIHZhciBfaXNDYWxpYnJhdGluZyAgICAgID0gZmFsc2U7ICAgICAgLy8gRmxhZyBpZiBjYWxpYnJhdGluZ1xuICB2YXIgX2NhbGlicmF0aW9uVmFsdWUgICA9IDA7ICAgICAgICAgIC8vIEFscGhhIG9mZnNldCB2YWx1ZVxuICB2YXIgX2dyYXZpdHlDb2VmZmljaWVudCA9IDA7ICAgICAgICAgIC8vIENvZWZmaWNpZW50IHRvIG5vcm1hbHplIGdyYXZpdHkgcmVsYXRlZCB2YWx1ZXNcbiAgdmFyIF9pc1J1bm5pbmcgICAgICAgICAgPSBmYWxzZTsgICAgICAvLyBCb29sZWFuIHZhbHVlIGlmIEd5cm9Ob3JtIGlzIHRyYWNraW5nXG4gIHZhciBfaXNSZWFkeSAgICAgICAgICAgID0gZmFsc2U7ICAgICAgLy8gQm9vbGVhbiB2YWx1ZSBpZiBHeXJvTm9ybSBpcyBpcyBpbml0aWFsaXplZFxuXG4gIHZhciBfZG8gICAgICAgICAgICAgICAgID0gbnVsbDsgICAgICAgLy8gT2JqZWN0IHRvIHN0b3JlIHRoZSBkZXZpY2Ugb3JpZW50YXRpb24gdmFsdWVzXG4gIHZhciBfZG0gICAgICAgICAgICAgICAgID0gbnVsbDsgICAgICAgLy8gT2JqZWN0IHRvIHN0b3JlIHRoZSBkZXZpY2UgbW90aW9uIHZhbHVlc1xuXG4gIC8qIE9QVElPTlMgKi9cbiAgdmFyIF9mcmVxdWVuY3kgICAgICAgICAgPSA1MDsgICAgICAgICAvLyBGcmVxdWVuY3kgZm9yIHRoZSByZXR1cm4gZGF0YSBpbiBtaWxsaXNlY29uZHNcbiAgdmFyIF9ncmF2aXR5Tm9ybWFsaXplZCAgPSB0cnVlOyAgICAgICAvLyBGbGFnIGlmIHRvIG5vcm1hbGl6ZSBncmF2aXR5IHZhbHVlc1xuICB2YXIgX29yaWVudGF0aW9uQmFzZSAgICA9IEdBTUU7ICAgICAgIC8vIENhbiBiZSBHeXJvTm9ybS5HQU1FIG9yIEd5cm9Ob3JtLldPUkxELiBHeXJvTm9ybS5HQU1FIHJldHVybnMgb3JpZW50YXRpb24gdmFsdWVzIHdpdGggcmVzcGVjdCB0byB0aGUgaGVhZCBkaXJlY3Rpb24gb2YgdGhlIGRldmljZS4gR3lyb05vcm0uV09STEQgcmV0dXJucyB0aGUgb3JpZW50YXRpb24gdmFsdWVzIHdpdGggcmVzcGVjdCB0byB0aGUgYWN0dWFsIG5vcnRoIGRpcmVjdGlvbiBvZiB0aGUgd29ybGQuXG4gIHZhciBfZGVjaW1hbENvdW50ICAgICAgID0gMjsgICAgICAgICAgLy8gTnVtYmVyIG9mIGRpZ2l0cyBhZnRlciB0aGUgZGVjaW1hbHMgcG9pbnQgZm9yIHRoZSByZXR1cm4gdmFsdWVzXG4gIHZhciBfbG9nZ2VyICAgICAgICAgICAgID0gbnVsbDsgICAgICAgLy8gRnVuY3Rpb24gdG8gY2FsbGJhY2sgb24gZXJyb3IuIFRoZXJlIGlzIG5vIGRlZmF1bHQgdmFsdWUuIEl0IGNhbiBvbmx5IGJlIHNldCBieSB0aGUgdXNlciBvbiBnbi5pbml0KClcbiAgdmFyIF9zY3JlZW5BZGp1c3RlZCAgICAgPSBmYWxzZTsgICAgICAvLyBJZiBzZXQgdG8gdHJ1ZSBpdCB3aWxsIHJldHVybiBzY3JlZW4gYWRqdXN0ZWQgdmFsdWVzLiAoZS5nLiBPbiBhIGhvcml6b250YWwgb3JpZW50YXRpb24gb2YgYSBtb2JpbGUgZGV2aWNlLCB0aGUgaGVhZCB3b3VsZCBiZSBvbmUgb2YgdGhlIHNpZGVzLCBpbnN0ZWFkIG9mICB0aGUgYWN0dWFsIGhlYWQgb2YgdGhlIGRldmljZS4pXG5cbiAgdmFyIF92YWx1ZXMgPSB7XG4gICAgZG86IHtcbiAgICAgIGFscGhhOiAwLFxuICAgICAgYmV0YTogMCxcbiAgICAgIGdhbW1hOiAwLFxuICAgICAgYWJzb2x1dGU6IGZhbHNlXG4gICAgfSxcbiAgICBkbToge1xuICAgICAgeDogMCxcbiAgICAgIHk6IDAsXG4gICAgICB6OiAwLFxuICAgICAgZ3g6IDAsXG4gICAgICBneTogMCxcbiAgICAgIGd6OiAwLFxuICAgICAgYWxwaGE6IDAsXG4gICAgICBiZXRhOiAwLFxuICAgICAgZ2FtbWE6IDBcbiAgICB9XG4gIH1cblxuICAvKi0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0qL1xuICAvKiBQVUJMSUMgRlVOQ1RJT05TICovXG5cbiAgLypcbiAgKlxuICAqIENvbnN0cnVjdG9yIGZ1bmN0aW9uXG4gICpcbiAgKi9cblxuICB2YXIgR3lyb05vcm0gPSBmdW5jdGlvbihvcHRpb25zKSB7fVxuXG4gIC8qIENvbnN0YW50cyAqL1xuICBHeXJvTm9ybS5HQU1FICAgICAgICAgICAgICAgICAgICAgICAgICAgICA9IEdBTUU7XG4gIEd5cm9Ob3JtLldPUkxEICAgICAgICAgICAgICAgICAgICAgICAgICAgID0gV09STEQ7XG4gIEd5cm9Ob3JtLkRFVklDRV9PUklFTlRBVElPTiAgICAgICAgICAgICAgID0gREVWSUNFX09SSUVOVEFUSU9OO1xuICBHeXJvTm9ybS5BQ0NFTEVSQVRJT04gICAgICAgICAgICAgICAgICAgICA9IEFDQ0VMRVJBVElPTjtcbiAgR3lyb05vcm0uQUNDRUxFUkFUSU9OX0lOQ0xVRElOR19HUkFWSVRZICAgPSBBQ0NFTEVSQVRJT05fSU5DTFVESU5HX0dSQVZJVFk7XG4gIEd5cm9Ob3JtLlJPVEFUSU9OX1JBVEUgICAgICAgICAgICAgICAgICAgID0gUk9UQVRJT05fUkFURTtcblxuICAvKlxuICAqXG4gICogSW5pdGlhbGl6ZSBHeXJvTm9ybSBpbnN0YW5jZSBmdW5jdGlvblxuICAqXG4gICogQHBhcmFtIG9iamVjdCBvcHRpb25zIC0gdmFsdWVzIGFyZSBhcyBmb2xsb3dzLiBJZiBzZXQgaW4gdGhlIGluaXQgZnVuY3Rpb24gdGhleSBvdmVyd3JpdGUgdGhlIGRlZmF1bHQgb3B0aW9uIHZhbHVlc1xuICAqIEBwYXJhbSBpbnQgb3B0aW9ucy5mcmVxdWVuY3lcbiAgKiBAcGFyYW0gYm9vbGVhbiBvcHRpb25zLmdyYXZpdHlOb3JtYWxpemVkXG4gICogQHBhcmFtIGJvb2xlYW4gb3B0aW9ucy5vcmllbnRhdGlvbkJhc2VcbiAgKiBAcGFyYW0gYm9vbGVhbiBvcHRpb25zLmRlY2ltYWxDb3VudFxuICAqIEBwYXJhbSBmdW5jdGlvbiBvcHRpb25zLmxvZ2dlclxuICAqIEBwYXJhbSBmdW5jdGlvbiBvcHRpb25zLnNjcmVlbkFkanVzdGVkXG4gICpcbiAgKi9cblxuICBHeXJvTm9ybS5wcm90b3R5cGUuaW5pdCA9IGZ1bmN0aW9uKG9wdGlvbnMpIHtcbiAgICAvLyBBc3NpZ24gb3B0aW9ucyB0aGF0IGFyZSBwYXNzZWQgd2l0aCB0aGUgY29uc3RydWN0b3IgZnVuY3Rpb25cbiAgICBpZiAob3B0aW9ucyAmJiBvcHRpb25zLmZyZXF1ZW5jeSkgX2ZyZXF1ZW5jeSA9IG9wdGlvbnMuZnJlcXVlbmN5O1xuICAgIGlmIChvcHRpb25zICYmIG9wdGlvbnMuZ3Jhdml0eU5vcm1hbGl6ZWQpIF9ncmF2aXR5Tm9ybWFsaXplZCA9IG9wdGlvbnMuZ3Jhdml0eU5vcm1hbGl6ZWQ7XG4gICAgaWYgKG9wdGlvbnMgJiYgb3B0aW9ucy5vcmllbnRhdGlvbkJhc2UpIF9vcmllbnRhdGlvbkJhc2UgPSBvcHRpb25zLm9yaWVudGF0aW9uQmFzZTtcbiAgICBpZiAob3B0aW9ucyAmJiB0eXBlb2Ygb3B0aW9ucy5kZWNpbWFsQ291bnQgPT09ICdudW1iZXInICYmIG9wdGlvbnMuZGVjaW1hbENvdW50ID49IDApIF9kZWNpbWFsQ291bnQgPSBwYXJzZUludChvcHRpb25zLmRlY2ltYWxDb3VudCk7XG4gICAgaWYgKG9wdGlvbnMgJiYgb3B0aW9ucy5sb2dnZXIpIF9sb2dnZXIgPSBvcHRpb25zLmxvZ2dlcjtcbiAgICBpZiAob3B0aW9ucyAmJiBvcHRpb25zLnNjcmVlbkFkanVzdGVkKSBfc2NyZWVuQWRqdXN0ZWQgPSBvcHRpb25zLnNjcmVlbkFkanVzdGVkO1xuXG4gICAgdmFyIGRldmljZU9yaWVudGF0aW9uUHJvbWlzZSA9IG5ldyBGVUxMVElMVC5nZXREZXZpY2VPcmllbnRhdGlvbih7ICd0eXBlJzogX29yaWVudGF0aW9uQmFzZSB9KS50aGVuKGZ1bmN0aW9uKGNvbnRyb2xsZXIpIHtcbiAgICAgIF9kbyA9IGNvbnRyb2xsZXI7XG4gICAgfSk7XG5cbiAgICB2YXIgZGV2aWNlTW90aW9uUHJvbWlzZSA9IG5ldyBGVUxMVElMVC5nZXREZXZpY2VNb3Rpb24oKS50aGVuKGZ1bmN0aW9uKGNvbnRyb2xsZXIpIHtcbiAgICAgIF9kbSA9IGNvbnRyb2xsZXI7XG4gICAgICAvLyBTZXQgZ3Jhdml0eSBjb2VmZmljaWVudFxuICAgICAgX2dyYXZpdHlDb2VmZmljaWVudCA9IChfZG0uZ2V0U2NyZWVuQWRqdXN0ZWRBY2NlbGVyYXRpb25JbmNsdWRpbmdHcmF2aXR5KCkueiA+IDApID8gLTEgOiAxO1xuICAgIH0pO1xuXG4gICAgcmV0dXJuIFByb21pc2UuYWxsKFtkZXZpY2VPcmllbnRhdGlvblByb21pc2UsIGRldmljZU1vdGlvblByb21pc2VdKS50aGVuKGZ1bmN0aW9uKCkge1xuICAgICAgX2lzUmVhZHkgPSB0cnVlO1xuICAgIH0pO1xuICB9XG5cbiAgLypcbiAgKlxuICAqIFN0b3BzIGFsbCB0aGUgdHJhY2tpbmcgYW5kIGxpc3RlbmluZyBvbiB0aGUgd2luZG93IG9iamVjdHNcbiAgKlxuICAqL1xuICBHeXJvTm9ybS5wcm90b3R5cGUuZW5kID0gZnVuY3Rpb24oKSB7XG4gICAgdHJ5IHtcbiAgICAgIF9pc1JlYWR5ID0gZmFsc2U7XG4gICAgICB0aGlzLnN0b3AoKTtcbiAgICAgIF9kbS5zdG9wKCk7XG4gICAgICBfZG8uc3RvcCgpO1xuICAgIH0gY2F0Y2goZXJyKXtcbiAgICAgIGxvZyhlcnIpO1xuICAgIH1cbiAgfVxuXG4gIC8qXG4gICpcbiAgKiBTdGFydHMgdHJhY2tpbmcgdGhlIHZhbHVlc1xuICAqXG4gICogQHBhcmFtIGZ1bmN0aW9uIGNhbGxiYWNrIC0gQ2FsbGJhY2sgZnVuY3Rpb24gdG8gcmVhZCB0aGUgdmFsdWVzXG4gICpcbiAgKi9cbiAgR3lyb05vcm0ucHJvdG90eXBlLnN0YXJ0ID0gZnVuY3Rpb24oY2FsbGJhY2spIHtcbiAgICBpZiAoIV9pc1JlYWR5KSB7XG4gICAgICBsb2coeyBtZXNzYWdlOiAnR3lyb05vcm0gaXMgbm90IGluaXRpYWxpemVkIHlldC4gRmlyc3QgY2FsbCB0aGUgXCJpbml0KClcIiBmdW5jdGlvbi4nLCBjb2RlOiAxIH0pO1xuICAgICAgcmV0dXJuO1xuICAgIH1cblxuICAgIF9pbnRlcnZhbCA9IHNldEludGVydmFsKGZ1bmN0aW9uKCkge1xuICAgICAgY2FsbGJhY2soc25hcFNob3QoKSk7XG4gICAgfSwgX2ZyZXF1ZW5jeSk7XG4gICAgX2lzUnVubmluZyA9IHRydWU7XG4gIH1cblxuICAvKlxuICAqXG4gICogU3RvcHMgdHJhY2tpbmcgdGhlIHZhbHVlc1xuICAqXG4gICovXG4gIEd5cm9Ob3JtLnByb3RvdHlwZS5zdG9wID0gZnVuY3Rpb24oKSB7XG4gICAgaWYgKF9pbnRlcnZhbCkge1xuICAgICAgY2xlYXJJbnRlcnZhbChfaW50ZXJ2YWwpO1xuICAgICAgX2lzUnVubmluZyA9IGZhbHNlO1xuICAgIH1cbiAgfVxuXG4gIC8qXG4gICpcbiAgKiBUb2dnbGVzIGlmIHRvIG5vcm1hbGl6ZSBncmF2aXR5IHJlbGF0ZWQgdmFsdWVzXG4gICpcbiAgKiBAcGFyYW0gYm9vbGVhbiBmbGFnXG4gICpcbiAgKi9cbiAgR3lyb05vcm0ucHJvdG90eXBlLm5vcm1hbGl6ZUdyYXZpdHkgPSBmdW5jdGlvbihmbGFnKSB7XG4gICAgX2dyYXZpdHlOb3JtYWxpemVkID0gKGZsYWcpID8gdHJ1ZSA6IGZhbHNlO1xuICB9XG5cblxuICAvKlxuICAqXG4gICogU2V0cyB0aGUgY3VycmVudCBoZWFkIGRpcmVjdGlvbiBhcyBhbHBoYSA9IDBcbiAgKiBDYW4gb25seSBiZSB1c2VkIGlmIGRldmljZSBvcmllbnRhdGlvbiBpcyBiZWluZyB0cmFja2VkLCB2YWx1ZXMgYXJlIG5vdCBzY3JlZW4gYWRqdXN0ZWQsIHZhbHVlIHR5cGUgaXMgR3lyb05vcm0uRVVMRVIgYW5kIG9yaWVudGF0aW9uIGJhc2UgaXMgR3lyb05vcm0uR0FNRVxuICAqXG4gICogQHJldHVybjogSWYgaGVhZCBkaXJlY3Rpb24gaXMgc2V0IHN1Y2Nlc3NmdWxseSByZXR1cm5zIHRydWUsIGVsc2UgZmFsc2VcbiAgKlxuICAqL1xuICBHeXJvTm9ybS5wcm90b3R5cGUuc2V0SGVhZERpcmVjdGlvbiA9IGZ1bmN0aW9uKCkge1xuICAgIGlmIChfc2NyZWVuQWRqdXN0ZWQgfHwgX29yaWVudGF0aW9uQmFzZSA9PT0gV09STEQpIHtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG5cbiAgICBfY2FsaWJyYXRpb25WYWx1ZSA9IF9kby5nZXRGaXhlZEZyYW1lRXVsZXIoKS5hbHBoYTtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuXG4gIC8qXG4gICpcbiAgKiBTZXRzIHRoZSBsb2cgZnVuY3Rpb25cbiAgKlxuICAqL1xuICBHeXJvTm9ybS5wcm90b3R5cGUuc3RhcnRMb2dnaW5nID0gZnVuY3Rpb24obG9nZ2VyKSB7XG4gICAgaWYgKGxvZ2dlcikge1xuICAgICAgX2xvZ2dlciA9IGxvZ2dlcjtcbiAgICB9XG4gIH1cblxuICAvKlxuICAqXG4gICogU2V0cyB0aGUgbG9nIGZ1bmN0aW9uIHRvIG51bGwgd2hpY2ggc3RvcHMgdGhlIGxvZ2dpbmdcbiAgKlxuICAqL1xuICBHeXJvTm9ybS5wcm90b3R5cGUuc3RvcExvZ2dpbmcgPSBmdW5jdGlvbigpIHtcbiAgICBfbG9nZ2VyID0gbnVsbDtcbiAgfVxuXG4gIC8qXG4gICpcbiAgKiBSZXR1cm5zIGlmIGNlcnRhaW4gdHlwZSBvZiBldmVudCBpcyBhdmFpbGFibGUgb24gdGhlIGRldmljZVxuICAqXG4gICogQHBhcmFtIHN0cmluZyBfZXZlbnRUeXBlIC0gcG9zc2libGUgdmFsdWVzIGFyZSBcImRldmljZW9yaWVudGF0aW9uXCIgLCBcImRldmljZW1vdGlvblwiICwgXCJjb21wYXNzbmVlZHNjYWxpYnJhdGlvblwiXG4gICpcbiAgKiBAcmV0dXJuIHRydWUgaWYgZXZlbnQgaXMgYXZhaWxhYmxlIGZhbHNlIGlmIG5vdFxuICAqXG4gICovXG4gIEd5cm9Ob3JtLnByb3RvdHlwZS5pc0F2YWlsYWJsZSA9IGZ1bmN0aW9uKF9ldmVudFR5cGUpIHtcblxuICAgIHZhciBkb1NuYXBTaG90ID0gX2RvLmdldFNjcmVlbkFkanVzdGVkRXVsZXIoKTtcbiAgICB2YXIgYWNjU25hcFNob3QgPSBfZG0uZ2V0U2NyZWVuQWRqdXN0ZWRBY2NlbGVyYXRpb24oKTtcbiAgICB2YXIgYWNjR3JhU25hcFNob3QgPSBfZG0uZ2V0U2NyZWVuQWRqdXN0ZWRBY2NlbGVyYXRpb25JbmNsdWRpbmdHcmF2aXR5KCk7XG4gICAgdmFyIHJvdFJhdGVTbmFwU2hvdCA9IF9kbS5nZXRTY3JlZW5BZGp1c3RlZFJvdGF0aW9uUmF0ZSgpO1xuXG4gICAgc3dpdGNoIChfZXZlbnRUeXBlKSB7XG4gICAgICBjYXNlIERFVklDRV9PUklFTlRBVElPTjpcbiAgICAgICAgcmV0dXJuICgoZG9TbmFwU2hvdC5hbHBoYSAmJiBkb1NuYXBTaG90LmFscGhhICE9PSBudWxsKSAmJiAoZG9TbmFwU2hvdC5iZXRhICYmIGRvU25hcFNob3QuYmV0YSAhPT0gbnVsbCkgJiYgKGRvU25hcFNob3QuZ2FtbWEgJiYgZG9TbmFwU2hvdC5nYW1tYSAhPT0gbnVsbCkpO1xuICAgICAgICBicmVhaztcblxuICAgICAgY2FzZSBBQ0NFTEVSQVRJT046XG4gICAgICAgIHJldHVybiAoYWNjU25hcFNob3QgJiYgYWNjU25hcFNob3QueCAmJiBhY2NTbmFwU2hvdC55ICYmIGFjY1NuYXBTaG90LnopO1xuICAgICAgICBicmVhaztcblxuICAgICAgY2FzZSBBQ0NFTEVSQVRJT05fSU5DTFVESU5HX0dSQVZJVFk6XG4gICAgICAgIHJldHVybiAoYWNjR3JhU25hcFNob3QgJiYgYWNjR3JhU25hcFNob3QueCAmJiBhY2NHcmFTbmFwU2hvdC55ICYmIGFjY0dyYVNuYXBTaG90LnopO1xuICAgICAgICBicmVhaztcblxuICAgICAgY2FzZSBST1RBVElPTl9SQVRFOlxuICAgICAgICByZXR1cm4gKHJvdFJhdGVTbmFwU2hvdCAmJiByb3RSYXRlU25hcFNob3QuYWxwaGEgJiYgcm90UmF0ZVNuYXBTaG90LmJldGEgJiYgcm90UmF0ZVNuYXBTaG90LmdhbW1hKTtcbiAgICAgICAgYnJlYWs7XG5cbiAgICAgIGRlZmF1bHQ6XG4gICAgICAgIHJldHVybiB7XG4gICAgICAgICAgZGV2aWNlT3JpZW50YXRpb25BdmFpbGFibGU6ICgoZG9TbmFwU2hvdC5hbHBoYSAmJiBkb1NuYXBTaG90LmFscGhhICE9PSBudWxsKSAmJiAoZG9TbmFwU2hvdC5iZXRhICYmIGRvU25hcFNob3QuYmV0YSAhPT0gbnVsbCkgJiYgKGRvU25hcFNob3QuZ2FtbWEgJiYgZG9TbmFwU2hvdC5nYW1tYSAhPT0gbnVsbCkpLFxuICAgICAgICAgIGFjY2VsZXJhdGlvbkF2YWlsYWJsZTogKGFjY1NuYXBTaG90ICYmIGFjY1NuYXBTaG90LnggJiYgYWNjU25hcFNob3QueSAmJiBhY2NTbmFwU2hvdC56KSxcbiAgICAgICAgICBhY2NlbGVyYXRpb25JbmNsdWRpbmdHcmF2aXR5QXZhaWxhYmxlOiAoYWNjR3JhU25hcFNob3QgJiYgYWNjR3JhU25hcFNob3QueCAmJiBhY2NHcmFTbmFwU2hvdC55ICYmIGFjY0dyYVNuYXBTaG90LnopLFxuICAgICAgICAgIHJvdGF0aW9uUmF0ZUF2YWlsYWJsZTogKHJvdFJhdGVTbmFwU2hvdCAmJiByb3RSYXRlU25hcFNob3QuYWxwaGEgJiYgcm90UmF0ZVNuYXBTaG90LmJldGEgJiYgcm90UmF0ZVNuYXBTaG90LmdhbW1hKVxuICAgICAgICB9XG4gICAgICAgIGJyZWFrO1xuICAgIH1cbiAgfVxuXG4gIC8qXG4gICpcbiAgKiBSZXR1cm5zIGJvb2xlYW4gdmFsdWUgaWYgdGhlIEd5cm9Ob3JtIGlzIHJ1bm5pbmdcbiAgKlxuICAqL1xuICBHeXJvTm9ybS5wcm90b3R5cGUuaXNSdW5uaW5nID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIF9pc1J1bm5pbmc7XG4gIH1cblxuICAvKi0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0qL1xuICAvKiBQUklWQVRFIEZVTkNUSU9OUyAqL1xuXG4gIC8qXG4gICpcbiAgKiBVdGlsaXR5IGZ1bmN0aW9uIHRvIHJvdW5kIHdpdGggZGlnaXRzIGFmdGVyIHRoZSBkZWNpbWFsIHBvaW50XG4gICpcbiAgKiBAcGFyYW0gZmxvYXQgbnVtYmVyIC0gdGhlIG9yaWdpbmFsIG51bWJlciB0byByb3VuZFxuICAqXG4gICovXG4gIGZ1bmN0aW9uIHJuZChudW1iZXIpIHtcbiAgICByZXR1cm4gTWF0aC5yb3VuZChudW1iZXIgKiBNYXRoLnBvdygxMCwgX2RlY2ltYWxDb3VudCkpIC8gTWF0aC5wb3coMTAsIF9kZWNpbWFsQ291bnQpO1xuICB9XG5cbiAgLypcbiAgKlxuICAqIFN0YXJ0cyBjYWxpYnJhdGlvblxuICAqXG4gICovXG4gIGZ1bmN0aW9uIGNhbGlicmF0ZSgpIHtcbiAgICBfaXNDYWxpYnJhdGluZyA9IHRydWU7XG4gICAgX2NhbGlicmF0aW9uVmFsdWVzID0gbmV3IEFycmF5KCk7XG4gIH1cblxuICAvKlxuICAqXG4gICogVGFrZXMgYSBzbmFwc2hvdCBvZiB0aGUgY3VycmVudCBkZXZpY2VvIG9yaWVudGFpb24gYW5kIGRldmljZSBtb3Rpb24gdmFsdWVzXG4gICpcbiAgKi9cbiAgZnVuY3Rpb24gc25hcFNob3QoKSB7XG4gICAgdmFyIGRvU25hcFNob3QgPSB7fTtcblxuICAgIGlmIChfc2NyZWVuQWRqdXN0ZWQpIHtcbiAgICAgIGRvU25hcFNob3QgPSBfZG8uZ2V0U2NyZWVuQWRqdXN0ZWRFdWxlcigpO1xuICAgIH0gZWxzZSB7XG4gICAgICBkb1NuYXBTaG90ID0gX2RvLmdldEZpeGVkRnJhbWVFdWxlcigpO1xuICAgIH1cblxuICAgIHZhciBhY2NTbmFwU2hvdCA9IF9kbS5nZXRTY3JlZW5BZGp1c3RlZEFjY2VsZXJhdGlvbigpO1xuICAgIHZhciBhY2NHcmFTbmFwU2hvdCA9IF9kbS5nZXRTY3JlZW5BZGp1c3RlZEFjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHkoKTtcbiAgICB2YXIgcm90UmF0ZVNuYXBTaG90ID0gX2RtLmdldFNjcmVlbkFkanVzdGVkUm90YXRpb25SYXRlKCk7XG5cbiAgICB2YXIgYWxwaGFUb1NlbmQgPSAwO1xuXG4gICAgaWYgKF9vcmllbnRhdGlvbkJhc2UgPT09IEdBTUUpIHtcbiAgICAgIGFscGhhVG9TZW5kID0gZG9TbmFwU2hvdC5hbHBoYSAtIF9jYWxpYnJhdGlvblZhbHVlO1xuICAgICAgYWxwaGFUb1NlbmQgPSAoYWxwaGFUb1NlbmQgPCAwKSA/ICgzNjAgLSBNYXRoLmFicyhhbHBoYVRvU2VuZCkpIDogYWxwaGFUb1NlbmQ7XG4gICAgfSBlbHNlIHtcbiAgICAgIGFscGhhVG9TZW5kID0gZG9TbmFwU2hvdC5hbHBoYTtcbiAgICB9XG5cbiAgICB2YXIgc25hcFNob3QgPSB7XG4gICAgICBkbzoge1xuICAgICAgICBhbHBoYTogcm5kKGFscGhhVG9TZW5kKSxcbiAgICAgICAgYmV0YTogcm5kKGRvU25hcFNob3QuYmV0YSksXG4gICAgICAgIGdhbW1hOiBybmQoZG9TbmFwU2hvdC5nYW1tYSksXG4gICAgICAgIGFic29sdXRlOiBfZG8uaXNBYnNvbHV0ZSgpXG4gICAgICB9LFxuICAgICAgZG06IHtcbiAgICAgICAgeDogcm5kKGFjY1NuYXBTaG90LngpLFxuICAgICAgICB5OiBybmQoYWNjU25hcFNob3QueSksXG4gICAgICAgIHo6IHJuZChhY2NTbmFwU2hvdC56KSxcbiAgICAgICAgZ3g6IHJuZChhY2NHcmFTbmFwU2hvdC54KSxcbiAgICAgICAgZ3k6IHJuZChhY2NHcmFTbmFwU2hvdC55KSxcbiAgICAgICAgZ3o6IHJuZChhY2NHcmFTbmFwU2hvdC56KSxcbiAgICAgICAgYWxwaGE6IHJuZChyb3RSYXRlU25hcFNob3QuYWxwaGEpLFxuICAgICAgICBiZXRhOiBybmQocm90UmF0ZVNuYXBTaG90LmJldGEpLFxuICAgICAgICBnYW1tYTogcm5kKHJvdFJhdGVTbmFwU2hvdC5nYW1tYSksXG4gICAgICAgIHRpbWVTdGFtcDogX2RtLmdldExhc3RSYXdFdmVudERhdGEoKS50aW1lU3RhbXBcbiAgICAgIH1cbiAgICB9O1xuXG4gICAgLy8gTm9ybWFsaXplIGdyYXZpdHlcbiAgICBpZiAoX2dyYXZpdHlOb3JtYWxpemVkKSB7XG4gICAgICBzbmFwU2hvdC5kbS5neCAqPSBfZ3Jhdml0eUNvZWZmaWNpZW50O1xuICAgICAgc25hcFNob3QuZG0uZ3kgKj0gX2dyYXZpdHlDb2VmZmljaWVudDtcbiAgICAgIHNuYXBTaG90LmRtLmd6ICo9IF9ncmF2aXR5Q29lZmZpY2llbnQ7XG4gICAgfVxuXG4gICAgcmV0dXJuIHNuYXBTaG90O1xuICB9XG5cblxuICAvKlxuICAqXG4gICogU3RhcnRzIGxpc3RlbmluZyB0byBvcmllbnRhdGlvbiBldmVudCBvbiB0aGUgd2luZG93IG9iamVjdFxuICAqXG4gICovXG4gIGZ1bmN0aW9uIGxvZyhlcnIpIHtcbiAgICBpZiAoX2xvZ2dlcikge1xuICAgICAgaWYgKHR5cGVvZihlcnIpID09ICdzdHJpbmcnKSB7XG4gICAgICAgIGVyciA9IHsgbWVzc2FnZTogZXJyLCBjb2RlOiAwIH1cbiAgICAgIH1cbiAgICAgIF9sb2dnZXIoZXJyKTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gR3lyb05vcm07XG59KSk7IiwiLy8gc2hpbSBmb3IgdXNpbmcgcHJvY2VzcyBpbiBicm93c2VyXG5cbnZhciBwcm9jZXNzID0gbW9kdWxlLmV4cG9ydHMgPSB7fTtcblxuLy8gY2FjaGVkIGZyb20gd2hhdGV2ZXIgZ2xvYmFsIGlzIHByZXNlbnQgc28gdGhhdCB0ZXN0IHJ1bm5lcnMgdGhhdCBzdHViIGl0XG4vLyBkb24ndCBicmVhayB0aGluZ3MuICBCdXQgd2UgbmVlZCB0byB3cmFwIGl0IGluIGEgdHJ5IGNhdGNoIGluIGNhc2UgaXQgaXNcbi8vIHdyYXBwZWQgaW4gc3RyaWN0IG1vZGUgY29kZSB3aGljaCBkb2Vzbid0IGRlZmluZSBhbnkgZ2xvYmFscy4gIEl0J3MgaW5zaWRlIGFcbi8vIGZ1bmN0aW9uIGJlY2F1c2UgdHJ5L2NhdGNoZXMgZGVvcHRpbWl6ZSBpbiBjZXJ0YWluIGVuZ2luZXMuXG5cbnZhciBjYWNoZWRTZXRUaW1lb3V0O1xudmFyIGNhY2hlZENsZWFyVGltZW91dDtcblxuKGZ1bmN0aW9uICgpIHtcbiAgdHJ5IHtcbiAgICBjYWNoZWRTZXRUaW1lb3V0ID0gc2V0VGltZW91dDtcbiAgfSBjYXRjaCAoZSkge1xuICAgIGNhY2hlZFNldFRpbWVvdXQgPSBmdW5jdGlvbiAoKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoJ3NldFRpbWVvdXQgaXMgbm90IGRlZmluZWQnKTtcbiAgICB9XG4gIH1cbiAgdHJ5IHtcbiAgICBjYWNoZWRDbGVhclRpbWVvdXQgPSBjbGVhclRpbWVvdXQ7XG4gIH0gY2F0Y2ggKGUpIHtcbiAgICBjYWNoZWRDbGVhclRpbWVvdXQgPSBmdW5jdGlvbiAoKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoJ2NsZWFyVGltZW91dCBpcyBub3QgZGVmaW5lZCcpO1xuICAgIH1cbiAgfVxufSAoKSlcbnZhciBxdWV1ZSA9IFtdO1xudmFyIGRyYWluaW5nID0gZmFsc2U7XG52YXIgY3VycmVudFF1ZXVlO1xudmFyIHF1ZXVlSW5kZXggPSAtMTtcblxuZnVuY3Rpb24gY2xlYW5VcE5leHRUaWNrKCkge1xuICAgIGlmICghZHJhaW5pbmcgfHwgIWN1cnJlbnRRdWV1ZSkge1xuICAgICAgICByZXR1cm47XG4gICAgfVxuICAgIGRyYWluaW5nID0gZmFsc2U7XG4gICAgaWYgKGN1cnJlbnRRdWV1ZS5sZW5ndGgpIHtcbiAgICAgICAgcXVldWUgPSBjdXJyZW50UXVldWUuY29uY2F0KHF1ZXVlKTtcbiAgICB9IGVsc2Uge1xuICAgICAgICBxdWV1ZUluZGV4ID0gLTE7XG4gICAgfVxuICAgIGlmIChxdWV1ZS5sZW5ndGgpIHtcbiAgICAgICAgZHJhaW5RdWV1ZSgpO1xuICAgIH1cbn1cblxuZnVuY3Rpb24gZHJhaW5RdWV1ZSgpIHtcbiAgICBpZiAoZHJhaW5pbmcpIHtcbiAgICAgICAgcmV0dXJuO1xuICAgIH1cbiAgICB2YXIgdGltZW91dCA9IGNhY2hlZFNldFRpbWVvdXQoY2xlYW5VcE5leHRUaWNrKTtcbiAgICBkcmFpbmluZyA9IHRydWU7XG5cbiAgICB2YXIgbGVuID0gcXVldWUubGVuZ3RoO1xuICAgIHdoaWxlKGxlbikge1xuICAgICAgICBjdXJyZW50UXVldWUgPSBxdWV1ZTtcbiAgICAgICAgcXVldWUgPSBbXTtcbiAgICAgICAgd2hpbGUgKCsrcXVldWVJbmRleCA8IGxlbikge1xuICAgICAgICAgICAgaWYgKGN1cnJlbnRRdWV1ZSkge1xuICAgICAgICAgICAgICAgIGN1cnJlbnRRdWV1ZVtxdWV1ZUluZGV4XS5ydW4oKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgICBxdWV1ZUluZGV4ID0gLTE7XG4gICAgICAgIGxlbiA9IHF1ZXVlLmxlbmd0aDtcbiAgICB9XG4gICAgY3VycmVudFF1ZXVlID0gbnVsbDtcbiAgICBkcmFpbmluZyA9IGZhbHNlO1xuICAgIGNhY2hlZENsZWFyVGltZW91dCh0aW1lb3V0KTtcbn1cblxucHJvY2Vzcy5uZXh0VGljayA9IGZ1bmN0aW9uIChmdW4pIHtcbiAgICB2YXIgYXJncyA9IG5ldyBBcnJheShhcmd1bWVudHMubGVuZ3RoIC0gMSk7XG4gICAgaWYgKGFyZ3VtZW50cy5sZW5ndGggPiAxKSB7XG4gICAgICAgIGZvciAodmFyIGkgPSAxOyBpIDwgYXJndW1lbnRzLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgICAgICBhcmdzW2kgLSAxXSA9IGFyZ3VtZW50c1tpXTtcbiAgICAgICAgfVxuICAgIH1cbiAgICBxdWV1ZS5wdXNoKG5ldyBJdGVtKGZ1biwgYXJncykpO1xuICAgIGlmIChxdWV1ZS5sZW5ndGggPT09IDEgJiYgIWRyYWluaW5nKSB7XG4gICAgICAgIGNhY2hlZFNldFRpbWVvdXQoZHJhaW5RdWV1ZSwgMCk7XG4gICAgfVxufTtcblxuLy8gdjggbGlrZXMgcHJlZGljdGlibGUgb2JqZWN0c1xuZnVuY3Rpb24gSXRlbShmdW4sIGFycmF5KSB7XG4gICAgdGhpcy5mdW4gPSBmdW47XG4gICAgdGhpcy5hcnJheSA9IGFycmF5O1xufVxuSXRlbS5wcm90b3R5cGUucnVuID0gZnVuY3Rpb24gKCkge1xuICAgIHRoaXMuZnVuLmFwcGx5KG51bGwsIHRoaXMuYXJyYXkpO1xufTtcbnByb2Nlc3MudGl0bGUgPSAnYnJvd3Nlcic7XG5wcm9jZXNzLmJyb3dzZXIgPSB0cnVlO1xucHJvY2Vzcy5lbnYgPSB7fTtcbnByb2Nlc3MuYXJndiA9IFtdO1xucHJvY2Vzcy52ZXJzaW9uID0gJyc7IC8vIGVtcHR5IHN0cmluZyB0byBhdm9pZCByZWdleHAgaXNzdWVzXG5wcm9jZXNzLnZlcnNpb25zID0ge307XG5cbmZ1bmN0aW9uIG5vb3AoKSB7fVxuXG5wcm9jZXNzLm9uID0gbm9vcDtcbnByb2Nlc3MuYWRkTGlzdGVuZXIgPSBub29wO1xucHJvY2Vzcy5vbmNlID0gbm9vcDtcbnByb2Nlc3Mub2ZmID0gbm9vcDtcbnByb2Nlc3MucmVtb3ZlTGlzdGVuZXIgPSBub29wO1xucHJvY2Vzcy5yZW1vdmVBbGxMaXN0ZW5lcnMgPSBub29wO1xucHJvY2Vzcy5lbWl0ID0gbm9vcDtcblxucHJvY2Vzcy5iaW5kaW5nID0gZnVuY3Rpb24gKG5hbWUpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoJ3Byb2Nlc3MuYmluZGluZyBpcyBub3Qgc3VwcG9ydGVkJyk7XG59O1xuXG5wcm9jZXNzLmN3ZCA9IGZ1bmN0aW9uICgpIHsgcmV0dXJuICcvJyB9O1xucHJvY2Vzcy5jaGRpciA9IGZ1bmN0aW9uIChkaXIpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoJ3Byb2Nlc3MuY2hkaXIgaXMgbm90IHN1cHBvcnRlZCcpO1xufTtcbnByb2Nlc3MudW1hc2sgPSBmdW5jdGlvbigpIHsgcmV0dXJuIDA7IH07XG4iLCIvKiFcbiAqIEBvdmVydmlldyBlczYtcHJvbWlzZSAtIGEgdGlueSBpbXBsZW1lbnRhdGlvbiBvZiBQcm9taXNlcy9BKy5cbiAqIEBjb3B5cmlnaHQgQ29weXJpZ2h0IChjKSAyMDE0IFllaHVkYSBLYXR6LCBUb20gRGFsZSwgU3RlZmFuIFBlbm5lciBhbmQgY29udHJpYnV0b3JzIChDb252ZXJzaW9uIHRvIEVTNiBBUEkgYnkgSmFrZSBBcmNoaWJhbGQpXG4gKiBAbGljZW5zZSAgIExpY2Vuc2VkIHVuZGVyIE1JVCBsaWNlbnNlXG4gKiAgICAgICAgICAgIFNlZSBodHRwczovL3Jhdy5naXRodWJ1c2VyY29udGVudC5jb20vamFrZWFyY2hpYmFsZC9lczYtcHJvbWlzZS9tYXN0ZXIvTElDRU5TRVxuICogQHZlcnNpb24gICAzLjIuMVxuICovXG5cbihmdW5jdGlvbigpIHtcbiAgICBcInVzZSBzdHJpY3RcIjtcbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkdXRpbHMkJG9iamVjdE9yRnVuY3Rpb24oeCkge1xuICAgICAgcmV0dXJuIHR5cGVvZiB4ID09PSAnZnVuY3Rpb24nIHx8ICh0eXBlb2YgeCA9PT0gJ29iamVjdCcgJiYgeCAhPT0gbnVsbCk7XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJHV0aWxzJCRpc0Z1bmN0aW9uKHgpIHtcbiAgICAgIHJldHVybiB0eXBlb2YgeCA9PT0gJ2Z1bmN0aW9uJztcbiAgICB9XG5cbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkdXRpbHMkJGlzTWF5YmVUaGVuYWJsZSh4KSB7XG4gICAgICByZXR1cm4gdHlwZW9mIHggPT09ICdvYmplY3QnICYmIHggIT09IG51bGw7XG4gICAgfVxuXG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSR1dGlscyQkX2lzQXJyYXk7XG4gICAgaWYgKCFBcnJheS5pc0FycmF5KSB7XG4gICAgICBsaWIkZXM2JHByb21pc2UkdXRpbHMkJF9pc0FycmF5ID0gZnVuY3Rpb24gKHgpIHtcbiAgICAgICAgcmV0dXJuIE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbCh4KSA9PT0gJ1tvYmplY3QgQXJyYXldJztcbiAgICAgIH07XG4gICAgfSBlbHNlIHtcbiAgICAgIGxpYiRlczYkcHJvbWlzZSR1dGlscyQkX2lzQXJyYXkgPSBBcnJheS5pc0FycmF5O1xuICAgIH1cblxuICAgIHZhciBsaWIkZXM2JHByb21pc2UkdXRpbHMkJGlzQXJyYXkgPSBsaWIkZXM2JHByb21pc2UkdXRpbHMkJF9pc0FycmF5O1xuICAgIHZhciBsaWIkZXM2JHByb21pc2UkYXNhcCQkbGVuID0gMDtcbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJGFzYXAkJHZlcnR4TmV4dDtcbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJGFzYXAkJGN1c3RvbVNjaGVkdWxlckZuO1xuXG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRhc2FwID0gZnVuY3Rpb24gYXNhcChjYWxsYmFjaywgYXJnKSB7XG4gICAgICBsaWIkZXM2JHByb21pc2UkYXNhcCQkcXVldWVbbGliJGVzNiRwcm9taXNlJGFzYXAkJGxlbl0gPSBjYWxsYmFjaztcbiAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRxdWV1ZVtsaWIkZXM2JHByb21pc2UkYXNhcCQkbGVuICsgMV0gPSBhcmc7XG4gICAgICBsaWIkZXM2JHByb21pc2UkYXNhcCQkbGVuICs9IDI7XG4gICAgICBpZiAobGliJGVzNiRwcm9taXNlJGFzYXAkJGxlbiA9PT0gMikge1xuICAgICAgICAvLyBJZiBsZW4gaXMgMiwgdGhhdCBtZWFucyB0aGF0IHdlIG5lZWQgdG8gc2NoZWR1bGUgYW4gYXN5bmMgZmx1c2guXG4gICAgICAgIC8vIElmIGFkZGl0aW9uYWwgY2FsbGJhY2tzIGFyZSBxdWV1ZWQgYmVmb3JlIHRoZSBxdWV1ZSBpcyBmbHVzaGVkLCB0aGV5XG4gICAgICAgIC8vIHdpbGwgYmUgcHJvY2Vzc2VkIGJ5IHRoaXMgZmx1c2ggdGhhdCB3ZSBhcmUgc2NoZWR1bGluZy5cbiAgICAgICAgaWYgKGxpYiRlczYkcHJvbWlzZSRhc2FwJCRjdXN0b21TY2hlZHVsZXJGbikge1xuICAgICAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRjdXN0b21TY2hlZHVsZXJGbihsaWIkZXM2JHByb21pc2UkYXNhcCQkZmx1c2gpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRzY2hlZHVsZUZsdXNoKCk7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG5cbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkYXNhcCQkc2V0U2NoZWR1bGVyKHNjaGVkdWxlRm4pIHtcbiAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRjdXN0b21TY2hlZHVsZXJGbiA9IHNjaGVkdWxlRm47XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJGFzYXAkJHNldEFzYXAoYXNhcEZuKSB7XG4gICAgICBsaWIkZXM2JHByb21pc2UkYXNhcCQkYXNhcCA9IGFzYXBGbjtcbiAgICB9XG5cbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJGFzYXAkJGJyb3dzZXJXaW5kb3cgPSAodHlwZW9mIHdpbmRvdyAhPT0gJ3VuZGVmaW5lZCcpID8gd2luZG93IDogdW5kZWZpbmVkO1xuICAgIHZhciBsaWIkZXM2JHByb21pc2UkYXNhcCQkYnJvd3Nlckdsb2JhbCA9IGxpYiRlczYkcHJvbWlzZSRhc2FwJCRicm93c2VyV2luZG93IHx8IHt9O1xuICAgIHZhciBsaWIkZXM2JHByb21pc2UkYXNhcCQkQnJvd3Nlck11dGF0aW9uT2JzZXJ2ZXIgPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkYnJvd3Nlckdsb2JhbC5NdXRhdGlvbk9ic2VydmVyIHx8IGxpYiRlczYkcHJvbWlzZSRhc2FwJCRicm93c2VyR2xvYmFsLldlYktpdE11dGF0aW9uT2JzZXJ2ZXI7XG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRpc05vZGUgPSB0eXBlb2Ygc2VsZiA9PT0gJ3VuZGVmaW5lZCcgJiYgdHlwZW9mIHByb2Nlc3MgIT09ICd1bmRlZmluZWQnICYmIHt9LnRvU3RyaW5nLmNhbGwocHJvY2VzcykgPT09ICdbb2JqZWN0IHByb2Nlc3NdJztcblxuICAgIC8vIHRlc3QgZm9yIHdlYiB3b3JrZXIgYnV0IG5vdCBpbiBJRTEwXG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRpc1dvcmtlciA9IHR5cGVvZiBVaW50OENsYW1wZWRBcnJheSAhPT0gJ3VuZGVmaW5lZCcgJiZcbiAgICAgIHR5cGVvZiBpbXBvcnRTY3JpcHRzICE9PSAndW5kZWZpbmVkJyAmJlxuICAgICAgdHlwZW9mIE1lc3NhZ2VDaGFubmVsICE9PSAndW5kZWZpbmVkJztcblxuICAgIC8vIG5vZGVcbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkYXNhcCQkdXNlTmV4dFRpY2soKSB7XG4gICAgICAvLyBub2RlIHZlcnNpb24gMC4xMC54IGRpc3BsYXlzIGEgZGVwcmVjYXRpb24gd2FybmluZyB3aGVuIG5leHRUaWNrIGlzIHVzZWQgcmVjdXJzaXZlbHlcbiAgICAgIC8vIHNlZSBodHRwczovL2dpdGh1Yi5jb20vY3Vqb2pzL3doZW4vaXNzdWVzLzQxMCBmb3IgZGV0YWlsc1xuICAgICAgcmV0dXJuIGZ1bmN0aW9uKCkge1xuICAgICAgICBwcm9jZXNzLm5leHRUaWNrKGxpYiRlczYkcHJvbWlzZSRhc2FwJCRmbHVzaCk7XG4gICAgICB9O1xuICAgIH1cblxuICAgIC8vIHZlcnR4XG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJGFzYXAkJHVzZVZlcnR4VGltZXIoKSB7XG4gICAgICByZXR1cm4gZnVuY3Rpb24oKSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCR2ZXJ0eE5leHQobGliJGVzNiRwcm9taXNlJGFzYXAkJGZsdXNoKTtcbiAgICAgIH07XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJGFzYXAkJHVzZU11dGF0aW9uT2JzZXJ2ZXIoKSB7XG4gICAgICB2YXIgaXRlcmF0aW9ucyA9IDA7XG4gICAgICB2YXIgb2JzZXJ2ZXIgPSBuZXcgbGliJGVzNiRwcm9taXNlJGFzYXAkJEJyb3dzZXJNdXRhdGlvbk9ic2VydmVyKGxpYiRlczYkcHJvbWlzZSRhc2FwJCRmbHVzaCk7XG4gICAgICB2YXIgbm9kZSA9IGRvY3VtZW50LmNyZWF0ZVRleHROb2RlKCcnKTtcbiAgICAgIG9ic2VydmVyLm9ic2VydmUobm9kZSwgeyBjaGFyYWN0ZXJEYXRhOiB0cnVlIH0pO1xuXG4gICAgICByZXR1cm4gZnVuY3Rpb24oKSB7XG4gICAgICAgIG5vZGUuZGF0YSA9IChpdGVyYXRpb25zID0gKytpdGVyYXRpb25zICUgMik7XG4gICAgICB9O1xuICAgIH1cblxuICAgIC8vIHdlYiB3b3JrZXJcbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkYXNhcCQkdXNlTWVzc2FnZUNoYW5uZWwoKSB7XG4gICAgICB2YXIgY2hhbm5lbCA9IG5ldyBNZXNzYWdlQ2hhbm5lbCgpO1xuICAgICAgY2hhbm5lbC5wb3J0MS5vbm1lc3NhZ2UgPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkZmx1c2g7XG4gICAgICByZXR1cm4gZnVuY3Rpb24gKCkge1xuICAgICAgICBjaGFubmVsLnBvcnQyLnBvc3RNZXNzYWdlKDApO1xuICAgICAgfTtcbiAgICB9XG5cbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkYXNhcCQkdXNlU2V0VGltZW91dCgpIHtcbiAgICAgIHJldHVybiBmdW5jdGlvbigpIHtcbiAgICAgICAgc2V0VGltZW91dChsaWIkZXM2JHByb21pc2UkYXNhcCQkZmx1c2gsIDEpO1xuICAgICAgfTtcbiAgICB9XG5cbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJGFzYXAkJHF1ZXVlID0gbmV3IEFycmF5KDEwMDApO1xuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRmbHVzaCgpIHtcbiAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbGliJGVzNiRwcm9taXNlJGFzYXAkJGxlbjsgaSs9Mikge1xuICAgICAgICB2YXIgY2FsbGJhY2sgPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkcXVldWVbaV07XG4gICAgICAgIHZhciBhcmcgPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkcXVldWVbaSsxXTtcblxuICAgICAgICBjYWxsYmFjayhhcmcpO1xuXG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRxdWV1ZVtpXSA9IHVuZGVmaW5lZDtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJGFzYXAkJHF1ZXVlW2krMV0gPSB1bmRlZmluZWQ7XG4gICAgICB9XG5cbiAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRsZW4gPSAwO1xuICAgIH1cblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRhdHRlbXB0VmVydHgoKSB7XG4gICAgICB0cnkge1xuICAgICAgICB2YXIgciA9IHJlcXVpcmU7XG4gICAgICAgIHZhciB2ZXJ0eCA9IHIoJ3ZlcnR4Jyk7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCR2ZXJ0eE5leHQgPSB2ZXJ0eC5ydW5Pbkxvb3AgfHwgdmVydHgucnVuT25Db250ZXh0O1xuICAgICAgICByZXR1cm4gbGliJGVzNiRwcm9taXNlJGFzYXAkJHVzZVZlcnR4VGltZXIoKTtcbiAgICAgIH0gY2F0Y2goZSkge1xuICAgICAgICByZXR1cm4gbGliJGVzNiRwcm9taXNlJGFzYXAkJHVzZVNldFRpbWVvdXQoKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJGFzYXAkJHNjaGVkdWxlRmx1c2g7XG4gICAgLy8gRGVjaWRlIHdoYXQgYXN5bmMgbWV0aG9kIHRvIHVzZSB0byB0cmlnZ2VyaW5nIHByb2Nlc3Npbmcgb2YgcXVldWVkIGNhbGxiYWNrczpcbiAgICBpZiAobGliJGVzNiRwcm9taXNlJGFzYXAkJGlzTm9kZSkge1xuICAgICAgbGliJGVzNiRwcm9taXNlJGFzYXAkJHNjaGVkdWxlRmx1c2ggPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkdXNlTmV4dFRpY2soKTtcbiAgICB9IGVsc2UgaWYgKGxpYiRlczYkcHJvbWlzZSRhc2FwJCRCcm93c2VyTXV0YXRpb25PYnNlcnZlcikge1xuICAgICAgbGliJGVzNiRwcm9taXNlJGFzYXAkJHNjaGVkdWxlRmx1c2ggPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkdXNlTXV0YXRpb25PYnNlcnZlcigpO1xuICAgIH0gZWxzZSBpZiAobGliJGVzNiRwcm9taXNlJGFzYXAkJGlzV29ya2VyKSB7XG4gICAgICBsaWIkZXM2JHByb21pc2UkYXNhcCQkc2NoZWR1bGVGbHVzaCA9IGxpYiRlczYkcHJvbWlzZSRhc2FwJCR1c2VNZXNzYWdlQ2hhbm5lbCgpO1xuICAgIH0gZWxzZSBpZiAobGliJGVzNiRwcm9taXNlJGFzYXAkJGJyb3dzZXJXaW5kb3cgPT09IHVuZGVmaW5lZCAmJiB0eXBlb2YgcmVxdWlyZSA9PT0gJ2Z1bmN0aW9uJykge1xuICAgICAgbGliJGVzNiRwcm9taXNlJGFzYXAkJHNjaGVkdWxlRmx1c2ggPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkYXR0ZW1wdFZlcnR4KCk7XG4gICAgfSBlbHNlIHtcbiAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRzY2hlZHVsZUZsdXNoID0gbGliJGVzNiRwcm9taXNlJGFzYXAkJHVzZVNldFRpbWVvdXQoKTtcbiAgICB9XG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJHRoZW4kJHRoZW4ob25GdWxmaWxsbWVudCwgb25SZWplY3Rpb24pIHtcbiAgICAgIHZhciBwYXJlbnQgPSB0aGlzO1xuXG4gICAgICB2YXIgY2hpbGQgPSBuZXcgdGhpcy5jb25zdHJ1Y3RvcihsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRub29wKTtcblxuICAgICAgaWYgKGNoaWxkW2xpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJFBST01JU0VfSURdID09PSB1bmRlZmluZWQpIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkbWFrZVByb21pc2UoY2hpbGQpO1xuICAgICAgfVxuXG4gICAgICB2YXIgc3RhdGUgPSBwYXJlbnQuX3N0YXRlO1xuXG4gICAgICBpZiAoc3RhdGUpIHtcbiAgICAgICAgdmFyIGNhbGxiYWNrID0gYXJndW1lbnRzW3N0YXRlIC0gMV07XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRhc2FwKGZ1bmN0aW9uKCl7XG4gICAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkaW52b2tlQ2FsbGJhY2soc3RhdGUsIGNoaWxkLCBjYWxsYmFjaywgcGFyZW50Ll9yZXN1bHQpO1xuICAgICAgICB9KTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHN1YnNjcmliZShwYXJlbnQsIGNoaWxkLCBvbkZ1bGZpbGxtZW50LCBvblJlamVjdGlvbik7XG4gICAgICB9XG5cbiAgICAgIHJldHVybiBjaGlsZDtcbiAgICB9XG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSR0aGVuJCRkZWZhdWx0ID0gbGliJGVzNiRwcm9taXNlJHRoZW4kJHRoZW47XG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJHByb21pc2UkcmVzb2x2ZSQkcmVzb2x2ZShvYmplY3QpIHtcbiAgICAgIC8qanNoaW50IHZhbGlkdGhpczp0cnVlICovXG4gICAgICB2YXIgQ29uc3RydWN0b3IgPSB0aGlzO1xuXG4gICAgICBpZiAob2JqZWN0ICYmIHR5cGVvZiBvYmplY3QgPT09ICdvYmplY3QnICYmIG9iamVjdC5jb25zdHJ1Y3RvciA9PT0gQ29uc3RydWN0b3IpIHtcbiAgICAgICAgcmV0dXJuIG9iamVjdDtcbiAgICAgIH1cblxuICAgICAgdmFyIHByb21pc2UgPSBuZXcgQ29uc3RydWN0b3IobGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkbm9vcCk7XG4gICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRyZXNvbHZlKHByb21pc2UsIG9iamVjdCk7XG4gICAgICByZXR1cm4gcHJvbWlzZTtcbiAgICB9XG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJHJlc29sdmUkJGRlZmF1bHQgPSBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSRyZXNvbHZlJCRyZXNvbHZlO1xuICAgIHZhciBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRQUk9NSVNFX0lEID0gTWF0aC5yYW5kb20oKS50b1N0cmluZygzNikuc3Vic3RyaW5nKDE2KTtcblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJG5vb3AoKSB7fVxuXG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJFBFTkRJTkcgICA9IHZvaWQgMDtcbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkRlVMRklMTEVEID0gMTtcbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUkVKRUNURUQgID0gMjtcblxuICAgIHZhciBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRHRVRfVEhFTl9FUlJPUiA9IG5ldyBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRFcnJvck9iamVjdCgpO1xuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkc2VsZkZ1bGZpbGxtZW50KCkge1xuICAgICAgcmV0dXJuIG5ldyBUeXBlRXJyb3IoXCJZb3UgY2Fubm90IHJlc29sdmUgYSBwcm9taXNlIHdpdGggaXRzZWxmXCIpO1xuICAgIH1cblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGNhbm5vdFJldHVybk93bigpIHtcbiAgICAgIHJldHVybiBuZXcgVHlwZUVycm9yKCdBIHByb21pc2VzIGNhbGxiYWNrIGNhbm5vdCByZXR1cm4gdGhhdCBzYW1lIHByb21pc2UuJyk7XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkZ2V0VGhlbihwcm9taXNlKSB7XG4gICAgICB0cnkge1xuICAgICAgICByZXR1cm4gcHJvbWlzZS50aGVuO1xuICAgICAgfSBjYXRjaChlcnJvcikge1xuICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRHRVRfVEhFTl9FUlJPUi5lcnJvciA9IGVycm9yO1xuICAgICAgICByZXR1cm4gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkR0VUX1RIRU5fRVJST1I7XG4gICAgICB9XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkdHJ5VGhlbih0aGVuLCB2YWx1ZSwgZnVsZmlsbG1lbnRIYW5kbGVyLCByZWplY3Rpb25IYW5kbGVyKSB7XG4gICAgICB0cnkge1xuICAgICAgICB0aGVuLmNhbGwodmFsdWUsIGZ1bGZpbGxtZW50SGFuZGxlciwgcmVqZWN0aW9uSGFuZGxlcik7XG4gICAgICB9IGNhdGNoKGUpIHtcbiAgICAgICAgcmV0dXJuIGU7XG4gICAgICB9XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkaGFuZGxlRm9yZWlnblRoZW5hYmxlKHByb21pc2UsIHRoZW5hYmxlLCB0aGVuKSB7XG4gICAgICAgbGliJGVzNiRwcm9taXNlJGFzYXAkJGFzYXAoZnVuY3Rpb24ocHJvbWlzZSkge1xuICAgICAgICB2YXIgc2VhbGVkID0gZmFsc2U7XG4gICAgICAgIHZhciBlcnJvciA9IGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHRyeVRoZW4odGhlbiwgdGhlbmFibGUsIGZ1bmN0aW9uKHZhbHVlKSB7XG4gICAgICAgICAgaWYgKHNlYWxlZCkgeyByZXR1cm47IH1cbiAgICAgICAgICBzZWFsZWQgPSB0cnVlO1xuICAgICAgICAgIGlmICh0aGVuYWJsZSAhPT0gdmFsdWUpIHtcbiAgICAgICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHJlc29sdmUocHJvbWlzZSwgdmFsdWUpO1xuICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRmdWxmaWxsKHByb21pc2UsIHZhbHVlKTtcbiAgICAgICAgICB9XG4gICAgICAgIH0sIGZ1bmN0aW9uKHJlYXNvbikge1xuICAgICAgICAgIGlmIChzZWFsZWQpIHsgcmV0dXJuOyB9XG4gICAgICAgICAgc2VhbGVkID0gdHJ1ZTtcblxuICAgICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHJlamVjdChwcm9taXNlLCByZWFzb24pO1xuICAgICAgICB9LCAnU2V0dGxlOiAnICsgKHByb21pc2UuX2xhYmVsIHx8ICcgdW5rbm93biBwcm9taXNlJykpO1xuXG4gICAgICAgIGlmICghc2VhbGVkICYmIGVycm9yKSB7XG4gICAgICAgICAgc2VhbGVkID0gdHJ1ZTtcbiAgICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRyZWplY3QocHJvbWlzZSwgZXJyb3IpO1xuICAgICAgICB9XG4gICAgICB9LCBwcm9taXNlKTtcbiAgICB9XG5cbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRoYW5kbGVPd25UaGVuYWJsZShwcm9taXNlLCB0aGVuYWJsZSkge1xuICAgICAgaWYgKHRoZW5hYmxlLl9zdGF0ZSA9PT0gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkRlVMRklMTEVEKSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGZ1bGZpbGwocHJvbWlzZSwgdGhlbmFibGUuX3Jlc3VsdCk7XG4gICAgICB9IGVsc2UgaWYgKHRoZW5hYmxlLl9zdGF0ZSA9PT0gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUkVKRUNURUQpIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcmVqZWN0KHByb21pc2UsIHRoZW5hYmxlLl9yZXN1bHQpO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkc3Vic2NyaWJlKHRoZW5hYmxlLCB1bmRlZmluZWQsIGZ1bmN0aW9uKHZhbHVlKSB7XG4gICAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcmVzb2x2ZShwcm9taXNlLCB2YWx1ZSk7XG4gICAgICAgIH0sIGZ1bmN0aW9uKHJlYXNvbikge1xuICAgICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHJlamVjdChwcm9taXNlLCByZWFzb24pO1xuICAgICAgICB9KTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRoYW5kbGVNYXliZVRoZW5hYmxlKHByb21pc2UsIG1heWJlVGhlbmFibGUsIHRoZW4pIHtcbiAgICAgIGlmIChtYXliZVRoZW5hYmxlLmNvbnN0cnVjdG9yID09PSBwcm9taXNlLmNvbnN0cnVjdG9yICYmXG4gICAgICAgICAgdGhlbiA9PT0gbGliJGVzNiRwcm9taXNlJHRoZW4kJGRlZmF1bHQgJiZcbiAgICAgICAgICBjb25zdHJ1Y3Rvci5yZXNvbHZlID09PSBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSRyZXNvbHZlJCRkZWZhdWx0KSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGhhbmRsZU93blRoZW5hYmxlKHByb21pc2UsIG1heWJlVGhlbmFibGUpO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgaWYgKHRoZW4gPT09IGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJEdFVF9USEVOX0VSUk9SKSB7XG4gICAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcmVqZWN0KHByb21pc2UsIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJEdFVF9USEVOX0VSUk9SLmVycm9yKTtcbiAgICAgICAgfSBlbHNlIGlmICh0aGVuID09PSB1bmRlZmluZWQpIHtcbiAgICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRmdWxmaWxsKHByb21pc2UsIG1heWJlVGhlbmFibGUpO1xuICAgICAgICB9IGVsc2UgaWYgKGxpYiRlczYkcHJvbWlzZSR1dGlscyQkaXNGdW5jdGlvbih0aGVuKSkge1xuICAgICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGhhbmRsZUZvcmVpZ25UaGVuYWJsZShwcm9taXNlLCBtYXliZVRoZW5hYmxlLCB0aGVuKTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRmdWxmaWxsKHByb21pc2UsIG1heWJlVGhlbmFibGUpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcmVzb2x2ZShwcm9taXNlLCB2YWx1ZSkge1xuICAgICAgaWYgKHByb21pc2UgPT09IHZhbHVlKSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHJlamVjdChwcm9taXNlLCBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRzZWxmRnVsZmlsbG1lbnQoKSk7XG4gICAgICB9IGVsc2UgaWYgKGxpYiRlczYkcHJvbWlzZSR1dGlscyQkb2JqZWN0T3JGdW5jdGlvbih2YWx1ZSkpIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkaGFuZGxlTWF5YmVUaGVuYWJsZShwcm9taXNlLCB2YWx1ZSwgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkZ2V0VGhlbih2YWx1ZSkpO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkZnVsZmlsbChwcm9taXNlLCB2YWx1ZSk7XG4gICAgICB9XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcHVibGlzaFJlamVjdGlvbihwcm9taXNlKSB7XG4gICAgICBpZiAocHJvbWlzZS5fb25lcnJvcikge1xuICAgICAgICBwcm9taXNlLl9vbmVycm9yKHByb21pc2UuX3Jlc3VsdCk7XG4gICAgICB9XG5cbiAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHB1Ymxpc2gocHJvbWlzZSk7XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkZnVsZmlsbChwcm9taXNlLCB2YWx1ZSkge1xuICAgICAgaWYgKHByb21pc2UuX3N0YXRlICE9PSBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRQRU5ESU5HKSB7IHJldHVybjsgfVxuXG4gICAgICBwcm9taXNlLl9yZXN1bHQgPSB2YWx1ZTtcbiAgICAgIHByb21pc2UuX3N0YXRlID0gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkRlVMRklMTEVEO1xuXG4gICAgICBpZiAocHJvbWlzZS5fc3Vic2NyaWJlcnMubGVuZ3RoICE9PSAwKSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRhc2FwKGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHB1Ymxpc2gsIHByb21pc2UpO1xuICAgICAgfVxuICAgIH1cblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHJlamVjdChwcm9taXNlLCByZWFzb24pIHtcbiAgICAgIGlmIChwcm9taXNlLl9zdGF0ZSAhPT0gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUEVORElORykgeyByZXR1cm47IH1cbiAgICAgIHByb21pc2UuX3N0YXRlID0gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUkVKRUNURUQ7XG4gICAgICBwcm9taXNlLl9yZXN1bHQgPSByZWFzb247XG5cbiAgICAgIGxpYiRlczYkcHJvbWlzZSRhc2FwJCRhc2FwKGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHB1Ymxpc2hSZWplY3Rpb24sIHByb21pc2UpO1xuICAgIH1cblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHN1YnNjcmliZShwYXJlbnQsIGNoaWxkLCBvbkZ1bGZpbGxtZW50LCBvblJlamVjdGlvbikge1xuICAgICAgdmFyIHN1YnNjcmliZXJzID0gcGFyZW50Ll9zdWJzY3JpYmVycztcbiAgICAgIHZhciBsZW5ndGggPSBzdWJzY3JpYmVycy5sZW5ndGg7XG5cbiAgICAgIHBhcmVudC5fb25lcnJvciA9IG51bGw7XG5cbiAgICAgIHN1YnNjcmliZXJzW2xlbmd0aF0gPSBjaGlsZDtcbiAgICAgIHN1YnNjcmliZXJzW2xlbmd0aCArIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJEZVTEZJTExFRF0gPSBvbkZ1bGZpbGxtZW50O1xuICAgICAgc3Vic2NyaWJlcnNbbGVuZ3RoICsgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUkVKRUNURURdICA9IG9uUmVqZWN0aW9uO1xuXG4gICAgICBpZiAobGVuZ3RoID09PSAwICYmIHBhcmVudC5fc3RhdGUpIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJGFzYXAkJGFzYXAobGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcHVibGlzaCwgcGFyZW50KTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRwdWJsaXNoKHByb21pc2UpIHtcbiAgICAgIHZhciBzdWJzY3JpYmVycyA9IHByb21pc2UuX3N1YnNjcmliZXJzO1xuICAgICAgdmFyIHNldHRsZWQgPSBwcm9taXNlLl9zdGF0ZTtcblxuICAgICAgaWYgKHN1YnNjcmliZXJzLmxlbmd0aCA9PT0gMCkgeyByZXR1cm47IH1cblxuICAgICAgdmFyIGNoaWxkLCBjYWxsYmFjaywgZGV0YWlsID0gcHJvbWlzZS5fcmVzdWx0O1xuXG4gICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHN1YnNjcmliZXJzLmxlbmd0aDsgaSArPSAzKSB7XG4gICAgICAgIGNoaWxkID0gc3Vic2NyaWJlcnNbaV07XG4gICAgICAgIGNhbGxiYWNrID0gc3Vic2NyaWJlcnNbaSArIHNldHRsZWRdO1xuXG4gICAgICAgIGlmIChjaGlsZCkge1xuICAgICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGludm9rZUNhbGxiYWNrKHNldHRsZWQsIGNoaWxkLCBjYWxsYmFjaywgZGV0YWlsKTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICBjYWxsYmFjayhkZXRhaWwpO1xuICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIHByb21pc2UuX3N1YnNjcmliZXJzLmxlbmd0aCA9IDA7XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkRXJyb3JPYmplY3QoKSB7XG4gICAgICB0aGlzLmVycm9yID0gbnVsbDtcbiAgICB9XG5cbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkVFJZX0NBVENIX0VSUk9SID0gbmV3IGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJEVycm9yT2JqZWN0KCk7XG5cbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCR0cnlDYXRjaChjYWxsYmFjaywgZGV0YWlsKSB7XG4gICAgICB0cnkge1xuICAgICAgICByZXR1cm4gY2FsbGJhY2soZGV0YWlsKTtcbiAgICAgIH0gY2F0Y2goZSkge1xuICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRUUllfQ0FUQ0hfRVJST1IuZXJyb3IgPSBlO1xuICAgICAgICByZXR1cm4gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkVFJZX0NBVENIX0VSUk9SO1xuICAgICAgfVxuICAgIH1cblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGludm9rZUNhbGxiYWNrKHNldHRsZWQsIHByb21pc2UsIGNhbGxiYWNrLCBkZXRhaWwpIHtcbiAgICAgIHZhciBoYXNDYWxsYmFjayA9IGxpYiRlczYkcHJvbWlzZSR1dGlscyQkaXNGdW5jdGlvbihjYWxsYmFjayksXG4gICAgICAgICAgdmFsdWUsIGVycm9yLCBzdWNjZWVkZWQsIGZhaWxlZDtcblxuICAgICAgaWYgKGhhc0NhbGxiYWNrKSB7XG4gICAgICAgIHZhbHVlID0gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkdHJ5Q2F0Y2goY2FsbGJhY2ssIGRldGFpbCk7XG5cbiAgICAgICAgaWYgKHZhbHVlID09PSBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRUUllfQ0FUQ0hfRVJST1IpIHtcbiAgICAgICAgICBmYWlsZWQgPSB0cnVlO1xuICAgICAgICAgIGVycm9yID0gdmFsdWUuZXJyb3I7XG4gICAgICAgICAgdmFsdWUgPSBudWxsO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIHN1Y2NlZWRlZCA9IHRydWU7XG4gICAgICAgIH1cblxuICAgICAgICBpZiAocHJvbWlzZSA9PT0gdmFsdWUpIHtcbiAgICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRyZWplY3QocHJvbWlzZSwgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkY2Fubm90UmV0dXJuT3duKCkpO1xuICAgICAgICAgIHJldHVybjtcbiAgICAgICAgfVxuXG4gICAgICB9IGVsc2Uge1xuICAgICAgICB2YWx1ZSA9IGRldGFpbDtcbiAgICAgICAgc3VjY2VlZGVkID0gdHJ1ZTtcbiAgICAgIH1cblxuICAgICAgaWYgKHByb21pc2UuX3N0YXRlICE9PSBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRQRU5ESU5HKSB7XG4gICAgICAgIC8vIG5vb3BcbiAgICAgIH0gZWxzZSBpZiAoaGFzQ2FsbGJhY2sgJiYgc3VjY2VlZGVkKSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHJlc29sdmUocHJvbWlzZSwgdmFsdWUpO1xuICAgICAgfSBlbHNlIGlmIChmYWlsZWQpIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcmVqZWN0KHByb21pc2UsIGVycm9yKTtcbiAgICAgIH0gZWxzZSBpZiAoc2V0dGxlZCA9PT0gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkRlVMRklMTEVEKSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGZ1bGZpbGwocHJvbWlzZSwgdmFsdWUpO1xuICAgICAgfSBlbHNlIGlmIChzZXR0bGVkID09PSBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRSRUpFQ1RFRCkge1xuICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRyZWplY3QocHJvbWlzZSwgdmFsdWUpO1xuICAgICAgfVxuICAgIH1cblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGluaXRpYWxpemVQcm9taXNlKHByb21pc2UsIHJlc29sdmVyKSB7XG4gICAgICB0cnkge1xuICAgICAgICByZXNvbHZlcihmdW5jdGlvbiByZXNvbHZlUHJvbWlzZSh2YWx1ZSl7XG4gICAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcmVzb2x2ZShwcm9taXNlLCB2YWx1ZSk7XG4gICAgICAgIH0sIGZ1bmN0aW9uIHJlamVjdFByb21pc2UocmVhc29uKSB7XG4gICAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcmVqZWN0KHByb21pc2UsIHJlYXNvbik7XG4gICAgICAgIH0pO1xuICAgICAgfSBjYXRjaChlKSB7XG4gICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHJlamVjdChwcm9taXNlLCBlKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkaWQgPSAwO1xuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJG5leHRJZCgpIHtcbiAgICAgIHJldHVybiBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRpZCsrO1xuICAgIH1cblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJG1ha2VQcm9taXNlKHByb21pc2UpIHtcbiAgICAgIHByb21pc2VbbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUFJPTUlTRV9JRF0gPSBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRpZCsrO1xuICAgICAgcHJvbWlzZS5fc3RhdGUgPSB1bmRlZmluZWQ7XG4gICAgICBwcm9taXNlLl9yZXN1bHQgPSB1bmRlZmluZWQ7XG4gICAgICBwcm9taXNlLl9zdWJzY3JpYmVycyA9IFtdO1xuICAgIH1cblxuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJGFsbCQkYWxsKGVudHJpZXMpIHtcbiAgICAgIHJldHVybiBuZXcgbGliJGVzNiRwcm9taXNlJGVudW1lcmF0b3IkJGRlZmF1bHQodGhpcywgZW50cmllcykucHJvbWlzZTtcbiAgICB9XG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJGFsbCQkZGVmYXVsdCA9IGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJGFsbCQkYWxsO1xuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJHJhY2UkJHJhY2UoZW50cmllcykge1xuICAgICAgLypqc2hpbnQgdmFsaWR0aGlzOnRydWUgKi9cbiAgICAgIHZhciBDb25zdHJ1Y3RvciA9IHRoaXM7XG5cbiAgICAgIGlmICghbGliJGVzNiRwcm9taXNlJHV0aWxzJCRpc0FycmF5KGVudHJpZXMpKSB7XG4gICAgICAgIHJldHVybiBuZXcgQ29uc3RydWN0b3IoZnVuY3Rpb24ocmVzb2x2ZSwgcmVqZWN0KSB7XG4gICAgICAgICAgcmVqZWN0KG5ldyBUeXBlRXJyb3IoJ1lvdSBtdXN0IHBhc3MgYW4gYXJyYXkgdG8gcmFjZS4nKSk7XG4gICAgICAgIH0pO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgcmV0dXJuIG5ldyBDb25zdHJ1Y3RvcihmdW5jdGlvbihyZXNvbHZlLCByZWplY3QpIHtcbiAgICAgICAgICB2YXIgbGVuZ3RoID0gZW50cmllcy5sZW5ndGg7XG4gICAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgQ29uc3RydWN0b3IucmVzb2x2ZShlbnRyaWVzW2ldKS50aGVuKHJlc29sdmUsIHJlamVjdCk7XG4gICAgICAgICAgfVxuICAgICAgICB9KTtcbiAgICAgIH1cbiAgICB9XG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJHJhY2UkJGRlZmF1bHQgPSBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSRyYWNlJCRyYWNlO1xuICAgIGZ1bmN0aW9uIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJHJlamVjdCQkcmVqZWN0KHJlYXNvbikge1xuICAgICAgLypqc2hpbnQgdmFsaWR0aGlzOnRydWUgKi9cbiAgICAgIHZhciBDb25zdHJ1Y3RvciA9IHRoaXM7XG4gICAgICB2YXIgcHJvbWlzZSA9IG5ldyBDb25zdHJ1Y3RvcihsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRub29wKTtcbiAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJHJlamVjdChwcm9taXNlLCByZWFzb24pO1xuICAgICAgcmV0dXJuIHByb21pc2U7XG4gICAgfVxuICAgIHZhciBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSRyZWplY3QkJGRlZmF1bHQgPSBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSRyZWplY3QkJHJlamVjdDtcblxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJHByb21pc2UkJG5lZWRzUmVzb2x2ZXIoKSB7XG4gICAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdZb3UgbXVzdCBwYXNzIGEgcmVzb2x2ZXIgZnVuY3Rpb24gYXMgdGhlIGZpcnN0IGFyZ3VtZW50IHRvIHRoZSBwcm9taXNlIGNvbnN0cnVjdG9yJyk7XG4gICAgfVxuXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJHByb21pc2UkJG5lZWRzTmV3KCkge1xuICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkZhaWxlZCB0byBjb25zdHJ1Y3QgJ1Byb21pc2UnOiBQbGVhc2UgdXNlIHRoZSAnbmV3JyBvcGVyYXRvciwgdGhpcyBvYmplY3QgY29uc3RydWN0b3IgY2Fubm90IGJlIGNhbGxlZCBhcyBhIGZ1bmN0aW9uLlwiKTtcbiAgICB9XG5cbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJHByb21pc2UkJGRlZmF1bHQgPSBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSQkUHJvbWlzZTtcbiAgICAvKipcbiAgICAgIFByb21pc2Ugb2JqZWN0cyByZXByZXNlbnQgdGhlIGV2ZW50dWFsIHJlc3VsdCBvZiBhbiBhc3luY2hyb25vdXMgb3BlcmF0aW9uLiBUaGVcbiAgICAgIHByaW1hcnkgd2F5IG9mIGludGVyYWN0aW5nIHdpdGggYSBwcm9taXNlIGlzIHRocm91Z2ggaXRzIGB0aGVuYCBtZXRob2QsIHdoaWNoXG4gICAgICByZWdpc3RlcnMgY2FsbGJhY2tzIHRvIHJlY2VpdmUgZWl0aGVyIGEgcHJvbWlzZSdzIGV2ZW50dWFsIHZhbHVlIG9yIHRoZSByZWFzb25cbiAgICAgIHdoeSB0aGUgcHJvbWlzZSBjYW5ub3QgYmUgZnVsZmlsbGVkLlxuXG4gICAgICBUZXJtaW5vbG9neVxuICAgICAgLS0tLS0tLS0tLS1cblxuICAgICAgLSBgcHJvbWlzZWAgaXMgYW4gb2JqZWN0IG9yIGZ1bmN0aW9uIHdpdGggYSBgdGhlbmAgbWV0aG9kIHdob3NlIGJlaGF2aW9yIGNvbmZvcm1zIHRvIHRoaXMgc3BlY2lmaWNhdGlvbi5cbiAgICAgIC0gYHRoZW5hYmxlYCBpcyBhbiBvYmplY3Qgb3IgZnVuY3Rpb24gdGhhdCBkZWZpbmVzIGEgYHRoZW5gIG1ldGhvZC5cbiAgICAgIC0gYHZhbHVlYCBpcyBhbnkgbGVnYWwgSmF2YVNjcmlwdCB2YWx1ZSAoaW5jbHVkaW5nIHVuZGVmaW5lZCwgYSB0aGVuYWJsZSwgb3IgYSBwcm9taXNlKS5cbiAgICAgIC0gYGV4Y2VwdGlvbmAgaXMgYSB2YWx1ZSB0aGF0IGlzIHRocm93biB1c2luZyB0aGUgdGhyb3cgc3RhdGVtZW50LlxuICAgICAgLSBgcmVhc29uYCBpcyBhIHZhbHVlIHRoYXQgaW5kaWNhdGVzIHdoeSBhIHByb21pc2Ugd2FzIHJlamVjdGVkLlxuICAgICAgLSBgc2V0dGxlZGAgdGhlIGZpbmFsIHJlc3Rpbmcgc3RhdGUgb2YgYSBwcm9taXNlLCBmdWxmaWxsZWQgb3IgcmVqZWN0ZWQuXG5cbiAgICAgIEEgcHJvbWlzZSBjYW4gYmUgaW4gb25lIG9mIHRocmVlIHN0YXRlczogcGVuZGluZywgZnVsZmlsbGVkLCBvciByZWplY3RlZC5cblxuICAgICAgUHJvbWlzZXMgdGhhdCBhcmUgZnVsZmlsbGVkIGhhdmUgYSBmdWxmaWxsbWVudCB2YWx1ZSBhbmQgYXJlIGluIHRoZSBmdWxmaWxsZWRcbiAgICAgIHN0YXRlLiAgUHJvbWlzZXMgdGhhdCBhcmUgcmVqZWN0ZWQgaGF2ZSBhIHJlamVjdGlvbiByZWFzb24gYW5kIGFyZSBpbiB0aGVcbiAgICAgIHJlamVjdGVkIHN0YXRlLiAgQSBmdWxmaWxsbWVudCB2YWx1ZSBpcyBuZXZlciBhIHRoZW5hYmxlLlxuXG4gICAgICBQcm9taXNlcyBjYW4gYWxzbyBiZSBzYWlkIHRvICpyZXNvbHZlKiBhIHZhbHVlLiAgSWYgdGhpcyB2YWx1ZSBpcyBhbHNvIGFcbiAgICAgIHByb21pc2UsIHRoZW4gdGhlIG9yaWdpbmFsIHByb21pc2UncyBzZXR0bGVkIHN0YXRlIHdpbGwgbWF0Y2ggdGhlIHZhbHVlJ3NcbiAgICAgIHNldHRsZWQgc3RhdGUuICBTbyBhIHByb21pc2UgdGhhdCAqcmVzb2x2ZXMqIGEgcHJvbWlzZSB0aGF0IHJlamVjdHMgd2lsbFxuICAgICAgaXRzZWxmIHJlamVjdCwgYW5kIGEgcHJvbWlzZSB0aGF0ICpyZXNvbHZlcyogYSBwcm9taXNlIHRoYXQgZnVsZmlsbHMgd2lsbFxuICAgICAgaXRzZWxmIGZ1bGZpbGwuXG5cblxuICAgICAgQmFzaWMgVXNhZ2U6XG4gICAgICAtLS0tLS0tLS0tLS1cblxuICAgICAgYGBganNcbiAgICAgIHZhciBwcm9taXNlID0gbmV3IFByb21pc2UoZnVuY3Rpb24ocmVzb2x2ZSwgcmVqZWN0KSB7XG4gICAgICAgIC8vIG9uIHN1Y2Nlc3NcbiAgICAgICAgcmVzb2x2ZSh2YWx1ZSk7XG5cbiAgICAgICAgLy8gb24gZmFpbHVyZVxuICAgICAgICByZWplY3QocmVhc29uKTtcbiAgICAgIH0pO1xuXG4gICAgICBwcm9taXNlLnRoZW4oZnVuY3Rpb24odmFsdWUpIHtcbiAgICAgICAgLy8gb24gZnVsZmlsbG1lbnRcbiAgICAgIH0sIGZ1bmN0aW9uKHJlYXNvbikge1xuICAgICAgICAvLyBvbiByZWplY3Rpb25cbiAgICAgIH0pO1xuICAgICAgYGBgXG5cbiAgICAgIEFkdmFuY2VkIFVzYWdlOlxuICAgICAgLS0tLS0tLS0tLS0tLS0tXG5cbiAgICAgIFByb21pc2VzIHNoaW5lIHdoZW4gYWJzdHJhY3RpbmcgYXdheSBhc3luY2hyb25vdXMgaW50ZXJhY3Rpb25zIHN1Y2ggYXNcbiAgICAgIGBYTUxIdHRwUmVxdWVzdGBzLlxuXG4gICAgICBgYGBqc1xuICAgICAgZnVuY3Rpb24gZ2V0SlNPTih1cmwpIHtcbiAgICAgICAgcmV0dXJuIG5ldyBQcm9taXNlKGZ1bmN0aW9uKHJlc29sdmUsIHJlamVjdCl7XG4gICAgICAgICAgdmFyIHhociA9IG5ldyBYTUxIdHRwUmVxdWVzdCgpO1xuXG4gICAgICAgICAgeGhyLm9wZW4oJ0dFVCcsIHVybCk7XG4gICAgICAgICAgeGhyLm9ucmVhZHlzdGF0ZWNoYW5nZSA9IGhhbmRsZXI7XG4gICAgICAgICAgeGhyLnJlc3BvbnNlVHlwZSA9ICdqc29uJztcbiAgICAgICAgICB4aHIuc2V0UmVxdWVzdEhlYWRlcignQWNjZXB0JywgJ2FwcGxpY2F0aW9uL2pzb24nKTtcbiAgICAgICAgICB4aHIuc2VuZCgpO1xuXG4gICAgICAgICAgZnVuY3Rpb24gaGFuZGxlcigpIHtcbiAgICAgICAgICAgIGlmICh0aGlzLnJlYWR5U3RhdGUgPT09IHRoaXMuRE9ORSkge1xuICAgICAgICAgICAgICBpZiAodGhpcy5zdGF0dXMgPT09IDIwMCkge1xuICAgICAgICAgICAgICAgIHJlc29sdmUodGhpcy5yZXNwb25zZSk7XG4gICAgICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICAgICAgcmVqZWN0KG5ldyBFcnJvcignZ2V0SlNPTjogYCcgKyB1cmwgKyAnYCBmYWlsZWQgd2l0aCBzdGF0dXM6IFsnICsgdGhpcy5zdGF0dXMgKyAnXScpKTtcbiAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuICAgICAgICAgIH07XG4gICAgICAgIH0pO1xuICAgICAgfVxuXG4gICAgICBnZXRKU09OKCcvcG9zdHMuanNvbicpLnRoZW4oZnVuY3Rpb24oanNvbikge1xuICAgICAgICAvLyBvbiBmdWxmaWxsbWVudFxuICAgICAgfSwgZnVuY3Rpb24ocmVhc29uKSB7XG4gICAgICAgIC8vIG9uIHJlamVjdGlvblxuICAgICAgfSk7XG4gICAgICBgYGBcblxuICAgICAgVW5saWtlIGNhbGxiYWNrcywgcHJvbWlzZXMgYXJlIGdyZWF0IGNvbXBvc2FibGUgcHJpbWl0aXZlcy5cblxuICAgICAgYGBganNcbiAgICAgIFByb21pc2UuYWxsKFtcbiAgICAgICAgZ2V0SlNPTignL3Bvc3RzJyksXG4gICAgICAgIGdldEpTT04oJy9jb21tZW50cycpXG4gICAgICBdKS50aGVuKGZ1bmN0aW9uKHZhbHVlcyl7XG4gICAgICAgIHZhbHVlc1swXSAvLyA9PiBwb3N0c0pTT05cbiAgICAgICAgdmFsdWVzWzFdIC8vID0+IGNvbW1lbnRzSlNPTlxuXG4gICAgICAgIHJldHVybiB2YWx1ZXM7XG4gICAgICB9KTtcbiAgICAgIGBgYFxuXG4gICAgICBAY2xhc3MgUHJvbWlzZVxuICAgICAgQHBhcmFtIHtmdW5jdGlvbn0gcmVzb2x2ZXJcbiAgICAgIFVzZWZ1bCBmb3IgdG9vbGluZy5cbiAgICAgIEBjb25zdHJ1Y3RvclxuICAgICovXG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJHByb21pc2UkJFByb21pc2UocmVzb2x2ZXIpIHtcbiAgICAgIHRoaXNbbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUFJPTUlTRV9JRF0gPSBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRuZXh0SWQoKTtcbiAgICAgIHRoaXMuX3Jlc3VsdCA9IHRoaXMuX3N0YXRlID0gdW5kZWZpbmVkO1xuICAgICAgdGhpcy5fc3Vic2NyaWJlcnMgPSBbXTtcblxuICAgICAgaWYgKGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJG5vb3AgIT09IHJlc29sdmVyKSB7XG4gICAgICAgIHR5cGVvZiByZXNvbHZlciAhPT0gJ2Z1bmN0aW9uJyAmJiBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSQkbmVlZHNSZXNvbHZlcigpO1xuICAgICAgICB0aGlzIGluc3RhbmNlb2YgbGliJGVzNiRwcm9taXNlJHByb21pc2UkJFByb21pc2UgPyBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRpbml0aWFsaXplUHJvbWlzZSh0aGlzLCByZXNvbHZlcikgOiBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSQkbmVlZHNOZXcoKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSQkUHJvbWlzZS5hbGwgPSBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSRhbGwkJGRlZmF1bHQ7XG4gICAgbGliJGVzNiRwcm9taXNlJHByb21pc2UkJFByb21pc2UucmFjZSA9IGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJHJhY2UkJGRlZmF1bHQ7XG4gICAgbGliJGVzNiRwcm9taXNlJHByb21pc2UkJFByb21pc2UucmVzb2x2ZSA9IGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJHJlc29sdmUkJGRlZmF1bHQ7XG4gICAgbGliJGVzNiRwcm9taXNlJHByb21pc2UkJFByb21pc2UucmVqZWN0ID0gbGliJGVzNiRwcm9taXNlJHByb21pc2UkcmVqZWN0JCRkZWZhdWx0O1xuICAgIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJCRQcm9taXNlLl9zZXRTY2hlZHVsZXIgPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkc2V0U2NoZWR1bGVyO1xuICAgIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJCRQcm9taXNlLl9zZXRBc2FwID0gbGliJGVzNiRwcm9taXNlJGFzYXAkJHNldEFzYXA7XG4gICAgbGliJGVzNiRwcm9taXNlJHByb21pc2UkJFByb21pc2UuX2FzYXAgPSBsaWIkZXM2JHByb21pc2UkYXNhcCQkYXNhcDtcblxuICAgIGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJCRQcm9taXNlLnByb3RvdHlwZSA9IHtcbiAgICAgIGNvbnN0cnVjdG9yOiBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSQkUHJvbWlzZSxcblxuICAgIC8qKlxuICAgICAgVGhlIHByaW1hcnkgd2F5IG9mIGludGVyYWN0aW5nIHdpdGggYSBwcm9taXNlIGlzIHRocm91Z2ggaXRzIGB0aGVuYCBtZXRob2QsXG4gICAgICB3aGljaCByZWdpc3RlcnMgY2FsbGJhY2tzIHRvIHJlY2VpdmUgZWl0aGVyIGEgcHJvbWlzZSdzIGV2ZW50dWFsIHZhbHVlIG9yIHRoZVxuICAgICAgcmVhc29uIHdoeSB0aGUgcHJvbWlzZSBjYW5ub3QgYmUgZnVsZmlsbGVkLlxuXG4gICAgICBgYGBqc1xuICAgICAgZmluZFVzZXIoKS50aGVuKGZ1bmN0aW9uKHVzZXIpe1xuICAgICAgICAvLyB1c2VyIGlzIGF2YWlsYWJsZVxuICAgICAgfSwgZnVuY3Rpb24ocmVhc29uKXtcbiAgICAgICAgLy8gdXNlciBpcyB1bmF2YWlsYWJsZSwgYW5kIHlvdSBhcmUgZ2l2ZW4gdGhlIHJlYXNvbiB3aHlcbiAgICAgIH0pO1xuICAgICAgYGBgXG5cbiAgICAgIENoYWluaW5nXG4gICAgICAtLS0tLS0tLVxuXG4gICAgICBUaGUgcmV0dXJuIHZhbHVlIG9mIGB0aGVuYCBpcyBpdHNlbGYgYSBwcm9taXNlLiAgVGhpcyBzZWNvbmQsICdkb3duc3RyZWFtJ1xuICAgICAgcHJvbWlzZSBpcyByZXNvbHZlZCB3aXRoIHRoZSByZXR1cm4gdmFsdWUgb2YgdGhlIGZpcnN0IHByb21pc2UncyBmdWxmaWxsbWVudFxuICAgICAgb3IgcmVqZWN0aW9uIGhhbmRsZXIsIG9yIHJlamVjdGVkIGlmIHRoZSBoYW5kbGVyIHRocm93cyBhbiBleGNlcHRpb24uXG5cbiAgICAgIGBgYGpzXG4gICAgICBmaW5kVXNlcigpLnRoZW4oZnVuY3Rpb24gKHVzZXIpIHtcbiAgICAgICAgcmV0dXJuIHVzZXIubmFtZTtcbiAgICAgIH0sIGZ1bmN0aW9uIChyZWFzb24pIHtcbiAgICAgICAgcmV0dXJuICdkZWZhdWx0IG5hbWUnO1xuICAgICAgfSkudGhlbihmdW5jdGlvbiAodXNlck5hbWUpIHtcbiAgICAgICAgLy8gSWYgYGZpbmRVc2VyYCBmdWxmaWxsZWQsIGB1c2VyTmFtZWAgd2lsbCBiZSB0aGUgdXNlcidzIG5hbWUsIG90aGVyd2lzZSBpdFxuICAgICAgICAvLyB3aWxsIGJlIGAnZGVmYXVsdCBuYW1lJ2BcbiAgICAgIH0pO1xuXG4gICAgICBmaW5kVXNlcigpLnRoZW4oZnVuY3Rpb24gKHVzZXIpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKCdGb3VuZCB1c2VyLCBidXQgc3RpbGwgdW5oYXBweScpO1xuICAgICAgfSwgZnVuY3Rpb24gKHJlYXNvbikge1xuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoJ2BmaW5kVXNlcmAgcmVqZWN0ZWQgYW5kIHdlJ3JlIHVuaGFwcHknKTtcbiAgICAgIH0pLnRoZW4oZnVuY3Rpb24gKHZhbHVlKSB7XG4gICAgICAgIC8vIG5ldmVyIHJlYWNoZWRcbiAgICAgIH0sIGZ1bmN0aW9uIChyZWFzb24pIHtcbiAgICAgICAgLy8gaWYgYGZpbmRVc2VyYCBmdWxmaWxsZWQsIGByZWFzb25gIHdpbGwgYmUgJ0ZvdW5kIHVzZXIsIGJ1dCBzdGlsbCB1bmhhcHB5Jy5cbiAgICAgICAgLy8gSWYgYGZpbmRVc2VyYCByZWplY3RlZCwgYHJlYXNvbmAgd2lsbCBiZSAnYGZpbmRVc2VyYCByZWplY3RlZCBhbmQgd2UncmUgdW5oYXBweScuXG4gICAgICB9KTtcbiAgICAgIGBgYFxuICAgICAgSWYgdGhlIGRvd25zdHJlYW0gcHJvbWlzZSBkb2VzIG5vdCBzcGVjaWZ5IGEgcmVqZWN0aW9uIGhhbmRsZXIsIHJlamVjdGlvbiByZWFzb25zIHdpbGwgYmUgcHJvcGFnYXRlZCBmdXJ0aGVyIGRvd25zdHJlYW0uXG5cbiAgICAgIGBgYGpzXG4gICAgICBmaW5kVXNlcigpLnRoZW4oZnVuY3Rpb24gKHVzZXIpIHtcbiAgICAgICAgdGhyb3cgbmV3IFBlZGFnb2dpY2FsRXhjZXB0aW9uKCdVcHN0cmVhbSBlcnJvcicpO1xuICAgICAgfSkudGhlbihmdW5jdGlvbiAodmFsdWUpIHtcbiAgICAgICAgLy8gbmV2ZXIgcmVhY2hlZFxuICAgICAgfSkudGhlbihmdW5jdGlvbiAodmFsdWUpIHtcbiAgICAgICAgLy8gbmV2ZXIgcmVhY2hlZFxuICAgICAgfSwgZnVuY3Rpb24gKHJlYXNvbikge1xuICAgICAgICAvLyBUaGUgYFBlZGdhZ29jaWFsRXhjZXB0aW9uYCBpcyBwcm9wYWdhdGVkIGFsbCB0aGUgd2F5IGRvd24gdG8gaGVyZVxuICAgICAgfSk7XG4gICAgICBgYGBcblxuICAgICAgQXNzaW1pbGF0aW9uXG4gICAgICAtLS0tLS0tLS0tLS1cblxuICAgICAgU29tZXRpbWVzIHRoZSB2YWx1ZSB5b3Ugd2FudCB0byBwcm9wYWdhdGUgdG8gYSBkb3duc3RyZWFtIHByb21pc2UgY2FuIG9ubHkgYmVcbiAgICAgIHJldHJpZXZlZCBhc3luY2hyb25vdXNseS4gVGhpcyBjYW4gYmUgYWNoaWV2ZWQgYnkgcmV0dXJuaW5nIGEgcHJvbWlzZSBpbiB0aGVcbiAgICAgIGZ1bGZpbGxtZW50IG9yIHJlamVjdGlvbiBoYW5kbGVyLiBUaGUgZG93bnN0cmVhbSBwcm9taXNlIHdpbGwgdGhlbiBiZSBwZW5kaW5nXG4gICAgICB1bnRpbCB0aGUgcmV0dXJuZWQgcHJvbWlzZSBpcyBzZXR0bGVkLiBUaGlzIGlzIGNhbGxlZCAqYXNzaW1pbGF0aW9uKi5cblxuICAgICAgYGBganNcbiAgICAgIGZpbmRVc2VyKCkudGhlbihmdW5jdGlvbiAodXNlcikge1xuICAgICAgICByZXR1cm4gZmluZENvbW1lbnRzQnlBdXRob3IodXNlcik7XG4gICAgICB9KS50aGVuKGZ1bmN0aW9uIChjb21tZW50cykge1xuICAgICAgICAvLyBUaGUgdXNlcidzIGNvbW1lbnRzIGFyZSBub3cgYXZhaWxhYmxlXG4gICAgICB9KTtcbiAgICAgIGBgYFxuXG4gICAgICBJZiB0aGUgYXNzaW1saWF0ZWQgcHJvbWlzZSByZWplY3RzLCB0aGVuIHRoZSBkb3duc3RyZWFtIHByb21pc2Ugd2lsbCBhbHNvIHJlamVjdC5cblxuICAgICAgYGBganNcbiAgICAgIGZpbmRVc2VyKCkudGhlbihmdW5jdGlvbiAodXNlcikge1xuICAgICAgICByZXR1cm4gZmluZENvbW1lbnRzQnlBdXRob3IodXNlcik7XG4gICAgICB9KS50aGVuKGZ1bmN0aW9uIChjb21tZW50cykge1xuICAgICAgICAvLyBJZiBgZmluZENvbW1lbnRzQnlBdXRob3JgIGZ1bGZpbGxzLCB3ZSdsbCBoYXZlIHRoZSB2YWx1ZSBoZXJlXG4gICAgICB9LCBmdW5jdGlvbiAocmVhc29uKSB7XG4gICAgICAgIC8vIElmIGBmaW5kQ29tbWVudHNCeUF1dGhvcmAgcmVqZWN0cywgd2UnbGwgaGF2ZSB0aGUgcmVhc29uIGhlcmVcbiAgICAgIH0pO1xuICAgICAgYGBgXG5cbiAgICAgIFNpbXBsZSBFeGFtcGxlXG4gICAgICAtLS0tLS0tLS0tLS0tLVxuXG4gICAgICBTeW5jaHJvbm91cyBFeGFtcGxlXG5cbiAgICAgIGBgYGphdmFzY3JpcHRcbiAgICAgIHZhciByZXN1bHQ7XG5cbiAgICAgIHRyeSB7XG4gICAgICAgIHJlc3VsdCA9IGZpbmRSZXN1bHQoKTtcbiAgICAgICAgLy8gc3VjY2Vzc1xuICAgICAgfSBjYXRjaChyZWFzb24pIHtcbiAgICAgICAgLy8gZmFpbHVyZVxuICAgICAgfVxuICAgICAgYGBgXG5cbiAgICAgIEVycmJhY2sgRXhhbXBsZVxuXG4gICAgICBgYGBqc1xuICAgICAgZmluZFJlc3VsdChmdW5jdGlvbihyZXN1bHQsIGVycil7XG4gICAgICAgIGlmIChlcnIpIHtcbiAgICAgICAgICAvLyBmYWlsdXJlXG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgLy8gc3VjY2Vzc1xuICAgICAgICB9XG4gICAgICB9KTtcbiAgICAgIGBgYFxuXG4gICAgICBQcm9taXNlIEV4YW1wbGU7XG5cbiAgICAgIGBgYGphdmFzY3JpcHRcbiAgICAgIGZpbmRSZXN1bHQoKS50aGVuKGZ1bmN0aW9uKHJlc3VsdCl7XG4gICAgICAgIC8vIHN1Y2Nlc3NcbiAgICAgIH0sIGZ1bmN0aW9uKHJlYXNvbil7XG4gICAgICAgIC8vIGZhaWx1cmVcbiAgICAgIH0pO1xuICAgICAgYGBgXG5cbiAgICAgIEFkdmFuY2VkIEV4YW1wbGVcbiAgICAgIC0tLS0tLS0tLS0tLS0tXG5cbiAgICAgIFN5bmNocm9ub3VzIEV4YW1wbGVcblxuICAgICAgYGBgamF2YXNjcmlwdFxuICAgICAgdmFyIGF1dGhvciwgYm9va3M7XG5cbiAgICAgIHRyeSB7XG4gICAgICAgIGF1dGhvciA9IGZpbmRBdXRob3IoKTtcbiAgICAgICAgYm9va3MgID0gZmluZEJvb2tzQnlBdXRob3IoYXV0aG9yKTtcbiAgICAgICAgLy8gc3VjY2Vzc1xuICAgICAgfSBjYXRjaChyZWFzb24pIHtcbiAgICAgICAgLy8gZmFpbHVyZVxuICAgICAgfVxuICAgICAgYGBgXG5cbiAgICAgIEVycmJhY2sgRXhhbXBsZVxuXG4gICAgICBgYGBqc1xuXG4gICAgICBmdW5jdGlvbiBmb3VuZEJvb2tzKGJvb2tzKSB7XG5cbiAgICAgIH1cblxuICAgICAgZnVuY3Rpb24gZmFpbHVyZShyZWFzb24pIHtcblxuICAgICAgfVxuXG4gICAgICBmaW5kQXV0aG9yKGZ1bmN0aW9uKGF1dGhvciwgZXJyKXtcbiAgICAgICAgaWYgKGVycikge1xuICAgICAgICAgIGZhaWx1cmUoZXJyKTtcbiAgICAgICAgICAvLyBmYWlsdXJlXG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgdHJ5IHtcbiAgICAgICAgICAgIGZpbmRCb29va3NCeUF1dGhvcihhdXRob3IsIGZ1bmN0aW9uKGJvb2tzLCBlcnIpIHtcbiAgICAgICAgICAgICAgaWYgKGVycikge1xuICAgICAgICAgICAgICAgIGZhaWx1cmUoZXJyKTtcbiAgICAgICAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgICAgICB0cnkge1xuICAgICAgICAgICAgICAgICAgZm91bmRCb29rcyhib29rcyk7XG4gICAgICAgICAgICAgICAgfSBjYXRjaChyZWFzb24pIHtcbiAgICAgICAgICAgICAgICAgIGZhaWx1cmUocmVhc29uKTtcbiAgICAgICAgICAgICAgICB9XG4gICAgICAgICAgICAgIH1cbiAgICAgICAgICAgIH0pO1xuICAgICAgICAgIH0gY2F0Y2goZXJyb3IpIHtcbiAgICAgICAgICAgIGZhaWx1cmUoZXJyKTtcbiAgICAgICAgICB9XG4gICAgICAgICAgLy8gc3VjY2Vzc1xuICAgICAgICB9XG4gICAgICB9KTtcbiAgICAgIGBgYFxuXG4gICAgICBQcm9taXNlIEV4YW1wbGU7XG5cbiAgICAgIGBgYGphdmFzY3JpcHRcbiAgICAgIGZpbmRBdXRob3IoKS5cbiAgICAgICAgdGhlbihmaW5kQm9va3NCeUF1dGhvcikuXG4gICAgICAgIHRoZW4oZnVuY3Rpb24oYm9va3Mpe1xuICAgICAgICAgIC8vIGZvdW5kIGJvb2tzXG4gICAgICB9KS5jYXRjaChmdW5jdGlvbihyZWFzb24pe1xuICAgICAgICAvLyBzb21ldGhpbmcgd2VudCB3cm9uZ1xuICAgICAgfSk7XG4gICAgICBgYGBcblxuICAgICAgQG1ldGhvZCB0aGVuXG4gICAgICBAcGFyYW0ge0Z1bmN0aW9ufSBvbkZ1bGZpbGxlZFxuICAgICAgQHBhcmFtIHtGdW5jdGlvbn0gb25SZWplY3RlZFxuICAgICAgVXNlZnVsIGZvciB0b29saW5nLlxuICAgICAgQHJldHVybiB7UHJvbWlzZX1cbiAgICAqL1xuICAgICAgdGhlbjogbGliJGVzNiRwcm9taXNlJHRoZW4kJGRlZmF1bHQsXG5cbiAgICAvKipcbiAgICAgIGBjYXRjaGAgaXMgc2ltcGx5IHN1Z2FyIGZvciBgdGhlbih1bmRlZmluZWQsIG9uUmVqZWN0aW9uKWAgd2hpY2ggbWFrZXMgaXQgdGhlIHNhbWVcbiAgICAgIGFzIHRoZSBjYXRjaCBibG9jayBvZiBhIHRyeS9jYXRjaCBzdGF0ZW1lbnQuXG5cbiAgICAgIGBgYGpzXG4gICAgICBmdW5jdGlvbiBmaW5kQXV0aG9yKCl7XG4gICAgICAgIHRocm93IG5ldyBFcnJvcignY291bGRuJ3QgZmluZCB0aGF0IGF1dGhvcicpO1xuICAgICAgfVxuXG4gICAgICAvLyBzeW5jaHJvbm91c1xuICAgICAgdHJ5IHtcbiAgICAgICAgZmluZEF1dGhvcigpO1xuICAgICAgfSBjYXRjaChyZWFzb24pIHtcbiAgICAgICAgLy8gc29tZXRoaW5nIHdlbnQgd3JvbmdcbiAgICAgIH1cblxuICAgICAgLy8gYXN5bmMgd2l0aCBwcm9taXNlc1xuICAgICAgZmluZEF1dGhvcigpLmNhdGNoKGZ1bmN0aW9uKHJlYXNvbil7XG4gICAgICAgIC8vIHNvbWV0aGluZyB3ZW50IHdyb25nXG4gICAgICB9KTtcbiAgICAgIGBgYFxuXG4gICAgICBAbWV0aG9kIGNhdGNoXG4gICAgICBAcGFyYW0ge0Z1bmN0aW9ufSBvblJlamVjdGlvblxuICAgICAgVXNlZnVsIGZvciB0b29saW5nLlxuICAgICAgQHJldHVybiB7UHJvbWlzZX1cbiAgICAqL1xuICAgICAgJ2NhdGNoJzogZnVuY3Rpb24ob25SZWplY3Rpb24pIHtcbiAgICAgICAgcmV0dXJuIHRoaXMudGhlbihudWxsLCBvblJlamVjdGlvbik7XG4gICAgICB9XG4gICAgfTtcbiAgICB2YXIgbGliJGVzNiRwcm9taXNlJGVudW1lcmF0b3IkJGRlZmF1bHQgPSBsaWIkZXM2JHByb21pc2UkZW51bWVyYXRvciQkRW51bWVyYXRvcjtcbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkZW51bWVyYXRvciQkRW51bWVyYXRvcihDb25zdHJ1Y3RvciwgaW5wdXQpIHtcbiAgICAgIHRoaXMuX2luc3RhbmNlQ29uc3RydWN0b3IgPSBDb25zdHJ1Y3RvcjtcbiAgICAgIHRoaXMucHJvbWlzZSA9IG5ldyBDb25zdHJ1Y3RvcihsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRub29wKTtcblxuICAgICAgaWYgKCF0aGlzLnByb21pc2VbbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUFJPTUlTRV9JRF0pIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkbWFrZVByb21pc2UodGhpcy5wcm9taXNlKTtcbiAgICAgIH1cblxuICAgICAgaWYgKGxpYiRlczYkcHJvbWlzZSR1dGlscyQkaXNBcnJheShpbnB1dCkpIHtcbiAgICAgICAgdGhpcy5faW5wdXQgICAgID0gaW5wdXQ7XG4gICAgICAgIHRoaXMubGVuZ3RoICAgICA9IGlucHV0Lmxlbmd0aDtcbiAgICAgICAgdGhpcy5fcmVtYWluaW5nID0gaW5wdXQubGVuZ3RoO1xuXG4gICAgICAgIHRoaXMuX3Jlc3VsdCA9IG5ldyBBcnJheSh0aGlzLmxlbmd0aCk7XG5cbiAgICAgICAgaWYgKHRoaXMubGVuZ3RoID09PSAwKSB7XG4gICAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkZnVsZmlsbCh0aGlzLnByb21pc2UsIHRoaXMuX3Jlc3VsdCk7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgdGhpcy5sZW5ndGggPSB0aGlzLmxlbmd0aCB8fCAwO1xuICAgICAgICAgIHRoaXMuX2VudW1lcmF0ZSgpO1xuICAgICAgICAgIGlmICh0aGlzLl9yZW1haW5pbmcgPT09IDApIHtcbiAgICAgICAgICAgIGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGZ1bGZpbGwodGhpcy5wcm9taXNlLCB0aGlzLl9yZXN1bHQpO1xuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgfSBlbHNlIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkcmVqZWN0KHRoaXMucHJvbWlzZSwgbGliJGVzNiRwcm9taXNlJGVudW1lcmF0b3IkJHZhbGlkYXRpb25FcnJvcigpKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICBmdW5jdGlvbiBsaWIkZXM2JHByb21pc2UkZW51bWVyYXRvciQkdmFsaWRhdGlvbkVycm9yKCkge1xuICAgICAgcmV0dXJuIG5ldyBFcnJvcignQXJyYXkgTWV0aG9kcyBtdXN0IGJlIHByb3ZpZGVkIGFuIEFycmF5Jyk7XG4gICAgfVxuXG4gICAgbGliJGVzNiRwcm9taXNlJGVudW1lcmF0b3IkJEVudW1lcmF0b3IucHJvdG90eXBlLl9lbnVtZXJhdGUgPSBmdW5jdGlvbigpIHtcbiAgICAgIHZhciBsZW5ndGggID0gdGhpcy5sZW5ndGg7XG4gICAgICB2YXIgaW5wdXQgICA9IHRoaXMuX2lucHV0O1xuXG4gICAgICBmb3IgKHZhciBpID0gMDsgdGhpcy5fc3RhdGUgPT09IGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJFBFTkRJTkcgJiYgaSA8IGxlbmd0aDsgaSsrKSB7XG4gICAgICAgIHRoaXMuX2VhY2hFbnRyeShpbnB1dFtpXSwgaSk7XG4gICAgICB9XG4gICAgfTtcblxuICAgIGxpYiRlczYkcHJvbWlzZSRlbnVtZXJhdG9yJCRFbnVtZXJhdG9yLnByb3RvdHlwZS5fZWFjaEVudHJ5ID0gZnVuY3Rpb24oZW50cnksIGkpIHtcbiAgICAgIHZhciBjID0gdGhpcy5faW5zdGFuY2VDb25zdHJ1Y3RvcjtcbiAgICAgIHZhciByZXNvbHZlID0gYy5yZXNvbHZlO1xuXG4gICAgICBpZiAocmVzb2x2ZSA9PT0gbGliJGVzNiRwcm9taXNlJHByb21pc2UkcmVzb2x2ZSQkZGVmYXVsdCkge1xuICAgICAgICB2YXIgdGhlbiA9IGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJGdldFRoZW4oZW50cnkpO1xuXG4gICAgICAgIGlmICh0aGVuID09PSBsaWIkZXM2JHByb21pc2UkdGhlbiQkZGVmYXVsdCAmJlxuICAgICAgICAgICAgZW50cnkuX3N0YXRlICE9PSBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRQRU5ESU5HKSB7XG4gICAgICAgICAgdGhpcy5fc2V0dGxlZEF0KGVudHJ5Ll9zdGF0ZSwgaSwgZW50cnkuX3Jlc3VsdCk7XG4gICAgICAgIH0gZWxzZSBpZiAodHlwZW9mIHRoZW4gIT09ICdmdW5jdGlvbicpIHtcbiAgICAgICAgICB0aGlzLl9yZW1haW5pbmctLTtcbiAgICAgICAgICB0aGlzLl9yZXN1bHRbaV0gPSBlbnRyeTtcbiAgICAgICAgfSBlbHNlIGlmIChjID09PSBsaWIkZXM2JHByb21pc2UkcHJvbWlzZSQkZGVmYXVsdCkge1xuICAgICAgICAgIHZhciBwcm9taXNlID0gbmV3IGMobGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkbm9vcCk7XG4gICAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkaGFuZGxlTWF5YmVUaGVuYWJsZShwcm9taXNlLCBlbnRyeSwgdGhlbik7XG4gICAgICAgICAgdGhpcy5fd2lsbFNldHRsZUF0KHByb21pc2UsIGkpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIHRoaXMuX3dpbGxTZXR0bGVBdChuZXcgYyhmdW5jdGlvbihyZXNvbHZlKSB7IHJlc29sdmUoZW50cnkpOyB9KSwgaSk7XG4gICAgICAgIH1cbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHRoaXMuX3dpbGxTZXR0bGVBdChyZXNvbHZlKGVudHJ5KSwgaSk7XG4gICAgICB9XG4gICAgfTtcblxuICAgIGxpYiRlczYkcHJvbWlzZSRlbnVtZXJhdG9yJCRFbnVtZXJhdG9yLnByb3RvdHlwZS5fc2V0dGxlZEF0ID0gZnVuY3Rpb24oc3RhdGUsIGksIHZhbHVlKSB7XG4gICAgICB2YXIgcHJvbWlzZSA9IHRoaXMucHJvbWlzZTtcblxuICAgICAgaWYgKHByb21pc2UuX3N0YXRlID09PSBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRQRU5ESU5HKSB7XG4gICAgICAgIHRoaXMuX3JlbWFpbmluZy0tO1xuXG4gICAgICAgIGlmIChzdGF0ZSA9PT0gbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkUkVKRUNURUQpIHtcbiAgICAgICAgICBsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRyZWplY3QocHJvbWlzZSwgdmFsdWUpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIHRoaXMuX3Jlc3VsdFtpXSA9IHZhbHVlO1xuICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIGlmICh0aGlzLl9yZW1haW5pbmcgPT09IDApIHtcbiAgICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkZnVsZmlsbChwcm9taXNlLCB0aGlzLl9yZXN1bHQpO1xuICAgICAgfVxuICAgIH07XG5cbiAgICBsaWIkZXM2JHByb21pc2UkZW51bWVyYXRvciQkRW51bWVyYXRvci5wcm90b3R5cGUuX3dpbGxTZXR0bGVBdCA9IGZ1bmN0aW9uKHByb21pc2UsIGkpIHtcbiAgICAgIHZhciBlbnVtZXJhdG9yID0gdGhpcztcblxuICAgICAgbGliJGVzNiRwcm9taXNlJCRpbnRlcm5hbCQkc3Vic2NyaWJlKHByb21pc2UsIHVuZGVmaW5lZCwgZnVuY3Rpb24odmFsdWUpIHtcbiAgICAgICAgZW51bWVyYXRvci5fc2V0dGxlZEF0KGxpYiRlczYkcHJvbWlzZSQkaW50ZXJuYWwkJEZVTEZJTExFRCwgaSwgdmFsdWUpO1xuICAgICAgfSwgZnVuY3Rpb24ocmVhc29uKSB7XG4gICAgICAgIGVudW1lcmF0b3IuX3NldHRsZWRBdChsaWIkZXM2JHByb21pc2UkJGludGVybmFsJCRSRUpFQ1RFRCwgaSwgcmVhc29uKTtcbiAgICAgIH0pO1xuICAgIH07XG4gICAgZnVuY3Rpb24gbGliJGVzNiRwcm9taXNlJHBvbHlmaWxsJCRwb2x5ZmlsbCgpIHtcbiAgICAgIHZhciBsb2NhbDtcblxuICAgICAgaWYgKHR5cGVvZiBnbG9iYWwgIT09ICd1bmRlZmluZWQnKSB7XG4gICAgICAgICAgbG9jYWwgPSBnbG9iYWw7XG4gICAgICB9IGVsc2UgaWYgKHR5cGVvZiBzZWxmICE9PSAndW5kZWZpbmVkJykge1xuICAgICAgICAgIGxvY2FsID0gc2VsZjtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgdHJ5IHtcbiAgICAgICAgICAgICAgbG9jYWwgPSBGdW5jdGlvbigncmV0dXJuIHRoaXMnKSgpO1xuICAgICAgICAgIH0gY2F0Y2ggKGUpIHtcbiAgICAgICAgICAgICAgdGhyb3cgbmV3IEVycm9yKCdwb2x5ZmlsbCBmYWlsZWQgYmVjYXVzZSBnbG9iYWwgb2JqZWN0IGlzIHVuYXZhaWxhYmxlIGluIHRoaXMgZW52aXJvbm1lbnQnKTtcbiAgICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIHZhciBQID0gbG9jYWwuUHJvbWlzZTtcblxuICAgICAgaWYgKFAgJiYgT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKFAucmVzb2x2ZSgpKSA9PT0gJ1tvYmplY3QgUHJvbWlzZV0nICYmICFQLmNhc3QpIHtcbiAgICAgICAgcmV0dXJuO1xuICAgICAgfVxuXG4gICAgICBsb2NhbC5Qcm9taXNlID0gbGliJGVzNiRwcm9taXNlJHByb21pc2UkJGRlZmF1bHQ7XG4gICAgfVxuICAgIHZhciBsaWIkZXM2JHByb21pc2UkcG9seWZpbGwkJGRlZmF1bHQgPSBsaWIkZXM2JHByb21pc2UkcG9seWZpbGwkJHBvbHlmaWxsO1xuXG4gICAgdmFyIGxpYiRlczYkcHJvbWlzZSR1bWQkJEVTNlByb21pc2UgPSB7XG4gICAgICAnUHJvbWlzZSc6IGxpYiRlczYkcHJvbWlzZSRwcm9taXNlJCRkZWZhdWx0LFxuICAgICAgJ3BvbHlmaWxsJzogbGliJGVzNiRwcm9taXNlJHBvbHlmaWxsJCRkZWZhdWx0XG4gICAgfTtcblxuICAgIC8qIGdsb2JhbCBkZWZpbmU6dHJ1ZSBtb2R1bGU6dHJ1ZSB3aW5kb3c6IHRydWUgKi9cbiAgICBpZiAodHlwZW9mIGRlZmluZSA9PT0gJ2Z1bmN0aW9uJyAmJiBkZWZpbmVbJ2FtZCddKSB7XG4gICAgICBkZWZpbmUoZnVuY3Rpb24oKSB7IHJldHVybiBsaWIkZXM2JHByb21pc2UkdW1kJCRFUzZQcm9taXNlOyB9KTtcbiAgICB9IGVsc2UgaWYgKHR5cGVvZiBtb2R1bGUgIT09ICd1bmRlZmluZWQnICYmIG1vZHVsZVsnZXhwb3J0cyddKSB7XG4gICAgICBtb2R1bGVbJ2V4cG9ydHMnXSA9IGxpYiRlczYkcHJvbWlzZSR1bWQkJEVTNlByb21pc2U7XG4gICAgfSBlbHNlIGlmICh0eXBlb2YgdGhpcyAhPT0gJ3VuZGVmaW5lZCcpIHtcbiAgICAgIHRoaXNbJ0VTNlByb21pc2UnXSA9IGxpYiRlczYkcHJvbWlzZSR1bWQkJEVTNlByb21pc2U7XG4gICAgfVxuXG4gICAgbGliJGVzNiRwcm9taXNlJHBvbHlmaWxsJCRkZWZhdWx0KCk7XG59KS5jYWxsKHRoaXMpO1xuXG4iLCIvKiBDb3B5cmlnaHQgKGMpIDIwMTMsIEJyYW5kb24gSm9uZXMsIENvbGluIE1hY0tlbnppZSBJVi4gQWxsIHJpZ2h0cyByZXNlcnZlZC5cblxuUmVkaXN0cmlidXRpb24gYW5kIHVzZSBpbiBzb3VyY2UgYW5kIGJpbmFyeSBmb3Jtcywgd2l0aCBvciB3aXRob3V0IG1vZGlmaWNhdGlvbixcbmFyZSBwZXJtaXR0ZWQgcHJvdmlkZWQgdGhhdCB0aGUgZm9sbG93aW5nIGNvbmRpdGlvbnMgYXJlIG1ldDpcblxuICAqIFJlZGlzdHJpYnV0aW9ucyBvZiBzb3VyY2UgY29kZSBtdXN0IHJldGFpbiB0aGUgYWJvdmUgY29weXJpZ2h0IG5vdGljZSwgdGhpc1xuICAgIGxpc3Qgb2YgY29uZGl0aW9ucyBhbmQgdGhlIGZvbGxvd2luZyBkaXNjbGFpbWVyLlxuICAqIFJlZGlzdHJpYnV0aW9ucyBpbiBiaW5hcnkgZm9ybSBtdXN0IHJlcHJvZHVjZSB0aGUgYWJvdmUgY29weXJpZ2h0IG5vdGljZSxcbiAgICB0aGlzIGxpc3Qgb2YgY29uZGl0aW9ucyBhbmQgdGhlIGZvbGxvd2luZyBkaXNjbGFpbWVyIGluIHRoZSBkb2N1bWVudGF0aW9uIFxuICAgIGFuZC9vciBvdGhlciBtYXRlcmlhbHMgcHJvdmlkZWQgd2l0aCB0aGUgZGlzdHJpYnV0aW9uLlxuXG5USElTIFNPRlRXQVJFIElTIFBST1ZJREVEIEJZIFRIRSBDT1BZUklHSFQgSE9MREVSUyBBTkQgQ09OVFJJQlVUT1JTIFwiQVMgSVNcIiBBTkRcbkFOWSBFWFBSRVNTIE9SIElNUExJRUQgV0FSUkFOVElFUywgSU5DTFVESU5HLCBCVVQgTk9UIExJTUlURUQgVE8sIFRIRSBJTVBMSUVEXG5XQVJSQU5USUVTIE9GIE1FUkNIQU5UQUJJTElUWSBBTkQgRklUTkVTUyBGT1IgQSBQQVJUSUNVTEFSIFBVUlBPU0UgQVJFIFxuRElTQ0xBSU1FRC4gSU4gTk8gRVZFTlQgU0hBTEwgVEhFIENPUFlSSUdIVCBIT0xERVIgT1IgQ09OVFJJQlVUT1JTIEJFIExJQUJMRSBGT1JcbkFOWSBESVJFQ1QsIElORElSRUNULCBJTkNJREVOVEFMLCBTUEVDSUFMLCBFWEVNUExBUlksIE9SIENPTlNFUVVFTlRJQUwgREFNQUdFU1xuKElOQ0xVRElORywgQlVUIE5PVCBMSU1JVEVEIFRPLCBQUk9DVVJFTUVOVCBPRiBTVUJTVElUVVRFIEdPT0RTIE9SIFNFUlZJQ0VTO1xuTE9TUyBPRiBVU0UsIERBVEEsIE9SIFBST0ZJVFM7IE9SIEJVU0lORVNTIElOVEVSUlVQVElPTikgSE9XRVZFUiBDQVVTRUQgQU5EIE9OXG5BTlkgVEhFT1JZIE9GIExJQUJJTElUWSwgV0hFVEhFUiBJTiBDT05UUkFDVCwgU1RSSUNUIExJQUJJTElUWSwgT1IgVE9SVFxuKElOQ0xVRElORyBORUdMSUdFTkNFIE9SIE9USEVSV0lTRSkgQVJJU0lORyBJTiBBTlkgV0FZIE9VVCBPRiBUSEUgVVNFIE9GIFRISVNcblNPRlRXQVJFLCBFVkVOIElGIEFEVklTRUQgT0YgVEhFIFBPU1NJQklMSVRZIE9GIFNVQ0ggREFNQUdFLiAqL1xuXG4vKipcbiAqIEBjbGFzcyBRdWF0ZXJuaW9uXG4gKiBAbmFtZSBxdWF0XG4gKi9cbnZhciBxdWF0ID0ge307XG5cbi8vIGltcG9ydHMuLi5cbnZhciB2ZWMzID0gcmVxdWlyZSgndmVjMycpO1xudmFyIHZlYzQgPSByZXF1aXJlKCd2ZWM0Jyk7XG52YXIgbWF0MyA9IHJlcXVpcmUoJ21hdDMnKTtcbnZhciBHTE1BVF9BUlJBWV9UWVBFID0gcmVxdWlyZSgnY29tbW9uJykuR0xNQVRfQVJSQVlfVFlQRTtcblxuLyoqXG4gKiBDcmVhdGVzIGEgbmV3IGlkZW50aXR5IHF1YXRcbiAqXG4gKiBAcmV0dXJucyB7cXVhdH0gYSBuZXcgcXVhdGVybmlvblxuICovXG5xdWF0LmNyZWF0ZSA9IGZ1bmN0aW9uKCkge1xuICAgIHZhciBvdXQgPSBuZXcgR0xNQVRfQVJSQVlfVFlQRSg0KTtcbiAgICBvdXRbMF0gPSAwO1xuICAgIG91dFsxXSA9IDA7XG4gICAgb3V0WzJdID0gMDtcbiAgICBvdXRbM10gPSAxO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFNldHMgYSBxdWF0ZXJuaW9uIHRvIHJlcHJlc2VudCB0aGUgc2hvcnRlc3Qgcm90YXRpb24gZnJvbSBvbmVcbiAqIHZlY3RvciB0byBhbm90aGVyLlxuICpcbiAqIEJvdGggdmVjdG9ycyBhcmUgYXNzdW1lZCB0byBiZSB1bml0IGxlbmd0aC5cbiAqXG4gKiBAcGFyYW0ge3F1YXR9IG91dCB0aGUgcmVjZWl2aW5nIHF1YXRlcm5pb24uXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdGhlIGluaXRpYWwgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzN9IGIgdGhlIGRlc3RpbmF0aW9uIHZlY3RvclxuICogQHJldHVybnMge3F1YXR9IG91dFxuICovXG5xdWF0LnJvdGF0aW9uVG8gPSAoZnVuY3Rpb24oKSB7XG4gICAgdmFyIHRtcHZlYzMgPSB2ZWMzLmNyZWF0ZSgpO1xuICAgIHZhciB4VW5pdFZlYzMgPSB2ZWMzLmZyb21WYWx1ZXMoMSwwLDApO1xuICAgIHZhciB5VW5pdFZlYzMgPSB2ZWMzLmZyb21WYWx1ZXMoMCwxLDApO1xuXG4gICAgcmV0dXJuIGZ1bmN0aW9uKG91dCwgYSwgYikge1xuICAgICAgICB2YXIgZG90ID0gdmVjMy5kb3QoYSwgYik7XG4gICAgICAgIGlmIChkb3QgPCAtMC45OTk5OTkpIHtcbiAgICAgICAgICAgIHZlYzMuY3Jvc3ModG1wdmVjMywgeFVuaXRWZWMzLCBhKTtcbiAgICAgICAgICAgIGlmICh2ZWMzLmxlbmd0aCh0bXB2ZWMzKSA8IDAuMDAwMDAxKVxuICAgICAgICAgICAgICAgIHZlYzMuY3Jvc3ModG1wdmVjMywgeVVuaXRWZWMzLCBhKTtcbiAgICAgICAgICAgIHZlYzMubm9ybWFsaXplKHRtcHZlYzMsIHRtcHZlYzMpO1xuICAgICAgICAgICAgcXVhdC5zZXRBeGlzQW5nbGUob3V0LCB0bXB2ZWMzLCBNYXRoLlBJKTtcbiAgICAgICAgICAgIHJldHVybiBvdXQ7XG4gICAgICAgIH0gZWxzZSBpZiAoZG90ID4gMC45OTk5OTkpIHtcbiAgICAgICAgICAgIG91dFswXSA9IDA7XG4gICAgICAgICAgICBvdXRbMV0gPSAwO1xuICAgICAgICAgICAgb3V0WzJdID0gMDtcbiAgICAgICAgICAgIG91dFszXSA9IDE7XG4gICAgICAgICAgICByZXR1cm4gb3V0O1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgdmVjMy5jcm9zcyh0bXB2ZWMzLCBhLCBiKTtcbiAgICAgICAgICAgIG91dFswXSA9IHRtcHZlYzNbMF07XG4gICAgICAgICAgICBvdXRbMV0gPSB0bXB2ZWMzWzFdO1xuICAgICAgICAgICAgb3V0WzJdID0gdG1wdmVjM1syXTtcbiAgICAgICAgICAgIG91dFszXSA9IDEgKyBkb3Q7XG4gICAgICAgICAgICByZXR1cm4gcXVhdC5ub3JtYWxpemUob3V0LCBvdXQpO1xuICAgICAgICB9XG4gICAgfTtcbn0pKCk7XG5cbi8qKlxuICogU2V0cyB0aGUgc3BlY2lmaWVkIHF1YXRlcm5pb24gd2l0aCB2YWx1ZXMgY29ycmVzcG9uZGluZyB0byB0aGUgZ2l2ZW5cbiAqIGF4ZXMuIEVhY2ggYXhpcyBpcyBhIHZlYzMgYW5kIGlzIGV4cGVjdGVkIHRvIGJlIHVuaXQgbGVuZ3RoIGFuZFxuICogcGVycGVuZGljdWxhciB0byBhbGwgb3RoZXIgc3BlY2lmaWVkIGF4ZXMuXG4gKlxuICogQHBhcmFtIHt2ZWMzfSB2aWV3ICB0aGUgdmVjdG9yIHJlcHJlc2VudGluZyB0aGUgdmlld2luZyBkaXJlY3Rpb25cbiAqIEBwYXJhbSB7dmVjM30gcmlnaHQgdGhlIHZlY3RvciByZXByZXNlbnRpbmcgdGhlIGxvY2FsIFwicmlnaHRcIiBkaXJlY3Rpb25cbiAqIEBwYXJhbSB7dmVjM30gdXAgICAgdGhlIHZlY3RvciByZXByZXNlbnRpbmcgdGhlIGxvY2FsIFwidXBcIiBkaXJlY3Rpb25cbiAqIEByZXR1cm5zIHtxdWF0fSBvdXRcbiAqL1xucXVhdC5zZXRBeGVzID0gKGZ1bmN0aW9uKCkge1xuICAgIHZhciBtYXRyID0gbWF0My5jcmVhdGUoKTtcblxuICAgIHJldHVybiBmdW5jdGlvbihvdXQsIHZpZXcsIHJpZ2h0LCB1cCkge1xuICAgICAgICBtYXRyWzBdID0gcmlnaHRbMF07XG4gICAgICAgIG1hdHJbM10gPSByaWdodFsxXTtcbiAgICAgICAgbWF0cls2XSA9IHJpZ2h0WzJdO1xuXG4gICAgICAgIG1hdHJbMV0gPSB1cFswXTtcbiAgICAgICAgbWF0cls0XSA9IHVwWzFdO1xuICAgICAgICBtYXRyWzddID0gdXBbMl07XG5cbiAgICAgICAgbWF0clsyXSA9IC12aWV3WzBdO1xuICAgICAgICBtYXRyWzVdID0gLXZpZXdbMV07XG4gICAgICAgIG1hdHJbOF0gPSAtdmlld1syXTtcblxuICAgICAgICByZXR1cm4gcXVhdC5ub3JtYWxpemUob3V0LCBxdWF0LmZyb21NYXQzKG91dCwgbWF0cikpO1xuICAgIH07XG59KSgpO1xuXG4vKipcbiAqIENyZWF0ZXMgYSBuZXcgcXVhdCBpbml0aWFsaXplZCB3aXRoIHZhbHVlcyBmcm9tIGFuIGV4aXN0aW5nIHF1YXRlcm5pb25cbiAqXG4gKiBAcGFyYW0ge3F1YXR9IGEgcXVhdGVybmlvbiB0byBjbG9uZVxuICogQHJldHVybnMge3F1YXR9IGEgbmV3IHF1YXRlcm5pb25cbiAqIEBmdW5jdGlvblxuICovXG5xdWF0LmNsb25lID0gdmVjNC5jbG9uZTtcblxuLyoqXG4gKiBDcmVhdGVzIGEgbmV3IHF1YXQgaW5pdGlhbGl6ZWQgd2l0aCB0aGUgZ2l2ZW4gdmFsdWVzXG4gKlxuICogQHBhcmFtIHtOdW1iZXJ9IHggWCBjb21wb25lbnRcbiAqIEBwYXJhbSB7TnVtYmVyfSB5IFkgY29tcG9uZW50XG4gKiBAcGFyYW0ge051bWJlcn0geiBaIGNvbXBvbmVudFxuICogQHBhcmFtIHtOdW1iZXJ9IHcgVyBjb21wb25lbnRcbiAqIEByZXR1cm5zIHtxdWF0fSBhIG5ldyBxdWF0ZXJuaW9uXG4gKiBAZnVuY3Rpb25cbiAqL1xucXVhdC5mcm9tVmFsdWVzID0gdmVjNC5mcm9tVmFsdWVzO1xuXG4vKipcbiAqIENvcHkgdGhlIHZhbHVlcyBmcm9tIG9uZSBxdWF0IHRvIGFub3RoZXJcbiAqXG4gKiBAcGFyYW0ge3F1YXR9IG91dCB0aGUgcmVjZWl2aW5nIHF1YXRlcm5pb25cbiAqIEBwYXJhbSB7cXVhdH0gYSB0aGUgc291cmNlIHF1YXRlcm5pb25cbiAqIEByZXR1cm5zIHtxdWF0fSBvdXRcbiAqIEBmdW5jdGlvblxuICovXG5xdWF0LmNvcHkgPSB2ZWM0LmNvcHk7XG5cbi8qKlxuICogU2V0IHRoZSBjb21wb25lbnRzIG9mIGEgcXVhdCB0byB0aGUgZ2l2ZW4gdmFsdWVzXG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgdGhlIHJlY2VpdmluZyBxdWF0ZXJuaW9uXG4gKiBAcGFyYW0ge051bWJlcn0geCBYIGNvbXBvbmVudFxuICogQHBhcmFtIHtOdW1iZXJ9IHkgWSBjb21wb25lbnRcbiAqIEBwYXJhbSB7TnVtYmVyfSB6IFogY29tcG9uZW50XG4gKiBAcGFyYW0ge051bWJlcn0gdyBXIGNvbXBvbmVudFxuICogQHJldHVybnMge3F1YXR9IG91dFxuICogQGZ1bmN0aW9uXG4gKi9cbnF1YXQuc2V0ID0gdmVjNC5zZXQ7XG5cbi8qKlxuICogU2V0IGEgcXVhdCB0byB0aGUgaWRlbnRpdHkgcXVhdGVybmlvblxuICpcbiAqIEBwYXJhbSB7cXVhdH0gb3V0IHRoZSByZWNlaXZpbmcgcXVhdGVybmlvblxuICogQHJldHVybnMge3F1YXR9IG91dFxuICovXG5xdWF0LmlkZW50aXR5ID0gZnVuY3Rpb24ob3V0KSB7XG4gICAgb3V0WzBdID0gMDtcbiAgICBvdXRbMV0gPSAwO1xuICAgIG91dFsyXSA9IDA7XG4gICAgb3V0WzNdID0gMTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBTZXRzIGEgcXVhdCBmcm9tIHRoZSBnaXZlbiBhbmdsZSBhbmQgcm90YXRpb24gYXhpcyxcbiAqIHRoZW4gcmV0dXJucyBpdC5cbiAqXG4gKiBAcGFyYW0ge3F1YXR9IG91dCB0aGUgcmVjZWl2aW5nIHF1YXRlcm5pb25cbiAqIEBwYXJhbSB7dmVjM30gYXhpcyB0aGUgYXhpcyBhcm91bmQgd2hpY2ggdG8gcm90YXRlXG4gKiBAcGFyYW0ge051bWJlcn0gcmFkIHRoZSBhbmdsZSBpbiByYWRpYW5zXG4gKiBAcmV0dXJucyB7cXVhdH0gb3V0XG4gKiovXG5xdWF0LnNldEF4aXNBbmdsZSA9IGZ1bmN0aW9uKG91dCwgYXhpcywgcmFkKSB7XG4gICAgcmFkID0gcmFkICogMC41O1xuICAgIHZhciBzID0gTWF0aC5zaW4ocmFkKTtcbiAgICBvdXRbMF0gPSBzICogYXhpc1swXTtcbiAgICBvdXRbMV0gPSBzICogYXhpc1sxXTtcbiAgICBvdXRbMl0gPSBzICogYXhpc1syXTtcbiAgICBvdXRbM10gPSBNYXRoLmNvcyhyYWQpO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIEFkZHMgdHdvIHF1YXQnc1xuICpcbiAqIEBwYXJhbSB7cXVhdH0gb3V0IHRoZSByZWNlaXZpbmcgcXVhdGVybmlvblxuICogQHBhcmFtIHtxdWF0fSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3F1YXR9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcmV0dXJucyB7cXVhdH0gb3V0XG4gKiBAZnVuY3Rpb25cbiAqL1xucXVhdC5hZGQgPSB2ZWM0LmFkZDtcblxuLyoqXG4gKiBNdWx0aXBsaWVzIHR3byBxdWF0J3NcbiAqXG4gKiBAcGFyYW0ge3F1YXR9IG91dCB0aGUgcmVjZWl2aW5nIHF1YXRlcm5pb25cbiAqIEBwYXJhbSB7cXVhdH0gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHtxdWF0fSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge3F1YXR9IG91dFxuICovXG5xdWF0Lm11bHRpcGx5ID0gZnVuY3Rpb24ob3V0LCBhLCBiKSB7XG4gICAgdmFyIGF4ID0gYVswXSwgYXkgPSBhWzFdLCBheiA9IGFbMl0sIGF3ID0gYVszXSxcbiAgICAgICAgYnggPSBiWzBdLCBieSA9IGJbMV0sIGJ6ID0gYlsyXSwgYncgPSBiWzNdO1xuXG4gICAgb3V0WzBdID0gYXggKiBidyArIGF3ICogYnggKyBheSAqIGJ6IC0gYXogKiBieTtcbiAgICBvdXRbMV0gPSBheSAqIGJ3ICsgYXcgKiBieSArIGF6ICogYnggLSBheCAqIGJ6O1xuICAgIG91dFsyXSA9IGF6ICogYncgKyBhdyAqIGJ6ICsgYXggKiBieSAtIGF5ICogYng7XG4gICAgb3V0WzNdID0gYXcgKiBidyAtIGF4ICogYnggLSBheSAqIGJ5IC0gYXogKiBiejtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBBbGlhcyBmb3Ige0BsaW5rIHF1YXQubXVsdGlwbHl9XG4gKiBAZnVuY3Rpb25cbiAqL1xucXVhdC5tdWwgPSBxdWF0Lm11bHRpcGx5O1xuXG4vKipcbiAqIFNjYWxlcyBhIHF1YXQgYnkgYSBzY2FsYXIgbnVtYmVyXG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7cXVhdH0gYSB0aGUgdmVjdG9yIHRvIHNjYWxlXG4gKiBAcGFyYW0ge051bWJlcn0gYiBhbW91bnQgdG8gc2NhbGUgdGhlIHZlY3RvciBieVxuICogQHJldHVybnMge3F1YXR9IG91dFxuICogQGZ1bmN0aW9uXG4gKi9cbnF1YXQuc2NhbGUgPSB2ZWM0LnNjYWxlO1xuXG4vKipcbiAqIFJvdGF0ZXMgYSBxdWF0ZXJuaW9uIGJ5IHRoZSBnaXZlbiBhbmdsZSBhYm91dCB0aGUgWCBheGlzXG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgcXVhdCByZWNlaXZpbmcgb3BlcmF0aW9uIHJlc3VsdFxuICogQHBhcmFtIHtxdWF0fSBhIHF1YXQgdG8gcm90YXRlXG4gKiBAcGFyYW0ge251bWJlcn0gcmFkIGFuZ2xlIChpbiByYWRpYW5zKSB0byByb3RhdGVcbiAqIEByZXR1cm5zIHtxdWF0fSBvdXRcbiAqL1xucXVhdC5yb3RhdGVYID0gZnVuY3Rpb24gKG91dCwgYSwgcmFkKSB7XG4gICAgcmFkICo9IDAuNTsgXG5cbiAgICB2YXIgYXggPSBhWzBdLCBheSA9IGFbMV0sIGF6ID0gYVsyXSwgYXcgPSBhWzNdLFxuICAgICAgICBieCA9IE1hdGguc2luKHJhZCksIGJ3ID0gTWF0aC5jb3MocmFkKTtcblxuICAgIG91dFswXSA9IGF4ICogYncgKyBhdyAqIGJ4O1xuICAgIG91dFsxXSA9IGF5ICogYncgKyBheiAqIGJ4O1xuICAgIG91dFsyXSA9IGF6ICogYncgLSBheSAqIGJ4O1xuICAgIG91dFszXSA9IGF3ICogYncgLSBheCAqIGJ4O1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFJvdGF0ZXMgYSBxdWF0ZXJuaW9uIGJ5IHRoZSBnaXZlbiBhbmdsZSBhYm91dCB0aGUgWSBheGlzXG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgcXVhdCByZWNlaXZpbmcgb3BlcmF0aW9uIHJlc3VsdFxuICogQHBhcmFtIHtxdWF0fSBhIHF1YXQgdG8gcm90YXRlXG4gKiBAcGFyYW0ge251bWJlcn0gcmFkIGFuZ2xlIChpbiByYWRpYW5zKSB0byByb3RhdGVcbiAqIEByZXR1cm5zIHtxdWF0fSBvdXRcbiAqL1xucXVhdC5yb3RhdGVZID0gZnVuY3Rpb24gKG91dCwgYSwgcmFkKSB7XG4gICAgcmFkICo9IDAuNTsgXG5cbiAgICB2YXIgYXggPSBhWzBdLCBheSA9IGFbMV0sIGF6ID0gYVsyXSwgYXcgPSBhWzNdLFxuICAgICAgICBieSA9IE1hdGguc2luKHJhZCksIGJ3ID0gTWF0aC5jb3MocmFkKTtcblxuICAgIG91dFswXSA9IGF4ICogYncgLSBheiAqIGJ5O1xuICAgIG91dFsxXSA9IGF5ICogYncgKyBhdyAqIGJ5O1xuICAgIG91dFsyXSA9IGF6ICogYncgKyBheCAqIGJ5O1xuICAgIG91dFszXSA9IGF3ICogYncgLSBheSAqIGJ5O1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFJvdGF0ZXMgYSBxdWF0ZXJuaW9uIGJ5IHRoZSBnaXZlbiBhbmdsZSBhYm91dCB0aGUgWiBheGlzXG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgcXVhdCByZWNlaXZpbmcgb3BlcmF0aW9uIHJlc3VsdFxuICogQHBhcmFtIHtxdWF0fSBhIHF1YXQgdG8gcm90YXRlXG4gKiBAcGFyYW0ge251bWJlcn0gcmFkIGFuZ2xlIChpbiByYWRpYW5zKSB0byByb3RhdGVcbiAqIEByZXR1cm5zIHtxdWF0fSBvdXRcbiAqL1xucXVhdC5yb3RhdGVaID0gZnVuY3Rpb24gKG91dCwgYSwgcmFkKSB7XG4gICAgcmFkICo9IDAuNTsgXG5cbiAgICB2YXIgYXggPSBhWzBdLCBheSA9IGFbMV0sIGF6ID0gYVsyXSwgYXcgPSBhWzNdLFxuICAgICAgICBieiA9IE1hdGguc2luKHJhZCksIGJ3ID0gTWF0aC5jb3MocmFkKTtcblxuICAgIG91dFswXSA9IGF4ICogYncgKyBheSAqIGJ6O1xuICAgIG91dFsxXSA9IGF5ICogYncgLSBheCAqIGJ6O1xuICAgIG91dFsyXSA9IGF6ICogYncgKyBhdyAqIGJ6O1xuICAgIG91dFszXSA9IGF3ICogYncgLSBheiAqIGJ6O1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIENhbGN1bGF0ZXMgdGhlIFcgY29tcG9uZW50IG9mIGEgcXVhdCBmcm9tIHRoZSBYLCBZLCBhbmQgWiBjb21wb25lbnRzLlxuICogQXNzdW1lcyB0aGF0IHF1YXRlcm5pb24gaXMgMSB1bml0IGluIGxlbmd0aC5cbiAqIEFueSBleGlzdGluZyBXIGNvbXBvbmVudCB3aWxsIGJlIGlnbm9yZWQuXG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgdGhlIHJlY2VpdmluZyBxdWF0ZXJuaW9uXG4gKiBAcGFyYW0ge3F1YXR9IGEgcXVhdCB0byBjYWxjdWxhdGUgVyBjb21wb25lbnQgb2ZcbiAqIEByZXR1cm5zIHtxdWF0fSBvdXRcbiAqL1xucXVhdC5jYWxjdWxhdGVXID0gZnVuY3Rpb24gKG91dCwgYSkge1xuICAgIHZhciB4ID0gYVswXSwgeSA9IGFbMV0sIHogPSBhWzJdO1xuXG4gICAgb3V0WzBdID0geDtcbiAgICBvdXRbMV0gPSB5O1xuICAgIG91dFsyXSA9IHo7XG4gICAgb3V0WzNdID0gLU1hdGguc3FydChNYXRoLmFicygxLjAgLSB4ICogeCAtIHkgKiB5IC0geiAqIHopKTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBDYWxjdWxhdGVzIHRoZSBkb3QgcHJvZHVjdCBvZiB0d28gcXVhdCdzXG4gKlxuICogQHBhcmFtIHtxdWF0fSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3F1YXR9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcmV0dXJucyB7TnVtYmVyfSBkb3QgcHJvZHVjdCBvZiBhIGFuZCBiXG4gKiBAZnVuY3Rpb25cbiAqL1xucXVhdC5kb3QgPSB2ZWM0LmRvdDtcblxuLyoqXG4gKiBQZXJmb3JtcyBhIGxpbmVhciBpbnRlcnBvbGF0aW9uIGJldHdlZW4gdHdvIHF1YXQnc1xuICpcbiAqIEBwYXJhbSB7cXVhdH0gb3V0IHRoZSByZWNlaXZpbmcgcXVhdGVybmlvblxuICogQHBhcmFtIHtxdWF0fSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3F1YXR9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcGFyYW0ge051bWJlcn0gdCBpbnRlcnBvbGF0aW9uIGFtb3VudCBiZXR3ZWVuIHRoZSB0d28gaW5wdXRzXG4gKiBAcmV0dXJucyB7cXVhdH0gb3V0XG4gKiBAZnVuY3Rpb25cbiAqL1xucXVhdC5sZXJwID0gdmVjNC5sZXJwO1xuXG4vKipcbiAqIFBlcmZvcm1zIGEgc3BoZXJpY2FsIGxpbmVhciBpbnRlcnBvbGF0aW9uIGJldHdlZW4gdHdvIHF1YXRcbiAqXG4gKiBAcGFyYW0ge3F1YXR9IG91dCB0aGUgcmVjZWl2aW5nIHF1YXRlcm5pb25cbiAqIEBwYXJhbSB7cXVhdH0gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHtxdWF0fSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHBhcmFtIHtOdW1iZXJ9IHQgaW50ZXJwb2xhdGlvbiBhbW91bnQgYmV0d2VlbiB0aGUgdHdvIGlucHV0c1xuICogQHJldHVybnMge3F1YXR9IG91dFxuICovXG5xdWF0LnNsZXJwID0gZnVuY3Rpb24gKG91dCwgYSwgYiwgdCkge1xuICAgIC8vIGJlbmNobWFya3M6XG4gICAgLy8gICAgaHR0cDovL2pzcGVyZi5jb20vcXVhdGVybmlvbi1zbGVycC1pbXBsZW1lbnRhdGlvbnNcblxuICAgIHZhciBheCA9IGFbMF0sIGF5ID0gYVsxXSwgYXogPSBhWzJdLCBhdyA9IGFbM10sXG4gICAgICAgIGJ4ID0gYlswXSwgYnkgPSBiWzFdLCBieiA9IGJbMl0sIGJ3ID0gYlszXTtcblxuICAgIHZhciAgICAgICAgb21lZ2EsIGNvc29tLCBzaW5vbSwgc2NhbGUwLCBzY2FsZTE7XG5cbiAgICAvLyBjYWxjIGNvc2luZVxuICAgIGNvc29tID0gYXggKiBieCArIGF5ICogYnkgKyBheiAqIGJ6ICsgYXcgKiBidztcbiAgICAvLyBhZGp1c3Qgc2lnbnMgKGlmIG5lY2Vzc2FyeSlcbiAgICBpZiAoIGNvc29tIDwgMC4wICkge1xuICAgICAgICBjb3NvbSA9IC1jb3NvbTtcbiAgICAgICAgYnggPSAtIGJ4O1xuICAgICAgICBieSA9IC0gYnk7XG4gICAgICAgIGJ6ID0gLSBiejtcbiAgICAgICAgYncgPSAtIGJ3O1xuICAgIH1cbiAgICAvLyBjYWxjdWxhdGUgY29lZmZpY2llbnRzXG4gICAgaWYgKCAoMS4wIC0gY29zb20pID4gMC4wMDAwMDEgKSB7XG4gICAgICAgIC8vIHN0YW5kYXJkIGNhc2UgKHNsZXJwKVxuICAgICAgICBvbWVnYSAgPSBNYXRoLmFjb3MoY29zb20pO1xuICAgICAgICBzaW5vbSAgPSBNYXRoLnNpbihvbWVnYSk7XG4gICAgICAgIHNjYWxlMCA9IE1hdGguc2luKCgxLjAgLSB0KSAqIG9tZWdhKSAvIHNpbm9tO1xuICAgICAgICBzY2FsZTEgPSBNYXRoLnNpbih0ICogb21lZ2EpIC8gc2lub207XG4gICAgfSBlbHNlIHsgICAgICAgIFxuICAgICAgICAvLyBcImZyb21cIiBhbmQgXCJ0b1wiIHF1YXRlcm5pb25zIGFyZSB2ZXJ5IGNsb3NlIFxuICAgICAgICAvLyAgLi4uIHNvIHdlIGNhbiBkbyBhIGxpbmVhciBpbnRlcnBvbGF0aW9uXG4gICAgICAgIHNjYWxlMCA9IDEuMCAtIHQ7XG4gICAgICAgIHNjYWxlMSA9IHQ7XG4gICAgfVxuICAgIC8vIGNhbGN1bGF0ZSBmaW5hbCB2YWx1ZXNcbiAgICBvdXRbMF0gPSBzY2FsZTAgKiBheCArIHNjYWxlMSAqIGJ4O1xuICAgIG91dFsxXSA9IHNjYWxlMCAqIGF5ICsgc2NhbGUxICogYnk7XG4gICAgb3V0WzJdID0gc2NhbGUwICogYXogKyBzY2FsZTEgKiBiejtcbiAgICBvdXRbM10gPSBzY2FsZTAgKiBhdyArIHNjYWxlMSAqIGJ3O1xuICAgIFxuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIENhbGN1bGF0ZXMgdGhlIGludmVyc2Ugb2YgYSBxdWF0XG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgdGhlIHJlY2VpdmluZyBxdWF0ZXJuaW9uXG4gKiBAcGFyYW0ge3F1YXR9IGEgcXVhdCB0byBjYWxjdWxhdGUgaW52ZXJzZSBvZlxuICogQHJldHVybnMge3F1YXR9IG91dFxuICovXG5xdWF0LmludmVydCA9IGZ1bmN0aW9uKG91dCwgYSkge1xuICAgIHZhciBhMCA9IGFbMF0sIGExID0gYVsxXSwgYTIgPSBhWzJdLCBhMyA9IGFbM10sXG4gICAgICAgIGRvdCA9IGEwKmEwICsgYTEqYTEgKyBhMiphMiArIGEzKmEzLFxuICAgICAgICBpbnZEb3QgPSBkb3QgPyAxLjAvZG90IDogMDtcbiAgICBcbiAgICAvLyBUT0RPOiBXb3VsZCBiZSBmYXN0ZXIgdG8gcmV0dXJuIFswLDAsMCwwXSBpbW1lZGlhdGVseSBpZiBkb3QgPT0gMFxuXG4gICAgb3V0WzBdID0gLWEwKmludkRvdDtcbiAgICBvdXRbMV0gPSAtYTEqaW52RG90O1xuICAgIG91dFsyXSA9IC1hMippbnZEb3Q7XG4gICAgb3V0WzNdID0gYTMqaW52RG90O1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIENhbGN1bGF0ZXMgdGhlIGNvbmp1Z2F0ZSBvZiBhIHF1YXRcbiAqIElmIHRoZSBxdWF0ZXJuaW9uIGlzIG5vcm1hbGl6ZWQsIHRoaXMgZnVuY3Rpb24gaXMgZmFzdGVyIHRoYW4gcXVhdC5pbnZlcnNlIGFuZCBwcm9kdWNlcyB0aGUgc2FtZSByZXN1bHQuXG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgdGhlIHJlY2VpdmluZyBxdWF0ZXJuaW9uXG4gKiBAcGFyYW0ge3F1YXR9IGEgcXVhdCB0byBjYWxjdWxhdGUgY29uanVnYXRlIG9mXG4gKiBAcmV0dXJucyB7cXVhdH0gb3V0XG4gKi9cbnF1YXQuY29uanVnYXRlID0gZnVuY3Rpb24gKG91dCwgYSkge1xuICAgIG91dFswXSA9IC1hWzBdO1xuICAgIG91dFsxXSA9IC1hWzFdO1xuICAgIG91dFsyXSA9IC1hWzJdO1xuICAgIG91dFszXSA9IGFbM107XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQ2FsY3VsYXRlcyB0aGUgbGVuZ3RoIG9mIGEgcXVhdFxuICpcbiAqIEBwYXJhbSB7cXVhdH0gYSB2ZWN0b3IgdG8gY2FsY3VsYXRlIGxlbmd0aCBvZlxuICogQHJldHVybnMge051bWJlcn0gbGVuZ3RoIG9mIGFcbiAqIEBmdW5jdGlvblxuICovXG5xdWF0Lmxlbmd0aCA9IHZlYzQubGVuZ3RoO1xuXG4vKipcbiAqIEFsaWFzIGZvciB7QGxpbmsgcXVhdC5sZW5ndGh9XG4gKiBAZnVuY3Rpb25cbiAqL1xucXVhdC5sZW4gPSBxdWF0Lmxlbmd0aDtcblxuLyoqXG4gKiBDYWxjdWxhdGVzIHRoZSBzcXVhcmVkIGxlbmd0aCBvZiBhIHF1YXRcbiAqXG4gKiBAcGFyYW0ge3F1YXR9IGEgdmVjdG9yIHRvIGNhbGN1bGF0ZSBzcXVhcmVkIGxlbmd0aCBvZlxuICogQHJldHVybnMge051bWJlcn0gc3F1YXJlZCBsZW5ndGggb2YgYVxuICogQGZ1bmN0aW9uXG4gKi9cbnF1YXQuc3F1YXJlZExlbmd0aCA9IHZlYzQuc3F1YXJlZExlbmd0aDtcblxuLyoqXG4gKiBBbGlhcyBmb3Ige0BsaW5rIHF1YXQuc3F1YXJlZExlbmd0aH1cbiAqIEBmdW5jdGlvblxuICovXG5xdWF0LnNxckxlbiA9IHF1YXQuc3F1YXJlZExlbmd0aDtcblxuLyoqXG4gKiBOb3JtYWxpemUgYSBxdWF0XG4gKlxuICogQHBhcmFtIHtxdWF0fSBvdXQgdGhlIHJlY2VpdmluZyBxdWF0ZXJuaW9uXG4gKiBAcGFyYW0ge3F1YXR9IGEgcXVhdGVybmlvbiB0byBub3JtYWxpemVcbiAqIEByZXR1cm5zIHtxdWF0fSBvdXRcbiAqIEBmdW5jdGlvblxuICovXG5xdWF0Lm5vcm1hbGl6ZSA9IHZlYzQubm9ybWFsaXplO1xuXG4vKipcbiAqIENyZWF0ZXMgYSBxdWF0ZXJuaW9uIGZyb20gdGhlIGdpdmVuIDN4MyByb3RhdGlvbiBtYXRyaXguXG4gKlxuICogTk9URTogVGhlIHJlc3VsdGFudCBxdWF0ZXJuaW9uIGlzIG5vdCBub3JtYWxpemVkLCBzbyB5b3Ugc2hvdWxkIGJlIHN1cmVcbiAqIHRvIHJlbm9ybWFsaXplIHRoZSBxdWF0ZXJuaW9uIHlvdXJzZWxmIHdoZXJlIG5lY2Vzc2FyeS5cbiAqXG4gKiBAcGFyYW0ge3F1YXR9IG91dCB0aGUgcmVjZWl2aW5nIHF1YXRlcm5pb25cbiAqIEBwYXJhbSB7bWF0M30gbSByb3RhdGlvbiBtYXRyaXhcbiAqIEByZXR1cm5zIHtxdWF0fSBvdXRcbiAqIEBmdW5jdGlvblxuICovXG5xdWF0LmZyb21NYXQzID0gZnVuY3Rpb24ob3V0LCBtKSB7XG4gICAgLy8gQWxnb3JpdGhtIGluIEtlbiBTaG9lbWFrZSdzIGFydGljbGUgaW4gMTk4NyBTSUdHUkFQSCBjb3Vyc2Ugbm90ZXNcbiAgICAvLyBhcnRpY2xlIFwiUXVhdGVybmlvbiBDYWxjdWx1cyBhbmQgRmFzdCBBbmltYXRpb25cIi5cbiAgICB2YXIgZlRyYWNlID0gbVswXSArIG1bNF0gKyBtWzhdO1xuICAgIHZhciBmUm9vdDtcblxuICAgIGlmICggZlRyYWNlID4gMC4wICkge1xuICAgICAgICAvLyB8d3wgPiAxLzIsIG1heSBhcyB3ZWxsIGNob29zZSB3ID4gMS8yXG4gICAgICAgIGZSb290ID0gTWF0aC5zcXJ0KGZUcmFjZSArIDEuMCk7ICAvLyAyd1xuICAgICAgICBvdXRbM10gPSAwLjUgKiBmUm9vdDtcbiAgICAgICAgZlJvb3QgPSAwLjUvZlJvb3Q7ICAvLyAxLyg0dylcbiAgICAgICAgb3V0WzBdID0gKG1bN10tbVs1XSkqZlJvb3Q7XG4gICAgICAgIG91dFsxXSA9IChtWzJdLW1bNl0pKmZSb290O1xuICAgICAgICBvdXRbMl0gPSAobVszXS1tWzFdKSpmUm9vdDtcbiAgICB9IGVsc2Uge1xuICAgICAgICAvLyB8d3wgPD0gMS8yXG4gICAgICAgIHZhciBpID0gMDtcbiAgICAgICAgaWYgKCBtWzRdID4gbVswXSApXG4gICAgICAgICAgaSA9IDE7XG4gICAgICAgIGlmICggbVs4XSA+IG1baSozK2ldIClcbiAgICAgICAgICBpID0gMjtcbiAgICAgICAgdmFyIGogPSAoaSsxKSUzO1xuICAgICAgICB2YXIgayA9IChpKzIpJTM7XG4gICAgICAgIFxuICAgICAgICBmUm9vdCA9IE1hdGguc3FydChtW2kqMytpXS1tW2oqMytqXS1tW2sqMytrXSArIDEuMCk7XG4gICAgICAgIG91dFtpXSA9IDAuNSAqIGZSb290O1xuICAgICAgICBmUm9vdCA9IDAuNSAvIGZSb290O1xuICAgICAgICBvdXRbM10gPSAobVtrKjMral0gLSBtW2oqMytrXSkgKiBmUm9vdDtcbiAgICAgICAgb3V0W2pdID0gKG1baiozK2ldICsgbVtpKjMral0pICogZlJvb3Q7XG4gICAgICAgIG91dFtrXSA9IChtW2sqMytpXSArIG1baSozK2tdKSAqIGZSb290O1xuICAgIH1cbiAgICBcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBSZXR1cm5zIGEgc3RyaW5nIHJlcHJlc2VudGF0aW9uIG9mIGEgcXVhdGVuaW9uXG4gKlxuICogQHBhcmFtIHtxdWF0fSB2ZWMgdmVjdG9yIHRvIHJlcHJlc2VudCBhcyBhIHN0cmluZ1xuICogQHJldHVybnMge1N0cmluZ30gc3RyaW5nIHJlcHJlc2VudGF0aW9uIG9mIHRoZSB2ZWN0b3JcbiAqL1xucXVhdC5zdHIgPSBmdW5jdGlvbiAoYSkge1xuICAgIHJldHVybiAncXVhdCgnICsgYVswXSArICcsICcgKyBhWzFdICsgJywgJyArIGFbMl0gKyAnLCAnICsgYVszXSArICcpJztcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gcXVhdDtcblxuIiwiLyogQ29weXJpZ2h0IChjKSAyMDEzLCBCcmFuZG9uIEpvbmVzLCBDb2xpbiBNYWNLZW56aWUgSVYuIEFsbCByaWdodHMgcmVzZXJ2ZWQuXG5cblJlZGlzdHJpYnV0aW9uIGFuZCB1c2UgaW4gc291cmNlIGFuZCBiaW5hcnkgZm9ybXMsIHdpdGggb3Igd2l0aG91dCBtb2RpZmljYXRpb24sXG5hcmUgcGVybWl0dGVkIHByb3ZpZGVkIHRoYXQgdGhlIGZvbGxvd2luZyBjb25kaXRpb25zIGFyZSBtZXQ6XG5cbiAgKiBSZWRpc3RyaWJ1dGlvbnMgb2Ygc291cmNlIGNvZGUgbXVzdCByZXRhaW4gdGhlIGFib3ZlIGNvcHlyaWdodCBub3RpY2UsIHRoaXNcbiAgICBsaXN0IG9mIGNvbmRpdGlvbnMgYW5kIHRoZSBmb2xsb3dpbmcgZGlzY2xhaW1lci5cbiAgKiBSZWRpc3RyaWJ1dGlvbnMgaW4gYmluYXJ5IGZvcm0gbXVzdCByZXByb2R1Y2UgdGhlIGFib3ZlIGNvcHlyaWdodCBub3RpY2UsXG4gICAgdGhpcyBsaXN0IG9mIGNvbmRpdGlvbnMgYW5kIHRoZSBmb2xsb3dpbmcgZGlzY2xhaW1lciBpbiB0aGUgZG9jdW1lbnRhdGlvbiBcbiAgICBhbmQvb3Igb3RoZXIgbWF0ZXJpYWxzIHByb3ZpZGVkIHdpdGggdGhlIGRpc3RyaWJ1dGlvbi5cblxuVEhJUyBTT0ZUV0FSRSBJUyBQUk9WSURFRCBCWSBUSEUgQ09QWVJJR0hUIEhPTERFUlMgQU5EIENPTlRSSUJVVE9SUyBcIkFTIElTXCIgQU5EXG5BTlkgRVhQUkVTUyBPUiBJTVBMSUVEIFdBUlJBTlRJRVMsIElOQ0xVRElORywgQlVUIE5PVCBMSU1JVEVEIFRPLCBUSEUgSU1QTElFRFxuV0FSUkFOVElFUyBPRiBNRVJDSEFOVEFCSUxJVFkgQU5EIEZJVE5FU1MgRk9SIEEgUEFSVElDVUxBUiBQVVJQT1NFIEFSRSBcbkRJU0NMQUlNRUQuIElOIE5PIEVWRU5UIFNIQUxMIFRIRSBDT1BZUklHSFQgSE9MREVSIE9SIENPTlRSSUJVVE9SUyBCRSBMSUFCTEUgRk9SXG5BTlkgRElSRUNULCBJTkRJUkVDVCwgSU5DSURFTlRBTCwgU1BFQ0lBTCwgRVhFTVBMQVJZLCBPUiBDT05TRVFVRU5USUFMIERBTUFHRVNcbihJTkNMVURJTkcsIEJVVCBOT1QgTElNSVRFRCBUTywgUFJPQ1VSRU1FTlQgT0YgU1VCU1RJVFVURSBHT09EUyBPUiBTRVJWSUNFUztcbkxPU1MgT0YgVVNFLCBEQVRBLCBPUiBQUk9GSVRTOyBPUiBCVVNJTkVTUyBJTlRFUlJVUFRJT04pIEhPV0VWRVIgQ0FVU0VEIEFORCBPTlxuQU5ZIFRIRU9SWSBPRiBMSUFCSUxJVFksIFdIRVRIRVIgSU4gQ09OVFJBQ1QsIFNUUklDVCBMSUFCSUxJVFksIE9SIFRPUlRcbihJTkNMVURJTkcgTkVHTElHRU5DRSBPUiBPVEhFUldJU0UpIEFSSVNJTkcgSU4gQU5ZIFdBWSBPVVQgT0YgVEhFIFVTRSBPRiBUSElTXG5TT0ZUV0FSRSwgRVZFTiBJRiBBRFZJU0VEIE9GIFRIRSBQT1NTSUJJTElUWSBPRiBTVUNIIERBTUFHRS4gKi9cblxuaWYoIUdMTUFUX0VQU0lMT04pIHtcbiAgICB2YXIgR0xNQVRfRVBTSUxPTiA9IDAuMDAwMDAxO1xufVxuXG5pZighR0xNQVRfQVJSQVlfVFlQRSkge1xuICAgIHZhciBHTE1BVF9BUlJBWV9UWVBFID0gKHR5cGVvZiBGbG9hdDMyQXJyYXkgIT09ICd1bmRlZmluZWQnKSA/IEZsb2F0MzJBcnJheSA6IEFycmF5O1xufVxuXG5pZighR0xNQVRfUkFORE9NKSB7XG4gICAgdmFyIEdMTUFUX1JBTkRPTSA9IE1hdGgucmFuZG9tO1xufVxuXG4vKipcbiAqIEBjbGFzcyBDb21tb24gdXRpbGl0aWVzXG4gKiBAbmFtZSBnbE1hdHJpeFxuICovXG52YXIgZ2xNYXRyaXggPSB7fTtcblxuLyoqXG4gKiBTZXRzIHRoZSB0eXBlIG9mIGFycmF5IHVzZWQgd2hlbiBjcmVhdGluZyBuZXcgdmVjdG9ycyBhbmQgbWF0cmljaWVzXG4gKlxuICogQHBhcmFtIHtUeXBlfSB0eXBlIEFycmF5IHR5cGUsIHN1Y2ggYXMgRmxvYXQzMkFycmF5IG9yIEFycmF5XG4gKi9cbmdsTWF0cml4LnNldE1hdHJpeEFycmF5VHlwZSA9IGZ1bmN0aW9uKHR5cGUpIHtcbiAgICBHTE1BVF9BUlJBWV9UWVBFID0gdHlwZTtcbn1cblxuXG52YXIgZGVncmVlID0gTWF0aC5QSSAvIDE4MDtcblxuLyoqXG4qIENvbnZlcnQgRGVncmVlIFRvIFJhZGlhblxuKlxuKiBAcGFyYW0ge051bWJlcn0gQW5nbGUgaW4gRGVncmVlc1xuKi9cbmdsTWF0cml4LnRvUmFkaWFuID0gZnVuY3Rpb24oYSl7XG4gICAgIHJldHVybiBhICogZGVncmVlO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IHtcbiAgR0xNQVRfRVBTSUxPTiA6IEdMTUFUX0VQU0lMT04sXG4gIEdMTUFUX0FSUkFZX1RZUEUgOiBHTE1BVF9BUlJBWV9UWVBFLFxuICBHTE1BVF9SQU5ET00gOiBHTE1BVF9SQU5ET00sXG4gIGdsTWF0cml4IDogZ2xNYXRyaXhcbn07XG4iLCIvKiBDb3B5cmlnaHQgKGMpIDIwMTMsIEJyYW5kb24gSm9uZXMsIENvbGluIE1hY0tlbnppZSBJVi4gQWxsIHJpZ2h0cyByZXNlcnZlZC5cblxuUmVkaXN0cmlidXRpb24gYW5kIHVzZSBpbiBzb3VyY2UgYW5kIGJpbmFyeSBmb3Jtcywgd2l0aCBvciB3aXRob3V0IG1vZGlmaWNhdGlvbixcbmFyZSBwZXJtaXR0ZWQgcHJvdmlkZWQgdGhhdCB0aGUgZm9sbG93aW5nIGNvbmRpdGlvbnMgYXJlIG1ldDpcblxuICAqIFJlZGlzdHJpYnV0aW9ucyBvZiBzb3VyY2UgY29kZSBtdXN0IHJldGFpbiB0aGUgYWJvdmUgY29weXJpZ2h0IG5vdGljZSwgdGhpc1xuICAgIGxpc3Qgb2YgY29uZGl0aW9ucyBhbmQgdGhlIGZvbGxvd2luZyBkaXNjbGFpbWVyLlxuICAqIFJlZGlzdHJpYnV0aW9ucyBpbiBiaW5hcnkgZm9ybSBtdXN0IHJlcHJvZHVjZSB0aGUgYWJvdmUgY29weXJpZ2h0IG5vdGljZSxcbiAgICB0aGlzIGxpc3Qgb2YgY29uZGl0aW9ucyBhbmQgdGhlIGZvbGxvd2luZyBkaXNjbGFpbWVyIGluIHRoZSBkb2N1bWVudGF0aW9uIFxuICAgIGFuZC9vciBvdGhlciBtYXRlcmlhbHMgcHJvdmlkZWQgd2l0aCB0aGUgZGlzdHJpYnV0aW9uLlxuXG5USElTIFNPRlRXQVJFIElTIFBST1ZJREVEIEJZIFRIRSBDT1BZUklHSFQgSE9MREVSUyBBTkQgQ09OVFJJQlVUT1JTIFwiQVMgSVNcIiBBTkRcbkFOWSBFWFBSRVNTIE9SIElNUExJRUQgV0FSUkFOVElFUywgSU5DTFVESU5HLCBCVVQgTk9UIExJTUlURUQgVE8sIFRIRSBJTVBMSUVEXG5XQVJSQU5USUVTIE9GIE1FUkNIQU5UQUJJTElUWSBBTkQgRklUTkVTUyBGT1IgQSBQQVJUSUNVTEFSIFBVUlBPU0UgQVJFIFxuRElTQ0xBSU1FRC4gSU4gTk8gRVZFTlQgU0hBTEwgVEhFIENPUFlSSUdIVCBIT0xERVIgT1IgQ09OVFJJQlVUT1JTIEJFIExJQUJMRSBGT1JcbkFOWSBESVJFQ1QsIElORElSRUNULCBJTkNJREVOVEFMLCBTUEVDSUFMLCBFWEVNUExBUlksIE9SIENPTlNFUVVFTlRJQUwgREFNQUdFU1xuKElOQ0xVRElORywgQlVUIE5PVCBMSU1JVEVEIFRPLCBQUk9DVVJFTUVOVCBPRiBTVUJTVElUVVRFIEdPT0RTIE9SIFNFUlZJQ0VTO1xuTE9TUyBPRiBVU0UsIERBVEEsIE9SIFBST0ZJVFM7IE9SIEJVU0lORVNTIElOVEVSUlVQVElPTikgSE9XRVZFUiBDQVVTRUQgQU5EIE9OXG5BTlkgVEhFT1JZIE9GIExJQUJJTElUWSwgV0hFVEhFUiBJTiBDT05UUkFDVCwgU1RSSUNUIExJQUJJTElUWSwgT1IgVE9SVFxuKElOQ0xVRElORyBORUdMSUdFTkNFIE9SIE9USEVSV0lTRSkgQVJJU0lORyBJTiBBTlkgV0FZIE9VVCBPRiBUSEUgVVNFIE9GIFRISVNcblNPRlRXQVJFLCBFVkVOIElGIEFEVklTRUQgT0YgVEhFIFBPU1NJQklMSVRZIE9GIFNVQ0ggREFNQUdFLiAqL1xuXG4vKipcbiAqIEBjbGFzcyAzeDMgTWF0cml4XG4gKiBAbmFtZSBtYXQzXG4gKi9cbnZhciBtYXQzID0ge307XG5cbnZhciBHTE1BVF9BUlJBWV9UWVBFID0gcmVxdWlyZSgnY29tbW9uJykuR0xNQVRfQVJSQVlfVFlQRTtcblxuLyoqXG4gKiBDcmVhdGVzIGEgbmV3IGlkZW50aXR5IG1hdDNcbiAqXG4gKiBAcmV0dXJucyB7bWF0M30gYSBuZXcgM3gzIG1hdHJpeFxuICovXG5tYXQzLmNyZWF0ZSA9IGZ1bmN0aW9uKCkge1xuICAgIHZhciBvdXQgPSBuZXcgR0xNQVRfQVJSQVlfVFlQRSg5KTtcbiAgICBvdXRbMF0gPSAxO1xuICAgIG91dFsxXSA9IDA7XG4gICAgb3V0WzJdID0gMDtcbiAgICBvdXRbM10gPSAwO1xuICAgIG91dFs0XSA9IDE7XG4gICAgb3V0WzVdID0gMDtcbiAgICBvdXRbNl0gPSAwO1xuICAgIG91dFs3XSA9IDA7XG4gICAgb3V0WzhdID0gMTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBDb3BpZXMgdGhlIHVwcGVyLWxlZnQgM3gzIHZhbHVlcyBpbnRvIHRoZSBnaXZlbiBtYXQzLlxuICpcbiAqIEBwYXJhbSB7bWF0M30gb3V0IHRoZSByZWNlaXZpbmcgM3gzIG1hdHJpeFxuICogQHBhcmFtIHttYXQ0fSBhICAgdGhlIHNvdXJjZSA0eDQgbWF0cml4XG4gKiBAcmV0dXJucyB7bWF0M30gb3V0XG4gKi9cbm1hdDMuZnJvbU1hdDQgPSBmdW5jdGlvbihvdXQsIGEpIHtcbiAgICBvdXRbMF0gPSBhWzBdO1xuICAgIG91dFsxXSA9IGFbMV07XG4gICAgb3V0WzJdID0gYVsyXTtcbiAgICBvdXRbM10gPSBhWzRdO1xuICAgIG91dFs0XSA9IGFbNV07XG4gICAgb3V0WzVdID0gYVs2XTtcbiAgICBvdXRbNl0gPSBhWzhdO1xuICAgIG91dFs3XSA9IGFbOV07XG4gICAgb3V0WzhdID0gYVsxMF07XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQ3JlYXRlcyBhIG5ldyBtYXQzIGluaXRpYWxpemVkIHdpdGggdmFsdWVzIGZyb20gYW4gZXhpc3RpbmcgbWF0cml4XG4gKlxuICogQHBhcmFtIHttYXQzfSBhIG1hdHJpeCB0byBjbG9uZVxuICogQHJldHVybnMge21hdDN9IGEgbmV3IDN4MyBtYXRyaXhcbiAqL1xubWF0My5jbG9uZSA9IGZ1bmN0aW9uKGEpIHtcbiAgICB2YXIgb3V0ID0gbmV3IEdMTUFUX0FSUkFZX1RZUEUoOSk7XG4gICAgb3V0WzBdID0gYVswXTtcbiAgICBvdXRbMV0gPSBhWzFdO1xuICAgIG91dFsyXSA9IGFbMl07XG4gICAgb3V0WzNdID0gYVszXTtcbiAgICBvdXRbNF0gPSBhWzRdO1xuICAgIG91dFs1XSA9IGFbNV07XG4gICAgb3V0WzZdID0gYVs2XTtcbiAgICBvdXRbN10gPSBhWzddO1xuICAgIG91dFs4XSA9IGFbOF07XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQ29weSB0aGUgdmFsdWVzIGZyb20gb25lIG1hdDMgdG8gYW5vdGhlclxuICpcbiAqIEBwYXJhbSB7bWF0M30gb3V0IHRoZSByZWNlaXZpbmcgbWF0cml4XG4gKiBAcGFyYW0ge21hdDN9IGEgdGhlIHNvdXJjZSBtYXRyaXhcbiAqIEByZXR1cm5zIHttYXQzfSBvdXRcbiAqL1xubWF0My5jb3B5ID0gZnVuY3Rpb24ob3V0LCBhKSB7XG4gICAgb3V0WzBdID0gYVswXTtcbiAgICBvdXRbMV0gPSBhWzFdO1xuICAgIG91dFsyXSA9IGFbMl07XG4gICAgb3V0WzNdID0gYVszXTtcbiAgICBvdXRbNF0gPSBhWzRdO1xuICAgIG91dFs1XSA9IGFbNV07XG4gICAgb3V0WzZdID0gYVs2XTtcbiAgICBvdXRbN10gPSBhWzddO1xuICAgIG91dFs4XSA9IGFbOF07XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogU2V0IGEgbWF0MyB0byB0aGUgaWRlbnRpdHkgbWF0cml4XG4gKlxuICogQHBhcmFtIHttYXQzfSBvdXQgdGhlIHJlY2VpdmluZyBtYXRyaXhcbiAqIEByZXR1cm5zIHttYXQzfSBvdXRcbiAqL1xubWF0My5pZGVudGl0eSA9IGZ1bmN0aW9uKG91dCkge1xuICAgIG91dFswXSA9IDE7XG4gICAgb3V0WzFdID0gMDtcbiAgICBvdXRbMl0gPSAwO1xuICAgIG91dFszXSA9IDA7XG4gICAgb3V0WzRdID0gMTtcbiAgICBvdXRbNV0gPSAwO1xuICAgIG91dFs2XSA9IDA7XG4gICAgb3V0WzddID0gMDtcbiAgICBvdXRbOF0gPSAxO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFRyYW5zcG9zZSB0aGUgdmFsdWVzIG9mIGEgbWF0M1xuICpcbiAqIEBwYXJhbSB7bWF0M30gb3V0IHRoZSByZWNlaXZpbmcgbWF0cml4XG4gKiBAcGFyYW0ge21hdDN9IGEgdGhlIHNvdXJjZSBtYXRyaXhcbiAqIEByZXR1cm5zIHttYXQzfSBvdXRcbiAqL1xubWF0My50cmFuc3Bvc2UgPSBmdW5jdGlvbihvdXQsIGEpIHtcbiAgICAvLyBJZiB3ZSBhcmUgdHJhbnNwb3Npbmcgb3Vyc2VsdmVzIHdlIGNhbiBza2lwIGEgZmV3IHN0ZXBzIGJ1dCBoYXZlIHRvIGNhY2hlIHNvbWUgdmFsdWVzXG4gICAgaWYgKG91dCA9PT0gYSkge1xuICAgICAgICB2YXIgYTAxID0gYVsxXSwgYTAyID0gYVsyXSwgYTEyID0gYVs1XTtcbiAgICAgICAgb3V0WzFdID0gYVszXTtcbiAgICAgICAgb3V0WzJdID0gYVs2XTtcbiAgICAgICAgb3V0WzNdID0gYTAxO1xuICAgICAgICBvdXRbNV0gPSBhWzddO1xuICAgICAgICBvdXRbNl0gPSBhMDI7XG4gICAgICAgIG91dFs3XSA9IGExMjtcbiAgICB9IGVsc2Uge1xuICAgICAgICBvdXRbMF0gPSBhWzBdO1xuICAgICAgICBvdXRbMV0gPSBhWzNdO1xuICAgICAgICBvdXRbMl0gPSBhWzZdO1xuICAgICAgICBvdXRbM10gPSBhWzFdO1xuICAgICAgICBvdXRbNF0gPSBhWzRdO1xuICAgICAgICBvdXRbNV0gPSBhWzddO1xuICAgICAgICBvdXRbNl0gPSBhWzJdO1xuICAgICAgICBvdXRbN10gPSBhWzVdO1xuICAgICAgICBvdXRbOF0gPSBhWzhdO1xuICAgIH1cbiAgICBcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBJbnZlcnRzIGEgbWF0M1xuICpcbiAqIEBwYXJhbSB7bWF0M30gb3V0IHRoZSByZWNlaXZpbmcgbWF0cml4XG4gKiBAcGFyYW0ge21hdDN9IGEgdGhlIHNvdXJjZSBtYXRyaXhcbiAqIEByZXR1cm5zIHttYXQzfSBvdXRcbiAqL1xubWF0My5pbnZlcnQgPSBmdW5jdGlvbihvdXQsIGEpIHtcbiAgICB2YXIgYTAwID0gYVswXSwgYTAxID0gYVsxXSwgYTAyID0gYVsyXSxcbiAgICAgICAgYTEwID0gYVszXSwgYTExID0gYVs0XSwgYTEyID0gYVs1XSxcbiAgICAgICAgYTIwID0gYVs2XSwgYTIxID0gYVs3XSwgYTIyID0gYVs4XSxcblxuICAgICAgICBiMDEgPSBhMjIgKiBhMTEgLSBhMTIgKiBhMjEsXG4gICAgICAgIGIxMSA9IC1hMjIgKiBhMTAgKyBhMTIgKiBhMjAsXG4gICAgICAgIGIyMSA9IGEyMSAqIGExMCAtIGExMSAqIGEyMCxcblxuICAgICAgICAvLyBDYWxjdWxhdGUgdGhlIGRldGVybWluYW50XG4gICAgICAgIGRldCA9IGEwMCAqIGIwMSArIGEwMSAqIGIxMSArIGEwMiAqIGIyMTtcblxuICAgIGlmICghZGV0KSB7IFxuICAgICAgICByZXR1cm4gbnVsbDsgXG4gICAgfVxuICAgIGRldCA9IDEuMCAvIGRldDtcblxuICAgIG91dFswXSA9IGIwMSAqIGRldDtcbiAgICBvdXRbMV0gPSAoLWEyMiAqIGEwMSArIGEwMiAqIGEyMSkgKiBkZXQ7XG4gICAgb3V0WzJdID0gKGExMiAqIGEwMSAtIGEwMiAqIGExMSkgKiBkZXQ7XG4gICAgb3V0WzNdID0gYjExICogZGV0O1xuICAgIG91dFs0XSA9IChhMjIgKiBhMDAgLSBhMDIgKiBhMjApICogZGV0O1xuICAgIG91dFs1XSA9ICgtYTEyICogYTAwICsgYTAyICogYTEwKSAqIGRldDtcbiAgICBvdXRbNl0gPSBiMjEgKiBkZXQ7XG4gICAgb3V0WzddID0gKC1hMjEgKiBhMDAgKyBhMDEgKiBhMjApICogZGV0O1xuICAgIG91dFs4XSA9IChhMTEgKiBhMDAgLSBhMDEgKiBhMTApICogZGV0O1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIENhbGN1bGF0ZXMgdGhlIGFkanVnYXRlIG9mIGEgbWF0M1xuICpcbiAqIEBwYXJhbSB7bWF0M30gb3V0IHRoZSByZWNlaXZpbmcgbWF0cml4XG4gKiBAcGFyYW0ge21hdDN9IGEgdGhlIHNvdXJjZSBtYXRyaXhcbiAqIEByZXR1cm5zIHttYXQzfSBvdXRcbiAqL1xubWF0My5hZGpvaW50ID0gZnVuY3Rpb24ob3V0LCBhKSB7XG4gICAgdmFyIGEwMCA9IGFbMF0sIGEwMSA9IGFbMV0sIGEwMiA9IGFbMl0sXG4gICAgICAgIGExMCA9IGFbM10sIGExMSA9IGFbNF0sIGExMiA9IGFbNV0sXG4gICAgICAgIGEyMCA9IGFbNl0sIGEyMSA9IGFbN10sIGEyMiA9IGFbOF07XG5cbiAgICBvdXRbMF0gPSAoYTExICogYTIyIC0gYTEyICogYTIxKTtcbiAgICBvdXRbMV0gPSAoYTAyICogYTIxIC0gYTAxICogYTIyKTtcbiAgICBvdXRbMl0gPSAoYTAxICogYTEyIC0gYTAyICogYTExKTtcbiAgICBvdXRbM10gPSAoYTEyICogYTIwIC0gYTEwICogYTIyKTtcbiAgICBvdXRbNF0gPSAoYTAwICogYTIyIC0gYTAyICogYTIwKTtcbiAgICBvdXRbNV0gPSAoYTAyICogYTEwIC0gYTAwICogYTEyKTtcbiAgICBvdXRbNl0gPSAoYTEwICogYTIxIC0gYTExICogYTIwKTtcbiAgICBvdXRbN10gPSAoYTAxICogYTIwIC0gYTAwICogYTIxKTtcbiAgICBvdXRbOF0gPSAoYTAwICogYTExIC0gYTAxICogYTEwKTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBDYWxjdWxhdGVzIHRoZSBkZXRlcm1pbmFudCBvZiBhIG1hdDNcbiAqXG4gKiBAcGFyYW0ge21hdDN9IGEgdGhlIHNvdXJjZSBtYXRyaXhcbiAqIEByZXR1cm5zIHtOdW1iZXJ9IGRldGVybWluYW50IG9mIGFcbiAqL1xubWF0My5kZXRlcm1pbmFudCA9IGZ1bmN0aW9uIChhKSB7XG4gICAgdmFyIGEwMCA9IGFbMF0sIGEwMSA9IGFbMV0sIGEwMiA9IGFbMl0sXG4gICAgICAgIGExMCA9IGFbM10sIGExMSA9IGFbNF0sIGExMiA9IGFbNV0sXG4gICAgICAgIGEyMCA9IGFbNl0sIGEyMSA9IGFbN10sIGEyMiA9IGFbOF07XG5cbiAgICByZXR1cm4gYTAwICogKGEyMiAqIGExMSAtIGExMiAqIGEyMSkgKyBhMDEgKiAoLWEyMiAqIGExMCArIGExMiAqIGEyMCkgKyBhMDIgKiAoYTIxICogYTEwIC0gYTExICogYTIwKTtcbn07XG5cbi8qKlxuICogTXVsdGlwbGllcyB0d28gbWF0MydzXG4gKlxuICogQHBhcmFtIHttYXQzfSBvdXQgdGhlIHJlY2VpdmluZyBtYXRyaXhcbiAqIEBwYXJhbSB7bWF0M30gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHttYXQzfSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge21hdDN9IG91dFxuICovXG5tYXQzLm11bHRpcGx5ID0gZnVuY3Rpb24gKG91dCwgYSwgYikge1xuICAgIHZhciBhMDAgPSBhWzBdLCBhMDEgPSBhWzFdLCBhMDIgPSBhWzJdLFxuICAgICAgICBhMTAgPSBhWzNdLCBhMTEgPSBhWzRdLCBhMTIgPSBhWzVdLFxuICAgICAgICBhMjAgPSBhWzZdLCBhMjEgPSBhWzddLCBhMjIgPSBhWzhdLFxuXG4gICAgICAgIGIwMCA9IGJbMF0sIGIwMSA9IGJbMV0sIGIwMiA9IGJbMl0sXG4gICAgICAgIGIxMCA9IGJbM10sIGIxMSA9IGJbNF0sIGIxMiA9IGJbNV0sXG4gICAgICAgIGIyMCA9IGJbNl0sIGIyMSA9IGJbN10sIGIyMiA9IGJbOF07XG5cbiAgICBvdXRbMF0gPSBiMDAgKiBhMDAgKyBiMDEgKiBhMTAgKyBiMDIgKiBhMjA7XG4gICAgb3V0WzFdID0gYjAwICogYTAxICsgYjAxICogYTExICsgYjAyICogYTIxO1xuICAgIG91dFsyXSA9IGIwMCAqIGEwMiArIGIwMSAqIGExMiArIGIwMiAqIGEyMjtcblxuICAgIG91dFszXSA9IGIxMCAqIGEwMCArIGIxMSAqIGExMCArIGIxMiAqIGEyMDtcbiAgICBvdXRbNF0gPSBiMTAgKiBhMDEgKyBiMTEgKiBhMTEgKyBiMTIgKiBhMjE7XG4gICAgb3V0WzVdID0gYjEwICogYTAyICsgYjExICogYTEyICsgYjEyICogYTIyO1xuXG4gICAgb3V0WzZdID0gYjIwICogYTAwICsgYjIxICogYTEwICsgYjIyICogYTIwO1xuICAgIG91dFs3XSA9IGIyMCAqIGEwMSArIGIyMSAqIGExMSArIGIyMiAqIGEyMTtcbiAgICBvdXRbOF0gPSBiMjAgKiBhMDIgKyBiMjEgKiBhMTIgKyBiMjIgKiBhMjI7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQWxpYXMgZm9yIHtAbGluayBtYXQzLm11bHRpcGx5fVxuICogQGZ1bmN0aW9uXG4gKi9cbm1hdDMubXVsID0gbWF0My5tdWx0aXBseTtcblxuLyoqXG4gKiBUcmFuc2xhdGUgYSBtYXQzIGJ5IHRoZSBnaXZlbiB2ZWN0b3JcbiAqXG4gKiBAcGFyYW0ge21hdDN9IG91dCB0aGUgcmVjZWl2aW5nIG1hdHJpeFxuICogQHBhcmFtIHttYXQzfSBhIHRoZSBtYXRyaXggdG8gdHJhbnNsYXRlXG4gKiBAcGFyYW0ge3ZlYzJ9IHYgdmVjdG9yIHRvIHRyYW5zbGF0ZSBieVxuICogQHJldHVybnMge21hdDN9IG91dFxuICovXG5tYXQzLnRyYW5zbGF0ZSA9IGZ1bmN0aW9uKG91dCwgYSwgdikge1xuICAgIHZhciBhMDAgPSBhWzBdLCBhMDEgPSBhWzFdLCBhMDIgPSBhWzJdLFxuICAgICAgICBhMTAgPSBhWzNdLCBhMTEgPSBhWzRdLCBhMTIgPSBhWzVdLFxuICAgICAgICBhMjAgPSBhWzZdLCBhMjEgPSBhWzddLCBhMjIgPSBhWzhdLFxuICAgICAgICB4ID0gdlswXSwgeSA9IHZbMV07XG5cbiAgICBvdXRbMF0gPSBhMDA7XG4gICAgb3V0WzFdID0gYTAxO1xuICAgIG91dFsyXSA9IGEwMjtcblxuICAgIG91dFszXSA9IGExMDtcbiAgICBvdXRbNF0gPSBhMTE7XG4gICAgb3V0WzVdID0gYTEyO1xuXG4gICAgb3V0WzZdID0geCAqIGEwMCArIHkgKiBhMTAgKyBhMjA7XG4gICAgb3V0WzddID0geCAqIGEwMSArIHkgKiBhMTEgKyBhMjE7XG4gICAgb3V0WzhdID0geCAqIGEwMiArIHkgKiBhMTIgKyBhMjI7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogUm90YXRlcyBhIG1hdDMgYnkgdGhlIGdpdmVuIGFuZ2xlXG4gKlxuICogQHBhcmFtIHttYXQzfSBvdXQgdGhlIHJlY2VpdmluZyBtYXRyaXhcbiAqIEBwYXJhbSB7bWF0M30gYSB0aGUgbWF0cml4IHRvIHJvdGF0ZVxuICogQHBhcmFtIHtOdW1iZXJ9IHJhZCB0aGUgYW5nbGUgdG8gcm90YXRlIHRoZSBtYXRyaXggYnlcbiAqIEByZXR1cm5zIHttYXQzfSBvdXRcbiAqL1xubWF0My5yb3RhdGUgPSBmdW5jdGlvbiAob3V0LCBhLCByYWQpIHtcbiAgICB2YXIgYTAwID0gYVswXSwgYTAxID0gYVsxXSwgYTAyID0gYVsyXSxcbiAgICAgICAgYTEwID0gYVszXSwgYTExID0gYVs0XSwgYTEyID0gYVs1XSxcbiAgICAgICAgYTIwID0gYVs2XSwgYTIxID0gYVs3XSwgYTIyID0gYVs4XSxcblxuICAgICAgICBzID0gTWF0aC5zaW4ocmFkKSxcbiAgICAgICAgYyA9IE1hdGguY29zKHJhZCk7XG5cbiAgICBvdXRbMF0gPSBjICogYTAwICsgcyAqIGExMDtcbiAgICBvdXRbMV0gPSBjICogYTAxICsgcyAqIGExMTtcbiAgICBvdXRbMl0gPSBjICogYTAyICsgcyAqIGExMjtcblxuICAgIG91dFszXSA9IGMgKiBhMTAgLSBzICogYTAwO1xuICAgIG91dFs0XSA9IGMgKiBhMTEgLSBzICogYTAxO1xuICAgIG91dFs1XSA9IGMgKiBhMTIgLSBzICogYTAyO1xuXG4gICAgb3V0WzZdID0gYTIwO1xuICAgIG91dFs3XSA9IGEyMTtcbiAgICBvdXRbOF0gPSBhMjI7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogU2NhbGVzIHRoZSBtYXQzIGJ5IHRoZSBkaW1lbnNpb25zIGluIHRoZSBnaXZlbiB2ZWMyXG4gKlxuICogQHBhcmFtIHttYXQzfSBvdXQgdGhlIHJlY2VpdmluZyBtYXRyaXhcbiAqIEBwYXJhbSB7bWF0M30gYSB0aGUgbWF0cml4IHRvIHJvdGF0ZVxuICogQHBhcmFtIHt2ZWMyfSB2IHRoZSB2ZWMyIHRvIHNjYWxlIHRoZSBtYXRyaXggYnlcbiAqIEByZXR1cm5zIHttYXQzfSBvdXRcbiAqKi9cbm1hdDMuc2NhbGUgPSBmdW5jdGlvbihvdXQsIGEsIHYpIHtcbiAgICB2YXIgeCA9IHZbMF0sIHkgPSB2WzFdO1xuXG4gICAgb3V0WzBdID0geCAqIGFbMF07XG4gICAgb3V0WzFdID0geCAqIGFbMV07XG4gICAgb3V0WzJdID0geCAqIGFbMl07XG5cbiAgICBvdXRbM10gPSB5ICogYVszXTtcbiAgICBvdXRbNF0gPSB5ICogYVs0XTtcbiAgICBvdXRbNV0gPSB5ICogYVs1XTtcblxuICAgIG91dFs2XSA9IGFbNl07XG4gICAgb3V0WzddID0gYVs3XTtcbiAgICBvdXRbOF0gPSBhWzhdO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIENvcGllcyB0aGUgdmFsdWVzIGZyb20gYSBtYXQyZCBpbnRvIGEgbWF0M1xuICpcbiAqIEBwYXJhbSB7bWF0M30gb3V0IHRoZSByZWNlaXZpbmcgbWF0cml4XG4gKiBAcGFyYW0ge21hdDJkfSBhIHRoZSBtYXRyaXggdG8gY29weVxuICogQHJldHVybnMge21hdDN9IG91dFxuICoqL1xubWF0My5mcm9tTWF0MmQgPSBmdW5jdGlvbihvdXQsIGEpIHtcbiAgICBvdXRbMF0gPSBhWzBdO1xuICAgIG91dFsxXSA9IGFbMV07XG4gICAgb3V0WzJdID0gMDtcblxuICAgIG91dFszXSA9IGFbMl07XG4gICAgb3V0WzRdID0gYVszXTtcbiAgICBvdXRbNV0gPSAwO1xuXG4gICAgb3V0WzZdID0gYVs0XTtcbiAgICBvdXRbN10gPSBhWzVdO1xuICAgIG91dFs4XSA9IDE7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuKiBDYWxjdWxhdGVzIGEgM3gzIG1hdHJpeCBmcm9tIHRoZSBnaXZlbiBxdWF0ZXJuaW9uXG4qXG4qIEBwYXJhbSB7bWF0M30gb3V0IG1hdDMgcmVjZWl2aW5nIG9wZXJhdGlvbiByZXN1bHRcbiogQHBhcmFtIHtxdWF0fSBxIFF1YXRlcm5pb24gdG8gY3JlYXRlIG1hdHJpeCBmcm9tXG4qXG4qIEByZXR1cm5zIHttYXQzfSBvdXRcbiovXG5tYXQzLmZyb21RdWF0ID0gZnVuY3Rpb24gKG91dCwgcSkge1xuICAgIHZhciB4ID0gcVswXSwgeSA9IHFbMV0sIHogPSBxWzJdLCB3ID0gcVszXSxcbiAgICAgICAgeDIgPSB4ICsgeCxcbiAgICAgICAgeTIgPSB5ICsgeSxcbiAgICAgICAgejIgPSB6ICsgeixcblxuICAgICAgICB4eCA9IHggKiB4MixcbiAgICAgICAgeXggPSB5ICogeDIsXG4gICAgICAgIHl5ID0geSAqIHkyLFxuICAgICAgICB6eCA9IHogKiB4MixcbiAgICAgICAgenkgPSB6ICogeTIsXG4gICAgICAgIHp6ID0geiAqIHoyLFxuICAgICAgICB3eCA9IHcgKiB4MixcbiAgICAgICAgd3kgPSB3ICogeTIsXG4gICAgICAgIHd6ID0gdyAqIHoyO1xuXG4gICAgb3V0WzBdID0gMSAtIHl5IC0geno7XG4gICAgb3V0WzNdID0geXggLSB3ejtcbiAgICBvdXRbNl0gPSB6eCArIHd5O1xuXG4gICAgb3V0WzFdID0geXggKyB3ejtcbiAgICBvdXRbNF0gPSAxIC0geHggLSB6ejtcbiAgICBvdXRbN10gPSB6eSAtIHd4O1xuXG4gICAgb3V0WzJdID0genggLSB3eTtcbiAgICBvdXRbNV0gPSB6eSArIHd4O1xuICAgIG91dFs4XSA9IDEgLSB4eCAtIHl5O1xuXG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuKiBDYWxjdWxhdGVzIGEgM3gzIG5vcm1hbCBtYXRyaXggKHRyYW5zcG9zZSBpbnZlcnNlKSBmcm9tIHRoZSA0eDQgbWF0cml4XG4qXG4qIEBwYXJhbSB7bWF0M30gb3V0IG1hdDMgcmVjZWl2aW5nIG9wZXJhdGlvbiByZXN1bHRcbiogQHBhcmFtIHttYXQ0fSBhIE1hdDQgdG8gZGVyaXZlIHRoZSBub3JtYWwgbWF0cml4IGZyb21cbipcbiogQHJldHVybnMge21hdDN9IG91dFxuKi9cbm1hdDMubm9ybWFsRnJvbU1hdDQgPSBmdW5jdGlvbiAob3V0LCBhKSB7XG4gICAgdmFyIGEwMCA9IGFbMF0sIGEwMSA9IGFbMV0sIGEwMiA9IGFbMl0sIGEwMyA9IGFbM10sXG4gICAgICAgIGExMCA9IGFbNF0sIGExMSA9IGFbNV0sIGExMiA9IGFbNl0sIGExMyA9IGFbN10sXG4gICAgICAgIGEyMCA9IGFbOF0sIGEyMSA9IGFbOV0sIGEyMiA9IGFbMTBdLCBhMjMgPSBhWzExXSxcbiAgICAgICAgYTMwID0gYVsxMl0sIGEzMSA9IGFbMTNdLCBhMzIgPSBhWzE0XSwgYTMzID0gYVsxNV0sXG5cbiAgICAgICAgYjAwID0gYTAwICogYTExIC0gYTAxICogYTEwLFxuICAgICAgICBiMDEgPSBhMDAgKiBhMTIgLSBhMDIgKiBhMTAsXG4gICAgICAgIGIwMiA9IGEwMCAqIGExMyAtIGEwMyAqIGExMCxcbiAgICAgICAgYjAzID0gYTAxICogYTEyIC0gYTAyICogYTExLFxuICAgICAgICBiMDQgPSBhMDEgKiBhMTMgLSBhMDMgKiBhMTEsXG4gICAgICAgIGIwNSA9IGEwMiAqIGExMyAtIGEwMyAqIGExMixcbiAgICAgICAgYjA2ID0gYTIwICogYTMxIC0gYTIxICogYTMwLFxuICAgICAgICBiMDcgPSBhMjAgKiBhMzIgLSBhMjIgKiBhMzAsXG4gICAgICAgIGIwOCA9IGEyMCAqIGEzMyAtIGEyMyAqIGEzMCxcbiAgICAgICAgYjA5ID0gYTIxICogYTMyIC0gYTIyICogYTMxLFxuICAgICAgICBiMTAgPSBhMjEgKiBhMzMgLSBhMjMgKiBhMzEsXG4gICAgICAgIGIxMSA9IGEyMiAqIGEzMyAtIGEyMyAqIGEzMixcblxuICAgICAgICAvLyBDYWxjdWxhdGUgdGhlIGRldGVybWluYW50XG4gICAgICAgIGRldCA9IGIwMCAqIGIxMSAtIGIwMSAqIGIxMCArIGIwMiAqIGIwOSArIGIwMyAqIGIwOCAtIGIwNCAqIGIwNyArIGIwNSAqIGIwNjtcblxuICAgIGlmICghZGV0KSB7IFxuICAgICAgICByZXR1cm4gbnVsbDsgXG4gICAgfVxuICAgIGRldCA9IDEuMCAvIGRldDtcblxuICAgIG91dFswXSA9IChhMTEgKiBiMTEgLSBhMTIgKiBiMTAgKyBhMTMgKiBiMDkpICogZGV0O1xuICAgIG91dFsxXSA9IChhMTIgKiBiMDggLSBhMTAgKiBiMTEgLSBhMTMgKiBiMDcpICogZGV0O1xuICAgIG91dFsyXSA9IChhMTAgKiBiMTAgLSBhMTEgKiBiMDggKyBhMTMgKiBiMDYpICogZGV0O1xuXG4gICAgb3V0WzNdID0gKGEwMiAqIGIxMCAtIGEwMSAqIGIxMSAtIGEwMyAqIGIwOSkgKiBkZXQ7XG4gICAgb3V0WzRdID0gKGEwMCAqIGIxMSAtIGEwMiAqIGIwOCArIGEwMyAqIGIwNykgKiBkZXQ7XG4gICAgb3V0WzVdID0gKGEwMSAqIGIwOCAtIGEwMCAqIGIxMCAtIGEwMyAqIGIwNikgKiBkZXQ7XG5cbiAgICBvdXRbNl0gPSAoYTMxICogYjA1IC0gYTMyICogYjA0ICsgYTMzICogYjAzKSAqIGRldDtcbiAgICBvdXRbN10gPSAoYTMyICogYjAyIC0gYTMwICogYjA1IC0gYTMzICogYjAxKSAqIGRldDtcbiAgICBvdXRbOF0gPSAoYTMwICogYjA0IC0gYTMxICogYjAyICsgYTMzICogYjAwKSAqIGRldDtcblxuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFJldHVybnMgYSBzdHJpbmcgcmVwcmVzZW50YXRpb24gb2YgYSBtYXQzXG4gKlxuICogQHBhcmFtIHttYXQzfSBtYXQgbWF0cml4IHRvIHJlcHJlc2VudCBhcyBhIHN0cmluZ1xuICogQHJldHVybnMge1N0cmluZ30gc3RyaW5nIHJlcHJlc2VudGF0aW9uIG9mIHRoZSBtYXRyaXhcbiAqL1xubWF0My5zdHIgPSBmdW5jdGlvbiAoYSkge1xuICAgIHJldHVybiAnbWF0MygnICsgYVswXSArICcsICcgKyBhWzFdICsgJywgJyArIGFbMl0gKyAnLCAnICsgXG4gICAgICAgICAgICAgICAgICAgIGFbM10gKyAnLCAnICsgYVs0XSArICcsICcgKyBhWzVdICsgJywgJyArIFxuICAgICAgICAgICAgICAgICAgICBhWzZdICsgJywgJyArIGFbN10gKyAnLCAnICsgYVs4XSArICcpJztcbn07XG5cbi8qKlxuICogUmV0dXJucyBGcm9iZW5pdXMgbm9ybSBvZiBhIG1hdDNcbiAqXG4gKiBAcGFyYW0ge21hdDN9IGEgdGhlIG1hdHJpeCB0byBjYWxjdWxhdGUgRnJvYmVuaXVzIG5vcm0gb2ZcbiAqIEByZXR1cm5zIHtOdW1iZXJ9IEZyb2Jlbml1cyBub3JtXG4gKi9cbm1hdDMuZnJvYiA9IGZ1bmN0aW9uIChhKSB7XG4gICAgcmV0dXJuKE1hdGguc3FydChNYXRoLnBvdyhhWzBdLCAyKSArIE1hdGgucG93KGFbMV0sIDIpICsgTWF0aC5wb3coYVsyXSwgMikgKyBNYXRoLnBvdyhhWzNdLCAyKSArIE1hdGgucG93KGFbNF0sIDIpICsgTWF0aC5wb3coYVs1XSwgMikgKyBNYXRoLnBvdyhhWzZdLCAyKSArIE1hdGgucG93KGFbN10sIDIpICsgTWF0aC5wb3coYVs4XSwgMikpKVxufTtcblxubW9kdWxlLmV4cG9ydHMgPSBtYXQzOyIsIi8qIENvcHlyaWdodCAoYykgMjAxMywgQnJhbmRvbiBKb25lcywgQ29saW4gTWFjS2VuemllIElWLiBBbGwgcmlnaHRzIHJlc2VydmVkLlxuXG5SZWRpc3RyaWJ1dGlvbiBhbmQgdXNlIGluIHNvdXJjZSBhbmQgYmluYXJ5IGZvcm1zLCB3aXRoIG9yIHdpdGhvdXQgbW9kaWZpY2F0aW9uLFxuYXJlIHBlcm1pdHRlZCBwcm92aWRlZCB0aGF0IHRoZSBmb2xsb3dpbmcgY29uZGl0aW9ucyBhcmUgbWV0OlxuXG4gICogUmVkaXN0cmlidXRpb25zIG9mIHNvdXJjZSBjb2RlIG11c3QgcmV0YWluIHRoZSBhYm92ZSBjb3B5cmlnaHQgbm90aWNlLCB0aGlzXG4gICAgbGlzdCBvZiBjb25kaXRpb25zIGFuZCB0aGUgZm9sbG93aW5nIGRpc2NsYWltZXIuXG4gICogUmVkaXN0cmlidXRpb25zIGluIGJpbmFyeSBmb3JtIG11c3QgcmVwcm9kdWNlIHRoZSBhYm92ZSBjb3B5cmlnaHQgbm90aWNlLFxuICAgIHRoaXMgbGlzdCBvZiBjb25kaXRpb25zIGFuZCB0aGUgZm9sbG93aW5nIGRpc2NsYWltZXIgaW4gdGhlIGRvY3VtZW50YXRpb24gXG4gICAgYW5kL29yIG90aGVyIG1hdGVyaWFscyBwcm92aWRlZCB3aXRoIHRoZSBkaXN0cmlidXRpb24uXG5cblRISVMgU09GVFdBUkUgSVMgUFJPVklERUQgQlkgVEhFIENPUFlSSUdIVCBIT0xERVJTIEFORCBDT05UUklCVVRPUlMgXCJBUyBJU1wiIEFORFxuQU5ZIEVYUFJFU1MgT1IgSU1QTElFRCBXQVJSQU5USUVTLCBJTkNMVURJTkcsIEJVVCBOT1QgTElNSVRFRCBUTywgVEhFIElNUExJRURcbldBUlJBTlRJRVMgT0YgTUVSQ0hBTlRBQklMSVRZIEFORCBGSVRORVNTIEZPUiBBIFBBUlRJQ1VMQVIgUFVSUE9TRSBBUkUgXG5ESVNDTEFJTUVELiBJTiBOTyBFVkVOVCBTSEFMTCBUSEUgQ09QWVJJR0hUIEhPTERFUiBPUiBDT05UUklCVVRPUlMgQkUgTElBQkxFIEZPUlxuQU5ZIERJUkVDVCwgSU5ESVJFQ1QsIElOQ0lERU5UQUwsIFNQRUNJQUwsIEVYRU1QTEFSWSwgT1IgQ09OU0VRVUVOVElBTCBEQU1BR0VTXG4oSU5DTFVESU5HLCBCVVQgTk9UIExJTUlURUQgVE8sIFBST0NVUkVNRU5UIE9GIFNVQlNUSVRVVEUgR09PRFMgT1IgU0VSVklDRVM7XG5MT1NTIE9GIFVTRSwgREFUQSwgT1IgUFJPRklUUzsgT1IgQlVTSU5FU1MgSU5URVJSVVBUSU9OKSBIT1dFVkVSIENBVVNFRCBBTkQgT05cbkFOWSBUSEVPUlkgT0YgTElBQklMSVRZLCBXSEVUSEVSIElOIENPTlRSQUNULCBTVFJJQ1QgTElBQklMSVRZLCBPUiBUT1JUXG4oSU5DTFVESU5HIE5FR0xJR0VOQ0UgT1IgT1RIRVJXSVNFKSBBUklTSU5HIElOIEFOWSBXQVkgT1VUIE9GIFRIRSBVU0UgT0YgVEhJU1xuU09GVFdBUkUsIEVWRU4gSUYgQURWSVNFRCBPRiBUSEUgUE9TU0lCSUxJVFkgT0YgU1VDSCBEQU1BR0UuICovXG5cbi8qKlxuICogQGNsYXNzIDQgRGltZW5zaW9uYWwgVmVjdG9yXG4gKiBAbmFtZSB2ZWM0XG4gKi9cbnZhciB2ZWM0ID0ge307XG5cbnZhciBHTE1BVF9BUlJBWV9UWVBFID0gcmVxdWlyZSgnY29tbW9uJykuR0xNQVRfQVJSQVlfVFlQRTtcbnZhciBHTE1BVF9SQU5ET00gPSByZXF1aXJlKCdjb21tb24nKS5HTE1BVF9SQU5ET007XG5cbi8qKlxuICogQ3JlYXRlcyBhIG5ldywgZW1wdHkgdmVjNFxuICpcbiAqIEByZXR1cm5zIHt2ZWM0fSBhIG5ldyA0RCB2ZWN0b3JcbiAqL1xudmVjNC5jcmVhdGUgPSBmdW5jdGlvbigpIHtcbiAgICB2YXIgb3V0ID0gbmV3IEdMTUFUX0FSUkFZX1RZUEUoNCk7XG4gICAgb3V0WzBdID0gMDtcbiAgICBvdXRbMV0gPSAwO1xuICAgIG91dFsyXSA9IDA7XG4gICAgb3V0WzNdID0gMDtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBDcmVhdGVzIGEgbmV3IHZlYzQgaW5pdGlhbGl6ZWQgd2l0aCB2YWx1ZXMgZnJvbSBhbiBleGlzdGluZyB2ZWN0b3JcbiAqXG4gKiBAcGFyYW0ge3ZlYzR9IGEgdmVjdG9yIHRvIGNsb25lXG4gKiBAcmV0dXJucyB7dmVjNH0gYSBuZXcgNEQgdmVjdG9yXG4gKi9cbnZlYzQuY2xvbmUgPSBmdW5jdGlvbihhKSB7XG4gICAgdmFyIG91dCA9IG5ldyBHTE1BVF9BUlJBWV9UWVBFKDQpO1xuICAgIG91dFswXSA9IGFbMF07XG4gICAgb3V0WzFdID0gYVsxXTtcbiAgICBvdXRbMl0gPSBhWzJdO1xuICAgIG91dFszXSA9IGFbM107XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQ3JlYXRlcyBhIG5ldyB2ZWM0IGluaXRpYWxpemVkIHdpdGggdGhlIGdpdmVuIHZhbHVlc1xuICpcbiAqIEBwYXJhbSB7TnVtYmVyfSB4IFggY29tcG9uZW50XG4gKiBAcGFyYW0ge051bWJlcn0geSBZIGNvbXBvbmVudFxuICogQHBhcmFtIHtOdW1iZXJ9IHogWiBjb21wb25lbnRcbiAqIEBwYXJhbSB7TnVtYmVyfSB3IFcgY29tcG9uZW50XG4gKiBAcmV0dXJucyB7dmVjNH0gYSBuZXcgNEQgdmVjdG9yXG4gKi9cbnZlYzQuZnJvbVZhbHVlcyA9IGZ1bmN0aW9uKHgsIHksIHosIHcpIHtcbiAgICB2YXIgb3V0ID0gbmV3IEdMTUFUX0FSUkFZX1RZUEUoNCk7XG4gICAgb3V0WzBdID0geDtcbiAgICBvdXRbMV0gPSB5O1xuICAgIG91dFsyXSA9IHo7XG4gICAgb3V0WzNdID0gdztcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBDb3B5IHRoZSB2YWx1ZXMgZnJvbSBvbmUgdmVjNCB0byBhbm90aGVyXG4gKlxuICogQHBhcmFtIHt2ZWM0fSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjNH0gYSB0aGUgc291cmNlIHZlY3RvclxuICogQHJldHVybnMge3ZlYzR9IG91dFxuICovXG52ZWM0LmNvcHkgPSBmdW5jdGlvbihvdXQsIGEpIHtcbiAgICBvdXRbMF0gPSBhWzBdO1xuICAgIG91dFsxXSA9IGFbMV07XG4gICAgb3V0WzJdID0gYVsyXTtcbiAgICBvdXRbM10gPSBhWzNdO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFNldCB0aGUgY29tcG9uZW50cyBvZiBhIHZlYzQgdG8gdGhlIGdpdmVuIHZhbHVlc1xuICpcbiAqIEBwYXJhbSB7dmVjNH0gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge051bWJlcn0geCBYIGNvbXBvbmVudFxuICogQHBhcmFtIHtOdW1iZXJ9IHkgWSBjb21wb25lbnRcbiAqIEBwYXJhbSB7TnVtYmVyfSB6IFogY29tcG9uZW50XG4gKiBAcGFyYW0ge051bWJlcn0gdyBXIGNvbXBvbmVudFxuICogQHJldHVybnMge3ZlYzR9IG91dFxuICovXG52ZWM0LnNldCA9IGZ1bmN0aW9uKG91dCwgeCwgeSwgeiwgdykge1xuICAgIG91dFswXSA9IHg7XG4gICAgb3V0WzFdID0geTtcbiAgICBvdXRbMl0gPSB6O1xuICAgIG91dFszXSA9IHc7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQWRkcyB0d28gdmVjNCdzXG4gKlxuICogQHBhcmFtIHt2ZWM0fSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjNH0gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHt2ZWM0fSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge3ZlYzR9IG91dFxuICovXG52ZWM0LmFkZCA9IGZ1bmN0aW9uKG91dCwgYSwgYikge1xuICAgIG91dFswXSA9IGFbMF0gKyBiWzBdO1xuICAgIG91dFsxXSA9IGFbMV0gKyBiWzFdO1xuICAgIG91dFsyXSA9IGFbMl0gKyBiWzJdO1xuICAgIG91dFszXSA9IGFbM10gKyBiWzNdO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFN1YnRyYWN0cyB2ZWN0b3IgYiBmcm9tIHZlY3RvciBhXG4gKlxuICogQHBhcmFtIHt2ZWM0fSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjNH0gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHt2ZWM0fSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge3ZlYzR9IG91dFxuICovXG52ZWM0LnN1YnRyYWN0ID0gZnVuY3Rpb24ob3V0LCBhLCBiKSB7XG4gICAgb3V0WzBdID0gYVswXSAtIGJbMF07XG4gICAgb3V0WzFdID0gYVsxXSAtIGJbMV07XG4gICAgb3V0WzJdID0gYVsyXSAtIGJbMl07XG4gICAgb3V0WzNdID0gYVszXSAtIGJbM107XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQWxpYXMgZm9yIHtAbGluayB2ZWM0LnN1YnRyYWN0fVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzQuc3ViID0gdmVjNC5zdWJ0cmFjdDtcblxuLyoqXG4gKiBNdWx0aXBsaWVzIHR3byB2ZWM0J3NcbiAqXG4gKiBAcGFyYW0ge3ZlYzR9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHt2ZWM0fSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3ZlYzR9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcmV0dXJucyB7dmVjNH0gb3V0XG4gKi9cbnZlYzQubXVsdGlwbHkgPSBmdW5jdGlvbihvdXQsIGEsIGIpIHtcbiAgICBvdXRbMF0gPSBhWzBdICogYlswXTtcbiAgICBvdXRbMV0gPSBhWzFdICogYlsxXTtcbiAgICBvdXRbMl0gPSBhWzJdICogYlsyXTtcbiAgICBvdXRbM10gPSBhWzNdICogYlszXTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBBbGlhcyBmb3Ige0BsaW5rIHZlYzQubXVsdGlwbHl9XG4gKiBAZnVuY3Rpb25cbiAqL1xudmVjNC5tdWwgPSB2ZWM0Lm11bHRpcGx5O1xuXG4vKipcbiAqIERpdmlkZXMgdHdvIHZlYzQnc1xuICpcbiAqIEBwYXJhbSB7dmVjNH0gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzR9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjNH0gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEByZXR1cm5zIHt2ZWM0fSBvdXRcbiAqL1xudmVjNC5kaXZpZGUgPSBmdW5jdGlvbihvdXQsIGEsIGIpIHtcbiAgICBvdXRbMF0gPSBhWzBdIC8gYlswXTtcbiAgICBvdXRbMV0gPSBhWzFdIC8gYlsxXTtcbiAgICBvdXRbMl0gPSBhWzJdIC8gYlsyXTtcbiAgICBvdXRbM10gPSBhWzNdIC8gYlszXTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBBbGlhcyBmb3Ige0BsaW5rIHZlYzQuZGl2aWRlfVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzQuZGl2ID0gdmVjNC5kaXZpZGU7XG5cbi8qKlxuICogUmV0dXJucyB0aGUgbWluaW11bSBvZiB0d28gdmVjNCdzXG4gKlxuICogQHBhcmFtIHt2ZWM0fSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjNH0gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHt2ZWM0fSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge3ZlYzR9IG91dFxuICovXG52ZWM0Lm1pbiA9IGZ1bmN0aW9uKG91dCwgYSwgYikge1xuICAgIG91dFswXSA9IE1hdGgubWluKGFbMF0sIGJbMF0pO1xuICAgIG91dFsxXSA9IE1hdGgubWluKGFbMV0sIGJbMV0pO1xuICAgIG91dFsyXSA9IE1hdGgubWluKGFbMl0sIGJbMl0pO1xuICAgIG91dFszXSA9IE1hdGgubWluKGFbM10sIGJbM10pO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFJldHVybnMgdGhlIG1heGltdW0gb2YgdHdvIHZlYzQnc1xuICpcbiAqIEBwYXJhbSB7dmVjNH0gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzR9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjNH0gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEByZXR1cm5zIHt2ZWM0fSBvdXRcbiAqL1xudmVjNC5tYXggPSBmdW5jdGlvbihvdXQsIGEsIGIpIHtcbiAgICBvdXRbMF0gPSBNYXRoLm1heChhWzBdLCBiWzBdKTtcbiAgICBvdXRbMV0gPSBNYXRoLm1heChhWzFdLCBiWzFdKTtcbiAgICBvdXRbMl0gPSBNYXRoLm1heChhWzJdLCBiWzJdKTtcbiAgICBvdXRbM10gPSBNYXRoLm1heChhWzNdLCBiWzNdKTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBTY2FsZXMgYSB2ZWM0IGJ5IGEgc2NhbGFyIG51bWJlclxuICpcbiAqIEBwYXJhbSB7dmVjNH0gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzR9IGEgdGhlIHZlY3RvciB0byBzY2FsZVxuICogQHBhcmFtIHtOdW1iZXJ9IGIgYW1vdW50IHRvIHNjYWxlIHRoZSB2ZWN0b3IgYnlcbiAqIEByZXR1cm5zIHt2ZWM0fSBvdXRcbiAqL1xudmVjNC5zY2FsZSA9IGZ1bmN0aW9uKG91dCwgYSwgYikge1xuICAgIG91dFswXSA9IGFbMF0gKiBiO1xuICAgIG91dFsxXSA9IGFbMV0gKiBiO1xuICAgIG91dFsyXSA9IGFbMl0gKiBiO1xuICAgIG91dFszXSA9IGFbM10gKiBiO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIEFkZHMgdHdvIHZlYzQncyBhZnRlciBzY2FsaW5nIHRoZSBzZWNvbmQgb3BlcmFuZCBieSBhIHNjYWxhciB2YWx1ZVxuICpcbiAqIEBwYXJhbSB7dmVjNH0gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzR9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjNH0gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEBwYXJhbSB7TnVtYmVyfSBzY2FsZSB0aGUgYW1vdW50IHRvIHNjYWxlIGIgYnkgYmVmb3JlIGFkZGluZ1xuICogQHJldHVybnMge3ZlYzR9IG91dFxuICovXG52ZWM0LnNjYWxlQW5kQWRkID0gZnVuY3Rpb24ob3V0LCBhLCBiLCBzY2FsZSkge1xuICAgIG91dFswXSA9IGFbMF0gKyAoYlswXSAqIHNjYWxlKTtcbiAgICBvdXRbMV0gPSBhWzFdICsgKGJbMV0gKiBzY2FsZSk7XG4gICAgb3V0WzJdID0gYVsyXSArIChiWzJdICogc2NhbGUpO1xuICAgIG91dFszXSA9IGFbM10gKyAoYlszXSAqIHNjYWxlKTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBDYWxjdWxhdGVzIHRoZSBldWNsaWRpYW4gZGlzdGFuY2UgYmV0d2VlbiB0d28gdmVjNCdzXG4gKlxuICogQHBhcmFtIHt2ZWM0fSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3ZlYzR9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcmV0dXJucyB7TnVtYmVyfSBkaXN0YW5jZSBiZXR3ZWVuIGEgYW5kIGJcbiAqL1xudmVjNC5kaXN0YW5jZSA9IGZ1bmN0aW9uKGEsIGIpIHtcbiAgICB2YXIgeCA9IGJbMF0gLSBhWzBdLFxuICAgICAgICB5ID0gYlsxXSAtIGFbMV0sXG4gICAgICAgIHogPSBiWzJdIC0gYVsyXSxcbiAgICAgICAgdyA9IGJbM10gLSBhWzNdO1xuICAgIHJldHVybiBNYXRoLnNxcnQoeCp4ICsgeSp5ICsgeip6ICsgdyp3KTtcbn07XG5cbi8qKlxuICogQWxpYXMgZm9yIHtAbGluayB2ZWM0LmRpc3RhbmNlfVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzQuZGlzdCA9IHZlYzQuZGlzdGFuY2U7XG5cbi8qKlxuICogQ2FsY3VsYXRlcyB0aGUgc3F1YXJlZCBldWNsaWRpYW4gZGlzdGFuY2UgYmV0d2VlbiB0d28gdmVjNCdzXG4gKlxuICogQHBhcmFtIHt2ZWM0fSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3ZlYzR9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcmV0dXJucyB7TnVtYmVyfSBzcXVhcmVkIGRpc3RhbmNlIGJldHdlZW4gYSBhbmQgYlxuICovXG52ZWM0LnNxdWFyZWREaXN0YW5jZSA9IGZ1bmN0aW9uKGEsIGIpIHtcbiAgICB2YXIgeCA9IGJbMF0gLSBhWzBdLFxuICAgICAgICB5ID0gYlsxXSAtIGFbMV0sXG4gICAgICAgIHogPSBiWzJdIC0gYVsyXSxcbiAgICAgICAgdyA9IGJbM10gLSBhWzNdO1xuICAgIHJldHVybiB4KnggKyB5KnkgKyB6KnogKyB3Knc7XG59O1xuXG4vKipcbiAqIEFsaWFzIGZvciB7QGxpbmsgdmVjNC5zcXVhcmVkRGlzdGFuY2V9XG4gKiBAZnVuY3Rpb25cbiAqL1xudmVjNC5zcXJEaXN0ID0gdmVjNC5zcXVhcmVkRGlzdGFuY2U7XG5cbi8qKlxuICogQ2FsY3VsYXRlcyB0aGUgbGVuZ3RoIG9mIGEgdmVjNFxuICpcbiAqIEBwYXJhbSB7dmVjNH0gYSB2ZWN0b3IgdG8gY2FsY3VsYXRlIGxlbmd0aCBvZlxuICogQHJldHVybnMge051bWJlcn0gbGVuZ3RoIG9mIGFcbiAqL1xudmVjNC5sZW5ndGggPSBmdW5jdGlvbiAoYSkge1xuICAgIHZhciB4ID0gYVswXSxcbiAgICAgICAgeSA9IGFbMV0sXG4gICAgICAgIHogPSBhWzJdLFxuICAgICAgICB3ID0gYVszXTtcbiAgICByZXR1cm4gTWF0aC5zcXJ0KHgqeCArIHkqeSArIHoqeiArIHcqdyk7XG59O1xuXG4vKipcbiAqIEFsaWFzIGZvciB7QGxpbmsgdmVjNC5sZW5ndGh9XG4gKiBAZnVuY3Rpb25cbiAqL1xudmVjNC5sZW4gPSB2ZWM0Lmxlbmd0aDtcblxuLyoqXG4gKiBDYWxjdWxhdGVzIHRoZSBzcXVhcmVkIGxlbmd0aCBvZiBhIHZlYzRcbiAqXG4gKiBAcGFyYW0ge3ZlYzR9IGEgdmVjdG9yIHRvIGNhbGN1bGF0ZSBzcXVhcmVkIGxlbmd0aCBvZlxuICogQHJldHVybnMge051bWJlcn0gc3F1YXJlZCBsZW5ndGggb2YgYVxuICovXG52ZWM0LnNxdWFyZWRMZW5ndGggPSBmdW5jdGlvbiAoYSkge1xuICAgIHZhciB4ID0gYVswXSxcbiAgICAgICAgeSA9IGFbMV0sXG4gICAgICAgIHogPSBhWzJdLFxuICAgICAgICB3ID0gYVszXTtcbiAgICByZXR1cm4geCp4ICsgeSp5ICsgeip6ICsgdyp3O1xufTtcblxuLyoqXG4gKiBBbGlhcyBmb3Ige0BsaW5rIHZlYzQuc3F1YXJlZExlbmd0aH1cbiAqIEBmdW5jdGlvblxuICovXG52ZWM0LnNxckxlbiA9IHZlYzQuc3F1YXJlZExlbmd0aDtcblxuLyoqXG4gKiBOZWdhdGVzIHRoZSBjb21wb25lbnRzIG9mIGEgdmVjNFxuICpcbiAqIEBwYXJhbSB7dmVjNH0gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzR9IGEgdmVjdG9yIHRvIG5lZ2F0ZVxuICogQHJldHVybnMge3ZlYzR9IG91dFxuICovXG52ZWM0Lm5lZ2F0ZSA9IGZ1bmN0aW9uKG91dCwgYSkge1xuICAgIG91dFswXSA9IC1hWzBdO1xuICAgIG91dFsxXSA9IC1hWzFdO1xuICAgIG91dFsyXSA9IC1hWzJdO1xuICAgIG91dFszXSA9IC1hWzNdO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIE5vcm1hbGl6ZSBhIHZlYzRcbiAqXG4gKiBAcGFyYW0ge3ZlYzR9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHt2ZWM0fSBhIHZlY3RvciB0byBub3JtYWxpemVcbiAqIEByZXR1cm5zIHt2ZWM0fSBvdXRcbiAqL1xudmVjNC5ub3JtYWxpemUgPSBmdW5jdGlvbihvdXQsIGEpIHtcbiAgICB2YXIgeCA9IGFbMF0sXG4gICAgICAgIHkgPSBhWzFdLFxuICAgICAgICB6ID0gYVsyXSxcbiAgICAgICAgdyA9IGFbM107XG4gICAgdmFyIGxlbiA9IHgqeCArIHkqeSArIHoqeiArIHcqdztcbiAgICBpZiAobGVuID4gMCkge1xuICAgICAgICBsZW4gPSAxIC8gTWF0aC5zcXJ0KGxlbik7XG4gICAgICAgIG91dFswXSA9IGFbMF0gKiBsZW47XG4gICAgICAgIG91dFsxXSA9IGFbMV0gKiBsZW47XG4gICAgICAgIG91dFsyXSA9IGFbMl0gKiBsZW47XG4gICAgICAgIG91dFszXSA9IGFbM10gKiBsZW47XG4gICAgfVxuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIENhbGN1bGF0ZXMgdGhlIGRvdCBwcm9kdWN0IG9mIHR3byB2ZWM0J3NcbiAqXG4gKiBAcGFyYW0ge3ZlYzR9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjNH0gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEByZXR1cm5zIHtOdW1iZXJ9IGRvdCBwcm9kdWN0IG9mIGEgYW5kIGJcbiAqL1xudmVjNC5kb3QgPSBmdW5jdGlvbiAoYSwgYikge1xuICAgIHJldHVybiBhWzBdICogYlswXSArIGFbMV0gKiBiWzFdICsgYVsyXSAqIGJbMl0gKyBhWzNdICogYlszXTtcbn07XG5cbi8qKlxuICogUGVyZm9ybXMgYSBsaW5lYXIgaW50ZXJwb2xhdGlvbiBiZXR3ZWVuIHR3byB2ZWM0J3NcbiAqXG4gKiBAcGFyYW0ge3ZlYzR9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHt2ZWM0fSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3ZlYzR9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcGFyYW0ge051bWJlcn0gdCBpbnRlcnBvbGF0aW9uIGFtb3VudCBiZXR3ZWVuIHRoZSB0d28gaW5wdXRzXG4gKiBAcmV0dXJucyB7dmVjNH0gb3V0XG4gKi9cbnZlYzQubGVycCA9IGZ1bmN0aW9uIChvdXQsIGEsIGIsIHQpIHtcbiAgICB2YXIgYXggPSBhWzBdLFxuICAgICAgICBheSA9IGFbMV0sXG4gICAgICAgIGF6ID0gYVsyXSxcbiAgICAgICAgYXcgPSBhWzNdO1xuICAgIG91dFswXSA9IGF4ICsgdCAqIChiWzBdIC0gYXgpO1xuICAgIG91dFsxXSA9IGF5ICsgdCAqIChiWzFdIC0gYXkpO1xuICAgIG91dFsyXSA9IGF6ICsgdCAqIChiWzJdIC0gYXopO1xuICAgIG91dFszXSA9IGF3ICsgdCAqIChiWzNdIC0gYXcpO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIEdlbmVyYXRlcyBhIHJhbmRvbSB2ZWN0b3Igd2l0aCB0aGUgZ2l2ZW4gc2NhbGVcbiAqXG4gKiBAcGFyYW0ge3ZlYzR9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHtOdW1iZXJ9IFtzY2FsZV0gTGVuZ3RoIG9mIHRoZSByZXN1bHRpbmcgdmVjdG9yLiBJZiBvbW1pdHRlZCwgYSB1bml0IHZlY3RvciB3aWxsIGJlIHJldHVybmVkXG4gKiBAcmV0dXJucyB7dmVjNH0gb3V0XG4gKi9cbnZlYzQucmFuZG9tID0gZnVuY3Rpb24gKG91dCwgc2NhbGUpIHtcbiAgICBzY2FsZSA9IHNjYWxlIHx8IDEuMDtcblxuICAgIC8vVE9ETzogVGhpcyBpcyBhIHByZXR0eSBhd2Z1bCB3YXkgb2YgZG9pbmcgdGhpcy4gRmluZCBzb21ldGhpbmcgYmV0dGVyLlxuICAgIG91dFswXSA9IEdMTUFUX1JBTkRPTSgpO1xuICAgIG91dFsxXSA9IEdMTUFUX1JBTkRPTSgpO1xuICAgIG91dFsyXSA9IEdMTUFUX1JBTkRPTSgpO1xuICAgIG91dFszXSA9IEdMTUFUX1JBTkRPTSgpO1xuICAgIHZlYzQubm9ybWFsaXplKG91dCwgb3V0KTtcbiAgICB2ZWM0LnNjYWxlKG91dCwgb3V0LCBzY2FsZSk7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogVHJhbnNmb3JtcyB0aGUgdmVjNCB3aXRoIGEgbWF0NC5cbiAqXG4gKiBAcGFyYW0ge3ZlYzR9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHt2ZWM0fSBhIHRoZSB2ZWN0b3IgdG8gdHJhbnNmb3JtXG4gKiBAcGFyYW0ge21hdDR9IG0gbWF0cml4IHRvIHRyYW5zZm9ybSB3aXRoXG4gKiBAcmV0dXJucyB7dmVjNH0gb3V0XG4gKi9cbnZlYzQudHJhbnNmb3JtTWF0NCA9IGZ1bmN0aW9uKG91dCwgYSwgbSkge1xuICAgIHZhciB4ID0gYVswXSwgeSA9IGFbMV0sIHogPSBhWzJdLCB3ID0gYVszXTtcbiAgICBvdXRbMF0gPSBtWzBdICogeCArIG1bNF0gKiB5ICsgbVs4XSAqIHogKyBtWzEyXSAqIHc7XG4gICAgb3V0WzFdID0gbVsxXSAqIHggKyBtWzVdICogeSArIG1bOV0gKiB6ICsgbVsxM10gKiB3O1xuICAgIG91dFsyXSA9IG1bMl0gKiB4ICsgbVs2XSAqIHkgKyBtWzEwXSAqIHogKyBtWzE0XSAqIHc7XG4gICAgb3V0WzNdID0gbVszXSAqIHggKyBtWzddICogeSArIG1bMTFdICogeiArIG1bMTVdICogdztcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBUcmFuc2Zvcm1zIHRoZSB2ZWM0IHdpdGggYSBxdWF0XG4gKlxuICogQHBhcmFtIHt2ZWM0fSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjNH0gYSB0aGUgdmVjdG9yIHRvIHRyYW5zZm9ybVxuICogQHBhcmFtIHtxdWF0fSBxIHF1YXRlcm5pb24gdG8gdHJhbnNmb3JtIHdpdGhcbiAqIEByZXR1cm5zIHt2ZWM0fSBvdXRcbiAqL1xudmVjNC50cmFuc2Zvcm1RdWF0ID0gZnVuY3Rpb24ob3V0LCBhLCBxKSB7XG4gICAgdmFyIHggPSBhWzBdLCB5ID0gYVsxXSwgeiA9IGFbMl0sXG4gICAgICAgIHF4ID0gcVswXSwgcXkgPSBxWzFdLCBxeiA9IHFbMl0sIHF3ID0gcVszXSxcblxuICAgICAgICAvLyBjYWxjdWxhdGUgcXVhdCAqIHZlY1xuICAgICAgICBpeCA9IHF3ICogeCArIHF5ICogeiAtIHF6ICogeSxcbiAgICAgICAgaXkgPSBxdyAqIHkgKyBxeiAqIHggLSBxeCAqIHosXG4gICAgICAgIGl6ID0gcXcgKiB6ICsgcXggKiB5IC0gcXkgKiB4LFxuICAgICAgICBpdyA9IC1xeCAqIHggLSBxeSAqIHkgLSBxeiAqIHo7XG5cbiAgICAvLyBjYWxjdWxhdGUgcmVzdWx0ICogaW52ZXJzZSBxdWF0XG4gICAgb3V0WzBdID0gaXggKiBxdyArIGl3ICogLXF4ICsgaXkgKiAtcXogLSBpeiAqIC1xeTtcbiAgICBvdXRbMV0gPSBpeSAqIHF3ICsgaXcgKiAtcXkgKyBpeiAqIC1xeCAtIGl4ICogLXF6O1xuICAgIG91dFsyXSA9IGl6ICogcXcgKyBpdyAqIC1xeiArIGl4ICogLXF5IC0gaXkgKiAtcXg7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogUGVyZm9ybSBzb21lIG9wZXJhdGlvbiBvdmVyIGFuIGFycmF5IG9mIHZlYzRzLlxuICpcbiAqIEBwYXJhbSB7QXJyYXl9IGEgdGhlIGFycmF5IG9mIHZlY3RvcnMgdG8gaXRlcmF0ZSBvdmVyXG4gKiBAcGFyYW0ge051bWJlcn0gc3RyaWRlIE51bWJlciBvZiBlbGVtZW50cyBiZXR3ZWVuIHRoZSBzdGFydCBvZiBlYWNoIHZlYzQuIElmIDAgYXNzdW1lcyB0aWdodGx5IHBhY2tlZFxuICogQHBhcmFtIHtOdW1iZXJ9IG9mZnNldCBOdW1iZXIgb2YgZWxlbWVudHMgdG8gc2tpcCBhdCB0aGUgYmVnaW5uaW5nIG9mIHRoZSBhcnJheVxuICogQHBhcmFtIHtOdW1iZXJ9IGNvdW50IE51bWJlciBvZiB2ZWMycyB0byBpdGVyYXRlIG92ZXIuIElmIDAgaXRlcmF0ZXMgb3ZlciBlbnRpcmUgYXJyYXlcbiAqIEBwYXJhbSB7RnVuY3Rpb259IGZuIEZ1bmN0aW9uIHRvIGNhbGwgZm9yIGVhY2ggdmVjdG9yIGluIHRoZSBhcnJheVxuICogQHBhcmFtIHtPYmplY3R9IFthcmddIGFkZGl0aW9uYWwgYXJndW1lbnQgdG8gcGFzcyB0byBmblxuICogQHJldHVybnMge0FycmF5fSBhXG4gKiBAZnVuY3Rpb25cbiAqL1xudmVjNC5mb3JFYWNoID0gKGZ1bmN0aW9uKCkge1xuICAgIHZhciB2ZWMgPSB2ZWM0LmNyZWF0ZSgpO1xuXG4gICAgcmV0dXJuIGZ1bmN0aW9uKGEsIHN0cmlkZSwgb2Zmc2V0LCBjb3VudCwgZm4sIGFyZykge1xuICAgICAgICB2YXIgaSwgbDtcbiAgICAgICAgaWYoIXN0cmlkZSkge1xuICAgICAgICAgICAgc3RyaWRlID0gNDtcbiAgICAgICAgfVxuXG4gICAgICAgIGlmKCFvZmZzZXQpIHtcbiAgICAgICAgICAgIG9mZnNldCA9IDA7XG4gICAgICAgIH1cbiAgICAgICAgXG4gICAgICAgIGlmKGNvdW50KSB7XG4gICAgICAgICAgICBsID0gTWF0aC5taW4oKGNvdW50ICogc3RyaWRlKSArIG9mZnNldCwgYS5sZW5ndGgpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgbCA9IGEubGVuZ3RoO1xuICAgICAgICB9XG5cbiAgICAgICAgZm9yKGkgPSBvZmZzZXQ7IGkgPCBsOyBpICs9IHN0cmlkZSkge1xuICAgICAgICAgICAgdmVjWzBdID0gYVtpXTsgdmVjWzFdID0gYVtpKzFdOyB2ZWNbMl0gPSBhW2krMl07IHZlY1szXSA9IGFbaSszXTtcbiAgICAgICAgICAgIGZuKHZlYywgdmVjLCBhcmcpO1xuICAgICAgICAgICAgYVtpXSA9IHZlY1swXTsgYVtpKzFdID0gdmVjWzFdOyBhW2krMl0gPSB2ZWNbMl07IGFbaSszXSA9IHZlY1szXTtcbiAgICAgICAgfVxuICAgICAgICBcbiAgICAgICAgcmV0dXJuIGE7XG4gICAgfTtcbn0pKCk7XG5cbi8qKlxuICogUmV0dXJucyBhIHN0cmluZyByZXByZXNlbnRhdGlvbiBvZiBhIHZlY3RvclxuICpcbiAqIEBwYXJhbSB7dmVjNH0gdmVjIHZlY3RvciB0byByZXByZXNlbnQgYXMgYSBzdHJpbmdcbiAqIEByZXR1cm5zIHtTdHJpbmd9IHN0cmluZyByZXByZXNlbnRhdGlvbiBvZiB0aGUgdmVjdG9yXG4gKi9cbnZlYzQuc3RyID0gZnVuY3Rpb24gKGEpIHtcbiAgICByZXR1cm4gJ3ZlYzQoJyArIGFbMF0gKyAnLCAnICsgYVsxXSArICcsICcgKyBhWzJdICsgJywgJyArIGFbM10gKyAnKSc7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IHZlYzQ7XG5cbiIsIi8qIENvcHlyaWdodCAoYykgMjAxMywgQnJhbmRvbiBKb25lcywgQ29saW4gTWFjS2VuemllIElWLiBBbGwgcmlnaHRzIHJlc2VydmVkLlxuXG5SZWRpc3RyaWJ1dGlvbiBhbmQgdXNlIGluIHNvdXJjZSBhbmQgYmluYXJ5IGZvcm1zLCB3aXRoIG9yIHdpdGhvdXQgbW9kaWZpY2F0aW9uLFxuYXJlIHBlcm1pdHRlZCBwcm92aWRlZCB0aGF0IHRoZSBmb2xsb3dpbmcgY29uZGl0aW9ucyBhcmUgbWV0OlxuXG4gICogUmVkaXN0cmlidXRpb25zIG9mIHNvdXJjZSBjb2RlIG11c3QgcmV0YWluIHRoZSBhYm92ZSBjb3B5cmlnaHQgbm90aWNlLCB0aGlzXG4gICAgbGlzdCBvZiBjb25kaXRpb25zIGFuZCB0aGUgZm9sbG93aW5nIGRpc2NsYWltZXIuXG4gICogUmVkaXN0cmlidXRpb25zIGluIGJpbmFyeSBmb3JtIG11c3QgcmVwcm9kdWNlIHRoZSBhYm92ZSBjb3B5cmlnaHQgbm90aWNlLFxuICAgIHRoaXMgbGlzdCBvZiBjb25kaXRpb25zIGFuZCB0aGUgZm9sbG93aW5nIGRpc2NsYWltZXIgaW4gdGhlIGRvY3VtZW50YXRpb24gXG4gICAgYW5kL29yIG90aGVyIG1hdGVyaWFscyBwcm92aWRlZCB3aXRoIHRoZSBkaXN0cmlidXRpb24uXG5cblRISVMgU09GVFdBUkUgSVMgUFJPVklERUQgQlkgVEhFIENPUFlSSUdIVCBIT0xERVJTIEFORCBDT05UUklCVVRPUlMgXCJBUyBJU1wiIEFORFxuQU5ZIEVYUFJFU1MgT1IgSU1QTElFRCBXQVJSQU5USUVTLCBJTkNMVURJTkcsIEJVVCBOT1QgTElNSVRFRCBUTywgVEhFIElNUExJRURcbldBUlJBTlRJRVMgT0YgTUVSQ0hBTlRBQklMSVRZIEFORCBGSVRORVNTIEZPUiBBIFBBUlRJQ1VMQVIgUFVSUE9TRSBBUkUgXG5ESVNDTEFJTUVELiBJTiBOTyBFVkVOVCBTSEFMTCBUSEUgQ09QWVJJR0hUIEhPTERFUiBPUiBDT05UUklCVVRPUlMgQkUgTElBQkxFIEZPUlxuQU5ZIERJUkVDVCwgSU5ESVJFQ1QsIElOQ0lERU5UQUwsIFNQRUNJQUwsIEVYRU1QTEFSWSwgT1IgQ09OU0VRVUVOVElBTCBEQU1BR0VTXG4oSU5DTFVESU5HLCBCVVQgTk9UIExJTUlURUQgVE8sIFBST0NVUkVNRU5UIE9GIFNVQlNUSVRVVEUgR09PRFMgT1IgU0VSVklDRVM7XG5MT1NTIE9GIFVTRSwgREFUQSwgT1IgUFJPRklUUzsgT1IgQlVTSU5FU1MgSU5URVJSVVBUSU9OKSBIT1dFVkVSIENBVVNFRCBBTkQgT05cbkFOWSBUSEVPUlkgT0YgTElBQklMSVRZLCBXSEVUSEVSIElOIENPTlRSQUNULCBTVFJJQ1QgTElBQklMSVRZLCBPUiBUT1JUXG4oSU5DTFVESU5HIE5FR0xJR0VOQ0UgT1IgT1RIRVJXSVNFKSBBUklTSU5HIElOIEFOWSBXQVkgT1VUIE9GIFRIRSBVU0UgT0YgVEhJU1xuU09GVFdBUkUsIEVWRU4gSUYgQURWSVNFRCBPRiBUSEUgUE9TU0lCSUxJVFkgT0YgU1VDSCBEQU1BR0UuICovXG5cbi8qKlxuICogQGNsYXNzIDMgRGltZW5zaW9uYWwgVmVjdG9yXG4gKiBAbmFtZSB2ZWMzXG4gKi9cbnZhciB2ZWMzID0ge307XG5cbnZhciBHTE1BVF9BUlJBWV9UWVBFID0gcmVxdWlyZSgnY29tbW9uJykuR0xNQVRfQVJSQVlfVFlQRTtcbnZhciBHTE1BVF9SQU5ET00gPSByZXF1aXJlKCdjb21tb24nKS5HTE1BVF9SQU5ET007XG5cbi8qKlxuICogQ3JlYXRlcyBhIG5ldywgZW1wdHkgdmVjM1xuICpcbiAqIEByZXR1cm5zIHt2ZWMzfSBhIG5ldyAzRCB2ZWN0b3JcbiAqL1xudmVjMy5jcmVhdGUgPSBmdW5jdGlvbigpIHtcbiAgICB2YXIgb3V0ID0gbmV3IEdMTUFUX0FSUkFZX1RZUEUoMyk7XG4gICAgb3V0WzBdID0gMDtcbiAgICBvdXRbMV0gPSAwO1xuICAgIG91dFsyXSA9IDA7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQ3JlYXRlcyBhIG5ldyB2ZWMzIGluaXRpYWxpemVkIHdpdGggdmFsdWVzIGZyb20gYW4gZXhpc3RpbmcgdmVjdG9yXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBhIHZlY3RvciB0byBjbG9uZVxuICogQHJldHVybnMge3ZlYzN9IGEgbmV3IDNEIHZlY3RvclxuICovXG52ZWMzLmNsb25lID0gZnVuY3Rpb24oYSkge1xuICAgIHZhciBvdXQgPSBuZXcgR0xNQVRfQVJSQVlfVFlQRSgzKTtcbiAgICBvdXRbMF0gPSBhWzBdO1xuICAgIG91dFsxXSA9IGFbMV07XG4gICAgb3V0WzJdID0gYVsyXTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBDcmVhdGVzIGEgbmV3IHZlYzMgaW5pdGlhbGl6ZWQgd2l0aCB0aGUgZ2l2ZW4gdmFsdWVzXG4gKlxuICogQHBhcmFtIHtOdW1iZXJ9IHggWCBjb21wb25lbnRcbiAqIEBwYXJhbSB7TnVtYmVyfSB5IFkgY29tcG9uZW50XG4gKiBAcGFyYW0ge051bWJlcn0geiBaIGNvbXBvbmVudFxuICogQHJldHVybnMge3ZlYzN9IGEgbmV3IDNEIHZlY3RvclxuICovXG52ZWMzLmZyb21WYWx1ZXMgPSBmdW5jdGlvbih4LCB5LCB6KSB7XG4gICAgdmFyIG91dCA9IG5ldyBHTE1BVF9BUlJBWV9UWVBFKDMpO1xuICAgIG91dFswXSA9IHg7XG4gICAgb3V0WzFdID0geTtcbiAgICBvdXRbMl0gPSB6O1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIENvcHkgdGhlIHZhbHVlcyBmcm9tIG9uZSB2ZWMzIHRvIGFub3RoZXJcbiAqXG4gKiBAcGFyYW0ge3ZlYzN9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHt2ZWMzfSBhIHRoZSBzb3VyY2UgdmVjdG9yXG4gKiBAcmV0dXJucyB7dmVjM30gb3V0XG4gKi9cbnZlYzMuY29weSA9IGZ1bmN0aW9uKG91dCwgYSkge1xuICAgIG91dFswXSA9IGFbMF07XG4gICAgb3V0WzFdID0gYVsxXTtcbiAgICBvdXRbMl0gPSBhWzJdO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFNldCB0aGUgY29tcG9uZW50cyBvZiBhIHZlYzMgdG8gdGhlIGdpdmVuIHZhbHVlc1xuICpcbiAqIEBwYXJhbSB7dmVjM30gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge051bWJlcn0geCBYIGNvbXBvbmVudFxuICogQHBhcmFtIHtOdW1iZXJ9IHkgWSBjb21wb25lbnRcbiAqIEBwYXJhbSB7TnVtYmVyfSB6IFogY29tcG9uZW50XG4gKiBAcmV0dXJucyB7dmVjM30gb3V0XG4gKi9cbnZlYzMuc2V0ID0gZnVuY3Rpb24ob3V0LCB4LCB5LCB6KSB7XG4gICAgb3V0WzBdID0geDtcbiAgICBvdXRbMV0gPSB5O1xuICAgIG91dFsyXSA9IHo7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQWRkcyB0d28gdmVjMydzXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjM30gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHt2ZWMzfSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge3ZlYzN9IG91dFxuICovXG52ZWMzLmFkZCA9IGZ1bmN0aW9uKG91dCwgYSwgYikge1xuICAgIG91dFswXSA9IGFbMF0gKyBiWzBdO1xuICAgIG91dFsxXSA9IGFbMV0gKyBiWzFdO1xuICAgIG91dFsyXSA9IGFbMl0gKyBiWzJdO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFN1YnRyYWN0cyB2ZWN0b3IgYiBmcm9tIHZlY3RvciBhXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjM30gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHt2ZWMzfSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge3ZlYzN9IG91dFxuICovXG52ZWMzLnN1YnRyYWN0ID0gZnVuY3Rpb24ob3V0LCBhLCBiKSB7XG4gICAgb3V0WzBdID0gYVswXSAtIGJbMF07XG4gICAgb3V0WzFdID0gYVsxXSAtIGJbMV07XG4gICAgb3V0WzJdID0gYVsyXSAtIGJbMl07XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogQWxpYXMgZm9yIHtAbGluayB2ZWMzLnN1YnRyYWN0fVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzMuc3ViID0gdmVjMy5zdWJ0cmFjdDtcblxuLyoqXG4gKiBNdWx0aXBsaWVzIHR3byB2ZWMzJ3NcbiAqXG4gKiBAcGFyYW0ge3ZlYzN9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHt2ZWMzfSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3ZlYzN9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcmV0dXJucyB7dmVjM30gb3V0XG4gKi9cbnZlYzMubXVsdGlwbHkgPSBmdW5jdGlvbihvdXQsIGEsIGIpIHtcbiAgICBvdXRbMF0gPSBhWzBdICogYlswXTtcbiAgICBvdXRbMV0gPSBhWzFdICogYlsxXTtcbiAgICBvdXRbMl0gPSBhWzJdICogYlsyXTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBBbGlhcyBmb3Ige0BsaW5rIHZlYzMubXVsdGlwbHl9XG4gKiBAZnVuY3Rpb25cbiAqL1xudmVjMy5tdWwgPSB2ZWMzLm11bHRpcGx5O1xuXG4vKipcbiAqIERpdmlkZXMgdHdvIHZlYzMnc1xuICpcbiAqIEBwYXJhbSB7dmVjM30gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjM30gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEByZXR1cm5zIHt2ZWMzfSBvdXRcbiAqL1xudmVjMy5kaXZpZGUgPSBmdW5jdGlvbihvdXQsIGEsIGIpIHtcbiAgICBvdXRbMF0gPSBhWzBdIC8gYlswXTtcbiAgICBvdXRbMV0gPSBhWzFdIC8gYlsxXTtcbiAgICBvdXRbMl0gPSBhWzJdIC8gYlsyXTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBBbGlhcyBmb3Ige0BsaW5rIHZlYzMuZGl2aWRlfVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzMuZGl2ID0gdmVjMy5kaXZpZGU7XG5cbi8qKlxuICogUmV0dXJucyB0aGUgbWluaW11bSBvZiB0d28gdmVjMydzXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjM30gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHt2ZWMzfSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge3ZlYzN9IG91dFxuICovXG52ZWMzLm1pbiA9IGZ1bmN0aW9uKG91dCwgYSwgYikge1xuICAgIG91dFswXSA9IE1hdGgubWluKGFbMF0sIGJbMF0pO1xuICAgIG91dFsxXSA9IE1hdGgubWluKGFbMV0sIGJbMV0pO1xuICAgIG91dFsyXSA9IE1hdGgubWluKGFbMl0sIGJbMl0pO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFJldHVybnMgdGhlIG1heGltdW0gb2YgdHdvIHZlYzMnc1xuICpcbiAqIEBwYXJhbSB7dmVjM30gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjM30gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEByZXR1cm5zIHt2ZWMzfSBvdXRcbiAqL1xudmVjMy5tYXggPSBmdW5jdGlvbihvdXQsIGEsIGIpIHtcbiAgICBvdXRbMF0gPSBNYXRoLm1heChhWzBdLCBiWzBdKTtcbiAgICBvdXRbMV0gPSBNYXRoLm1heChhWzFdLCBiWzFdKTtcbiAgICBvdXRbMl0gPSBNYXRoLm1heChhWzJdLCBiWzJdKTtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBTY2FsZXMgYSB2ZWMzIGJ5IGEgc2NhbGFyIG51bWJlclxuICpcbiAqIEBwYXJhbSB7dmVjM30gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdGhlIHZlY3RvciB0byBzY2FsZVxuICogQHBhcmFtIHtOdW1iZXJ9IGIgYW1vdW50IHRvIHNjYWxlIHRoZSB2ZWN0b3IgYnlcbiAqIEByZXR1cm5zIHt2ZWMzfSBvdXRcbiAqL1xudmVjMy5zY2FsZSA9IGZ1bmN0aW9uKG91dCwgYSwgYikge1xuICAgIG91dFswXSA9IGFbMF0gKiBiO1xuICAgIG91dFsxXSA9IGFbMV0gKiBiO1xuICAgIG91dFsyXSA9IGFbMl0gKiBiO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIEFkZHMgdHdvIHZlYzMncyBhZnRlciBzY2FsaW5nIHRoZSBzZWNvbmQgb3BlcmFuZCBieSBhIHNjYWxhciB2YWx1ZVxuICpcbiAqIEBwYXJhbSB7dmVjM30gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjM30gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEBwYXJhbSB7TnVtYmVyfSBzY2FsZSB0aGUgYW1vdW50IHRvIHNjYWxlIGIgYnkgYmVmb3JlIGFkZGluZ1xuICogQHJldHVybnMge3ZlYzN9IG91dFxuICovXG52ZWMzLnNjYWxlQW5kQWRkID0gZnVuY3Rpb24ob3V0LCBhLCBiLCBzY2FsZSkge1xuICAgIG91dFswXSA9IGFbMF0gKyAoYlswXSAqIHNjYWxlKTtcbiAgICBvdXRbMV0gPSBhWzFdICsgKGJbMV0gKiBzY2FsZSk7XG4gICAgb3V0WzJdID0gYVsyXSArIChiWzJdICogc2NhbGUpO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIENhbGN1bGF0ZXMgdGhlIGV1Y2xpZGlhbiBkaXN0YW5jZSBiZXR3ZWVuIHR3byB2ZWMzJ3NcbiAqXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjM30gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEByZXR1cm5zIHtOdW1iZXJ9IGRpc3RhbmNlIGJldHdlZW4gYSBhbmQgYlxuICovXG52ZWMzLmRpc3RhbmNlID0gZnVuY3Rpb24oYSwgYikge1xuICAgIHZhciB4ID0gYlswXSAtIGFbMF0sXG4gICAgICAgIHkgPSBiWzFdIC0gYVsxXSxcbiAgICAgICAgeiA9IGJbMl0gLSBhWzJdO1xuICAgIHJldHVybiBNYXRoLnNxcnQoeCp4ICsgeSp5ICsgeip6KTtcbn07XG5cbi8qKlxuICogQWxpYXMgZm9yIHtAbGluayB2ZWMzLmRpc3RhbmNlfVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzMuZGlzdCA9IHZlYzMuZGlzdGFuY2U7XG5cbi8qKlxuICogQ2FsY3VsYXRlcyB0aGUgc3F1YXJlZCBldWNsaWRpYW4gZGlzdGFuY2UgYmV0d2VlbiB0d28gdmVjMydzXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3ZlYzN9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcmV0dXJucyB7TnVtYmVyfSBzcXVhcmVkIGRpc3RhbmNlIGJldHdlZW4gYSBhbmQgYlxuICovXG52ZWMzLnNxdWFyZWREaXN0YW5jZSA9IGZ1bmN0aW9uKGEsIGIpIHtcbiAgICB2YXIgeCA9IGJbMF0gLSBhWzBdLFxuICAgICAgICB5ID0gYlsxXSAtIGFbMV0sXG4gICAgICAgIHogPSBiWzJdIC0gYVsyXTtcbiAgICByZXR1cm4geCp4ICsgeSp5ICsgeip6O1xufTtcblxuLyoqXG4gKiBBbGlhcyBmb3Ige0BsaW5rIHZlYzMuc3F1YXJlZERpc3RhbmNlfVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzMuc3FyRGlzdCA9IHZlYzMuc3F1YXJlZERpc3RhbmNlO1xuXG4vKipcbiAqIENhbGN1bGF0ZXMgdGhlIGxlbmd0aCBvZiBhIHZlYzNcbiAqXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdmVjdG9yIHRvIGNhbGN1bGF0ZSBsZW5ndGggb2ZcbiAqIEByZXR1cm5zIHtOdW1iZXJ9IGxlbmd0aCBvZiBhXG4gKi9cbnZlYzMubGVuZ3RoID0gZnVuY3Rpb24gKGEpIHtcbiAgICB2YXIgeCA9IGFbMF0sXG4gICAgICAgIHkgPSBhWzFdLFxuICAgICAgICB6ID0gYVsyXTtcbiAgICByZXR1cm4gTWF0aC5zcXJ0KHgqeCArIHkqeSArIHoqeik7XG59O1xuXG4vKipcbiAqIEFsaWFzIGZvciB7QGxpbmsgdmVjMy5sZW5ndGh9XG4gKiBAZnVuY3Rpb25cbiAqL1xudmVjMy5sZW4gPSB2ZWMzLmxlbmd0aDtcblxuLyoqXG4gKiBDYWxjdWxhdGVzIHRoZSBzcXVhcmVkIGxlbmd0aCBvZiBhIHZlYzNcbiAqXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdmVjdG9yIHRvIGNhbGN1bGF0ZSBzcXVhcmVkIGxlbmd0aCBvZlxuICogQHJldHVybnMge051bWJlcn0gc3F1YXJlZCBsZW5ndGggb2YgYVxuICovXG52ZWMzLnNxdWFyZWRMZW5ndGggPSBmdW5jdGlvbiAoYSkge1xuICAgIHZhciB4ID0gYVswXSxcbiAgICAgICAgeSA9IGFbMV0sXG4gICAgICAgIHogPSBhWzJdO1xuICAgIHJldHVybiB4KnggKyB5KnkgKyB6Kno7XG59O1xuXG4vKipcbiAqIEFsaWFzIGZvciB7QGxpbmsgdmVjMy5zcXVhcmVkTGVuZ3RofVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzMuc3FyTGVuID0gdmVjMy5zcXVhcmVkTGVuZ3RoO1xuXG4vKipcbiAqIE5lZ2F0ZXMgdGhlIGNvbXBvbmVudHMgb2YgYSB2ZWMzXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjM30gYSB2ZWN0b3IgdG8gbmVnYXRlXG4gKiBAcmV0dXJucyB7dmVjM30gb3V0XG4gKi9cbnZlYzMubmVnYXRlID0gZnVuY3Rpb24ob3V0LCBhKSB7XG4gICAgb3V0WzBdID0gLWFbMF07XG4gICAgb3V0WzFdID0gLWFbMV07XG4gICAgb3V0WzJdID0gLWFbMl07XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogTm9ybWFsaXplIGEgdmVjM1xuICpcbiAqIEBwYXJhbSB7dmVjM30gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdmVjdG9yIHRvIG5vcm1hbGl6ZVxuICogQHJldHVybnMge3ZlYzN9IG91dFxuICovXG52ZWMzLm5vcm1hbGl6ZSA9IGZ1bmN0aW9uKG91dCwgYSkge1xuICAgIHZhciB4ID0gYVswXSxcbiAgICAgICAgeSA9IGFbMV0sXG4gICAgICAgIHogPSBhWzJdO1xuICAgIHZhciBsZW4gPSB4KnggKyB5KnkgKyB6Kno7XG4gICAgaWYgKGxlbiA+IDApIHtcbiAgICAgICAgLy9UT0RPOiBldmFsdWF0ZSB1c2Ugb2YgZ2xtX2ludnNxcnQgaGVyZT9cbiAgICAgICAgbGVuID0gMSAvIE1hdGguc3FydChsZW4pO1xuICAgICAgICBvdXRbMF0gPSBhWzBdICogbGVuO1xuICAgICAgICBvdXRbMV0gPSBhWzFdICogbGVuO1xuICAgICAgICBvdXRbMl0gPSBhWzJdICogbGVuO1xuICAgIH1cbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBDYWxjdWxhdGVzIHRoZSBkb3QgcHJvZHVjdCBvZiB0d28gdmVjMydzXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBhIHRoZSBmaXJzdCBvcGVyYW5kXG4gKiBAcGFyYW0ge3ZlYzN9IGIgdGhlIHNlY29uZCBvcGVyYW5kXG4gKiBAcmV0dXJucyB7TnVtYmVyfSBkb3QgcHJvZHVjdCBvZiBhIGFuZCBiXG4gKi9cbnZlYzMuZG90ID0gZnVuY3Rpb24gKGEsIGIpIHtcbiAgICByZXR1cm4gYVswXSAqIGJbMF0gKyBhWzFdICogYlsxXSArIGFbMl0gKiBiWzJdO1xufTtcblxuLyoqXG4gKiBDb21wdXRlcyB0aGUgY3Jvc3MgcHJvZHVjdCBvZiB0d28gdmVjMydzXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjM30gYSB0aGUgZmlyc3Qgb3BlcmFuZFxuICogQHBhcmFtIHt2ZWMzfSBiIHRoZSBzZWNvbmQgb3BlcmFuZFxuICogQHJldHVybnMge3ZlYzN9IG91dFxuICovXG52ZWMzLmNyb3NzID0gZnVuY3Rpb24ob3V0LCBhLCBiKSB7XG4gICAgdmFyIGF4ID0gYVswXSwgYXkgPSBhWzFdLCBheiA9IGFbMl0sXG4gICAgICAgIGJ4ID0gYlswXSwgYnkgPSBiWzFdLCBieiA9IGJbMl07XG5cbiAgICBvdXRbMF0gPSBheSAqIGJ6IC0gYXogKiBieTtcbiAgICBvdXRbMV0gPSBheiAqIGJ4IC0gYXggKiBiejtcbiAgICBvdXRbMl0gPSBheCAqIGJ5IC0gYXkgKiBieDtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLyoqXG4gKiBQZXJmb3JtcyBhIGxpbmVhciBpbnRlcnBvbGF0aW9uIGJldHdlZW4gdHdvIHZlYzMnc1xuICpcbiAqIEBwYXJhbSB7dmVjM30gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdGhlIGZpcnN0IG9wZXJhbmRcbiAqIEBwYXJhbSB7dmVjM30gYiB0aGUgc2Vjb25kIG9wZXJhbmRcbiAqIEBwYXJhbSB7TnVtYmVyfSB0IGludGVycG9sYXRpb24gYW1vdW50IGJldHdlZW4gdGhlIHR3byBpbnB1dHNcbiAqIEByZXR1cm5zIHt2ZWMzfSBvdXRcbiAqL1xudmVjMy5sZXJwID0gZnVuY3Rpb24gKG91dCwgYSwgYiwgdCkge1xuICAgIHZhciBheCA9IGFbMF0sXG4gICAgICAgIGF5ID0gYVsxXSxcbiAgICAgICAgYXogPSBhWzJdO1xuICAgIG91dFswXSA9IGF4ICsgdCAqIChiWzBdIC0gYXgpO1xuICAgIG91dFsxXSA9IGF5ICsgdCAqIChiWzFdIC0gYXkpO1xuICAgIG91dFsyXSA9IGF6ICsgdCAqIChiWzJdIC0gYXopO1xuICAgIHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIEdlbmVyYXRlcyBhIHJhbmRvbSB2ZWN0b3Igd2l0aCB0aGUgZ2l2ZW4gc2NhbGVcbiAqXG4gKiBAcGFyYW0ge3ZlYzN9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHtOdW1iZXJ9IFtzY2FsZV0gTGVuZ3RoIG9mIHRoZSByZXN1bHRpbmcgdmVjdG9yLiBJZiBvbW1pdHRlZCwgYSB1bml0IHZlY3RvciB3aWxsIGJlIHJldHVybmVkXG4gKiBAcmV0dXJucyB7dmVjM30gb3V0XG4gKi9cbnZlYzMucmFuZG9tID0gZnVuY3Rpb24gKG91dCwgc2NhbGUpIHtcbiAgICBzY2FsZSA9IHNjYWxlIHx8IDEuMDtcblxuICAgIHZhciByID0gR0xNQVRfUkFORE9NKCkgKiAyLjAgKiBNYXRoLlBJO1xuICAgIHZhciB6ID0gKEdMTUFUX1JBTkRPTSgpICogMi4wKSAtIDEuMDtcbiAgICB2YXIgelNjYWxlID0gTWF0aC5zcXJ0KDEuMC16KnopICogc2NhbGU7XG5cbiAgICBvdXRbMF0gPSBNYXRoLmNvcyhyKSAqIHpTY2FsZTtcbiAgICBvdXRbMV0gPSBNYXRoLnNpbihyKSAqIHpTY2FsZTtcbiAgICBvdXRbMl0gPSB6ICogc2NhbGU7XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogVHJhbnNmb3JtcyB0aGUgdmVjMyB3aXRoIGEgbWF0NC5cbiAqIDR0aCB2ZWN0b3IgY29tcG9uZW50IGlzIGltcGxpY2l0bHkgJzEnXG4gKlxuICogQHBhcmFtIHt2ZWMzfSBvdXQgdGhlIHJlY2VpdmluZyB2ZWN0b3JcbiAqIEBwYXJhbSB7dmVjM30gYSB0aGUgdmVjdG9yIHRvIHRyYW5zZm9ybVxuICogQHBhcmFtIHttYXQ0fSBtIG1hdHJpeCB0byB0cmFuc2Zvcm0gd2l0aFxuICogQHJldHVybnMge3ZlYzN9IG91dFxuICovXG52ZWMzLnRyYW5zZm9ybU1hdDQgPSBmdW5jdGlvbihvdXQsIGEsIG0pIHtcbiAgICB2YXIgeCA9IGFbMF0sIHkgPSBhWzFdLCB6ID0gYVsyXTtcbiAgICBvdXRbMF0gPSBtWzBdICogeCArIG1bNF0gKiB5ICsgbVs4XSAqIHogKyBtWzEyXTtcbiAgICBvdXRbMV0gPSBtWzFdICogeCArIG1bNV0gKiB5ICsgbVs5XSAqIHogKyBtWzEzXTtcbiAgICBvdXRbMl0gPSBtWzJdICogeCArIG1bNl0gKiB5ICsgbVsxMF0gKiB6ICsgbVsxNF07XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogVHJhbnNmb3JtcyB0aGUgdmVjMyB3aXRoIGEgbWF0My5cbiAqXG4gKiBAcGFyYW0ge3ZlYzN9IG91dCB0aGUgcmVjZWl2aW5nIHZlY3RvclxuICogQHBhcmFtIHt2ZWMzfSBhIHRoZSB2ZWN0b3IgdG8gdHJhbnNmb3JtXG4gKiBAcGFyYW0ge21hdDR9IG0gdGhlIDN4MyBtYXRyaXggdG8gdHJhbnNmb3JtIHdpdGhcbiAqIEByZXR1cm5zIHt2ZWMzfSBvdXRcbiAqL1xudmVjMy50cmFuc2Zvcm1NYXQzID0gZnVuY3Rpb24ob3V0LCBhLCBtKSB7XG4gICAgdmFyIHggPSBhWzBdLCB5ID0gYVsxXSwgeiA9IGFbMl07XG4gICAgb3V0WzBdID0geCAqIG1bMF0gKyB5ICogbVszXSArIHogKiBtWzZdO1xuICAgIG91dFsxXSA9IHggKiBtWzFdICsgeSAqIG1bNF0gKyB6ICogbVs3XTtcbiAgICBvdXRbMl0gPSB4ICogbVsyXSArIHkgKiBtWzVdICsgeiAqIG1bOF07XG4gICAgcmV0dXJuIG91dDtcbn07XG5cbi8qKlxuICogVHJhbnNmb3JtcyB0aGUgdmVjMyB3aXRoIGEgcXVhdFxuICpcbiAqIEBwYXJhbSB7dmVjM30gb3V0IHRoZSByZWNlaXZpbmcgdmVjdG9yXG4gKiBAcGFyYW0ge3ZlYzN9IGEgdGhlIHZlY3RvciB0byB0cmFuc2Zvcm1cbiAqIEBwYXJhbSB7cXVhdH0gcSBxdWF0ZXJuaW9uIHRvIHRyYW5zZm9ybSB3aXRoXG4gKiBAcmV0dXJucyB7dmVjM30gb3V0XG4gKi9cbnZlYzMudHJhbnNmb3JtUXVhdCA9IGZ1bmN0aW9uKG91dCwgYSwgcSkge1xuICAgIC8vIGJlbmNobWFya3M6IGh0dHA6Ly9qc3BlcmYuY29tL3F1YXRlcm5pb24tdHJhbnNmb3JtLXZlYzMtaW1wbGVtZW50YXRpb25zXG5cbiAgICB2YXIgeCA9IGFbMF0sIHkgPSBhWzFdLCB6ID0gYVsyXSxcbiAgICAgICAgcXggPSBxWzBdLCBxeSA9IHFbMV0sIHF6ID0gcVsyXSwgcXcgPSBxWzNdLFxuXG4gICAgICAgIC8vIGNhbGN1bGF0ZSBxdWF0ICogdmVjXG4gICAgICAgIGl4ID0gcXcgKiB4ICsgcXkgKiB6IC0gcXogKiB5LFxuICAgICAgICBpeSA9IHF3ICogeSArIHF6ICogeCAtIHF4ICogeixcbiAgICAgICAgaXogPSBxdyAqIHogKyBxeCAqIHkgLSBxeSAqIHgsXG4gICAgICAgIGl3ID0gLXF4ICogeCAtIHF5ICogeSAtIHF6ICogejtcblxuICAgIC8vIGNhbGN1bGF0ZSByZXN1bHQgKiBpbnZlcnNlIHF1YXRcbiAgICBvdXRbMF0gPSBpeCAqIHF3ICsgaXcgKiAtcXggKyBpeSAqIC1xeiAtIGl6ICogLXF5O1xuICAgIG91dFsxXSA9IGl5ICogcXcgKyBpdyAqIC1xeSArIGl6ICogLXF4IC0gaXggKiAtcXo7XG4gICAgb3V0WzJdID0gaXogKiBxdyArIGl3ICogLXF6ICsgaXggKiAtcXkgLSBpeSAqIC1xeDtcbiAgICByZXR1cm4gb3V0O1xufTtcblxuLypcbiogUm90YXRlIGEgM0QgdmVjdG9yIGFyb3VuZCB0aGUgeC1heGlzXG4qIEBwYXJhbSB7dmVjM30gb3V0IFRoZSByZWNlaXZpbmcgdmVjM1xuKiBAcGFyYW0ge3ZlYzN9IGEgVGhlIHZlYzMgcG9pbnQgdG8gcm90YXRlXG4qIEBwYXJhbSB7dmVjM30gYiBUaGUgb3JpZ2luIG9mIHRoZSByb3RhdGlvblxuKiBAcGFyYW0ge051bWJlcn0gYyBUaGUgYW5nbGUgb2Ygcm90YXRpb25cbiogQHJldHVybnMge3ZlYzN9IG91dFxuKi9cbnZlYzMucm90YXRlWCA9IGZ1bmN0aW9uKG91dCwgYSwgYiwgYyl7XG4gICB2YXIgcCA9IFtdLCByPVtdO1xuXHQgIC8vVHJhbnNsYXRlIHBvaW50IHRvIHRoZSBvcmlnaW5cblx0ICBwWzBdID0gYVswXSAtIGJbMF07XG5cdCAgcFsxXSA9IGFbMV0gLSBiWzFdO1xuICBcdHBbMl0gPSBhWzJdIC0gYlsyXTtcblxuXHQgIC8vcGVyZm9ybSByb3RhdGlvblxuXHQgIHJbMF0gPSBwWzBdO1xuXHQgIHJbMV0gPSBwWzFdKk1hdGguY29zKGMpIC0gcFsyXSpNYXRoLnNpbihjKTtcblx0ICByWzJdID0gcFsxXSpNYXRoLnNpbihjKSArIHBbMl0qTWF0aC5jb3MoYyk7XG5cblx0ICAvL3RyYW5zbGF0ZSB0byBjb3JyZWN0IHBvc2l0aW9uXG5cdCAgb3V0WzBdID0gclswXSArIGJbMF07XG5cdCAgb3V0WzFdID0gclsxXSArIGJbMV07XG5cdCAgb3V0WzJdID0gclsyXSArIGJbMl07XG5cbiAgXHRyZXR1cm4gb3V0O1xufTtcblxuLypcbiogUm90YXRlIGEgM0QgdmVjdG9yIGFyb3VuZCB0aGUgeS1heGlzXG4qIEBwYXJhbSB7dmVjM30gb3V0IFRoZSByZWNlaXZpbmcgdmVjM1xuKiBAcGFyYW0ge3ZlYzN9IGEgVGhlIHZlYzMgcG9pbnQgdG8gcm90YXRlXG4qIEBwYXJhbSB7dmVjM30gYiBUaGUgb3JpZ2luIG9mIHRoZSByb3RhdGlvblxuKiBAcGFyYW0ge051bWJlcn0gYyBUaGUgYW5nbGUgb2Ygcm90YXRpb25cbiogQHJldHVybnMge3ZlYzN9IG91dFxuKi9cbnZlYzMucm90YXRlWSA9IGZ1bmN0aW9uKG91dCwgYSwgYiwgYyl7XG4gIFx0dmFyIHAgPSBbXSwgcj1bXTtcbiAgXHQvL1RyYW5zbGF0ZSBwb2ludCB0byB0aGUgb3JpZ2luXG4gIFx0cFswXSA9IGFbMF0gLSBiWzBdO1xuICBcdHBbMV0gPSBhWzFdIC0gYlsxXTtcbiAgXHRwWzJdID0gYVsyXSAtIGJbMl07XG4gIFxuICBcdC8vcGVyZm9ybSByb3RhdGlvblxuICBcdHJbMF0gPSBwWzJdKk1hdGguc2luKGMpICsgcFswXSpNYXRoLmNvcyhjKTtcbiAgXHRyWzFdID0gcFsxXTtcbiAgXHRyWzJdID0gcFsyXSpNYXRoLmNvcyhjKSAtIHBbMF0qTWF0aC5zaW4oYyk7XG4gIFxuICBcdC8vdHJhbnNsYXRlIHRvIGNvcnJlY3QgcG9zaXRpb25cbiAgXHRvdXRbMF0gPSByWzBdICsgYlswXTtcbiAgXHRvdXRbMV0gPSByWzFdICsgYlsxXTtcbiAgXHRvdXRbMl0gPSByWzJdICsgYlsyXTtcbiAgXG4gIFx0cmV0dXJuIG91dDtcbn07XG5cbi8qXG4qIFJvdGF0ZSBhIDNEIHZlY3RvciBhcm91bmQgdGhlIHotYXhpc1xuKiBAcGFyYW0ge3ZlYzN9IG91dCBUaGUgcmVjZWl2aW5nIHZlYzNcbiogQHBhcmFtIHt2ZWMzfSBhIFRoZSB2ZWMzIHBvaW50IHRvIHJvdGF0ZVxuKiBAcGFyYW0ge3ZlYzN9IGIgVGhlIG9yaWdpbiBvZiB0aGUgcm90YXRpb25cbiogQHBhcmFtIHtOdW1iZXJ9IGMgVGhlIGFuZ2xlIG9mIHJvdGF0aW9uXG4qIEByZXR1cm5zIHt2ZWMzfSBvdXRcbiovXG52ZWMzLnJvdGF0ZVogPSBmdW5jdGlvbihvdXQsIGEsIGIsIGMpe1xuICBcdHZhciBwID0gW10sIHI9W107XG4gIFx0Ly9UcmFuc2xhdGUgcG9pbnQgdG8gdGhlIG9yaWdpblxuICBcdHBbMF0gPSBhWzBdIC0gYlswXTtcbiAgXHRwWzFdID0gYVsxXSAtIGJbMV07XG4gIFx0cFsyXSA9IGFbMl0gLSBiWzJdO1xuICBcbiAgXHQvL3BlcmZvcm0gcm90YXRpb25cbiAgXHRyWzBdID0gcFswXSpNYXRoLmNvcyhjKSAtIHBbMV0qTWF0aC5zaW4oYyk7XG4gIFx0clsxXSA9IHBbMF0qTWF0aC5zaW4oYykgKyBwWzFdKk1hdGguY29zKGMpO1xuICBcdHJbMl0gPSBwWzJdO1xuICBcbiAgXHQvL3RyYW5zbGF0ZSB0byBjb3JyZWN0IHBvc2l0aW9uXG4gIFx0b3V0WzBdID0gclswXSArIGJbMF07XG4gIFx0b3V0WzFdID0gclsxXSArIGJbMV07XG4gIFx0b3V0WzJdID0gclsyXSArIGJbMl07XG4gIFxuICBcdHJldHVybiBvdXQ7XG59O1xuXG4vKipcbiAqIFBlcmZvcm0gc29tZSBvcGVyYXRpb24gb3ZlciBhbiBhcnJheSBvZiB2ZWMzcy5cbiAqXG4gKiBAcGFyYW0ge0FycmF5fSBhIHRoZSBhcnJheSBvZiB2ZWN0b3JzIHRvIGl0ZXJhdGUgb3ZlclxuICogQHBhcmFtIHtOdW1iZXJ9IHN0cmlkZSBOdW1iZXIgb2YgZWxlbWVudHMgYmV0d2VlbiB0aGUgc3RhcnQgb2YgZWFjaCB2ZWMzLiBJZiAwIGFzc3VtZXMgdGlnaHRseSBwYWNrZWRcbiAqIEBwYXJhbSB7TnVtYmVyfSBvZmZzZXQgTnVtYmVyIG9mIGVsZW1lbnRzIHRvIHNraXAgYXQgdGhlIGJlZ2lubmluZyBvZiB0aGUgYXJyYXlcbiAqIEBwYXJhbSB7TnVtYmVyfSBjb3VudCBOdW1iZXIgb2YgdmVjM3MgdG8gaXRlcmF0ZSBvdmVyLiBJZiAwIGl0ZXJhdGVzIG92ZXIgZW50aXJlIGFycmF5XG4gKiBAcGFyYW0ge0Z1bmN0aW9ufSBmbiBGdW5jdGlvbiB0byBjYWxsIGZvciBlYWNoIHZlY3RvciBpbiB0aGUgYXJyYXlcbiAqIEBwYXJhbSB7T2JqZWN0fSBbYXJnXSBhZGRpdGlvbmFsIGFyZ3VtZW50IHRvIHBhc3MgdG8gZm5cbiAqIEByZXR1cm5zIHtBcnJheX0gYVxuICogQGZ1bmN0aW9uXG4gKi9cbnZlYzMuZm9yRWFjaCA9IChmdW5jdGlvbigpIHtcbiAgICB2YXIgdmVjID0gdmVjMy5jcmVhdGUoKTtcblxuICAgIHJldHVybiBmdW5jdGlvbihhLCBzdHJpZGUsIG9mZnNldCwgY291bnQsIGZuLCBhcmcpIHtcbiAgICAgICAgdmFyIGksIGw7XG4gICAgICAgIGlmKCFzdHJpZGUpIHtcbiAgICAgICAgICAgIHN0cmlkZSA9IDM7XG4gICAgICAgIH1cblxuICAgICAgICBpZighb2Zmc2V0KSB7XG4gICAgICAgICAgICBvZmZzZXQgPSAwO1xuICAgICAgICB9XG4gICAgICAgIFxuICAgICAgICBpZihjb3VudCkge1xuICAgICAgICAgICAgbCA9IE1hdGgubWluKChjb3VudCAqIHN0cmlkZSkgKyBvZmZzZXQsIGEubGVuZ3RoKTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgIGwgPSBhLmxlbmd0aDtcbiAgICAgICAgfVxuXG4gICAgICAgIGZvcihpID0gb2Zmc2V0OyBpIDwgbDsgaSArPSBzdHJpZGUpIHtcbiAgICAgICAgICAgIHZlY1swXSA9IGFbaV07IHZlY1sxXSA9IGFbaSsxXTsgdmVjWzJdID0gYVtpKzJdO1xuICAgICAgICAgICAgZm4odmVjLCB2ZWMsIGFyZyk7XG4gICAgICAgICAgICBhW2ldID0gdmVjWzBdOyBhW2krMV0gPSB2ZWNbMV07IGFbaSsyXSA9IHZlY1syXTtcbiAgICAgICAgfVxuICAgICAgICBcbiAgICAgICAgcmV0dXJuIGE7XG4gICAgfTtcbn0pKCk7XG5cbi8qKlxuICogUmV0dXJucyBhIHN0cmluZyByZXByZXNlbnRhdGlvbiBvZiBhIHZlY3RvclxuICpcbiAqIEBwYXJhbSB7dmVjM30gdmVjIHZlY3RvciB0byByZXByZXNlbnQgYXMgYSBzdHJpbmdcbiAqIEByZXR1cm5zIHtTdHJpbmd9IHN0cmluZyByZXByZXNlbnRhdGlvbiBvZiB0aGUgdmVjdG9yXG4gKi9cbnZlYzMuc3RyID0gZnVuY3Rpb24gKGEpIHtcbiAgICByZXR1cm4gJ3ZlYzMoJyArIGFbMF0gKyAnLCAnICsgYVsxXSArICcsICcgKyBhWzJdICsgJyknO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSB2ZWMzO1xuXG4iLCIvKlxuICogQ29weXJpZ2h0IDIwMTUgR29vZ2xlIEluYy4gQWxsIFJpZ2h0cyBSZXNlcnZlZC5cbiAqIExpY2Vuc2VkIHVuZGVyIHRoZSBBcGFjaGUgTGljZW5zZSwgVmVyc2lvbiAyLjAgKHRoZSBcIkxpY2Vuc2VcIik7XG4gKiB5b3UgbWF5IG5vdCB1c2UgdGhpcyBmaWxlIGV4Y2VwdCBpbiBjb21wbGlhbmNlIHdpdGggdGhlIExpY2Vuc2UuXG4gKiBZb3UgbWF5IG9idGFpbiBhIGNvcHkgb2YgdGhlIExpY2Vuc2UgYXRcbiAqXG4gKiAgICAgaHR0cDovL3d3dy5hcGFjaGUub3JnL2xpY2Vuc2VzL0xJQ0VOU0UtMi4wXG4gKlxuICogVW5sZXNzIHJlcXVpcmVkIGJ5IGFwcGxpY2FibGUgbGF3IG9yIGFncmVlZCB0byBpbiB3cml0aW5nLCBzb2Z0d2FyZVxuICogZGlzdHJpYnV0ZWQgdW5kZXIgdGhlIExpY2Vuc2UgaXMgZGlzdHJpYnV0ZWQgb24gYW4gXCJBUyBJU1wiIEJBU0lTLFxuICogV0lUSE9VVCBXQVJSQU5USUVTIE9SIENPTkRJVElPTlMgT0YgQU5ZIEtJTkQsIGVpdGhlciBleHByZXNzIG9yIGltcGxpZWQuXG4gKiBTZWUgdGhlIExpY2Vuc2UgZm9yIHRoZSBzcGVjaWZpYyBsYW5ndWFnZSBnb3Zlcm5pbmcgcGVybWlzc2lvbnMgYW5kXG4gKiBsaW1pdGF0aW9ucyB1bmRlciB0aGUgTGljZW5zZS5cbiAqL1xuXG4vKipcbiAqIFRPRE86IEZpeCB1cCBhbGwgXCJuZXcgVEhSRUVcIiBpbnN0YW50aWF0aW9ucyB0byBpbXByb3ZlIHBlcmZvcm1hbmNlLlxuICovXG5cbi8vIHNob3VsZCByZXF1aXJlXG4vLyBVdGlsIFNlbnNvclNhbXBsZSBcbi8vIHF1YXQsIHZlYzMgXG52YXIgdmVjMyA9IHJlcXVpcmUoJ2dsLW1hdHJpeC12ZWMzJyk7XG52YXIgcXVhdCA9IHJlcXVpcmUoJ2dsLW1hdHJpeC1xdWF0Jyk7XG5cbi8vdmFyIFV0aWwgPSByZXF1aXJlKCcuL3V0aWwuanMnKTtcbnZhciBTZW5zb3JTYW1wbGUgPSByZXF1aXJlKCcuL3NlbnNvci1zYW1wbGUuanMnKTtcblxudmFyIERFQlVHID0gZmFsc2U7XG5cbnZhciBNSU5fVElNRVNURVAgPSAwLjAwMTtcbnZhciBNQVhfVElNRVNURVAgPSAxO1xuXG4vLyBIZWxwZXIgbWV0aG9kIHRvIHZhbGlkYXRlIHRoZSB0aW1lIHN0ZXBzIG9mIHNlbnNvciB0aW1lc3RhbXBzLlxuZnVuY3Rpb24gaXNUaW1lc3RhbXBEZWx0YVZhbGlkKHRpbWVzdGFtcERlbHRhUykge1xuICBpZiAoaXNOYU4odGltZXN0YW1wRGVsdGFTKSkge1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuICBpZiAodGltZXN0YW1wRGVsdGFTIDw9IE1JTl9USU1FU1RFUCkge1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuICBpZiAodGltZXN0YW1wRGVsdGFTID4gTUFYX1RJTUVTVEVQKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG4gIHJldHVybiB0cnVlO1xufVxuXG5cbi8qKlxuICogQW4gaW1wbGVtZW50YXRpb24gb2YgYSBzaW1wbGUgY29tcGxlbWVudGFyeSBmaWx0ZXIsIHdoaWNoIGZ1c2VzIGd5cm9zY29wZSBhbmRcbiAqIGFjY2VsZXJvbWV0ZXIgZGF0YSBmcm9tIHRoZSAnZGV2aWNlbW90aW9uJyBldmVudC5cbiAqXG4gKiBBY2NlbGVyb21ldGVyIGRhdGEgaXMgdmVyeSBub2lzeSwgYnV0IHN0YWJsZSBvdmVyIHRoZSBsb25nIHRlcm0uXG4gKiBHeXJvc2NvcGUgZGF0YSBpcyBzbW9vdGgsIGJ1dCB0ZW5kcyB0byBkcmlmdCBvdmVyIHRoZSBsb25nIHRlcm0uXG4gKlxuICogVGhpcyBmdXNpb24gaXMgcmVsYXRpdmVseSBzaW1wbGU6XG4gKiAxLiBHZXQgb3JpZW50YXRpb24gZXN0aW1hdGVzIGZyb20gYWNjZWxlcm9tZXRlciBieSBhcHBseWluZyBhIGxvdy1wYXNzIGZpbHRlclxuICogICAgb24gdGhhdCBkYXRhLlxuICogMi4gR2V0IG9yaWVudGF0aW9uIGVzdGltYXRlcyBmcm9tIGd5cm9zY29wZSBieSBpbnRlZ3JhdGluZyBvdmVyIHRpbWUuXG4gKiAzLiBDb21iaW5lIHRoZSB0d28gZXN0aW1hdGVzLCB3ZWlnaGluZyAoMSkgaW4gdGhlIGxvbmcgdGVybSwgYnV0ICgyKSBmb3IgdGhlXG4gKiAgICBzaG9ydCB0ZXJtLlxuICovXG5mdW5jdGlvbiBDb21wbGVtZW50YXJ5RmlsdGVyKGtGaWx0ZXIpIHtcbiAgdGhpcy5rRmlsdGVyID0ga0ZpbHRlcjtcblxuICAvLyBSYXcgc2Vuc29yIG1lYXN1cmVtZW50cy5cbiAgdGhpcy5jdXJyZW50QWNjZWxNZWFzdXJlbWVudCA9IG5ldyBTZW5zb3JTYW1wbGUoKTtcbiAgdGhpcy5jdXJyZW50R3lyb01lYXN1cmVtZW50ID0gbmV3IFNlbnNvclNhbXBsZSgpO1xuICB0aGlzLnByZXZpb3VzR3lyb01lYXN1cmVtZW50ID0gbmV3IFNlbnNvclNhbXBsZSgpO1xuXG4gIC8vIEN1cnJlbnQgZmlsdGVyIG9yaWVudGF0aW9uXG4vKlxuICB0aGlzLmZpbHRlclEgPSBuZXcgVEhSRUUuUXVhdGVybmlvbigpO1xuICB0aGlzLnByZXZpb3VzRmlsdGVyUSA9IG5ldyBUSFJFRS5RdWF0ZXJuaW9uKCk7XG4qL1xuICB0aGlzLmZpbHRlclEgPSBxdWF0LmNyZWF0ZSgpO1xuICB0aGlzLnByZXZpb3VzRmlsdGVyUSA9IHF1YXQuY3JlYXRlKCk7XG4gIFxuICAvLyBPcmllbnRhdGlvbiBiYXNlZCBvbiB0aGUgYWNjZWxlcm9tZXRlci5cbi8vICB0aGlzLmFjY2VsUSA9IG5ldyBUSFJFRS5RdWF0ZXJuaW9uKCk7XG4gIHRoaXMuYWNjZWxRID0gcXVhdC5jcmVhdGUoKTtcblxuICAvLyBXaGV0aGVyIG9yIG5vdCB0aGUgb3JpZW50YXRpb24gaGFzIGJlZW4gaW5pdGlhbGl6ZWQuXG4gIHRoaXMuaXNPcmllbnRhdGlvbkluaXRpYWxpemVkID0gZmFsc2U7XG4gIC8vIFJ1bm5pbmcgZXN0aW1hdGUgb2YgZ3Jhdml0eSBiYXNlZCBvbiB0aGUgY3VycmVudCBvcmllbnRhdGlvbi5cbi8vICB0aGlzLmVzdGltYXRlZEdyYXZpdHkgPSBuZXcgVEhSRUUuVmVjdG9yMygpO1xuICB0aGlzLmVzdGltYXRlZEdyYXZpdHkgPSB2ZWMzLmNyZWF0ZSgpO1xuXG4gIC8vIE1lYXN1cmVkIGdyYXZpdHkgYmFzZWQgb24gYWNjZWxlcm9tZXRlci5cbi8vICB0aGlzLm1lYXN1cmVkR3Jhdml0eSA9IG5ldyBUSFJFRS5WZWN0b3IzKCk7XG4gIHRoaXMubWVhc3VyZWRHcmF2aXR5ID0gdmVjMy5jcmVhdGUoKTtcblxuICAvLyBEZWJ1ZyBvbmx5IHF1YXRlcm5pb24gb2YgZ3lyby1iYXNlZCBvcmllbnRhdGlvbi5cbi8vICB0aGlzLmd5cm9JbnRlZ3JhbFEgPSBuZXcgVEhSRUUuUXVhdGVybmlvbigpO1xuICB0aGlzLmd5cm9JbnRlZ3JhbFEgPSBxdWF0LmNyZWF0ZSgpO1xuXG59XG5cbkNvbXBsZW1lbnRhcnlGaWx0ZXIucHJvdG90eXBlLmFkZEFjY2VsTWVhc3VyZW1lbnQgPSBmdW5jdGlvbih2ZWN0b3IsIHRpbWVzdGFtcFMpIHtcbiAgdGhpcy5jdXJyZW50QWNjZWxNZWFzdXJlbWVudC5zZXQodmVjdG9yLCB0aW1lc3RhbXBTKTtcbn07XG5cbkNvbXBsZW1lbnRhcnlGaWx0ZXIucHJvdG90eXBlLmFkZEd5cm9NZWFzdXJlbWVudCA9IGZ1bmN0aW9uKHZlY3RvciwgdGltZXN0YW1wUykge1xuICB0aGlzLmN1cnJlbnRHeXJvTWVhc3VyZW1lbnQuc2V0KHZlY3RvciwgdGltZXN0YW1wUyk7XG5cbiAgdmFyIGRlbHRhVCA9IHRpbWVzdGFtcFMgLSB0aGlzLnByZXZpb3VzR3lyb01lYXN1cmVtZW50LnRpbWVzdGFtcFM7XG4gIGlmIChpc1RpbWVzdGFtcERlbHRhVmFsaWQoZGVsdGFUKSkge1xuICAgIHRoaXMucnVuXygpO1xuICB9XG4gIFxuICB0aGlzLnByZXZpb3VzR3lyb01lYXN1cmVtZW50LmNvcHkodGhpcy5jdXJyZW50R3lyb01lYXN1cmVtZW50KTtcbn07XG5cbkNvbXBsZW1lbnRhcnlGaWx0ZXIucHJvdG90eXBlLnJ1bl8gPSBmdW5jdGlvbigpIHtcbiAgdGhpcy5hY2NlbFEgPSB0aGlzLmFjY2VsVG9RdWF0ZXJuaW9uXyh0aGlzLmN1cnJlbnRBY2NlbE1lYXN1cmVtZW50LnNhbXBsZSk7XG5cbiAgaWYgKCF0aGlzLmlzT3JpZW50YXRpb25Jbml0aWFsaXplZCkge1xuLy8gICAgdGhpcy5wcmV2aW91c0ZpbHRlclEuY29weSh0aGlzLmFjY2VsUSk7XG4gICAgcXVhdC5jb3B5KHRoaXMucHJldmlvdXNGaWx0ZXJRLCB0aGlzLmFjY2VsUSk7XG5cbiAgICB0aGlzLmlzT3JpZW50YXRpb25Jbml0aWFsaXplZCA9IHRydWU7XG4gICAgcmV0dXJuO1xuICB9XG5cbiAgdmFyIGRlbHRhVCA9IHRoaXMuY3VycmVudEd5cm9NZWFzdXJlbWVudC50aW1lc3RhbXBTIC1cbiAgICAgIHRoaXMucHJldmlvdXNHeXJvTWVhc3VyZW1lbnQudGltZXN0YW1wUztcblxuICAvLyBDb252ZXJ0IGd5cm8gcm90YXRpb24gdmVjdG9yIHRvIGEgcXVhdGVybmlvbiBkZWx0YS5cbiAgdmFyIGd5cm9EZWx0YVEgPSB0aGlzLmd5cm9Ub1F1YXRlcm5pb25EZWx0YV8odGhpcy5jdXJyZW50R3lyb01lYXN1cmVtZW50LnNhbXBsZSwgZGVsdGFUKTtcbi8vICB0aGlzLmd5cm9JbnRlZ3JhbFEubXVsdGlwbHkoZ3lyb0RlbHRhUSk7XG4gIHF1YXQubXVsdGlwbHkodGhpcy5neXJvSW50ZWdyYWxRLCB0aGlzLmd5cm9JbnRlZ3JhbFEsIGd5cm9EZWx0YVEpO1xuXG5cbiAgLy8gZmlsdGVyXzEgPSBLICogKGZpbHRlcl8wICsgZ3lybyAqIGRUKSArICgxIC0gSykgKiBhY2NlbC5cbi8qXG4gIHRoaXMuZmlsdGVyUS5jb3B5KHRoaXMucHJldmlvdXNGaWx0ZXJRKTtcbiAgdGhpcy5maWx0ZXJRLm11bHRpcGx5KGd5cm9EZWx0YVEpO1xuKi9cbiAgcXVhdC5jb3B5KHRoaXMuZmlsdGVyUSwgdGhpcy5wcmV2aW91c0ZpbHRlclEpO1xuICBxdWF0Lm11bHRpcGx5KHRoaXMuZmlsdGVyUSwgdGhpcy5maWx0ZXJRLCBneXJvRGVsdGFRKTtcbiAgXG4gIC8vIENhbGN1bGF0ZSB0aGUgZGVsdGEgYmV0d2VlbiB0aGUgY3VycmVudCBlc3RpbWF0ZWQgZ3Jhdml0eSBhbmQgdGhlIHJlYWxcbiAgLy8gZ3Jhdml0eSB2ZWN0b3IgZnJvbSBhY2NlbGVyb21ldGVyLlxuLypcbiAgdmFyIGludkZpbHRlclEgPSBuZXcgVEhSRUUuUXVhdGVybmlvbigpO1xuICBpbnZGaWx0ZXJRLmNvcHkodGhpcy5maWx0ZXJRKTtcbiAgaW52RmlsdGVyUS5pbnZlcnNlKCk7XG4qL1xuICB2YXIgaW52RmlsdGVyUSA9IHF1YXQuY3JlYXRlKCk7XG4gIHF1YXQuY29weShpbnZGaWx0ZXJRLCB0aGlzLmZpbHRlclEpO1xuICBxdWF0LmNvbmp1Z2F0ZShpbnZGaWx0ZXJRLCBpbnZGaWx0ZXJRKVxuICBcbi8qXG4gIHRoaXMuZXN0aW1hdGVkR3Jhdml0eS5zZXQoMCwgMCwgLTEpO1xuICB0aGlzLmVzdGltYXRlZEdyYXZpdHkuYXBwbHlRdWF0ZXJuaW9uKGludkZpbHRlclEpO1xuICB0aGlzLmVzdGltYXRlZEdyYXZpdHkubm9ybWFsaXplKCk7XG5cbiAgdGhpcy5tZWFzdXJlZEdyYXZpdHkuY29weSh0aGlzLmN1cnJlbnRBY2NlbE1lYXN1cmVtZW50LnNhbXBsZSk7XG4gIHRoaXMubWVhc3VyZWRHcmF2aXR5Lm5vcm1hbGl6ZSgpO1xuKi9cbiAgdmVjMy5zZXQodGhpcy5lc3RpbWF0ZWRHcmF2aXR5LCAwLCAwLCAtMSk7XG4gIHZlYzMudHJhbnNmb3JtUXVhdCh0aGlzLmVzdGltYXRlZEdyYXZpdHksIHRoaXMuZXN0aW1hdGVkR3Jhdml0eSwgaW52RmlsdGVyUSk7XG4gIHZlYzMubm9ybWFsaXplKHRoaXMuZXN0aW1hdGVkR3Jhdml0eSwgdGhpcy5lc3RpbWF0ZWRHcmF2aXR5KTtcblxuICB2ZWMzLmNvcHkodGhpcy5tZWFzdXJlZEdyYXZpdHksIHRoaXMuY3VycmVudEFjY2VsTWVhc3VyZW1lbnQuc2FtcGxlKTtcbiAgdmVjMy5ub3JtYWxpemUodGhpcy5tZWFzdXJlZEdyYXZpdHksIHRoaXMubWVhc3VyZWRHcmF2aXR5KTtcbiAgXG4gIC8vIENvbXBhcmUgZXN0aW1hdGVkIGdyYXZpdHkgd2l0aCBtZWFzdXJlZCBncmF2aXR5LCBnZXQgdGhlIGRlbHRhIHF1YXRlcm5pb25cbiAgLy8gYmV0d2VlbiB0aGUgdHdvLlxuLypcbiAgdmFyIGRlbHRhUSA9IG5ldyBUSFJFRS5RdWF0ZXJuaW9uKCk7XG4gIGRlbHRhUS5zZXRGcm9tVW5pdFZlY3RvcnModGhpcy5lc3RpbWF0ZWRHcmF2aXR5LCB0aGlzLm1lYXN1cmVkR3Jhdml0eSk7XG4gIGRlbHRhUS5pbnZlcnNlKCk7XG4qL1xuXG4gIHZhciBkZWx0YVEgPSBzZXRGcm9tVW5pdFZlY3RvcnModGhpcy5lc3RpbWF0ZWRHcmF2aXR5LCB0aGlzLm1lYXN1cmVkR3Jhdml0eSk7XG4gIHF1YXQuY29uanVnYXRlKGRlbHRhUSwgZGVsdGFRKVxuICBcbiAgaWYgKERFQlVHKSB7XG4vKlxuICAgIGNvbnNvbGUubG9nKCdEZWx0YTogTmFOIHJhZCwgR19lc3Q6ICglcywgJXMsICVzKSwgR19tZWFzOiAoJXMsICVzLCAlcyknLFxuICAgICAgICAgICAgICAgIC8vVXRpbC5nZXRRdWF0ZXJuaW9uQW5nbGUoZGVsdGFRKSxcbiAgICAgICAgICAgICAgICAodGhpcy5lc3RpbWF0ZWRHcmF2aXR5LngpLnRvRml4ZWQoMSksXG4gICAgICAgICAgICAgICAgKHRoaXMuZXN0aW1hdGVkR3Jhdml0eS55KS50b0ZpeGVkKDEpLFxuICAgICAgICAgICAgICAgICh0aGlzLmVzdGltYXRlZEdyYXZpdHkueikudG9GaXhlZCgxKSxcbiAgICAgICAgICAgICAgICAodGhpcy5tZWFzdXJlZEdyYXZpdHkueCkudG9GaXhlZCgxKSxcbiAgICAgICAgICAgICAgICAodGhpcy5tZWFzdXJlZEdyYXZpdHkueSkudG9GaXhlZCgxKSxcbiAgICAgICAgICAgICAgICAodGhpcy5tZWFzdXJlZEdyYXZpdHkueikudG9GaXhlZCgxKSk7XG4qL1xuICB9XG5cbiAgLy8gQ2FsY3VsYXRlIHRoZSBTTEVSUCB0YXJnZXQ6IGN1cnJlbnQgb3JpZW50YXRpb24gcGx1cyB0aGUgbWVhc3VyZWQtZXN0aW1hdGVkXG4gIC8vIHF1YXRlcm5pb24gZGVsdGEuXG4vKlxuICB2YXIgdGFyZ2V0USA9IG5ldyBUSFJFRS5RdWF0ZXJuaW9uKCk7XG4gIHRhcmdldFEuY29weSh0aGlzLmZpbHRlclEpO1xuICB0YXJnZXRRLm11bHRpcGx5KGRlbHRhUSk7XG4qL1xuICB2YXIgdGFyZ2V0USA9IHF1YXQuY3JlYXRlKCk7XG4gIHF1YXQuY29weSh0YXJnZXRRLCB0aGlzLmZpbHRlclEpO1xuICBxdWF0Lm11bHRpcGx5KHRhcmdldFEsIHRhcmdldFEsIGRlbHRhUSk7XG4gIFxuICAvLyBTTEVSUCBmYWN0b3I6IDAgaXMgcHVyZSBneXJvLCAxIGlzIHB1cmUgYWNjZWwuXG4vLyAgdGhpcy5maWx0ZXJRLnNsZXJwKHRhcmdldFEsIDEgLSB0aGlzLmtGaWx0ZXIpO1xuICBxdWF0LnNsZXJwKHRoaXMuZmlsdGVyUSwgdGhpcy5maWx0ZXJRLCB0YXJnZXRRLCAgMSAtIHRoaXMua0ZpbHRlcik7XG5cbi8vICB0aGlzLnByZXZpb3VzRmlsdGVyUS5jb3B5KHRoaXMuZmlsdGVyUSk7XG4gIHF1YXQuY29weSh0aGlzLnByZXZpb3VzRmlsdGVyUSwgdGhpcy5maWx0ZXJRKTtcbn07XG5cbkNvbXBsZW1lbnRhcnlGaWx0ZXIucHJvdG90eXBlLmdldE9yaWVudGF0aW9uID0gZnVuY3Rpb24oKSB7XG4gIHJldHVybiB0aGlzLmZpbHRlclE7XG59O1xuXG5Db21wbGVtZW50YXJ5RmlsdGVyLnByb3RvdHlwZS5hY2NlbFRvUXVhdGVybmlvbl8gPSBmdW5jdGlvbihhY2NlbCkge1xuLypcbiAgdmFyIG5vcm1BY2NlbCA9IG5ldyBUSFJFRS5WZWN0b3IzKCk7XG4gIG5vcm1BY2NlbC5jb3B5KGFjY2VsKTtcbiAgbm9ybUFjY2VsLm5vcm1hbGl6ZSgpO1xuICB2YXIgcXVhdCA9IG5ldyBUSFJFRS5RdWF0ZXJuaW9uKCk7XG4gIHF1YXQuc2V0RnJvbVVuaXRWZWN0b3JzKG5ldyBUSFJFRS5WZWN0b3IzKDAsIDAsIC0xKSwgbm9ybUFjY2VsKTtcbiAgcmV0dXJuIHF1YXQ7XG4qL1xuICB2YXIgbm9ybUFjY2VsID0gdmVjMy5jcmVhdGUoKTtcbiAgdmVjMy5jb3B5KG5vcm1BY2NlbCwgYWNjZWwpO1xuICB2ZWMzLm5vcm1hbGl6ZShub3JtQWNjZWwsIG5vcm1BY2NlbCk7XG4gIHZhciBxdWF0ID0gc2V0RnJvbVVuaXRWZWN0b3JzKCB2ZWMzLmZyb21WYWx1ZXMoMCwgMCwgLTEpLCBub3JtQWNjZWwpO1xuICByZXR1cm4gcXVhdDtcbn07XG5cbkNvbXBsZW1lbnRhcnlGaWx0ZXIucHJvdG90eXBlLmd5cm9Ub1F1YXRlcm5pb25EZWx0YV8gPSBmdW5jdGlvbihneXJvLCBkdCkge1xuICAvLyBFeHRyYWN0IGF4aXMgYW5kIGFuZ2xlIGZyb20gdGhlIGd5cm9zY29wZSBkYXRhLlxuLypcbiAgdmFyIHF1YXQgPSBuZXcgVEhSRUUuUXVhdGVybmlvbigpO1xuICB2YXIgYXhpcyA9IG5ldyBUSFJFRS5WZWN0b3IzKCk7XG4qL1xuICB2YXIgb3V0ID0gcXVhdC5jcmVhdGUoKTtcbiAgdmFyIGF4aXMgPSB2ZWMzLmNyZWF0ZSgpO1xuICBcbi8qXG4gIGF4aXMuY29weShneXJvKTtcbiAgYXhpcy5ub3JtYWxpemUoKTtcbiAgcXVhdC5zZXRGcm9tQXhpc0FuZ2xlKGF4aXMsIGd5cm8ubGVuZ3RoKCkgKiBkdCk7XG4qL1xuICB2ZWMzLmNvcHkoYXhpcywgZ3lybyk7XG4gIHZlYzMubm9ybWFsaXplKGF4aXMsIGF4aXMpO1xuICBxdWF0LnNldEF4aXNBbmdsZShvdXQsIGF4aXMsIHZlYzMubGVuZ3RoKGd5cm8pICogZHQpO1xuICBcbiAgcmV0dXJuIG91dDtcbn07XG5cbi8vIGdsLW1hdHJpeCB2ZXIuXG52YXIgc2V0RnJvbVVuaXRWZWN0b3JzID0gZnVuY3Rpb24oKSB7XG5cblx0Ly8gaHR0cDovL2xvbGVuZ2luZS5uZXQvYmxvZy8yMDE0LzAyLzI0L3F1YXRlcm5pb24tZnJvbS10d28tdmVjdG9ycy1maW5hbFxuXG5cdC8vIGFzc3VtZXMgZGlyZWN0aW9uIHZlY3RvcnMgdkZyb20gYW5kIHZUbyBhcmUgbm9ybWFsaXplZFxuXG5cdHZhciB2MSwgcjtcblxuXHR2YXIgRVBTID0gMC4wMDAwMDE7XG5cblx0cmV0dXJuIGZ1bmN0aW9uIHNldEZyb21Vbml0VmVjdG9ycyggdkZyb20sIHZUbyApIHtcblxuLy9cdFx0aWYgKCB2MSA9PT0gdW5kZWZpbmVkICkgdjEgPSBuZXcgVEhSRUUuVmVjdG9yMygpO1xuXHRcdGlmICggdjEgPT09IHVuZGVmaW5lZCApIHYxID0gdmVjMy5jcmVhdGUoKTtcblx0XHRcblxuLy9cdFx0ciA9IHZGcm9tLmRvdCggdlRvICkgKyAxO1xuXHRcdHIgPSB2ZWMzLmRvdCggdkZyb20sIHZUbyApICsgMTtcblxuXHRcdGlmICggciA8IEVQUyApIHtcblxuXHRcdFx0ciA9IDA7XG5cbi8vXHRcdFx0aWYgKCBNYXRoLmFicyggdkZyb20ueCApID4gTWF0aC5hYnMoIHZGcm9tLnogKSApIHtcblx0XHRcdGlmICggTWF0aC5hYnMoIHZGcm9tWzBdICkgPiBNYXRoLmFicyggdkZyb21bMl0gKSApIHtcbi8vXHRcdFx0XHR2MS5zZXQoIC0gdkZyb20ueSwgdkZyb20ueCwgMCApO1xuXHRcdFx0XHR2ZWMzLnNldCh2MSwgIC0gdkZyb21bMV0sIHZGcm9tWzBdLCAwICk7XG5cblx0XHRcdH0gZWxzZSB7XG5cbi8vXHRcdFx0XHR2MS5zZXQoIDAsIC0gdkZyb20ueiwgdkZyb20ueSApO1xuXHRcdFx0XHR2ZWMzLnNldCh2MSwgMCwgLSB2RnJvbVsyXSwgdkZyb21bMV0gKTtcblxuXHRcdFx0fVxuXG5cdFx0fSBlbHNlIHtcblxuLy9cdFx0XHR2MS5jcm9zc1ZlY3RvcnMoIHZGcm9tLCB2VG8gKTtcblx0XHRcdHZlYzMuY3Jvc3MoIHYxLCB2RnJvbSwgdlRvICk7XG5cblx0XHR9XG5cblx0XHQvLyBvdXRRXG5cdFx0dmFyIG91dCA9IHF1YXQuZnJvbVZhbHVlcyh2MVswXSwgdjFbMV0sIHYxWzJdLCByKTtcblxuXHRcdHJldHVybiBxdWF0Lm5vcm1hbGl6ZShvdXQsIG91dCk7XG5cblx0fTtcblxufSgpXG5cdFxubW9kdWxlLmV4cG9ydHMgPSBDb21wbGVtZW50YXJ5RmlsdGVyOyIsIi8vIHZlYzNcbi8vIHF1YXRcbnZhciB2ZWMzID0gcmVxdWlyZSgnZ2wtbWF0cml4LXZlYzMnKTtcbnZhciBxdWF0ID0gcmVxdWlyZSgnZ2wtbWF0cml4LXF1YXQnKTtcblxudmFyIEd5cm9Ob3JtID0gcmVxdWlyZSgnLi4vbGliL2d5cm9ub3JtL2d5cm9ub3JtLmpzJyk7XG5cbnZhciBDb21wbGVtZW50YXJ5RmlsdGVyID0gcmVxdWlyZSgnLi9jb21wbGVtZW50YXJ5LWZpbHRlci5qcycpO1xudmFyIFBvc2VQcmVkaWN0b3IgPSByZXF1aXJlKCcuL3Bvc2UtcHJlZGljdG9yLmpzJyk7XG5cblxudmFyIE1JTl9USU1FU1RFUCA9IDAuMDAxO1xudmFyIE1BWF9USU1FU1RFUCA9IDE7XG5cbnZhciBmbGFnID0gdHJ1ZTtcbmZ1bmN0aW9uIENvbXBsZW1lbnRhcnlPcmllbnRhdGlvbigpIHtcbi8qXG4gIHRoaXMuYWNjZWxlcm9tZXRlciA9IG5ldyBUSFJFRS5WZWN0b3IzKCk7XG4gIHRoaXMuZ3lyb3Njb3BlID0gbmV3IFRIUkVFLlZlY3RvcjMoKTtcbiovXG4gIHRoaXMuYWNjZWxlcm9tZXRlciA9IHZlYzMuY3JlYXRlKCk7XG4gIHRoaXMuZ3lyb3Njb3BlID0gdmVjMy5jcmVhdGUoKTtcblxuXG5cdFx0XHRcblxuICB2YXIgc2NvcGUgPSB0aGlzO1xuICB0aGlzLmduID0gbmV3IEd5cm9Ob3JtKCk7XG5cbiAgdGhpcy5nbi5pbml0KHtcbiAgICBmcmVxdWVuY3k6IDEwMDAvNjAsICAgICAgICAgICAgICAgICAgIC8vICggSG93IG9mdGVuIHRoZSBvYmplY3Qgc2VuZHMgdGhlIHZhbHVlcyAtIG1pbGxpc2Vjb25kcyApXG4gICAgZ3Jhdml0eU5vcm1hbGl6ZWQ6dHJ1ZSwgICAgICAgICAvLyAoIElmIHRoZSBnYXJ2aXR5IHJlbGF0ZWQgdmFsdWVzIHRvIGJlIG5vcm1hbGl6ZWQgKVxuICAgIG9yaWVudGF0aW9uQmFzZTpHeXJvTm9ybS5HQU1FLCAgICAgIC8vICggQ2FuIGJlIEd5cm9Ob3JtLkdBTUUgb3IgR3lyb05vcm0uV09STEQuIGduLkdBTUUgcmV0dXJucyBvcmllbnRhdGlvbiB2YWx1ZXMgd2l0aCByZXNwZWN0IHRvIHRoZSBoZWFkIGRpcmVjdGlvbiBvZiB0aGUgZGV2aWNlLiBnbi5XT1JMRCByZXR1cm5zIHRoZSBvcmllbnRhdGlvbiB2YWx1ZXMgd2l0aCByZXNwZWN0IHRvIHRoZSBhY3R1YWwgbm9ydGggZGlyZWN0aW9uIG9mIHRoZSB3b3JsZC4gKVxuICAgIGRlY2ltYWxDb3VudDoyLCAgICAgICAgICAgICAgICAgLy8gKCBIb3cgbWFueSBkaWdpdHMgYWZ0ZXIgdGhlIGRlY2ltYWwgcG9pbnQgd2lsbCB0aGVyZSBiZSBpbiB0aGUgcmV0dXJuIHZhbHVlcyApXG4gICAgbG9nZ2VyOm51bGwsICAgICAgICAgICAgICAgICAgICAvLyAoIEZ1bmN0aW9uIHRvIGJlIGNhbGxlZCB0byBsb2cgbWVzc2FnZXMgZnJvbSBneXJvbm9ybS5qcyApXG4gICAgc2NyZWVuQWRqdXN0ZWQ6ZmFsc2UgICAgICAgICAgICAvLyAoIElmIHNldCB0byB0cnVlIGl0IHdpbGwgcmV0dXJuIHNjcmVlbiBhZGp1c3RlZCB2YWx1ZXMuIClcbn0pXG4udGhlbihmdW5jdGlvbigpe1xuXG5cdFx0XG5cdCAgICBzY29wZS5nbi5zdGFydChmdW5jdGlvbihkYXRhKXtcblx0XHQgICAgXG5cdFx0XHRcblx0XHRcdHNjb3BlLm9uRGV2aWNlTW90aW9uQ2hhbmdlXyh7XG5cdFx0XHRcdGFjY2VsZXJhdGlvbkluY2x1ZGluZ0dyYXZpdHk6IHtcblx0XHRcdFx0XHR4OiBkYXRhLmRtLmd4LFxuXHRcdFx0XHRcdHk6IGRhdGEuZG0uZ3ksXG5cdFx0XHRcdFx0ejogZGF0YS5kbS5neixcblx0XHRcdFx0fSxcblx0XHRcdFx0dGltZVN0YW1wOiBkYXRhLmRtLnRpbWVTdGFtcCxcblx0XHRcdFx0cm90YXRpb25SYXRlOiB7XG5cdFx0XHRcdFx0YWxwaGE6IGRhdGEuZG0uYWxwaGEsXG5cdFx0XHRcdFx0YmV0YTogZGF0YS5kbS5iZXRhLFxuXHRcdFx0XHRcdGdhbW1hOiBkYXRhLmRtLmdhbW1hXG5cdFx0XHRcdH1cblx0XHRcdH0pO1xuXG5cdCAgICB9KTtcblx0fSk7XG5cblxuLy8gIHdpbmRvdy5hZGRFdmVudExpc3RlbmVyKCdkZXZpY2Vtb3Rpb24nLCB0aGlzLm9uRGV2aWNlTW90aW9uQ2hhbmdlXy5iaW5kKHRoaXMpKTtcbiAgd2luZG93LmFkZEV2ZW50TGlzdGVuZXIoJ29yaWVudGF0aW9uY2hhbmdlJywgdGhpcy5vblNjcmVlbk9yaWVudGF0aW9uQ2hhbmdlXy5iaW5kKHRoaXMpKTtcblxuICAvLyDslYTrnpgg65GQIOqwneyytOydmCDsnbjsiJjrpbwg7KGw7KCV7ZW07IScIOqwkOuPhOulvCDsoJzslrTtlZzri6QuXG4gIHRoaXMuZmlsdGVyID0gbmV3IENvbXBsZW1lbnRhcnlGaWx0ZXIoMC43KTsgLy8ga0ZpbHRlciDsnZgg6rCS7J20IDHsl5Ag6rCA6rmM7Jq4IOyImOuhnSDrr7zqsJDrj4TqsIAg65ao7Ja07KeE64ukLlxuICB0aGlzLnBvc2VQcmVkaWN0b3IgPSBuZXcgUG9zZVByZWRpY3RvcigwLjA1MCk7XG5cbi8vICB0aGlzLmZpbHRlclRvV29ybGRRID0gbmV3IFRIUkVFLlF1YXRlcm5pb24oKTtcbiAgdGhpcy5maWx0ZXJUb1dvcmxkUSA9IHF1YXQuY3JlYXRlKCk7XG5cbiAgLy8gU2V0IHRoZSBmaWx0ZXIgdG8gd29ybGQgdHJhbnNmb3JtLCBidXQgb25seSBmb3IgQW5kcm9pZC5cbiAgaWYgKGlzSU9TKCkpIHtcbi8vICAgIHRoaXMuZmlsdGVyVG9Xb3JsZFEuc2V0RnJvbUF4aXNBbmdsZShuZXcgVEhSRUUuVmVjdG9yMygxLCAwLCAwKSwgTWF0aC5QSS8yKTtcbiAgICBxdWF0LnNldEF4aXNBbmdsZSh0aGlzLmZpbHRlclRvV29ybGRRLCB2ZWMzLmZyb21WYWx1ZXMoMSwgMCwgMCksIE1hdGguUEkvMik7XG4gIH0gZWxzZSB7XG4vLyAgICB0aGlzLmZpbHRlclRvV29ybGRRLnNldEZyb21BeGlzQW5nbGUobmV3IFRIUkVFLlZlY3RvcjMoMSwgMCwgMCksIC1NYXRoLlBJLzIpO1xuICAgIHF1YXQuc2V0QXhpc0FuZ2xlKHRoaXMuZmlsdGVyVG9Xb3JsZFEsIHZlYzMuZnJvbVZhbHVlcygxLCAwLCAwKSwgLU1hdGguUEkvMik7XG4gIH1cblxuLy8gIHRoaXMud29ybGRUb1NjcmVlblEgPSBuZXcgVEhSRUUuUXVhdGVybmlvbigpO1xuICB0aGlzLndvcmxkVG9TY3JlZW5RID0gcXVhdC5jcmVhdGUoKTtcblxuICB0aGlzLnNldFNjcmVlblRyYW5zZm9ybV8oKTtcbn1cblxuQ29tcGxlbWVudGFyeU9yaWVudGF0aW9uLnByb3RvdHlwZS5vbkRldmljZU1vdGlvbkNoYW5nZV8gPSBmdW5jdGlvbihkZXZpY2VNb3Rpb24pIHtcbiAgdmFyIGFjY0dyYXZpdHkgPSBkZXZpY2VNb3Rpb24uYWNjZWxlcmF0aW9uSW5jbHVkaW5nR3Jhdml0eTtcbiAgdmFyIHJvdFJhdGUgPSBkZXZpY2VNb3Rpb24ucm90YXRpb25SYXRlO1xuICB2YXIgdGltZXN0YW1wUyA9IGRldmljZU1vdGlvbi50aW1lU3RhbXAgLyAxMDAwO1xuXG4gIHZhciBkZWx0YVMgPSB0aW1lc3RhbXBTIC0gdGhpcy5wcmV2aW91c1RpbWVzdGFtcFM7XG4gIGlmIChkZWx0YVMgPD0gTUlOX1RJTUVTVEVQIHx8IGRlbHRhUyA+IE1BWF9USU1FU1RFUCkge1xuICAgIGNvbnNvbGUud2FybignSW52YWxpZCB0aW1lc3RhbXBzIGRldGVjdGVkLiBUaW1lIHN0ZXAgYmV0d2VlbiBzdWNjZXNzaXZlICcgK1xuICAgICAgICAgICAgICAgICAnZ3lyb3Njb3BlIHNlbnNvciBzYW1wbGVzIGlzIHZlcnkgc21hbGwgb3Igbm90IG1vbm90b25pYycpO1xuICAgIHRoaXMucHJldmlvdXNUaW1lc3RhbXBTID0gdGltZXN0YW1wUztcbiAgICByZXR1cm47XG4gIH1cbi8qXG4gIHRoaXMuYWNjZWxlcm9tZXRlci5zZXQoLWFjY0dyYXZpdHkueCwgLWFjY0dyYXZpdHkueSwgLWFjY0dyYXZpdHkueik7XG4gIHRoaXMuZ3lyb3Njb3BlLnNldChyb3RSYXRlLmFscGhhLCByb3RSYXRlLmJldGEsIHJvdFJhdGUuZ2FtbWEpO1xuKi9cbiAgdmVjMy5zZXQodGhpcy5hY2NlbGVyb21ldGVyLCAtYWNjR3Jhdml0eS54LCAtYWNjR3Jhdml0eS55LCAtYWNjR3Jhdml0eS56KTtcbiAgdmVjMy5zZXQodGhpcy5neXJvc2NvcGUsIHJvdFJhdGUuYWxwaGEsIHJvdFJhdGUuYmV0YSwgcm90UmF0ZS5nYW1tYSk7XG4gIFxuICAvLyBJbiBpT1MsIHJvdGF0aW9uUmF0ZSBpcyByZXBvcnRlZCBpbiBkZWdyZWVzLCBzbyB3ZSBmaXJzdCBjb252ZXJ0IHRvXG4gIC8vIHJhZGlhbnMuXG4gIGlmIChpc0lPUygpKSB7XG4vLyAgICAgdGhpcy5neXJvc2NvcGUubXVsdGlwbHlTY2FsYXIoTWF0aC5QSSAvIDE4MCk7XG5cdHZlYzMuc2NhbGUodGhpcy5neXJvc2NvcGUsIHRoaXMuZ3lyb3Njb3BlLCBNYXRoLlBJIC8gMTgwKVxuICB9XG5cbiAgdGhpcy5maWx0ZXIuYWRkQWNjZWxNZWFzdXJlbWVudCh0aGlzLmFjY2VsZXJvbWV0ZXIsIHRpbWVzdGFtcFMpO1xuICB0aGlzLmZpbHRlci5hZGRHeXJvTWVhc3VyZW1lbnQodGhpcy5neXJvc2NvcGUsIHRpbWVzdGFtcFMpO1xuXG4gIHRoaXMucHJldmlvdXNUaW1lc3RhbXBTID0gdGltZXN0YW1wUztcbn07XG5cbkNvbXBsZW1lbnRhcnlPcmllbnRhdGlvbi5wcm90b3R5cGUub25TY3JlZW5PcmllbnRhdGlvbkNoYW5nZV8gPVxuICAgIGZ1bmN0aW9uKHNjcmVlbk9yaWVudGF0aW9uKSB7XG4gIHRoaXMuc2V0U2NyZWVuVHJhbnNmb3JtXygpO1xufTtcblxuQ29tcGxlbWVudGFyeU9yaWVudGF0aW9uLnByb3RvdHlwZS5zZXRTY3JlZW5UcmFuc2Zvcm1fID0gZnVuY3Rpb24oKSB7XG4gIHF1YXQuc2V0KHRoaXMud29ybGRUb1NjcmVlblEsIDAsIDAsIDAsIDEpO1xuICBzd2l0Y2ggKHdpbmRvdy5vcmllbnRhdGlvbikge1xuICAgIGNhc2UgMDpcbiAgICAgIGJyZWFrO1xuICAgIGNhc2UgOTA6XG4vLyAgICAgIHRoaXMud29ybGRUb1NjcmVlblEuc2V0RnJvbUF4aXNBbmdsZShuZXcgVEhSRUUuVmVjdG9yMygwLCAwLCAxKSwgLU1hdGguUEkvMik7XG5cdCAgICBxdWF0LnNldEF4aXNBbmdsZSh0aGlzLndvcmxkVG9TY3JlZW5RLCB2ZWMzLmZyb21WYWx1ZXMoMCwgMCwgMSksIC1NYXRoLlBJLzIpO1xuXG4gICAgICBicmVhaztcbiAgICBjYXNlIC05MDogXG4vLyAgICAgIHRoaXMud29ybGRUb1NjcmVlblEuc2V0RnJvbUF4aXNBbmdsZShuZXcgVEhSRUUuVmVjdG9yMygwLCAwLCAxKSwgTWF0aC5QSS8yKTtcblx0ICAgIHF1YXQuc2V0QXhpc0FuZ2xlKHRoaXMud29ybGRUb1NjcmVlblEsIHZlYzMuZnJvbVZhbHVlcygwLCAwLCAxKSwgTWF0aC5QSS8yKTtcbiAgICAgIGJyZWFrO1xuICAgIGNhc2UgMTgwOlxuICAgICAgYnJlYWs7XG4gIH1cbn07XG5cbkNvbXBsZW1lbnRhcnlPcmllbnRhdGlvbi5wcm90b3R5cGUuZ2V0T3JpZW50YXRpb24gPSBmdW5jdGlvbigpIHtcbiAgLy8gQ29udmVydCBmcm9tIGZpbHRlciBzcGFjZSB0byB0aGUgdGhlIHNhbWUgc3lzdGVtIHVzZWQgYnkgdGhlXG4gIC8vIGRldmljZW9yaWVudGF0aW9uIGV2ZW50LlxuICB2YXIgb3JpZW50YXRpb24gPSB0aGlzLmZpbHRlci5nZXRPcmllbnRhdGlvbigpO1xuXG4gIC8vIFByZWRpY3Qgb3JpZW50YXRpb24uXG4gIHRoaXMucHJlZGljdGVkUSA9IHRoaXMucG9zZVByZWRpY3Rvci5nZXRQcmVkaWN0aW9uKG9yaWVudGF0aW9uLCB0aGlzLmd5cm9zY29wZSwgdGhpcy5wcmV2aW91c1RpbWVzdGFtcFMpO1xuXG4gIC8vIENvbnZlcnQgdG8gVEhSRUUgY29vcmRpbmF0ZSBzeXN0ZW06IC1aIGZvcndhcmQsIFkgdXAsIFggcmlnaHQuXG4vLyAgdmFyIG91dCA9IG5ldyBUSFJFRS5RdWF0ZXJuaW9uKCk7XG4gIHZhciBvdXQgPSBxdWF0LmNyZWF0ZSgpO1xuXG4vKlxuICBvdXQuY29weSh0aGlzLmZpbHRlclRvV29ybGRRKTtcbiAgb3V0Lm11bHRpcGx5KHRoaXMucHJlZGljdGVkUSk7XG4gIG91dC5tdWx0aXBseSh0aGlzLndvcmxkVG9TY3JlZW5RKTtcbiovXG4gIHF1YXQuY29weShvdXQsIHRoaXMuZmlsdGVyVG9Xb3JsZFEpO1xuICBxdWF0Lm11bHRpcGx5KG91dCwgb3V0LCB0aGlzLnByZWRpY3RlZFEpO1xuICBxdWF0Lm11bHRpcGx5KG91dCwgb3V0LCB0aGlzLndvcmxkVG9TY3JlZW5RKVxuXG4gIHJldHVybiBvdXQ7XG59O1xuXG5mdW5jdGlvbiBpc0lPUygpIHtcblx0cmV0dXJuIC9pUGFkfGlQaG9uZXxpUG9kLy50ZXN0KG5hdmlnYXRvci5wbGF0Zm9ybSk7XG59XG5cbndpbmRvdy5Db21wbGVtZW50YXJ5T3JpZW50YXRpb24gPSBtb2R1bGUuZXhwb3J0cyA9IENvbXBsZW1lbnRhcnlPcmllbnRhdGlvbjsiLCIvKlxuICogQ29weXJpZ2h0IDIwMTUgR29vZ2xlIEluYy4gQWxsIFJpZ2h0cyBSZXNlcnZlZC5cbiAqIExpY2Vuc2VkIHVuZGVyIHRoZSBBcGFjaGUgTGljZW5zZSwgVmVyc2lvbiAyLjAgKHRoZSBcIkxpY2Vuc2VcIik7XG4gKiB5b3UgbWF5IG5vdCB1c2UgdGhpcyBmaWxlIGV4Y2VwdCBpbiBjb21wbGlhbmNlIHdpdGggdGhlIExpY2Vuc2UuXG4gKiBZb3UgbWF5IG9idGFpbiBhIGNvcHkgb2YgdGhlIExpY2Vuc2UgYXRcbiAqXG4gKiAgICAgaHR0cDovL3d3dy5hcGFjaGUub3JnL2xpY2Vuc2VzL0xJQ0VOU0UtMi4wXG4gKlxuICogVW5sZXNzIHJlcXVpcmVkIGJ5IGFwcGxpY2FibGUgbGF3IG9yIGFncmVlZCB0byBpbiB3cml0aW5nLCBzb2Z0d2FyZVxuICogZGlzdHJpYnV0ZWQgdW5kZXIgdGhlIExpY2Vuc2UgaXMgZGlzdHJpYnV0ZWQgb24gYW4gXCJBUyBJU1wiIEJBU0lTLFxuICogV0lUSE9VVCBXQVJSQU5USUVTIE9SIENPTkRJVElPTlMgT0YgQU5ZIEtJTkQsIGVpdGhlciBleHByZXNzIG9yIGltcGxpZWQuXG4gKiBTZWUgdGhlIExpY2Vuc2UgZm9yIHRoZSBzcGVjaWZpYyBsYW5ndWFnZSBnb3Zlcm5pbmcgcGVybWlzc2lvbnMgYW5kXG4gKiBsaW1pdGF0aW9ucyB1bmRlciB0aGUgTGljZW5zZS5cbiAqL1xudmFyIERFQlVHID0gZmFsc2U7XG5cbi8qKlxuICogR2l2ZW4gYW4gb3JpZW50YXRpb24gYW5kIHRoZSBneXJvc2NvcGUgZGF0YSwgcHJlZGljdHMgdGhlIGZ1dHVyZSBvcmllbnRhdGlvblxuICogb2YgdGhlIGhlYWQuIFRoaXMgbWFrZXMgcmVuZGVyaW5nIGFwcGVhciBmYXN0ZXIuXG4gKlxuICogQWxzbyBzZWU6IGh0dHA6Ly9tc2wuY3MudWl1Yy5lZHUvfmxhdmFsbGUvcGFwZXJzL0xhdlllckthdEFudDE0LnBkZlxuICpcbiAqIEBwYXJhbSB7TnVtYmVyfSBwcmVkaWN0aW9uVGltZVMgdGltZSBmcm9tIGhlYWQgbW92ZW1lbnQgdG8gdGhlIGFwcGVhcmFuY2Ugb2ZcbiAqIHRoZSBjb3JyZXNwb25kaW5nIGltYWdlLlxuICovXG4gXG4vLyBxdWF0XG52YXIgcXVhdCA9IHJlcXVpcmUoJ2dsLW1hdHJpeC1xdWF0Jyk7XG52YXIgdmVjMyA9IHJlcXVpcmUoJ2dsLW1hdHJpeC12ZWMzJyk7XG5cblxuZnVuY3Rpb24gUG9zZVByZWRpY3RvcihwcmVkaWN0aW9uVGltZVMpIHtcbiAgdGhpcy5wcmVkaWN0aW9uVGltZVMgPSBwcmVkaWN0aW9uVGltZVM7XG5cbiAgLy8gVGhlIHF1YXRlcm5pb24gY29ycmVzcG9uZGluZyB0byB0aGUgcHJldmlvdXMgc3RhdGUuXG4vLyAgdGhpcy5wcmV2aW91c1EgPSBuZXcgVEhSRUUuUXVhdGVybmlvbigpO1xuICB0aGlzLnByZXZpb3VzUSA9IHF1YXQuY3JlYXRlKCk7XG5cbiAgLy8gUHJldmlvdXMgdGltZSBhIHByZWRpY3Rpb24gb2NjdXJyZWQuXG4gIHRoaXMucHJldmlvdXNUaW1lc3RhbXBTID0gbnVsbDtcblxuICAvLyBUaGUgZGVsdGEgcXVhdGVybmlvbiB0aGF0IGFkanVzdHMgdGhlIGN1cnJlbnQgcG9zZS5cbi8vICB0aGlzLmRlbHRhUSA9IG5ldyBUSFJFRS5RdWF0ZXJuaW9uKCk7XG4gIHRoaXMuZGVsdGFRID0gcXVhdC5jcmVhdGUoKTtcblxuICAvLyBUaGUgb3V0cHV0IHF1YXRlcm5pb24uXG4vLyAgdGhpcy5vdXRRID0gbmV3IFRIUkVFLlF1YXRlcm5pb24oKTtcbiAgdGhpcy5vdXRRID0gcXVhdC5jcmVhdGUoKTtcblxufVxuXG5Qb3NlUHJlZGljdG9yLnByb3RvdHlwZS5nZXRQcmVkaWN0aW9uID0gZnVuY3Rpb24oY3VycmVudFEsIGd5cm8sIHRpbWVzdGFtcFMpIHtcbiAgaWYgKCF0aGlzLnByZXZpb3VzVGltZXN0YW1wUykge1xuLy8gICAgdGhpcy5wcmV2aW91c1EuY29weShjdXJyZW50USk7XG4gIFx0cXVhdC5jb3B5KHRoaXMucHJldmlvdXNRLCBjdXJyZW50USlcbiAgICB0aGlzLnByZXZpb3VzVGltZXN0YW1wUyA9IHRpbWVzdGFtcFM7XG4gICAgcmV0dXJuIGN1cnJlbnRRO1xuICB9XG5cbiAgLy8gQ2FsY3VsYXRlIGF4aXMgYW5kIGFuZ2xlIGJhc2VkIG9uIGd5cm9zY29wZSByb3RhdGlvbiByYXRlIGRhdGEuXG4gIHZhciBheGlzID0gdmVjMy5jcmVhdGUoKTtcbi8qXG4gIGF4aXMuY29weShneXJvKTtcbiAgYXhpcy5ub3JtYWxpemUoKTtcbiovXG4gIHZlYzMuY29weShheGlzLCBneXJvKTtcbiAgdmVjMy5ub3JtYWxpemUoYXhpcywgYXhpcyk7XG4gIFxuICB2YXIgYW5ndWxhclNwZWVkID0gdmVjMy5sZW5ndGgoZ3lybyk7XG5cbiAgLy8gSWYgd2UncmUgcm90YXRpbmcgc2xvd2x5LCBkb24ndCBkbyBwcmVkaWN0aW9uLlxuICBpZiAoYW5ndWxhclNwZWVkIDwgMjAgKiAoIE1hdGguUEkgLyAxODAgKSApIHtcbiAgICBpZiAoREVCVUcpIHtcbiAgICAgIGNvbnNvbGUubG9nKCdNb3Zpbmcgc2xvd2x5LCBhdCAlcyBkZWcvczogbm8gcHJlZGljdGlvbicsXG4gICAgICAgICAgICAgICAgICAoYW5ndWxhclNwZWVkICogKDE4MCAvIE1hdGguUEkpKS50b0ZpeGVkKDEpKTtcbiAgICB9XG4vKlxuICAgIHRoaXMub3V0US5jb3B5KGN1cnJlbnRRKTtcbiAgICB0aGlzLnByZXZpb3VzUS5jb3B5KGN1cnJlbnRRKTtcbiovXG4gICAgcXVhdC5jb3B5KHRoaXMub3V0USwgY3VycmVudFEpO1xuICAgIHF1YXQuY29weSh0aGlzLnByZXZpb3VzUSwgY3VycmVudFEpO1xuICAgIHJldHVybiB0aGlzLm91dFE7XG4gIH1cblxuICAvLyBHZXQgdGhlIHByZWRpY3RlZCBhbmdsZSBiYXNlZCBvbiB0aGUgdGltZSBkZWx0YSBhbmQgbGF0ZW5jeS5cbiAgdmFyIGRlbHRhVCA9IHRpbWVzdGFtcFMgLSB0aGlzLnByZXZpb3VzVGltZXN0YW1wUztcbiAgdmFyIHByZWRpY3RBbmdsZSA9IGFuZ3VsYXJTcGVlZCAqIHRoaXMucHJlZGljdGlvblRpbWVTO1xuXG4vKlxuICB0aGlzLmRlbHRhUS5zZXRGcm9tQXhpc0FuZ2xlKGF4aXMsIHByZWRpY3RBbmdsZSk7XG4gIHRoaXMub3V0US5jb3B5KHRoaXMucHJldmlvdXNRKTtcbiAgdGhpcy5vdXRRLm11bHRpcGx5KHRoaXMuZGVsdGFRKTtcblxuICB0aGlzLnByZXZpb3VzUS5jb3B5KGN1cnJlbnRRKTtcbiovXG4gIHF1YXQuc2V0QXhpc0FuZ2xlKHRoaXMuZGVsdGFRLCBheGlzLCBwcmVkaWN0QW5nbGUpO1xuICBxdWF0LmNvcHkodGhpcy5vdXRRLCB0aGlzLnByZXZpb3VzUSk7XG4gIHF1YXQubXVsdGlwbHkodGhpcy5vdXRRLCB0aGlzLm91dFEsIHRoaXMuZGVsdGFRKTtcblxuICBxdWF0LmNvcHkodGhpcy5wcmV2aW91c1EsIGN1cnJlbnRRKTtcblxuICByZXR1cm4gdGhpcy5vdXRRO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSBQb3NlUHJlZGljdG9yOyIsImZ1bmN0aW9uIFNlbnNvclNhbXBsZShzYW1wbGUsIHRpbWVzdGFtcFMpIHtcbiAgdGhpcy5zZXQoc2FtcGxlLCB0aW1lc3RhbXBTKTtcbn07XG5cblNlbnNvclNhbXBsZS5wcm90b3R5cGUuc2V0ID0gZnVuY3Rpb24oc2FtcGxlLCB0aW1lc3RhbXBTKSB7XG4gIHRoaXMuc2FtcGxlID0gc2FtcGxlO1xuICB0aGlzLnRpbWVzdGFtcFMgPSB0aW1lc3RhbXBTO1xufTtcblxuU2Vuc29yU2FtcGxlLnByb3RvdHlwZS5jb3B5ID0gZnVuY3Rpb24oc2Vuc29yU2FtcGxlKSB7XG4gIHRoaXMuc2V0KHNlbnNvclNhbXBsZS5zYW1wbGUsIHNlbnNvclNhbXBsZS50aW1lc3RhbXBTKTtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gU2Vuc29yU2FtcGxlOyJdfQ==
