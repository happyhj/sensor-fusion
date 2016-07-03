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
var vec3 = require('gl-matrix-vec3');
var quat = require('gl-matrix-quat');

//var Util = require('./util.js');
var SensorSample = require('./sensor-sample.js');

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