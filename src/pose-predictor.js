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
var quat = require('gl-matrix-quat');
var vec3 = require('gl-matrix-vec3');


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