// vec3
// quat
var vec3 = require('gl-matrix-vec3');
var quat = require('gl-matrix-quat');

var GyroNorm = require('../lib/gyronorm/gyronorm.js');

var ComplementaryFilter = require('./complementary-filter.js');
var PosePredictor = require('./pose-predictor.js');


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