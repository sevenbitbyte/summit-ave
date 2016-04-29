(function () {
	"use strict";
	angular.module('app')
		.controller('Control', ['JoystickService', 'ROSService', 'ROSAudioService', 'ROSMotorsService', '$timeout', Control])

	function Control(JoystickService, ROSService, ROSAudioService, ROSMotorsService, $timeout) {
		console.log("ControlCtrl");

		var controlCtrl = this;

		controlCtrl.selected = 'drive';

		controlCtrl.type = {
			drive: {
				label: "Drive",
			},
			audio: {
				label: "Audio"
			},
			// gantry: {
			// 	label: "Gantry"
			// },
			// head: {
			// 	label: "Head",
			// 	rotation: 0
			// },
			jammer: {
				label: "Jammer",
				elbow: {
					rotation: 0
				},
				shoulder: {
					rotation: 0
				}
			}
		}

		controlCtrl.turnElbow = function (r100) {
			ROSMotorsService.turnElbow(r100/100);
		}

		controlCtrl.turnShoulder = function (r100) {
			ROSMotorsService.turnShoulder(r100/100);
		}


		ROSAudioService.getAvailableClips()
			.then(
				function (result) {
					controlCtrl.soundclips = result.data;
				},
				function (err) {
					console.log(err);
				}
			);

		controlCtrl.playClip = function (clip) {
			ROSAudioService.playClip(clip, function (result) {
				console.log(result);
			})
		}

		var driveJoystick;

		controlCtrl.updateJoysticks = function () {
			switch(controlCtrl.selected) {
			case 'drive':
				var joyfield = document.getElementById('joyfield');

				if(!joyfield) {
					$timeout(function () {
						controlCtrl.updateJoysticks()
					}, 1);
					return
				}

				var options = {
					zone: joyfield,
					size: 180,
					color: 'black',
					mode: 'static',
					position: {
						left: '50%',
						top: '45%'
					}
				};

				var e = ROSService.eyePos();

				var vc = ROSService.velocityCtrl();

				driveJoystick = nipplejs.create(options);

				driveJoystick.on('move', function (evt, data) {
					// console.log(data);
					var angular = Math.cos(data.angle.radian);

					var distance = data.distance / 180;

					if(angular > 0.5 || angular < 0.0) {
						distance = -distance / 2.5
					} else {
						distance = distance / 2
					}

					var movement = new ROSLIB.Message({
						linear: {
							x: distance,
							y: 0,
							z: 0
						},
						angular: {
							z: angular,
							x: 0,
							y: 0
						}
					})

					var eye = new ROSLIB.Message({
						data: [
							Math.round((angular * 7) + 8), 7,
							Math.round((angular * 7) + 23), 7,
						]
					})

					vc.topic.publish(movement);

					e.topic.publish(eye);
				})

				break;
			default:
				return;
			}
		}

		JoystickService.start()
			.then(function () {
				ROSService.start()
					.then(function (ros) {
						controlCtrl.updateJoysticks();
					})
			})
	}
}())
