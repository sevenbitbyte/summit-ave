(function () {
	"use strict";
	angular.module('app')
		.controller('Control', ['JoystickService', 'ROSService', 'ROSAudioService', '$timeout', Control])

	function Control(JoystickService, ROSService, ROSAudioService, $timeout) {
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
			gantry: {
				label: "Gantry"
			},
			head: {
				label: "Head"
			},
			jammer: {
				label: "Jammer"
			}
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
					color: 'black',
					mode: 'static',
					position: {
						left: '50%',
						top: '45%'
					}
				};

				var e = ROSService.eyePos();

				driveJoystick = nipplejs.create(options);

				driveJoystick.on('move', function (evt, data) {
					// console.log(data);
					var angular = Math.cos(data.angle.radian);

					var eye = new ROSLIB.Message({
						data: [
							Math.round((angular * 7) + 8), 7,
							Math.round((angular * 7) + 23), 7,
						]
					})

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
