"use strict()";

function Control($http, $state, $ionicLoading, $ionicPopup, DataStore, JoystickService, ROSService, $timeout) {
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

	controlCtrl.sounds = [
		"a", "b", "c", "d", "Swqfwqf", "c", "d", "Swqfwqf", "c", "d", "Swqfwqf", "c", "d", "Swqfwqf", "Swqfwqf", "c", "d", "Swqfwqf", "Swqfwqf", "c", "d", "Swqfwqf"
	]

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

			// var angular = 1;

			driveJoystick = nipplejs.create(options);

			driveJoystick.on('move', function (evt, data) {
				// console.log(data);
				var angular = Math.cos(data.angle.radian) * 0.5;

				var eye = new ROSLIB.Message({
					data: [
							Math.round((angular * 7) + 8), 7,
							Math.round((angular * 7) + 23), 7,
						]
				})
				e.publish(eye);
			})

			// driveJoystick.on('plain:right', function (evt, data) {
			// 	console.log("Turned right");
			// 	console.log(evt);
			//
			// 	var eye = new ROSLIB.Message({
			// 		data: [
			// 			Math.round((angular * 7) + 8), 7,
			// 			Math.round((angular * 7) + 23), 7,
			// 		]
			// 	})
			//
			// 	e.publish(eye);
			// })
			//
			// driveJoystick.on('plain:left', function (evt, data) {
			// 	console.log("Turned left");
			// 	console.log(evt);
			//
			// 	var eye = new ROSLIB.Message({
			// 		data: [
			// 			Math.round((-angular * 7) + 8), 7,
			// 			Math.round((-angular * 7) + 23), 7,
			// 		]
			// 	})
			//
			// 	e.publish(eye);
			// })

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
