"use strict()";

function Control($http, $state, $ionicLoading, $ionicPopup, DataStore, JoystickService, $timeout) {
	console.log("ControlCtrl");

	var controlCtrl = this;

	var dev = true;

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

			driveJoystick = nipplejs.create(options);
			break;
		default:
			return;
		}
	}

	JoystickService.start(function () {
		controlCtrl.updateJoysticks();
	})

}
