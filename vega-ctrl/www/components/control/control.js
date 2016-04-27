"use strict()";

function Control($http, $state, $ionicLoading, $ionicPopup, DataStore, JoystickService) {
	console.log("ControlCtrl");

	var controlCtrl = this;

	var dev = true;

	controlCtrl.selected = 'joystick';

	controlCtrl.type = {
		joystick: {
			label: "Joystick",
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

	JoystickService.start(function () {

		var options = {
        zone: document.getElementById('joyfield'),
				color: 'black',
				mode : 'static',
				position: {left: '50%', top: '45%'}
    };

		var manager = nipplejs.create(options);
	})

}
