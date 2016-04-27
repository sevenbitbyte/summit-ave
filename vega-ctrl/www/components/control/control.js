"use strict()";

function Control($http, $state, $ionicLoading, $ionicPopup, DataStore) {
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

}
