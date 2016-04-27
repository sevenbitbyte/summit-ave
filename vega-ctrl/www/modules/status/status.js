"use strict()";

function Status($http, $state, $ionicLoading, $ionicPopup, DataStore) {
	console.log("StatusCtrl");

	var statusCtrl = this;

	var dev = true;

	statusCtrl.cards = {
		users: {
			label: 'Connected Users'
		},
		level: {
			label: 'Battery Level'
		},
		hd: {
			label: 'Hard Drives Space'
		},
		bandwidth: {
			label: 'Bandwidth VS'
		},
		version: {
			label: 'Software Version'
		},
		wifi: {
			label: 'Wifi Networks'
		},
		services: {
			label: 'Services'
		}
	}

}
