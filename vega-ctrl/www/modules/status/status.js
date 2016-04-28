(function () {
	"use strict";
	angular.module('app')
		.controller('Status', ['$scope', 'ROSService', Status])

	function Status($scope, ROSService) {
		console.log("StatusCtrl");

		var statusCtrl = this;

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
			}
		}

		ROSService.start()
			.then(function (ros) {

				console.log(ros);

			})
	}
}())
