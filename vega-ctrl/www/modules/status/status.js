(function () {
	"use strict";
	angular.module('app')
		.controller('Status', ['$scope', '$interval', 'ROSSensorsService', Status])

	function Status($scope, $interval, ROSSensorsService) {
		console.log("StatusCtrl");

		var statusCtrl = this;

		statusCtrl.cards = {
			users: {
				label: 'Connected Users'
			},
			battery: {
				label: 'Battery Level',
				percentage : 100,
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
			temperature : {
				label: "Temperature"
			}
		}

		$interval(function () {
			statusCtrl.cards.battery.percentage = ROSSensorsService.getBatteryPercentage();

			statusCtrl.cards.temperature.celsius = ROSSensorsService.getTemperature();
		}, 1800);

	}
}())
