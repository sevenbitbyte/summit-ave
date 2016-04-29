(function () {
	"use strict";
	angular.module('app')
		.factory('ROSSensorsService', ['ROSService', ROSSensorsService]);

	function ROSSensorsService(ROSService) {
		console.log('ROSSensorsService');

		var topicName = '/turtlebot/sensor_state';
		var messageType = 'create_node/TurtlebotSensorState';

		var keys = {
			bump: 'bumps_wheeldrops',
			temperature: 'temperature',
			charge: 'charge',
			capacity: 'capacity'
		}

		var sensors = {};

		function vibrationHandle(msg) {
			var bumpDegree = msg[keys.bump];

			if(navigator.vibrate && bumpDegree > 0 && bumpDegree < 4) {
				// vibration API supported
				console.log("Should be vibrating");
				navigator.vibrate(1);
			}
		}

		function getBatteryPercentage() {
			if (!sensors.msg[keys.charge]) return null;
			return sensors.msg[keys.charge] / sensors.msg[keys.capacity] * 100;
		}

		function getTemperature() {
			if (!sensors.msg[keys.temperature]) return null;
			return sensors.msg[keys.temperature];
		}

		function sensorsHandle(msg) {
			sensors.msg = msg;
			vibrationHandle(msg);
		}

		ROSService.start()
			.then(function (ros) {
				sensors.topic = ROSService.generateTopic(ros, topicName, messageType);
				sensors.topic.subscribe(sensorsHandle);
			})

		return {
			battery: sensors.battery,
			getBatteryPercentage: getBatteryPercentage,
			getTemperature: getTemperature
		};
	}
}())
