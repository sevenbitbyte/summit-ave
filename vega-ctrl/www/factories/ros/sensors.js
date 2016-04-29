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
		}

		var sensors = {};

		function vibrationHandle(msg) {
			var bumpDegree = msg[keys.bump];

			if(navigator.vibrate && bumpDegree > 0 && bumpDegree < 4) {
				// vibration API supported
				console.log("Should be vibrating");
				navigator.vibrate(bumpDegree);
			}
		}

    function getBatteryRate(msg) {

    }

		function sensorsHandle (msg) {

      vibrationHandle(msg);

      // console.log(msg);

			// console.log('Received Sensor message : ', msg[keys.bump]);
		}

		ROSService.start()
			.then(function (ros) {
				sensors.listener = ROSService.generateTopicListener(ros, topicName, messageType, sensorsHandle);
      })

		return {

		};
	}
}())
