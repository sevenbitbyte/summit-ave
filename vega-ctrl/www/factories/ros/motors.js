(function () {
	"use strict";
	angular.module('app')
		.factory('ROSMotorsService', ['ROSService', ROSMotorsService]);

	function ROSMotorsService(ROSService) {
		console.log('ROSMotorsService');

		var elbow = {
			topic: '/vega4_scara_elbow_controller/command',
			messageType: 'std_msgs/Float64'
		}

		var shoulder = {
			topic: '/vega4_scara_shoulder_controller/command',
			messageType: 'std_msgs/Float64'
		}

		function turnElbow(rad) {
			var msg = new ROSLIB.Message({
				data: rad
			})
			console.log("TURNING ELBOW BY :", rad);
			return elbow.topic.publish(msg);
		}

		function turnShoulder(rad) {
			var msg = new ROSLIB.Message({
				data: rad
			})

			console.log("TURNING SHOULDER BY :", rad);
			return shoulder.topic.publish(msg)
		}


		ROSService.start()
			.then(function (ros) {
				elbow.topic = ROSService.generateTopic(ros, elbow.topic, elbow.messageType);
				shoulder.topic = ROSService.generateTopic(ros, shoulder.topic, shoulder.messageType);
			})

		return {
			turnElbow: turnElbow,
			turnShoulder: turnShoulder
		};
	}
}())
