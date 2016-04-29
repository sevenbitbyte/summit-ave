(function () {
	"use strict";

	angular.module('app')
		.factory('ROSService', ['Utils', '$location', '$q', ROSService]);

	function ROSService(Utils, $location, $q) {
		console.log('ROSService');

		var videoPort = 8080;

		var rosPort = 4000;

		var host = (($location.host() === 'localhost') || Utils.isDev()) ? '192.168.1.104' : $location.host();

		var eyePos = {
			topic: '/head/eye_positions',
			messageType: 'std_msgs/Int8MultiArray'
		};

		var velocityCtrl = {
			topic: '/cmd_vel',
			messageType: 'geometry_msgs/Twist'
		};

		/*
		  CDN links:
		    http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js

		    http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js
		*/

		function connectionHandler() {
			console.log('Connected to websocket server.');
		}

		function errorHandler(error) {
			console.log('Error connecting to websocket server: ', error);
		}

		function closeHandler() {
			console.log('Connection to websocket server closed.');
		}

		function generateGoal(actionClient, goalMessage) {
			var goal = new ROSLIB.Goal({
				actionClient: actionClient,
				goalMessage: goalMessage
			});

			return goal;
		}

		function generateActionClient(ros, serverName, actionName) {
			var action = new ROSLIB.ActionClient({
				ros: ros,
				serverName: serverName,
				actionName: actionName
			});

			return action;
		}

		function generateTopic(ros, topic, messageType) {
			var listener = new ROSLIB.Topic({
				ros: ros,
				name: topic,
				messageType: messageType
			});

			return listener;
		}

		var deferred = $q.defer();

		Utils.loadScript("./lib/eventemitter2/lib/eventemitter2.js",
			function () {
				console.log("Event Emitter Loaded!");

				Utils.loadScript("./lib/roslib/build/roslib.min.js",
					function () {
						console.log("Ros Initialized!");

						var ros = new ROSLIB.Ros({
							url: 'ws://' + host + ':' + rosPort
						});

						ros.on('connection', connectionHandler);
						ros.on('error', errorHandler);
						ros.on('close', closeHandler);

						eyePos.topic = generateTopic(ros, eyePos.topic, eyePos.messageType);

						velocityCtrl.topic = generateTopic(ros, velocityCtrl.topic, velocityCtrl.messageType);

						console.log(ros);

						deferred.resolve(ros);
					});
			});

		function start() {
			return deferred.promise;
		}

		return {
			host: host,
			videoPort: videoPort,
			start: start,
			eyePos: function () {
				return eyePos;
			},
			velocityCtrl: function () {
				return velocityCtrl;
			},
			generateActionClient: generateActionClient,
			generateTopic: generateTopic,
			generateGoal: generateGoal
		};
	}
}())
