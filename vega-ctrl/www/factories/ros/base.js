(function () {
	"use strict";

	angular.module('app')
		.factory('ROSService', ['Utils', '$location', '$q', ROSService]);

	function ROSService(Utils, $location, $q) {
		console.log('ROSService');

		var videoPort = 8000;

		var rosPort = 4000;

		var host = (($location.host() === 'localhost') || Utils.isDev()) ? '192.168.1.127' : $location.host();

		var eyePos = {
			name: '/head/eye_positions',
			messageType: 'std_msgs/Int8MultiArray'
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

		function generateTopicListener(ros, name, messageType, scb) {
			var listener = new ROSLIB.Topic({
				ros: ros,
				name: name,
				messageType: messageType
			});

			if(scb) listener.subscribe(scb)

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

						eyePos.topic = generateTopicListener(ros, eyePos.name, eyePos.messageType);

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
			generateActionClient: generateActionClient,
			generateTopicListener: generateTopicListener,
			generateGoal: generateGoal
		};
	}
}())
