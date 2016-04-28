function ROSService(Utils, $location, $q) {
	console.log('ROSService');

	var videoPort = 8000;

	var rosPort = 4000;

	var host = (($location.host() === 'localhost') || Utils.isDev()) ? '192.168.1.127' : $location.host();

	var ros;

	var eyePos = {
		name: '/head/eye_positions',
		messageType: 'std_msgs/Int8MultiArray'
	};

	var bumpSensor = {
		name: '/turtlebot/sensor_state',
		messageType: 'create_node/TurtlebotSensorState',
		bumpkey: 'bumps_wheeldrops',
		scb: function (msg) {
			// console.log('Received message on ' + bumpSensor.topic.name + ': ', msg[bumpSensor.bumpkey]);
			var bumpDegree = msg[bumpSensor.bumpkey];

			if(navigator.vibrate && bumpDegree != 0) {
				// vibration API supported
				console.log("Should be vibrating");
				navigator.vibrate(bumpDegree);
			}

		}
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

	function generateTopicListener(name, messageType, scb) {
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

					ros = new ROSLIB.Ros({
						url: 'ws://' + host + ':' + rosPort
					});

					ros.on('connection', connectionHandler);
					ros.on('error', errorHandler);
					ros.on('close', closeHandler);

					eyePos.topic = generateTopicListener(eyePos.name, eyePos.messageType);

					bumpSensor.topic = generateTopicListener(bumpSensor.name, bumpSensor.messageType, bumpSensor.scb);

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
		ros: function () {
			return ros;
		},
		eyePos: function () {
			return eyePos;
		}
	};
}
