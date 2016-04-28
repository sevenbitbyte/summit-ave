function ROSService(Utils, $location, $q) {
	console.log('ROSService');

	var videoPort = 8000;

	var rosPort = 4000;

	var host = (($location.host() === 'localhost') || Utils.isDev()) ? '192.168.1.127' : $location.host();

	var ros;

	var eyePos;

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

					eyePos = new ROSLIB.Topic({
						ros: ros,
						name: '/head/eye_positions',
						messageType: 'std_msgs/Int8MultiArray'
					})

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
