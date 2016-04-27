function ROSService(Utils, $location) {
	console.log('ROSService');

	var videoPort = 8000;

	var rosPort = 4000;

	var host = $location.host() === 'localhost' ? '192.168.1.127' : $location.host();

	var isReady = false;

	var ros;

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

	var start = function (callback) {
		if(isReady) {
			return callback(ros);
		} else {
			Utils.loadScript("./lib/eventemitter2/lib/eventemitter2.js",
				function () {
					console.log("Event Emitter Loaded!");

					Utils.loadScript("./lib/roslib/build/roslib.min.js",
						function () {
							console.log("Ros Loaded!");

							isReady = true;

							var ros = new ROSLIB.Ros({
								url: 'ws://' + host + ':' + rosPort
							});

							ros.on('connection', connectionHandler);
							ros.on('error', errorHandler);
							ros.on('close', closeHandler);

							return callback(ros);
						});
				});
		}
	}

	return {
		host: host,
		videoPort: videoPort,
		start: start,
		ros: ros
	};
}
