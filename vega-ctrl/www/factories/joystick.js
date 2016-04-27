function JoystickService(Utils) {
	console.log('JoystickService');

	var isReady = false;

	var start = function (callback) {
		if(isReady) {
			return callback();
		} else {
			Utils.loadScript("./lib/nipplejs/dist/nipplejs.min.js",
				function () {
					console.log("Nipple Loaded!");

          isReady = true;

					return callback();
				});
		}
	}

	return {
		start: start
	};
}
