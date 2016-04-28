(function () {
	"use strict";
	angular.module('app')
		.factory('JoystickService', ['Utils', '$q', JoystickService])

	function JoystickService(Utils, $q) {
		console.log('JoystickService');

		var deferred = $q.defer();

		Utils.loadScript("./lib/nipplejs/dist/nipplejs.min.js",
			function () {
				console.log("Nipple Loaded!");
				deferred.resolve();
			});

		function start() {
			return deferred.promise;
		}

		return {
			start: start
		};
	}
}())
