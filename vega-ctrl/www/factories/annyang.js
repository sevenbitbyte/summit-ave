(function () {
	"use strict";
	angular.module('app')
		.factory('AnnyangService', ['Utils', AnnyangService])

	function AnnyangService(Utils) {
		console.log('AnnyangService');

		var isReady = false;

		var start = function (callback) {
			if(isReady) {
				return callback();
			} else {
				Utils.loadScript("https://cdnjs.cloudflare.com/ajax/libs/annyang/2.4.0/annyang.min.js",
					function () {
						console.log("Annyang Loaded!");
						isReady = true;
						return callback();
					});
			}
		}

		return {
			start: start
		};
	}
}())
