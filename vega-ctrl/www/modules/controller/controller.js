(function () {
	"use strict";
	angular.module('app')
		.controller('Controller', ['ROSSensorsService', Controller])

	function Controller(ROSSensorsService) {
		console.log("ControllerCtrl");

		var controllerCtrl = this;

	}
}())
