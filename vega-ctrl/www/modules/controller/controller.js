(function () {
	"use strict";
	angular.module('app')
		.controller('Controller', [Controller])

	function Controller() {
		console.log("ControllerCtrl");

		var controllerCtrl = this;

		controllerCtrl.colors = ["positive", "royal", "balanced"];
	}
}())
