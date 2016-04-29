(function () {
	"use strict";
	angular.module('app')
		.controller('Utility', ['$scope', 'ROSService', Utility])

	function Utility($scope, ROSService) {
		console.log("utilityCtrl");

		var utilityCtrl = this;

		utilityCtrl.cards = {
			services: {
				label: 'Services'
			},
			nodes: {
				label: 'Nodes'
			},
			topics: {
				label: 'Topics'
			},
			params: {
				label: 'Params'
			}
		}

		ROSService.start()
			.then(function (ros) {

				ros.getServices(function (msg) {

					$scope.$evalAsync(function () {
						utilityCtrl.cards.services.msg = msg;
					})
				}, function (err) {
					return console.log(err);
				});

				ros.getTopics(function (msg) {

					$scope.$evalAsync(function () {
						utilityCtrl.cards.topics.msg = msg;
					})
				}, function (err) {
					return console.log(err);
				});

				ros.getNodes(function (msg) {

					$scope.$evalAsync(function () {
						utilityCtrl.cards.nodes.msg = msg;
					})

				}, function (err) {
					return console.log(err);
				});

				ros.getParams(function (msg) {

					$scope.$evalAsync(function () {
						utilityCtrl.cards.params.msg = msg;
					})

				}, function (err) {
					return console.log(err);
				});

			})
	}
}())
