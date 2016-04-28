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
			}
		}

		ROSService.start()
			.then(function (ros) {

				console.log(ros);

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
			})
	}
}())
