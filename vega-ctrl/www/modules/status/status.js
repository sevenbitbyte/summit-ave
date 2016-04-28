"use strict()";

function Status($http, $scope, $state, $ionicLoading, $ionicPopup, DataStore, ROSService) {
	console.log("StatusCtrl");

	var statusCtrl = this;

	statusCtrl.cards = {
		users: {
			label: 'Connected Users'
		},
		level: {
			label: 'Battery Level'
		},
		hd: {
			label: 'Hard Drives Space'
		},
		bandwidth: {
			label: 'Bandwidth VS'
		},
		version: {
			label: 'Software Version'
		},
		wifi: {
			label: 'Wifi Networks'
		},
		services: {
			label: 'Services'
		}
	}

	ROSService.start()
		.then(function (ros) {
			console.log(ros);
			ros.getServices(function (msg) {

				$scope.$evalAsync(function () {
					statusCtrl.cards.services.msg = msg;
					console.log(statusCtrl.cards.services);
				})



			}, function (err) {
				return console.log(err);
			});



		})

}
