// Ionic Starter App

// angular.module is a global place for creating, registering and retrieving Angular modules
// 'starter' is the name of this angular module example (also set in a <body> attribute in index.html)
// the 2nd parameter is an array of 'requires'
// 'starter.controllers' is found in controllers.js

(function () {
	"use strict";

	angular.module('app', [
    'ionic',
		'angular.circular-slider'
  ])
		.config(['$stateProvider', '$urlRouterProvider', '$ionicConfigProvider', config])
		.run(['$ionicPlatform', run])

	function config($stateProvider, $urlRouterProvider, $ionicConfigProvider) {

		// if none of the above states are matched, use this as the fallback
		console.log('config');

		// $ionicConfigProvider.views.maxCache(0);

		$ionicConfigProvider.tabs.position('bottom');

		$urlRouterProvider.otherwise('/controller');

		$stateProvider
			.state('tabs', {
				url: "",
				abstract: true,
				templateUrl: "modules/tabs.html"
			})
			.state('tabs.controller', {
				url: '/controller',
				views: {
					'controller-tab': {
						templateUrl: 'modules/controller/controller.html',
						controller: 'Controller as controllerCtrl'
					}
				}
			})
			.state('tabs.status', {
				url: '/status',
				views: {
					'status-tab': {
						templateUrl: 'modules/status/status.html',
						controller: 'Status as statusCtrl'
					}
				}
			})
			.state('tabs.utility', {
				url: '/utility',
				views: {
					'utility-tab': {
						templateUrl: 'modules/utility/utility.html',
						controller: 'Utility as utilityCtrl'
					}
				}
			})
	}

	function run($ionicPlatform) {
		$ionicPlatform.ready(function () {
			// Hide the accessory bar by default (remove this to show the accessory bar above the keyboard
			// for form inputs)
			console.log('run');
			if(window.cordova && window.cordova.plugins.Keyboard) {
				cordova.plugins.Keyboard.hideKeyboardAccessoryBar(true);
				cordova.plugins.Keyboard.disableScroll(true);
			}
			if(window.StatusBar) {
				// org.apache.cordova.statusbar required
				StatusBar.styleDefault();
			}
		});
	}
}())
