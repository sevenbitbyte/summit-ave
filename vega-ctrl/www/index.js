// Ionic Starter App

// angular.module is a global place for creating, registering and retrieving Angular modules
// 'starter' is the name of this angular module example (also set in a <body> attribute in index.html)
// the 2nd parameter is an array of 'requires'
// 'starter.controllers' is found in controllers.js

angular.module('app', [
    'ionic'
  ])
	.config(config)
	.run(run)
	.factory('DataStore', DataStore)
	.factory('Utils', Utils)
	.factory('AnnyangService', AnnyangService)
  .factory('ROSService', ROSService)
  .factory('JoystickService', JoystickService)
	.controller('Home', Home)
	.controller('Status', Status)
  .controller('Control', Control)
  .controller('Stream', Stream)

function config($stateProvider, $urlRouterProvider, $ionicConfigProvider) {

	// if none of the above states are matched, use this as the fallback
	console.log('config');

	// $ionicConfigProvider.views.maxCache(0);

  $ionicConfigProvider.tabs.position('bottom');

	$urlRouterProvider.otherwise('/home');

	$stateProvider
		.state('tabs', {
			url: "",
			abstract: true,
			templateUrl: "modules/tabs.html"
		})
		.state('tabs.home', {
			url: '/home',
			views: {
				'home-tab': {
					templateUrl: 'modules/home/home.html',
					controller: 'Home as homeCtrl'
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
		// .state('app.deal', {
		//   url: '/deallists/:dealId',
		//   views: {
		//     'menuContent': {
		//       templateUrl: 'templates/playlist.html',
		//       controller: 'DealListCtrl'
		//     }
		//   }
		// });
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
