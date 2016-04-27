"use strict()";

function Stream($http, $state, $ionicLoading, $ionicPopup, DataStore, ROSService) {
	console.log("StreamCtrl");

	var streamCtrl = this;

	var dev = true;

	streamCtrl.url = ROSService.host;
  streamCtrl.port = ROSService.videoPort;

	streamCtrl.topics = {
    "Color": '/camera/rgb/image_rect_color',
    "Depth": '/camera/depth/image',
    // "Infrared": '/camera/ir/image',
	}

	streamCtrl.selectedTopic = streamCtrl.topics["Color"];

	ROSService.start(function (ros) {
		// $scope.$apply(function () {
		// streamCtrl.url = "192.168.1.127";
		// })
	})

}
