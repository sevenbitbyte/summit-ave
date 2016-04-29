(function () {
	"use strict";
	angular.module('app')
		.controller('Stream', ['ROSService', Stream])

	function Stream(ROSService) {
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

		ROSService.start()
			.then(function (ros) {

			})
	}
}())
