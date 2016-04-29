(function () {
	"use strict";
	angular.module('app')
		.factory('ROSAudioService', ['ROSService','$http', ROSAudioService]);

	function ROSAudioService(ROSService, $http) {
		console.log('ROSAudioService');

		var serverName = '/housebot/audio_action';
		var actionName = 'housebot_msgs/PlayAudioAction';

		var audioPlayer = {}

		var audioGoal = undefined;

		var clipsAPI = './db/clips.json'

		var configureAudio = function (source, path, clip) {
			var goal = {
				source: source,
				path: path,
				start_ms: clip.startms,
				end_ms: clip.endms
			}

			audioGoal = ROSService.generateGoal(audioPlayer.action, goal);

			console.log(audioGoal);
		}

		var playAudio = function (timeoutCb, resultCb) {
			// Print out their output into the terminal.
			audioGoal.on('feedback', function (feedback) {
				console.log('Feedback: ', feedback);
			});

			audioGoal.on('status', function (status) {
				console.log('status: ', status);
			});

			audioGoal.on('result', function (result) {
				console.log('Final Result: ', result);

				if(resultCb) {
					resultCb()
				}

				audioGoal = undefined;
			});

			audioGoal.once('timeout', function () {
				if(!audioGoal.status) {
					console.log('AudioGoalTimeout');
					console.log(audioGoal);

					if(timeoutCb) {
						timeoutCb()
					}

					audioGoal.cancel();
					audioGoal = undefined;
				}
			});

			audioGoal.send(1800);
		}

		var playClip = function (clip, finishCallback) {
			configureAudio(1, 'r2d2.mp3', clip);

			playAudio(function () {
				alert("Couldn't play the clip...");

			}, finishCallback)
		}

		ROSService.start()
			.then(function (ros) {
				audioPlayer.action = ROSService.generateActionClient(ros, serverName, actionName);
			})


		return {
			playClip: playClip,
			getAvailableClips: function () {
				return $http.get(clipsAPI);
			}
		};
	}
}())
