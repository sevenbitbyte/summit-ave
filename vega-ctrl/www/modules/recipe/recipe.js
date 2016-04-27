"use strict()";

function Recipe($state, $stateParams, $ionicScrollDelegate, DataStore, $interval) {
  console.log("Recipe");

  var recipeCtrl = this;

  var fields = ['ingredients', 'tools', 'steps'];

  recipeCtrl.show = null;

  recipeCtrl.nextStep = 0;

  recipeCtrl.goHome = function () {
    $state.go('home');
  }

  recipeCtrl.stateChanged = function (checked) {
    if(checked) {
      recipeCtrl.nextStep++;
    } else {
      recipeCtrl.nextStep--;
    }
  }

  recipeCtrl.toNextStage = function () {
    recipeCtrl.nextStep = 0;
    ++recipeCtrl.show;
    $ionicScrollDelegate.scrollTop(true);
  }

  recipeCtrl.finish = function () {
    if(DataStore.user.finished[recipeCtrl.content.name]) {
      DataStore.user.exp += 100;
    } else {
      DataStore.user.finished[recipeCtrl.content.name] = true;
      DataStore.user.exp += 300 * recipeCtrl.content.level;
    }
    if(DataStore.user.exp >= DataStore.user.level * 900) {
      ++DataStore.user.level;
    }
    DataStore.user.$save()
      .then(function (ref) {
        ref.key() === DataStore.user; // true
      }, function (error) {
        console.log("Error:", error);
      });

    $state.go('home');
  }

  if(!DataStore.recipes || $stateParams.index == null) {
    console.log(DataStore.recipes);
    $state.go('home');
  } else {

    console.log($stateParams.index);
    console.log(DataStore.recipes);

    recipeCtrl.content = angular.copy(DataStore.recipes[$stateParams.index]);
    recipeCtrl.show = 0;
    console.log(recipeCtrl.content);
  }
}
