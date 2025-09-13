angular.module('beamng.apps')
.directive('uiAppTemplate', [function () {
  return {
    templateUrl: '/ui/modules/apps/uiAppTemplate/app.html',
    replace: true,
    restrict: 'EA',
    scope: true,
    controller: ['$scope', function ($scope) {
      // List any required streams here.
      var streamsList = ['electrics'];
      StreamsManager.add(streamsList);

      // Clean up when the app is closed.
      $scope.$on('$destroy', function () {
        StreamsManager.remove(streamsList);
      });

      // Example data exposed to the template.
      $scope.exampleData = 0;

      $scope.reset = function () {
        $scope.exampleData = 0;
      };

      // Update exampleData whenever streams are updated.
      $scope.$on('streamsUpdate', function (event, streams) {
        $scope.$evalAsync(function () {
          if (!streams.electrics) { return; }
          // Example: show current wheel speed.
          $scope.exampleData = streams.electrics.wheelspeed;
        });
      });
    }]
  };
}]);
