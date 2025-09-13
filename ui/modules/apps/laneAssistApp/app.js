angular.module('beamng.apps')
.directive('laneAssistApp', [function () {
  return {
    templateUrl: '/ui/modules/apps/laneAssistApp/app.html',
    replace: true,
    restrict: 'EA',
    scope: true,
    controller: ['$scope', '$element', function ($scope, $element) {
      var canvas = $element.find('canvas')[0];
      var ctx = canvas.getContext('2d');

      function draw(data) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (!data || !data.lane_width) { return; }
        var laneWidth = data.lane_width;
        var offset = data.lateral_offset || 0;
        var scale = canvas.width / laneWidth;
        var center = canvas.width / 2 - offset * scale;
        var halfLane = laneWidth * 0.5 * scale;
        var left = center - halfLane;
        var right = center + halfLane;
        ctx.strokeStyle = 'yellow';
        ctx.beginPath(); ctx.moveTo(left, 0); ctx.lineTo(left, canvas.height); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(right, 0); ctx.lineTo(right, canvas.height); ctx.stroke();
        ctx.fillStyle = 'red';
        ctx.fillRect(center - 5, canvas.height - 20, 10, 20);
      }

      function update() {
        bngApi.engineLua('extensions.driver_assistance_angelo234.getLaneSensorData()', function (data) {
          $scope.$evalAsync(function () {
            draw(data);
          });
        });
      }

      var interval = setInterval(update, 100);
      $scope.$on('$destroy', function () {
        clearInterval(interval);
      });
    }]
  };
}]);
