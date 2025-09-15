angular.module('beamng.apps')
.directive('laneCenteringAssistApp', [function () {
  return {
    templateUrl: '/ui/modules/apps/laneCenteringAssistApp/app.html',
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
        var bottom = canvas.height;
        var horizon = 0;
        var yaw = 0;
        if (data.road_dir) {
          yaw = Math.atan2(data.road_dir.y, data.road_dir.x);
        }
        var shift = Math.tan(yaw) * bottom;
        ctx.lineWidth = 2;
        ctx.strokeStyle = 'yellow';
        ctx.beginPath(); ctx.moveTo(left, bottom); ctx.lineTo(left + shift, horizon); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(right, bottom); ctx.lineTo(right + shift, horizon); ctx.stroke();
        ctx.strokeStyle = 'green';
        ctx.beginPath(); ctx.moveTo(center, bottom); ctx.lineTo(center + shift, horizon); ctx.stroke();
      }

      function update() {
        bngApi.engineLua('extensions.driver_assistance_angelo234.getLaneCenteringData()', function (data) {
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
