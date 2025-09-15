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
      var carColor = [255, 255, 255];

      function drawVehicle(x) {
        ctx.save();
        ctx.fillStyle = 'rgba(' +
          Math.round(carColor[0]) + ',' +
          Math.round(carColor[1]) + ',' +
          Math.round(carColor[2]) + ',0.5)';
        var carWidth = 10;
        var carLength = 20;
        ctx.translate(x, canvas.height - carLength);
        ctx.fillRect(-carWidth / 2, 0, carWidth, carLength);
        ctx.restore();
      }

      function draw(data) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        var center = canvas.width / 2;
        if (!data || !data.lane_width) {
          drawVehicle(center);
          return;
        }
        var laneWidth = data.lane_width;
        var offset = data.lateral_offset || 0;
        var scale = canvas.width / laneWidth;
        center = canvas.width / 2 - offset * scale;
        var halfLane = laneWidth * 0.5 * scale;
        var left = center - halfLane;
        var right = center + halfLane;
        var bottom = canvas.height;
        var horizon = 0;
        var yaw = 0;
        if (data.road_dir) {
          yaw = Math.atan2(data.road_dir.y, data.road_dir.x);
        }
        var curve = data.curvature || 0;
        var shift0 = Math.tan(yaw) * bottom;
        var shift1 = Math.tan(yaw + curve) * bottom;
        var midShift = (shift0 + shift1) / 2;
        ctx.lineWidth = 2;
        ctx.strokeStyle = 'yellow';
        ctx.beginPath(); ctx.moveTo(left, bottom); ctx.quadraticCurveTo(left + midShift, bottom * 0.5, left + shift1, horizon); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(right, bottom); ctx.quadraticCurveTo(right + midShift, bottom * 0.5, right + shift1, horizon); ctx.stroke();
        ctx.strokeStyle = 'green';
        ctx.beginPath(); ctx.moveTo(center, bottom); ctx.quadraticCurveTo(center + midShift, bottom * 0.5, center + shift1, horizon); ctx.stroke();
        drawVehicle(center);
      }

      function update() {
        bngApi.engineLua('extensions.driver_assistance_angelo234.getLaneCenteringData()', function (data) {
          $scope.$evalAsync(function () {
            if (data && data.color) {
              carColor = [
                typeof data.color.r === 'number' ? data.color.r : 255,
                typeof data.color.g === 'number' ? data.color.g : 255,
                typeof data.color.b === 'number' ? data.color.b : 255
              ];
            }
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
