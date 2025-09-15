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
      var CAR_WIDTH = 10;
      var CAR_LENGTH = 20;
      var CAR_POINT_SIZE = 2;
      var vehiclePointCloud = createVehiclePointCloud(CAR_WIDTH, CAR_LENGTH, CAR_POINT_SIZE);

      function createVehiclePointCloud(width, length, dotSize) {
        var points = [];
        var halfW = width / 2;
        var halfL = length / 2;
        var safeHalfW = halfW || 1;
        var safeHalfL = halfL || 1;
        var baseSpacing = dotSize * 1.5;
        var spacingScale = dotSize * 0.8;
        var jitter = dotSize * 0.35;
        var maxRadius = Math.sqrt(halfW * halfW + halfL * halfL) || 1;

        for (var y = -halfL; y <= halfL + 0.001;) {
          var depthRatio = Math.abs(y) / safeHalfL;
          var ySpacing = baseSpacing + depthRatio * spacingScale;
          for (var x = -halfW; x <= halfW + 0.001;) {
            var lateralRatio = Math.abs(x) / safeHalfW;
            var spacing = baseSpacing + Math.max(depthRatio, lateralRatio) * spacingScale;
            var skipProbability = 0.05 + Math.max(depthRatio, lateralRatio) * 0.1;
            var nextX = x + spacing;
            if (Math.random() >= skipProbability) {
              var distRatio = Math.sqrt(x * x + y * y) / maxRadius;
              points.push({
                x: x + (Math.random() - 0.5) * jitter,
                y: y + (Math.random() - 0.5) * jitter,
                alpha: Math.min(1, Math.max(0.3, 0.35 + (1 - distRatio) * 0.4))
              });
            }
            x = nextX;
          }
          y += ySpacing;
        }

        var outlineStep = dotSize * 1.25;
        for (var ox = -halfW; ox <= halfW + 0.001; ox += outlineStep) {
          points.push({ x: ox, y: -halfL, alpha: 0.55 });
          points.push({ x: ox, y: halfL, alpha: 0.55 });
        }
        for (var oy = -halfL; oy <= halfL + 0.001; oy += outlineStep) {
          points.push({ x: -halfW, y: oy, alpha: 0.55 });
          points.push({ x: halfW, y: oy, alpha: 0.55 });
        }

        points.push({ x: 0, y: 0, alpha: 0.75 });

        return points;
      }

      function drawVehicle(x) {
        ctx.save();
        ctx.translate(x, canvas.height - CAR_LENGTH / 2);
        var colorPrefix = 'rgba(' +
          Math.round(carColor[0]) + ',' +
          Math.round(carColor[1]) + ',' +
          Math.round(carColor[2]) + ',';
        vehiclePointCloud.forEach(function (point) {
          ctx.fillStyle = colorPrefix + point.alpha + ')';
          ctx.fillRect(
            point.x - CAR_POINT_SIZE / 2,
            point.y - CAR_POINT_SIZE / 2,
            CAR_POINT_SIZE,
            CAR_POINT_SIZE
          );
        });
        ctx.restore();
      }

      function draw(data) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        var carX = canvas.width / 2;
        if (!data || !data.lane_width) {
          drawVehicle(carX);
          return;
        }
        var laneWidth = data.lane_width;
        var offset = data.lateral_offset || 0;
        var scale = canvas.width / laneWidth;
        var laneCenter = carX - offset * scale;
        var halfLane = laneWidth * 0.5 * scale;
        var left = laneCenter - halfLane;
        var right = laneCenter + halfLane;
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
        ctx.beginPath(); ctx.moveTo(laneCenter, bottom); ctx.quadraticCurveTo(laneCenter + midShift, bottom * 0.5, laneCenter + shift1, horizon); ctx.stroke();

        ctx.strokeStyle = 'red';
        ctx.beginPath(); ctx.moveTo(carX, bottom); ctx.lineTo(laneCenter, bottom); ctx.stroke();

        if (data.future_dir) {
          var fYaw = Math.atan2(data.future_dir.y, data.future_dir.x);
          var fShift = Math.tan(fYaw) * bottom;
          ctx.strokeStyle = 'blue';
          ctx.beginPath(); ctx.moveTo(laneCenter, bottom); ctx.lineTo(laneCenter + fShift, horizon); ctx.stroke();
        }

        drawVehicle(carX);
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
