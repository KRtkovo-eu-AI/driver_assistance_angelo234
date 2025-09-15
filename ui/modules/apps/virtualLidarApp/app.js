angular.module('beamng.apps')
.directive('virtualLidarApp', [function () {
  return {
    templateUrl: '/ui/modules/apps/virtualLidarApp/app.html',
    replace: true,
    restrict: 'EA',
    scope: true,
    controller: ['$scope', '$element', function ($scope, $element) {
      var canvas = $element.find('canvas')[0];
      var ctx = canvas.getContext('2d');
      // Fixed world range for drawing (in meters). Keeps zoom stable.
      var FIXED_RANGE = 60;
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

      function drawVehicle() {
        ctx.save();
        ctx.translate(canvas.width / 2, canvas.height / 2);
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

      function draw(points) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (!points || !points.length) {
          drawVehicle();
          return;
        }

        var minD = Infinity, maxD = -Infinity;

        points.forEach(function (p) {
          var d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
          if (d < minD) { minD = d; }
          if (d > maxD) { maxD = d; }
          p._d = d;
        });

        var scale = Math.min(
          canvas.width / (2 * FIXED_RANGE),
          canvas.height / (2 * FIXED_RANGE)
        );
        var distRange = Math.max(1, maxD - minD);

        points.forEach(function (p) {
          var x = canvas.width / 2 + p.x * scale;
          var y = canvas.height / 2 - p.y * scale;
          var hue = ((p._d - minD) / distRange) * 240;
          ctx.fillStyle = 'hsl(' + hue + ', 100%, 50%)';
          ctx.fillRect(x, y, 2, 2);
        });

        drawVehicle();
      }

      function update() {
        bngApi.engineLua(
          'extensions.driver_assistance_angelo234.getVirtualLidarData()',
          function (data) {
            $scope.$evalAsync(function () {
              if (data.color) {
                carColor = [
                  typeof data.color.r === 'number' ? data.color.r : 255,
                  typeof data.color.g === 'number' ? data.color.g : 255,
                  typeof data.color.b === 'number' ? data.color.b : 255
                ];
              }
              draw(data.points);
            });
          }
        );
      }

      var interval = setInterval(update, 100);

      $scope.$on('$destroy', function () {
        clearInterval(interval);
      });
    }]
  };
}]);
