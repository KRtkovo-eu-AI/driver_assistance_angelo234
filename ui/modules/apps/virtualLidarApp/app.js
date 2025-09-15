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
      var carBounds = null;
      var DEFAULT_BOUNDS = { width: 2, length: 4 }; // meters

      function drawVehicle(scale) {
        ctx.save();
        ctx.fillStyle = 'rgba(' +
          Math.round(carColor[0]) + ',' +
          Math.round(carColor[1]) + ',' +
          Math.round(carColor[2]) + ',0.5)';
        var bounds = carBounds || DEFAULT_BOUNDS;
        var s = typeof scale === 'number' ? scale : 1;
        var carWidth = bounds.width * 1.50 * s;
        var carLength = bounds.length * 1.20 * s;
        ctx.translate(canvas.width / 2, canvas.height / 2);
        ctx.fillRect(-carWidth / 2, -carLength / 2, carWidth, carLength);
        ctx.restore();
      }

      function draw(points, groundPoints) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        var scale = Math.min(
          canvas.width / (2 * FIXED_RANGE),
          canvas.height / (2 * FIXED_RANGE)
        );
        var regular = Array.isArray(points) ? points : [];
        var low = Array.isArray(groundPoints) ? groundPoints : [];
        if (!regular.length && !low.length) {
          drawVehicle(scale);
          return;
        }

        var minD = Infinity, maxD = -Infinity;

        [regular, low].forEach(function (list) {
          list.forEach(function (p) {
            var d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (d < minD) { minD = d; }
            if (d > maxD) { maxD = d; }
            p._d = d;
          });
        });

        var distRange = Math.max(1, maxD - minD);

        function drawPoints(list, lightness, alpha) {
          var opacity = typeof alpha === 'number' ? alpha : 1;
          list.forEach(function (p) {
            var x = canvas.width / 2 + p.x * scale;
            var y = canvas.height / 2 - p.y * scale;
            var hue = ((p._d - minD) / distRange) * 240;
            ctx.fillStyle =
              'hsla(' + hue + ', 100%, ' + lightness + '%, ' + opacity + ')';
            ctx.fillRect(x, y, 2, 2);
          });
        }

        drawPoints(low, 30, 0.45);
        drawPoints(regular, 50, 1);

        drawVehicle(scale);
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
              if (
                data.bounds &&
                typeof data.bounds.width === 'number' &&
                typeof data.bounds.length === 'number'
              ) {
                carBounds = {
                  width: data.bounds.width,
                  length: data.bounds.length
                };
              }
              draw(data.points, data.groundPoints);
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
