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
      var ORTHOGONAL_OFFSETS = [
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1]
      ];
      var DIAGONAL_OFFSETS = [
        [1, 1],
        [1, -1],
        [-1, 1],
        [-1, -1]
      ];

      function drawRasterizedPoint(x, y, color, size) {
        if (!isFinite(size) || size <= 0) {
          size = 2;
        }

        var primarySize = size;
        var primaryHalf = primarySize * 0.5;
        var neighborSpacing = Math.max(0.8, primarySize * 0.9);
        var neighborSize = Math.max(0.6, primarySize * 0.75);
        var diagSpacing = neighborSpacing * 1.35;
        var diagSize = Math.max(0.45, primarySize * 0.55);

        ctx.fillStyle = color;

        ctx.globalAlpha = 0.85;
        ctx.fillRect(x - primaryHalf, y - primaryHalf, primarySize, primarySize);

        ctx.globalAlpha = 0.32;
        for (var i = 0; i < ORTHOGONAL_OFFSETS.length; i++) {
          var orth = ORTHOGONAL_OFFSETS[i];
          var ox = x + orth[0] * neighborSpacing;
          var oy = y + orth[1] * neighborSpacing;
          ctx.fillRect(ox - neighborSize * 0.5, oy - neighborSize * 0.5, neighborSize, neighborSize);
        }

        ctx.globalAlpha = 0.18;
        for (var j = 0; j < DIAGONAL_OFFSETS.length; j++) {
          var diag = DIAGONAL_OFFSETS[j];
          var dx = x + diag[0] * diagSpacing;
          var dy = y + diag[1] * diagSpacing;
          ctx.fillRect(dx - diagSize * 0.5, dy - diagSize * 0.5, diagSize, diagSize);
        }

        ctx.globalAlpha = 1;
      }

      function drawVehicle() {
        ctx.save();
        ctx.fillStyle = 'rgba(' +
          Math.round(carColor[0]) + ',' +
          Math.round(carColor[1]) + ',' +
          Math.round(carColor[2]) + ',0.5)';
        var carWidth = 10;
        var carLength = 20;
        ctx.translate(canvas.width / 2, canvas.height / 2);
        ctx.fillRect(-carWidth / 2, -carLength / 2, carWidth, carLength);
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

        var basePointSize = Math.max(2, Math.min(5, scale * 1.1));

        points.forEach(function (p) {
          var x = canvas.width / 2 + p.x * scale;
          var y = canvas.height / 2 - p.y * scale;
          var hue = ((p._d - minD) / distRange) * 240;
          var color = 'hsl(' + hue + ', 100%, 50%)';
          var depthFactor = distRange > 0 ? 1 - (p._d - minD) / distRange : 0;
          if (depthFactor < 0) { depthFactor = 0; }
          if (depthFactor > 1) { depthFactor = 1; }
          var pointSize = basePointSize + depthFactor * 1.2;
          drawRasterizedPoint(x, y, color, pointSize);
        });

        ctx.globalAlpha = 1;
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
