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

      function drawVehicle() {
        ctx.save();
        ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
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

        var maxAbsX = 0, maxAbsY = 0;
        var minD = Infinity, maxD = -Infinity;

        points.forEach(function (p) {
          if (Math.abs(p.x) > maxAbsX) { maxAbsX = Math.abs(p.x); }
          if (Math.abs(p.y) > maxAbsY) { maxAbsY = Math.abs(p.y); }

          var d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
          if (d < minD) { minD = d; }
          if (d > maxD) { maxD = d; }
          p._d = d;
        });

        var range = Math.max(1, Math.max(maxAbsX, maxAbsY));
        var scale = Math.min(
          canvas.width / (2 * range),
          canvas.height / (2 * range)
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
        bngApi.engineLua('extensions.driver_assistance_angelo234.getVirtualLidarPointCloud()', function (data) {
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
