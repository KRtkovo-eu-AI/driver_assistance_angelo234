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

      function draw(points) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (!points || !points.length) { return; }

        var minX = points[0].x, maxX = points[0].x;
        var minY = points[0].y, maxY = points[0].y;
        var minD = Infinity, maxD = -Infinity;

        points.forEach(function (p) {
          if (p.x < minX) { minX = p.x; }
          if (p.x > maxX) { maxX = p.x; }
          if (p.y < minY) { minY = p.y; }
          if (p.y > maxY) { maxY = p.y; }

          var d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
          if (d < minD) { minD = d; }
          if (d > maxD) { maxD = d; }
          p._d = d;
        });

        var scale = Math.min(
          canvas.width / Math.max(1, maxX - minX),
          canvas.height / Math.max(1, maxY - minY)
        );
        var distRange = Math.max(1, maxD - minD);

        points.forEach(function (p) {
          var x = (p.x - minX) * scale;
          var y = canvas.height - (p.y - minY) * scale;
          var hue = ((p._d - minD) / distRange) * 240;
          ctx.fillStyle = 'hsl(' + hue + ', 100%, 50%)';
          ctx.fillRect(x, y, 2, 2);
        });
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
