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
      var perspective = 5; // larger = stronger perspective

      function project(f, l) {
        var z = f + perspective;
        var px = canvas.width / 2 + (l / z) * canvas.width;
        var py = canvas.height - (f / z) * canvas.height;
        return [px, py];
      }

      function drawVehicle() {
        ctx.save();
        ctx.fillStyle = 'rgba(' +
          Math.round(carColor[0]) + ',' +
          Math.round(carColor[1]) + ',' +
          Math.round(carColor[2]) + ',0.5)';
        var carWidth = 20;
        var carLength = 40;
        ctx.translate(canvas.width / 2, canvas.height - carLength);
        ctx.fillRect(-carWidth / 2, 0, carWidth, carLength);
        ctx.restore();
      }

      function drawLine(points, style) {
        if (!points || points.length < 2) return;
        ctx.strokeStyle = style;
        ctx.beginPath();
        for (var i = 0; i < points.length; i++) {
          var p = project(points[i][0], points[i][1]);
          if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
        }
        ctx.stroke();
      }

      function drawAssist(steer) {
        if (typeof steer !== 'number') return;
        var baseX = canvas.width / 2;
        var baseY = canvas.height - 40;
        ctx.strokeStyle = 'red';
        ctx.beginPath();
        ctx.moveTo(baseX, baseY);
        ctx.lineTo(baseX + steer * 100, baseY - 50);
        ctx.stroke();
      }

      function draw(data) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (data) {
          drawLine(data.left_line, 'yellow');
          drawLine(data.right_line, 'yellow');
          drawLine(data.center_line, 'white');
          drawAssist(data.assist_steer);
        }
        drawVehicle();
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

