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
      var scale = 5; // pixels per meter

      function drawVehicle(cx, cy) {
        ctx.save();
        ctx.fillStyle = 'rgba(' +
          Math.round(carColor[0]) + ',' +
          Math.round(carColor[1]) + ',' +
          Math.round(carColor[2]) + ',0.5)';
        var carWidth = 10;
        var carLength = 20;
        ctx.translate(cx, cy);
        ctx.fillRect(-carWidth / 2, -carLength / 2, carWidth, carLength);
        ctx.restore();
      }

      function drawLine(points, style) {
        if (!points || points.length < 2) return;
        ctx.strokeStyle = style;
        ctx.beginPath();
        for (var i = 0; i < points.length; i++) {
          var f = points[i][0];
          var l = points[i][1];
          var x = canvas.width / 2 + l * scale;
          var y = canvas.height / 2 - f * scale;
          if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
      }

      function draw(data) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (data) {
          drawLine(data.left_line, 'yellow');
          drawLine(data.right_line, 'yellow');
          drawLine(data.center_line, 'white');
        }
        drawVehicle(canvas.width / 2, canvas.height / 2);
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

