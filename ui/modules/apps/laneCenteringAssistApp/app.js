angular.module('beamng.apps')
.directive('laneCenteringAssistApp', [function () {
  return {
    templateUrl: '/ui/modules/apps/laneCenteringAssistApp/app.html',
    replace: true,
    restrict: 'EA',
    scope: true,
    controller: ['$scope', '$element', function ($scope, $element) {
      var canvas = $element.find('canvas')[0]
      var ctx = canvas.getContext('2d')
      var timer = null
      var carColor = [204, 236, 255]

      var vm = $scope.vm = {
        statusText: 'Unavailable',
        statusClass: 'lca-status-off',
        statusDetail: '',
        speed: '—',
        driverSteer: '—',
        aiSteer: '—',
        blend: '—',
        throttle: '—',
        brake: '—'
      }

      function formatNumber(value, digits) {
        if (typeof value !== 'number' || !isFinite(value)) return '—'
        return value.toFixed(digits)
      }

      function syncCanvas() {
        if (!canvas) return {width: 0, height: 0}
        var width = canvas.clientWidth || 0
        var height = canvas.clientHeight || 0
        var dpr = window.devicePixelRatio || 1
        var pixelWidth = Math.max(1, Math.round(width * dpr))
        var pixelHeight = Math.max(1, Math.round(height * dpr))
        if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
          canvas.width = pixelWidth
          canvas.height = pixelHeight
        }
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
        return {width: width, height: height}
      }

      function updateStatus(data) {
        var status = (data && data.status) || {}
        var state = 'Unavailable'
        var clazz = 'lca-status-off'
        var detail = ''

        if (!status.installed) {
          state = 'Not installed'
          clazz = 'lca-status-missing'
        } else if (status.active) {
          state = 'Active'
          clazz = 'lca-status-active'
        } else if (!status.enabled) {
          state = 'Standby'
          clazz = 'lca-status-idle'
          detail = 'Disabled'
        } else if (status.driverOverride) {
          state = 'Standby'
          clazz = 'lca-status-warning'
          detail = 'Driver override'
        } else if (status.reason === 'cooldown') {
          state = 'Cooling down'
          clazz = 'lca-status-warning'
        } else if (status.available) {
          state = 'Ready'
          clazz = 'lca-status-ready'
        } else {
          state = 'Searching lane'
          clazz = 'lca-status-idle'
        }

        if (status.reason === 'low_speed') {
          detail = 'Waiting for speed'
        } else if (status.reason === 'not_installed') {
          detail = 'Install system to enable'
        }

        vm.statusText = state
        vm.statusClass = clazz
        vm.statusDetail = detail
      }

      function drawBackground(width, height) {
        ctx.clearRect(0, 0, width, height)
        var gradient = ctx.createLinearGradient(0, 0, 0, height)
        gradient.addColorStop(0, 'rgba(8, 16, 24, 0.92)')
        gradient.addColorStop(1, 'rgba(10, 22, 32, 0.75)')
        ctx.fillStyle = gradient
        ctx.fillRect(0, 0, width, height)

        ctx.strokeStyle = 'rgba(40, 140, 150, 0.35)'
        ctx.lineWidth = 1
        ctx.beginPath()
        ctx.moveTo(0, height * 0.75)
        ctx.lineTo(width, height * 0.75)
        ctx.stroke()
      }

      function drawGrid(width, height, scale) {
        var spacing = 8 * scale
        var centerX = width / 2
        var bottom = height * 0.78
        ctx.strokeStyle = 'rgba(52, 92, 112, 0.4)'
        ctx.lineWidth = 1
        ctx.beginPath()
        for (var offset = spacing; offset < width / 2; offset += spacing) {
          ctx.moveTo(centerX + offset, bottom)
          ctx.lineTo(centerX + offset, bottom - height * 0.6)
          ctx.moveTo(centerX - offset, bottom)
          ctx.lineTo(centerX - offset, bottom - height * 0.6)
        }
        ctx.stroke()
      }

      function drawTrajectory(data, width, height) {
        var points = (data && data.ai && data.ai.trajectory && data.ai.trajectory.points) || []
        if (!points.length) return

        var maxX = 1
        var maxY = 1
        for (var i = 0; i < points.length; i++) {
          var p = points[i]
          maxX = Math.max(maxX, Math.abs(p.x))
          maxY = Math.max(maxY, Math.abs(p.y))
        }
        var baseY = height * 0.78
        var spanY = height * 0.65
        var scaleY = spanY / (Math.max(5, maxY) + 1)
        var scaleX = (width * 0.42) / (Math.max(3, maxX) + 1)
        var scale = Math.max(1.5, Math.min(scaleX, scaleY))

        drawGrid(width, height, scale / 10)

        ctx.strokeStyle = 'rgba(70, 210, 170, 0.85)'
        ctx.lineWidth = Math.max(2, scale * 0.15)
        ctx.lineJoin = 'round'
        ctx.beginPath()
        ctx.moveTo(width / 2 + points[0].x * scale, baseY - points[0].y * scale)
        for (var j = 1; j < points.length; j++) {
          var pt = points[j]
          ctx.lineTo(width / 2 + pt.x * scale, baseY - pt.y * scale)
        }
        ctx.stroke()

        ctx.fillStyle = 'rgba(140, 220, 240, 0.9)'
        var markerRadius = Math.max(2, scale * 0.18)
        for (var k = 0; k < points.length; k += 3) {
          var mp = points[k]
          ctx.beginPath()
          ctx.arc(width / 2 + mp.x * scale, baseY - mp.y * scale, markerRadius, 0, Math.PI * 2)
          ctx.fill()
        }
      }

      function drawVehicle(width, height) {
        var baseY = height * 0.78
        var carW = width * 0.12
        var carH = height * 0.18
        var radius = carW * 0.15
        var left = width / 2 - carW / 2
        var right = left + carW
        var top = baseY - carH
        var bottom = baseY

        ctx.fillStyle = 'rgba(' + Math.round(carColor[0]) + ',' + Math.round(carColor[1]) + ',' + Math.round(carColor[2]) + ',0.75)'
        ctx.strokeStyle = 'rgba(20, 80, 120, 0.9)'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.moveTo(left + radius, top)
        ctx.lineTo(right - radius, top)
        ctx.quadraticCurveTo(right, top, right, top + radius)
        ctx.lineTo(right, bottom - radius)
        ctx.quadraticCurveTo(right, bottom, right - radius, bottom)
        ctx.lineTo(left + radius, bottom)
        ctx.quadraticCurveTo(left, bottom, left, bottom - radius)
        ctx.lineTo(left, top + radius)
        ctx.quadraticCurveTo(left, top, left + radius, top)
        ctx.closePath()
        ctx.fill()
        ctx.stroke()
      }

      function drawOverlay(width, height) {
        ctx.strokeStyle = 'rgba(70, 180, 160, 0.7)'
        ctx.lineWidth = 2
        ctx.strokeRect(1, 1, width - 2, height - 2)
      }

      function render(data) {
        var view = syncCanvas()
        var width = view.width
        var height = view.height
        if (!width || !height) return

        drawBackground(width, height)
        drawTrajectory(data, width, height)
        drawVehicle(width, height)
        drawOverlay(width, height)
      }

      function updateNumbers(data) {
        var ai = data && data.ai && data.ai.command
        var driver = data && data.driver
        var vehicle = data && data.vehicle

        vm.speed = formatNumber(vehicle && vehicle.speed, 1)
        vm.driverSteer = formatNumber(driver && driver.steering, 3)
        vm.aiSteer = formatNumber(ai && ai.aiSteer, 3)
        vm.blend = formatNumber(ai && ai.blend, 2)
        vm.throttle = formatNumber(driver && driver.throttle, 2)
        vm.brake = formatNumber(driver && driver.brake, 2)
      }

      function handleColor(data) {
        if (!data || !data.color) return
        var c = data.color
        if (typeof c.r === 'number' && typeof c.g === 'number' && typeof c.b === 'number') {
          carColor = [c.r, c.g, c.b]
        }
      }

      function tick() {
        bngApi.engineLua('extensions.driver_assistance_angelo234.getLaneCenteringData()', function (data) {
          $scope.$evalAsync(function () {
            handleColor(data)
            updateStatus(data)
            updateNumbers(data)
            render(data)
          })
        })
      }

      tick()
      timer = setInterval(tick, 130)

      $scope.$on('$destroy', function () {
        if (timer) {
          clearInterval(timer)
          timer = null
        }
      })
    }]
  }
}])
