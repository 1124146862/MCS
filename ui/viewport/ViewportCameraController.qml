import QtQuick
import QtQuick3D

Item {
    id: root

    signal axisSnapRequested(real pitch, real yaw)

    required property Node origin
    required property Camera camera
    property bool navigationEnabled: true
    property vector3d focusTarget: Qt.vector3d(0, 110, 0)
    property real focusDistance: 520

    property real orbitSensitivity: 0.35
    property real panSensitivity: 1.0
    property real zoomStep: 0.12
    property real minDistance: 80
    property real maxDistance: 2400
    property real minPitch: -89
    property real maxPitch: 89
    property bool snapShortcutLatched: false

    readonly property bool navigationActive: orbitHandler.active || panHandler.active

    focus: true

    signal distanceChanged(real distance)

    function clamp(value, minimumValue, maximumValue) {
        return Math.max(minimumValue, Math.min(maximumValue, value))
    }

    function normalizeAngle(angle) {
        let normalized = angle % 360
        if (normalized <= -180)
            normalized += 360
        if (normalized > 180)
            normalized -= 360
        return normalized
    }

    function cameraForwardVector() {
        const pitchRadians = root.origin.eulerRotation.x * Math.PI / 180
        const yawRadians = root.origin.eulerRotation.y * Math.PI / 180
        const cosPitch = Math.cos(pitchRadians)

        return {
            x: -cosPitch * Math.sin(yawRadians),
            y: Math.sin(pitchRadians),
            z: -cosPitch * Math.cos(yawRadians)
        }
    }

    function snapToNearestAxis() {
        if (!root.navigationEnabled)
            return

        const forward = root.cameraForwardVector()
        const standardViews = [
            { pitch: 0, yaw: 0, direction: { x: 0, y: 0, z: -1 } },
            { pitch: 0, yaw: 180, direction: { x: 0, y: 0, z: 1 } },
            { pitch: 0, yaw: 90, direction: { x: -1, y: 0, z: 0 } },
            { pitch: 0, yaw: -90, direction: { x: 1, y: 0, z: 0 } },
            { pitch: -89, yaw: 0, direction: { x: 0, y: -1, z: 0 } },
            { pitch: 89, yaw: 0, direction: { x: 0, y: 1, z: 0 } }
        ]

        let nearestView = standardViews[0]
        let bestScore = -Infinity

        for (const candidate of standardViews) {
            const direction = candidate.direction
            const score = forward.x * direction.x
                        + forward.y * direction.y
                        + forward.z * direction.z

            if (score > bestScore) {
                bestScore = score
                nearestView = candidate
            }
        }

        root.origin.eulerRotation = Qt.vector3d(nearestView.pitch, root.normalizeAngle(nearestView.yaw), 0)
        root.axisSnapRequested(nearestView.pitch, root.normalizeAngle(nearestView.yaw))
    }

    function zoomByPixels(deltaY) {
        if (!root.navigationEnabled || !deltaY)
            return

        const factor = 1 + (deltaY / Math.max(root.height, 1))
        const safeFactor = factor <= 0.05 ? 0.05 : factor

        const nextDistance = root.clamp(root.camera.z * safeFactor, root.minDistance, root.maxDistance)
        root.camera.z = nextDistance
        root.distanceChanged(nextDistance)
    }

    function zoomBySteps(steps) {
        if (!root.navigationEnabled || !steps)
            return

        const factor = 1 - (steps * root.zoomStep)
        const safeFactor = factor <= 0.05 ? 0.05 : factor

        const nextDistance = root.clamp(root.camera.z * safeFactor, root.minDistance, root.maxDistance)
        root.camera.z = nextDistance
        root.distanceChanged(nextDistance)
    }

    function panBy(deltaX, deltaY) {
        if (!root.navigationEnabled)
            return

        const widthFactor = Math.max(root.width, 1)
        const heightFactor = Math.max(root.height, 1)
        const panScale = root.camera.z
        const scaledX = -(deltaX / widthFactor) * panScale * root.panSensitivity
        const scaledY = (deltaY / heightFactor) * panScale * root.panSensitivity

        let movement = Qt.vector3d(0, 0, 0)
        const right = root.origin.right
        const up = root.origin.up

        movement = movement.plus(Qt.vector3d(right.x * scaledX, right.y * scaledX, right.z * scaledX))
        movement = movement.plus(Qt.vector3d(up.x * scaledY, up.y * scaledY, up.z * scaledY))
        root.origin.position = root.origin.position.plus(movement)
    }

    function focusOnSelection() {
        root.origin.position = root.focusTarget

        const nextDistance = root.clamp(root.focusDistance, root.minDistance, root.maxDistance)
        root.camera.z = nextDistance
        root.distanceChanged(nextDistance)
    }

    DragHandler {
        id: orbitHandler

        target: null
        acceptedButtons: Qt.LeftButton
        acceptedModifiers: Qt.AltModifier
        enabled: root.navigationEnabled

        property point lastPoint: Qt.point(0, 0)

        onActiveChanged: {
            if (active)
                lastPoint = Qt.point(centroid.position.x, centroid.position.y)
        }

        onCentroidChanged: {
            if (!active)
                return

            const currentPoint = Qt.point(centroid.position.x, centroid.position.y)
            const deltaX = currentPoint.x - lastPoint.x
            const deltaY = currentPoint.y - lastPoint.y
            const rotation = root.origin.eulerRotation

            const nextPitch = root.clamp(rotation.x - deltaY * root.orbitSensitivity, root.minPitch, root.maxPitch)
            const nextYaw = rotation.y - deltaX * root.orbitSensitivity

            root.origin.eulerRotation = Qt.vector3d(nextPitch, nextYaw, rotation.z)
            lastPoint = currentPoint
        }
    }

    DragHandler {
        id: panHandler

        target: null
        acceptedButtons: Qt.MiddleButton
        acceptedModifiers: Qt.AltModifier
        enabled: root.navigationEnabled

        property point lastPoint: Qt.point(0, 0)

        onActiveChanged: {
            if (active)
                lastPoint = Qt.point(centroid.position.x, centroid.position.y)
        }

        onCentroidChanged: {
            if (!active)
                return

            const currentPoint = Qt.point(centroid.position.x, centroid.position.y)
            root.panBy(currentPoint.x - lastPoint.x, currentPoint.y - lastPoint.y)
            lastPoint = currentPoint
        }
    }

    DragHandler {
        id: zoomDragHandler

        target: null
        acceptedButtons: Qt.RightButton
        acceptedModifiers: Qt.AltModifier
        enabled: root.navigationEnabled

        property point lastPoint: Qt.point(0, 0)

        onActiveChanged: {
            if (active)
                lastPoint = Qt.point(centroid.position.x, centroid.position.y)
        }

        onCentroidChanged: {
            if (!active)
                return

            const currentPoint = Qt.point(centroid.position.x, centroid.position.y)
            root.zoomByPixels(currentPoint.y - lastPoint.y)
            lastPoint = currentPoint
        }
    }

    WheelHandler {
        target: null
        orientation: Qt.Vertical
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        enabled: root.navigationEnabled

        onWheel: event => {
            const steps = event.angleDelta.y / 120
            root.zoomBySteps(steps)
        }
    }

    Keys.onPressed: event => {
        if (!root.navigationEnabled)
            return

        const ctrlAltPressed = (event.modifiers & Qt.ControlModifier) && (event.modifiers & Qt.AltModifier)
        const isModifierKey = event.key === Qt.Key_Control || event.key === Qt.Key_Alt

        if (ctrlAltPressed && isModifierKey && !root.snapShortcutLatched) {
            root.snapShortcutLatched = true
            root.snapToNearestAxis()
            event.accepted = true
            return
        }

        if (event.key === Qt.Key_T) {
            root.focusOnSelection()
            event.accepted = true
        }
    }

    Keys.onReleased: event => {
        const ctrlStillPressed = (event.modifiers & Qt.ControlModifier) !== 0
        const altStillPressed = (event.modifiers & Qt.AltModifier) !== 0

        if (!ctrlStillPressed || !altStillPressed)
            root.snapShortcutLatched = false
    }
}
