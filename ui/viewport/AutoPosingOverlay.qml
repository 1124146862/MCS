import QtQuick
import QtQuick.Controls

Item {
    id: root

    required property var sceneView
    required property var cameraOrbit
    property var documentViewModel: null
    property real cameraDistance: 520
    property real fieldOfView: 45
    property bool horizontalFieldOfView: false
    property real visibleWorldHeight: 200
    property real dragSensitivity: 1.0
    readonly property real visibleWorldWidth: visibleWorldHeight * (Math.max(sceneView.width, 1) / Math.max(sceneView.height, 1))

    readonly property bool autoPosingVisible: documentViewModel !== null && documentViewModel.isAutoPosingMode
    property int projectionRevision: 0
    property string draggingControllerId: ""
    property vector3d draggingWorldPosition: Qt.vector3d(0, 0, 0)

    anchors.fill: parent
    visible: autoPosingVisible
    enabled: autoPosingVisible
    z: 20
    focus: autoPosingVisible

    Timer {
        interval: 16
        running: root.autoPosingVisible
        repeat: true
        onTriggered: root.projectionRevision++
    }

    function projectPoint(x, y, z) {
        const refreshTick = root.projectionRevision
        return sceneView.mapFrom3DScene(Qt.vector3d(x, y, z))
    }

    function controllerSize(controller) {
        if (controller.controllerType === "main")
            return 18
        if (controller.controllerType === "direction")
            return 14
        return 12
    }

    function controllerFillColor(controller) {
        if (controller.active)
            return "#2a98ff"
        return "#3fae56"
    }

    function controllerBorderColor(controller) {
        if (controller.selected)
            return "#fff5bf"
        if (controller.fixed)
            return "#ffb347"
        return "#0f2718"
    }

    function lineColor(segment) {
        return segment.color ? segment.color : "#3b7e48"
    }

    function vectorAdd(left, right) {
        return Qt.vector3d(left.x + right.x, left.y + right.y, left.z + right.z)
    }

    function vectorSubtract(left, right) {
        return Qt.vector3d(left.x - right.x, left.y - right.y, left.z - right.z)
    }

    function vectorScale(vector, factor) {
        return Qt.vector3d(vector.x * factor, vector.y * factor, vector.z * factor)
    }

    function vectorDot(left, right) {
        return left.x * right.x + left.y * right.y + left.z * right.z
    }

    function vectorLength(vector) {
        return Math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
    }

    function normalizeVector(vector) {
        const length = vectorLength(vector)
        if (length <= 0.00001)
            return Qt.vector3d(0, 0, -1)
        return Qt.vector3d(vector.x / length, vector.y / length, vector.z / length)
    }

    function cameraForwardVector() {
        return normalizeVector(Qt.vector3d(cameraOrbit.forward.x, cameraOrbit.forward.y, cameraOrbit.forward.z))
    }

    function cameraRightVector() {
        return normalizeVector(Qt.vector3d(cameraOrbit.right.x, cameraOrbit.right.y, cameraOrbit.right.z))
    }

    function cameraUpVector() {
        return normalizeVector(Qt.vector3d(cameraOrbit.up.x, cameraOrbit.up.y, cameraOrbit.up.z))
    }

    function cameraWorldPosition() {
        return vectorSubtract(
            Qt.vector3d(cameraOrbit.position.x, cameraOrbit.position.y, cameraOrbit.position.z),
            vectorScale(cameraForwardVector(), root.cameraDistance)
        )
    }

    function rayDirectionForPoint(point) {
        const aspect = Math.max(root.width, 1) / Math.max(root.height, 1)
        const fovRadians = Math.max(root.fieldOfView, 0.1) * Math.PI / 180
        let halfWidth = 0
        let halfHeight = 0

        if (root.horizontalFieldOfView) {
            halfWidth = Math.tan(fovRadians / 2)
            halfHeight = halfWidth / aspect
        } else {
            halfHeight = Math.tan(fovRadians / 2)
            halfWidth = halfHeight * aspect
        }

        const normalizedX = (point.x / Math.max(root.width, 1)) * 2 - 1
        const normalizedY = 1 - (point.y / Math.max(root.height, 1)) * 2
        const forward = cameraForwardVector()
        const horizontalOffset = vectorScale(cameraRightVector(), normalizedX * halfWidth * root.dragSensitivity)
        const verticalOffset = vectorScale(cameraUpVector(), normalizedY * halfHeight * root.dragSensitivity)
        return normalizeVector(vectorAdd(forward, vectorAdd(horizontalOffset, verticalOffset)))
    }

    function intersectRayWithPlane(rayOrigin, rayDirection, planePoint, planeNormal, fallbackPoint) {
        const denominator = vectorDot(rayDirection, planeNormal)
        if (Math.abs(denominator) <= 0.00001)
            return fallbackPoint

        const distance = vectorDot(vectorSubtract(planePoint, rayOrigin), planeNormal) / denominator
        return vectorAdd(rayOrigin, vectorScale(rayDirection, distance))
    }

    function controllerDragPlaneNormal(controller) {
        if (controller.controllerType === "direction") {
            const worldUp = Qt.vector3d(0, 1, 0)
            const forward = cameraForwardVector()
            const alignment = Math.abs(vectorDot(forward, worldUp))

            if (alignment < 0.92)
                return worldUp
        }

        return cameraForwardVector()
    }

    function overlayPointFromScenePosition(scenePosition) {
        return root.mapFromItem(null, scenePosition.x, scenePosition.y)
    }

    function draggedWorldPosition(basePosition, planePoint, planeNormal, screenPoint) {
        const rayOrigin = cameraWorldPosition()
        const rayDirection = rayDirectionForPoint(screenPoint)

        return intersectRayWithPlane(
            rayOrigin,
            rayDirection,
            planePoint,
            planeNormal,
            basePosition
        )
    }

    function draggedPositionOrFallback(controllerId, fallbackX, fallbackY, fallbackZ) {
        if (draggingControllerId === controllerId)
            return draggingWorldPosition
        return Qt.vector3d(fallbackX, fallbackY, fallbackZ)
    }

    Keys.onPressed: event => {
        if (!root.autoPosingVisible || root.documentViewModel === null)
            return

        if (event.key === Qt.Key_R) {
            root.documentViewModel.toggleSelectedAutoPosingFixed()
            event.accepted = true
            return
        }

        if (event.key === Qt.Key_Z && (event.modifiers & Qt.ShiftModifier)) {
            root.documentViewModel.resetSelectedAutoPosingController()
            event.accepted = true
        }
    }

    Repeater {
        model: root.documentViewModel ? root.documentViewModel.autoPosingSegments : []

        delegate: Item {
            id: segmentItem

            required property var modelData

            readonly property vector3d startWorldPosition: root.draggedPositionOrFallback(
                                                               modelData.startId,
                                                               modelData.startX,
                                                               modelData.startY,
                                                               modelData.startZ)
            readonly property vector3d endWorldPosition: root.draggedPositionOrFallback(
                                                             modelData.endId,
                                                             modelData.endX,
                                                             modelData.endY,
                                                             modelData.endZ)
            readonly property vector3d startScreen: root.projectPoint(startWorldPosition.x, startWorldPosition.y, startWorldPosition.z)
            readonly property vector3d endScreen: root.projectPoint(endWorldPosition.x, endWorldPosition.y, endWorldPosition.z)
            readonly property real deltaX: endScreen.x - startScreen.x
            readonly property real deltaY: endScreen.y - startScreen.y
            readonly property real segmentLength: Math.sqrt(deltaX * deltaX + deltaY * deltaY)

            visible: root.autoPosingVisible && startScreen.z > 0 && endScreen.z > 0 && segmentLength > 6

            x: startScreen.x
            y: startScreen.y
            width: segmentLength
            height: 2
            rotation: Math.atan2(deltaY, deltaX) * 180 / Math.PI
            transformOrigin: Item.Left

            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: parent.width
                height: 2
                radius: 1
                color: root.lineColor(segmentItem.modelData)
                opacity: 0.95
            }
        }
    }

    Repeater {
        model: root.documentViewModel ? root.documentViewModel.autoPosingControllerVisuals : []

        delegate: Item {
            id: targetGhostItem

            required property var modelData

            readonly property vector3d targetWorldPosition: root.draggedPositionOrFallback(
                                                                modelData.id,
                                                                modelData.targetX,
                                                                modelData.targetY,
                                                                modelData.targetZ)
            readonly property vector3d solvedWorldPosition: Qt.vector3d(
                                                                modelData.solvedX,
                                                                modelData.solvedY,
                                                                modelData.solvedZ)
            readonly property vector3d projected: root.projectPoint(
                                                      targetWorldPosition.x,
                                                      targetWorldPosition.y,
                                                      targetWorldPosition.z)
            readonly property real ghostDiameter: Math.max(root.controllerSize(modelData) - 6, 8)

            visible: root.autoPosingVisible
                     && modelData.visible
                     && projected.z > 0
                     && (modelData.showGhostTarget || root.draggingControllerId === modelData.id)
            width: ghostDiameter + 8
            height: ghostDiameter + 8
            x: projected.x - width / 2
            y: projected.y - height / 2

            Rectangle {
                anchors.centerIn: parent
                width: targetGhostItem.ghostDiameter
                height: width
                radius: modelData.controllerType === "direction" ? 2 : width / 2
                color: "#4cffffff"
                border.color: modelData.pressureState === "stretch"
                              ? "#d94c45"
                              : (modelData.pressureState === "squeeze" ? "#2a98ff" : "#b9f5c4")
                border.width: 1
                opacity: 0.85
            }
        }
    }

    Repeater {
        model: root.documentViewModel ? root.documentViewModel.autoPosingControllerVisuals : []

        delegate: Item {
            id: controllerItem

            required property var modelData

            readonly property vector3d displayWorldPosition: Qt.vector3d(
                                                                 modelData.displayX,
                                                                 modelData.displayY,
                                                                 modelData.displayZ)
            readonly property vector3d projected: root.projectPoint(
                                                      displayWorldPosition.x,
                                                      displayWorldPosition.y,
                                                      displayWorldPosition.z)
            readonly property real controllerDiameter: root.controllerSize(modelData)

            visible: root.autoPosingVisible && modelData.visible && projected.z > 0
            width: controllerDiameter + 8
            height: controllerDiameter + 8
            x: projected.x - width / 2
            y: projected.y - height / 2

            Rectangle {
                id: halo
                anchors.centerIn: parent
                width: controllerItem.controllerDiameter + ((controllerItem.modelData.selected || root.draggingControllerId === controllerItem.modelData.id) ? 10 : 4)
                height: width
                radius: width / 2
                color: (controllerItem.modelData.selected || root.draggingControllerId === controllerItem.modelData.id) ? "#55ffd44d" : "#14000000"
                border.color: controllerItem.modelData.fixed ? "#ffb347" : "transparent"
                border.width: controllerItem.modelData.fixed ? 2 : 0
            }

            Rectangle {
                id: controllerVisual
                anchors.centerIn: parent
                width: controllerItem.controllerDiameter
                height: controllerItem.controllerDiameter
                radius: controllerItem.modelData.controllerType === "direction" ? 3 : width / 2
                color: root.controllerFillColor(controllerItem.modelData)
                border.color: (root.draggingControllerId === controllerItem.modelData.id)
                              ? "#fff5bf"
                              : (controllerItem.modelData.clamped
                                 ? "#d94c45"
                                 : root.controllerBorderColor(controllerItem.modelData))
                border.width: (controllerItem.modelData.selected || root.draggingControllerId === controllerItem.modelData.id) ? 2 : 1
            }
        }
    }

    Repeater {
        model: root.documentViewModel ? root.documentViewModel.autoPosingControllerHandles : []

        delegate: Item {
            id: handleItem

            required property var modelData

            readonly property vector3d handleWorldPosition: root.draggedPositionOrFallback(
                                                                 modelData.id,
                                                                 modelData.handleX,
                                                                 modelData.handleY,
                                                                 modelData.handleZ)
            readonly property vector3d projected: root.projectPoint(
                                                      handleWorldPosition.x,
                                                      handleWorldPosition.y,
                                                      handleWorldPosition.z)
            readonly property real controllerDiameter: root.controllerSize(modelData)
            property vector3d dragStartWorldPosition: Qt.vector3d(0, 0, 0)
            property vector3d dragPlanePoint: Qt.vector3d(0, 0, 0)
            property vector3d dragPlaneNormal: Qt.vector3d(0, 0, -1)

            visible: root.autoPosingVisible && modelData.visible && projected.z > 0
            width: controllerDiameter + 14
            height: controllerDiameter + 14
            x: projected.x - width / 2
            y: projected.y - height / 2

            Rectangle {
                anchors.centerIn: parent
                width: parent.width
                height: parent.height
                radius: width / 2
                color: "transparent"
            }

            DragHandler {
                id: handleDragHandler

                target: null
                acceptedButtons: Qt.LeftButton

                onActiveChanged: {
                    if (!active) {
                        root.documentViewModel.endAutoPosingDrag(handleItem.modelData.id)
                        root.draggingControllerId = ""
                        return
                    }

                    root.forceActiveFocus()
                    root.documentViewModel.beginAutoPosingDrag(handleItem.modelData.id)
                    root.draggingControllerId = handleItem.modelData.id
                    handleItem.dragStartWorldPosition = Qt.vector3d(
                        handleItem.modelData.targetX,
                        handleItem.modelData.targetY,
                        handleItem.modelData.targetZ
                    )
                    handleItem.dragPlanePoint = handleItem.dragStartWorldPosition
                    handleItem.dragPlaneNormal = root.controllerDragPlaneNormal(handleItem.modelData)
                    root.draggingWorldPosition = handleItem.dragStartWorldPosition
                }

                onCentroidChanged: {
                    if (!active)
                        return

                    const rootPoint = root.overlayPointFromScenePosition(centroid.scenePosition)
                    const nextScenePosition = root.draggedWorldPosition(
                        handleItem.dragStartWorldPosition,
                        handleItem.dragPlanePoint,
                        handleItem.dragPlaneNormal,
                        rootPoint
                    )
                    root.draggingWorldPosition = nextScenePosition
                    root.documentViewModel.previewAutoPosingController(
                        handleItem.modelData.id,
                        nextScenePosition.x,
                        nextScenePosition.y,
                        nextScenePosition.z
                    )
                }
            }

            HoverHandler {
                id: hoverHandler
                acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
                cursorShape: Qt.PointingHandCursor
            }

            TapHandler {
                acceptedButtons: Qt.LeftButton

                onTapped: eventPoint => {
                    root.forceActiveFocus()
                    root.documentViewModel.selectAutoPosingController(handleItem.modelData.id)
                }

                onDoubleTapped: eventPoint => {
                    root.forceActiveFocus()
                    root.documentViewModel.selectAutoPosingController(handleItem.modelData.id)
                }
            }

            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.NoButton
                hoverEnabled: true

                ToolTip.visible: containsMouse
                ToolTip.text: handleItem.modelData.name
            }
        }
    }
}
