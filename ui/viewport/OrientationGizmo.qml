import QtQuick
import QtQuick3D

Item {
    id: root

    required property Node cameraOrbit
    property bool orthographicView: false
    property bool navigationLocked: false

    signal xAxisClicked()
    signal yAxisClicked()
    signal zAxisClicked()
    signal projectionClicked()
    signal lockClicked()

    readonly property real orbitRadius: 28
    readonly property vector3d xAxisVector: Qt.vector3d(1, 0, 0)
    readonly property vector3d yAxisVector: Qt.vector3d(0, 1, 0)
    readonly property vector3d zAxisVector: Qt.vector3d(0, 0, 1)
    readonly property color xAxisColor: "#f54b4b"
    readonly property color yAxisColor: "#8fbc38"
    readonly property color zAxisColor: "#4285f4"

    width: 76
    height: 76

    function dot(a, b) {
        return a.x * b.x + a.y * b.y + a.z * b.z
    }

    function axisScreenVector(axis) {
        const right = root.cameraOrbit.right
        const up = root.cameraOrbit.up
        return Qt.point(root.dot(axis, right), root.dot(axis, up))
    }

    function axisDepth(axis) {
        return -root.dot(axis, root.cameraOrbit.forward)
    }

    function axisPoint(axis, positive) {
        const direction = positive
                ? axis
                : Qt.vector3d(-axis.x, -axis.y, -axis.z)
        const screenVector = root.axisScreenVector(direction)
        return Qt.point(
            bezel.width / 2 + screenVector.x * root.orbitRadius,
            bezel.height / 2 - screenVector.y * root.orbitRadius
        )
    }

    function axisFrontPoint(axis) {
        return root.axisDepth(axis) >= 0
                ? root.axisPoint(axis, true)
                : root.axisPoint(axis, false)
    }

    function axisBackPoint(axis) {
        return root.axisDepth(axis) >= 0
                ? root.axisPoint(axis, false)
                : root.axisPoint(axis, true)
    }

    function axisLineAngle(axis) {
        const start = root.axisPoint(axis, false)
        const end = root.axisPoint(axis, true)
        return Math.atan2(end.y - start.y, end.x - start.x) * 180 / Math.PI
    }

    function axisLineLength(axis) {
        const start = root.axisPoint(axis, false)
        const end = root.axisPoint(axis, true)
        const deltaX = end.x - start.x
        const deltaY = end.y - start.y
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY)
    }

    function axisLabelPoint(axis) {
        const frontPoint = root.axisFrontPoint(axis)
        const centerPoint = Qt.point(bezel.width / 2, bezel.height / 2)
        const deltaX = frontPoint.x - centerPoint.x
        const deltaY = frontPoint.y - centerPoint.y
        const length = Math.max(Math.sqrt(deltaX * deltaX + deltaY * deltaY), 0.001)
        const offset = 14
        return Qt.point(
            frontPoint.x + (deltaX / length) * offset,
            frontPoint.y + (deltaY / length) * offset
        )
    }

    Rectangle {
        id: bezel
        anchors.fill: parent
        radius: width / 2
        color: "#282a2e"
        opacity: 0.9
    }

    Item {
        anchors.fill: bezel

        Item {
            x: root.axisPoint(root.xAxisVector, false).x
            y: root.axisPoint(root.xAxisVector, false).y
            width: root.axisLineLength(root.xAxisVector)
            height: 2
            rotation: root.axisLineAngle(root.xAxisVector)
            transformOrigin: Item.Left

            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: parent.width
                height: 2
                radius: 1
                color: root.xAxisColor
                opacity: 0.9
            }
        }

        Item {
            x: root.axisPoint(root.yAxisVector, false).x
            y: root.axisPoint(root.yAxisVector, false).y
            width: root.axisLineLength(root.yAxisVector)
            height: 2
            rotation: root.axisLineAngle(root.yAxisVector)
            transformOrigin: Item.Left

            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: parent.width
                height: 2
                radius: 1
                color: root.yAxisColor
                opacity: 0.9
            }
        }

        Item {
            x: root.axisPoint(root.zAxisVector, false).x
            y: root.axisPoint(root.zAxisVector, false).y
            width: root.axisLineLength(root.zAxisVector)
            height: 2
            rotation: root.axisLineAngle(root.zAxisVector)
            transformOrigin: Item.Left

            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: parent.width
                height: 2
                radius: 1
                color: root.zAxisColor
                opacity: 0.9
            }
        }

        Rectangle {
            x: root.axisBackPoint(root.xAxisVector).x - width / 2
            y: root.axisBackPoint(root.xAxisVector).y - height / 2
            width: 14
            height: 14
            radius: 7
            color: root.xAxisColor
            opacity: 0.4
        }

        Rectangle {
            x: root.axisBackPoint(root.yAxisVector).x - width / 2
            y: root.axisBackPoint(root.yAxisVector).y - height / 2
            width: 14
            height: 14
            radius: 7
            color: root.yAxisColor
            opacity: 0.4
        }

        Rectangle {
            x: root.axisBackPoint(root.zAxisVector).x - width / 2
            y: root.axisBackPoint(root.zAxisVector).y - height / 2
            width: 14
            height: 14
            radius: 7
            color: root.zAxisColor
            opacity: 0.4
        }
    }

    View3D {
        anchors.centerIn: bezel
        width: 30
        height: 30
        camera: gizmoCamera

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Transparent
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            eulerRotation: root.cameraOrbit.eulerRotation

            OrthographicCamera {
                id: gizmoCamera
                z: 32
                clipNear: 1
                clipFar: 100
                horizontalMagnification: 12
                verticalMagnification: 12
            }

            DirectionalLight {
                eulerRotation: Qt.vector3d(-20, 25, 0)
                brightness: 1.1
                ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
                color: "#ffffff"
            }
        }

        Model {
            source: "#Cube"
            scale: Qt.vector3d(5.0, 5.0, 5.0)
            materials: [
                PrincipledMaterial {
                    baseColor: root.orthographicView ? "#e6e6e6" : "#cccccc"
                    roughness: 0.3
                    metalness: 0.0
                }
            ]
        }
    }

    Item {
        anchors.fill: bezel

        Rectangle {
            anchors.centerIn: parent
            width: 24
            height: 24
            radius: 2
            color: "transparent"

            MouseArea {
                anchors.fill: parent
                enabled: !root.navigationLocked
                cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                onClicked: root.projectionClicked()
            }
        }

        Rectangle {
            x: root.axisLabelPoint(root.xAxisVector).x - width / 2
            y: root.axisLabelPoint(root.xAxisVector).y - height / 2
            width: 20
            height: 20
            radius: 10
            color: root.xAxisColor

            Text {
                anchors.centerIn: parent
                text: "X"
                color: "#111111"
                font.pixelSize: 10
                font.bold: true
            }

            MouseArea {
                anchors.fill: parent
                enabled: !root.navigationLocked
                cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                onClicked: root.xAxisClicked()
            }
        }

        Rectangle {
            x: root.axisLabelPoint(root.yAxisVector).x - width / 2
            y: root.axisLabelPoint(root.yAxisVector).y - height / 2
            width: 20
            height: 20
            radius: 10
            color: root.yAxisColor

            Text {
                anchors.centerIn: parent
                text: "Y"
                color: "#111111"
                font.pixelSize: 10
                font.bold: true
            }

            MouseArea {
                anchors.fill: parent
                enabled: !root.navigationLocked
                cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                onClicked: root.yAxisClicked()
            }
        }

        Rectangle {
            x: root.axisLabelPoint(root.zAxisVector).x - width / 2
            y: root.axisLabelPoint(root.zAxisVector).y - height / 2
            width: 20
            height: 20
            radius: 10
            color: root.zAxisColor

            Text {
                anchors.centerIn: parent
                text: "Z"
                color: "#111111"
                font.pixelSize: 10
                font.bold: true
            }

            MouseArea {
                anchors.fill: parent
                enabled: !root.navigationLocked
                cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                onClicked: root.zAxisClicked()
            }
        }
    }

    Image {
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.rightMargin: 0
        anchors.bottomMargin: 0
        source: root.navigationLocked ? "../../assets/lock.svg" : "../../assets/unlock.svg"
        sourceSize.width: 14
        sourceSize.height: 14
        opacity: root.navigationLocked ? 0.9 : 0.3

        MouseArea {
            anchors.fill: parent
            anchors.margins: -4
            cursorShape: Qt.PointingHandCursor
            onClicked: root.lockClicked()
        }
    }
}
