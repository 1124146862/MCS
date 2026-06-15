import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import "../components" as Components

Item {
    id: root

    property var documentViewModel: null
    readonly property bool importedModelReady: documentViewModel !== null && documentViewModel.hasGeneratedModel
    readonly property bool autoPosingMode: documentViewModel !== null && documentViewModel.isAutoPosingMode
    readonly property bool jointMode: documentViewModel !== null && documentViewModel.currentEditMode === "joint"
    readonly property real viewportAspectRatio: Math.max(sceneView.width, 1) / Math.max(sceneView.height, 1)
    property bool orthographicView: false
    property bool navigationLocked: false
    property bool xAxisForward: true
    property bool yAxisTop: true
    property bool zAxisRight: true
    property real cameraDistance: 520
    readonly property real perspectiveFieldOfView: 45
    readonly property real isometricFieldOfView: 4

    function normalizeAngle(angle) {
        let normalized = angle % 360
        if (normalized <= -180)
            normalized += 360
        if (normalized > 180)
            normalized -= 360
        return normalized
    }

    function angleDistance(a, b) {
        return Math.abs(normalizeAngle(a - b))
    }

    function syncAxisIndicators() {
        const rotation = cameraOrbit.eulerRotation
        const yaw = normalizeAngle(rotation.y)
        const pitch = rotation.x

        xAxisForward = angleDistance(yaw, 90) <= angleDistance(yaw, -90)
        zAxisRight = angleDistance(yaw, 0) <= angleDistance(yaw, 180)
        yAxisTop = Math.abs(pitch - (-89)) <= Math.abs(pitch - 89)
    }

    function applyViewAngles(pitch, yaw) {
        cameraOrbit.eulerRotation = Qt.vector3d(pitch, normalizeAngle(yaw), 0)
        syncAxisIndicators()
    }

    function visibleHeightFor(distance, fieldOfView) {
        const safeDistance = Math.max(distance, 1)
        const safeFieldOfView = Math.max(fieldOfView, 0.1)
        const radians = safeFieldOfView * Math.PI / 180

        if (perspectiveCamera.fieldOfViewOrientation === PerspectiveCamera.Horizontal) {
            const visibleWidth = 2 * safeDistance * Math.tan(radians / 2)
            return visibleWidth / viewportAspectRatio
        }

        return 2 * safeDistance * Math.tan(radians / 2)
    }

    function toggleXAxisView() {
        if (navigationLocked)
            return

        xAxisForward = !xAxisForward
        applyViewAngles(0, xAxisForward ? 90 : -90)
    }

    function toggleYAxisView() {
        if (navigationLocked)
            return

        yAxisTop = !yAxisTop
        applyViewAngles(yAxisTop ? -89 : 89, 0)
    }

    function toggleZAxisView() {
        if (navigationLocked)
            return

        zAxisRight = !zAxisRight
        applyViewAngles(0, zAxisRight ? 0 : 180)
    }

    function toggleProjectionMode() {
        if (navigationLocked)
            return

        const currentFieldOfView = orthographicView ? isometricFieldOfView : perspectiveFieldOfView
        const nextFieldOfView = orthographicView ? perspectiveFieldOfView : isometricFieldOfView
        const currentVisibleHeight = visibleHeightFor(cameraDistance, currentFieldOfView)
        const nextRadians = nextFieldOfView * Math.PI / 180

        cameraDistance = currentVisibleHeight / (2 * Math.tan(nextRadians / 2))

        orthographicView = !orthographicView
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 32
            color: "#2d2d30"

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 8
                spacing: 8

                Label {
                    text: qsTr("Viewport")
                    color: "#ffffff"
                    font.pixelSize: 12
                    font.bold: true
                }

                Label {
                    text: root.orthographicView ? qsTr("Isometric") : qsTr("Perspective")
                    color: "#007acc"
                    font.pixelSize: 12
                }

                Item { Layout.fillWidth: true }

                Components.StyledToolButton { text: qsTr("Grid"); implicitHeight: 24 }
                Components.StyledToolButton { text: qsTr("Shaded"); implicitHeight: 24 }
            }
        }

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true

            Item {
                id: viewportBackground
                anchors.fill: parent

                Rectangle {
                    anchors.fill: parent

                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "#5d6065" }
                        GradientStop { position: 1.0; color: "#70747a" }
                    }
                }
            }

            View3D {
                id: sceneView
                anchors.fill: parent
                anchors.margins: 18
                camera: perspectiveCamera

                environment: SceneEnvironment {
                    backgroundMode: SceneEnvironment.Transparent
                    antialiasingMode: SceneEnvironment.MSAA
                    antialiasingQuality: SceneEnvironment.VeryHigh
                }

                Node {
                    id: cameraOrbit
                    position: Qt.vector3d(0, 110, 0)
                    eulerRotation: Qt.vector3d(-18, 0, 0)

                    onEulerRotationChanged: root.syncAxisIndicators()

                    PerspectiveCamera {
                        id: perspectiveCamera
                        z: root.cameraDistance
                        clipNear: 1
                        clipFar: 10000
                        fieldOfView: root.orthographicView ? root.isometricFieldOfView : root.perspectiveFieldOfView
                        onZChanged: root.cameraDistance = z
                    }

                    DirectionalLight {
                        brightness: 0.3
                        ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
                        color: "#efefef"
                    }
                }

                DirectionalLight {
                    eulerRotation: Qt.vector3d(-92, 0, 0)
                    brightness: 0.7
                    color: "#f3f3f1"
                }

                Node {
                    DirectionalLight {
                        eulerRotation: Qt.vector3d(88, 0, 0)
                        brightness: 0.3
                        color: "#cfd4de"
                    }

                    Model {
                        position: Qt.vector3d(0, 0.4, 0)
                        eulerRotation: Qt.vector3d(-90, 0, 0)
                        geometry: GridGeometry {
                            horizontalLines: 61
                            verticalLines: 61
                            horizontalStep: 20
                            verticalStep: 20
                        }

                        materials: [
                            DefaultMaterial {
                                lighting: DefaultMaterial.NoLighting
                                diffuseColor: "#70747a"
                            }
                        ]
                    }

                    Model {
                        position: Qt.vector3d(0, 0.45, 0)
                        eulerRotation: Qt.vector3d(-90, 0, 0)
                        geometry: GridGeometry {
                            horizontalLines: 13
                            verticalLines: 13
                            horizontalStep: 100
                            verticalStep: 100
                        }

                        materials: [
                            DefaultMaterial {
                                lighting: DefaultMaterial.NoLighting
                                diffuseColor: "#8b9096"
                            }
                        ]
                    }

                    Loader3D {
                        id: runtimeModelLoader
                        visible: root.importedModelReady
                        source: root.documentViewModel ? root.documentViewModel.generatedComponentUrl : ""
                        scale: Qt.vector3d(1, 1, 1)

                        onItemChanged: {
                            if (!root.documentViewModel)
                                return

                            if (item)
                                root.documentViewModel.bindRuntimeModel(item)
                            else
                                root.documentViewModel.clearRuntimeModel()
                        }
                    }

                    Node {
                        visible: !root.importedModelReady

                        Model {
                            source: "#Sphere"
                            position: Qt.vector3d(0, 192, 0)
                            scale: Qt.vector3d(28, 28, 28)

                            materials: [
                                PrincipledMaterial {
                                    baseColor: "#d9dde5"
                                    roughness: 0.42
                                }
                            ]
                        }

                        Model {
                            source: "#Cube"
                            position: Qt.vector3d(0, 120, 0)
                            scale: Qt.vector3d(52, 90, 26)

                            materials: [
                                PrincipledMaterial {
                                    baseColor: "#d9dde5"
                                    roughness: 0.42
                                }
                            ]
                        }

                        Model {
                            source: "#Cube"
                            position: Qt.vector3d(-42, 126, 0)
                            scale: Qt.vector3d(14, 68, 14)

                            materials: [
                                PrincipledMaterial {
                                    baseColor: "#cfd5df"
                                    roughness: 0.48
                                }
                            ]
                        }

                        Model {
                            source: "#Cube"
                            position: Qt.vector3d(42, 126, 0)
                            scale: Qt.vector3d(14, 68, 14)

                            materials: [
                                PrincipledMaterial {
                                    baseColor: "#cfd5df"
                                    roughness: 0.48
                                }
                            ]
                        }

                        Model {
                            source: "#Cube"
                            position: Qt.vector3d(-16, 42, 0)
                            scale: Qt.vector3d(16, 82, 16)

                            materials: [
                                PrincipledMaterial {
                                    baseColor: "#cfd5df"
                                    roughness: 0.48
                                }
                            ]
                        }

                        Model {
                            source: "#Cube"
                            position: Qt.vector3d(16, 42, 0)
                            scale: Qt.vector3d(16, 82, 16)

                            materials: [
                                PrincipledMaterial {
                                    baseColor: "#cfd5df"
                                    roughness: 0.48
                                }
                            ]
                        }
                    }
                }
            }

            View3D {
                anchors.fill: sceneView
                enabled: false
                camera: skeletonOverlayCamera
                visible: root.jointMode || root.autoPosingMode

                environment: SceneEnvironment {
                    backgroundMode: SceneEnvironment.Transparent
                    antialiasingMode: SceneEnvironment.MSAA
                    antialiasingQuality: SceneEnvironment.VeryHigh
                }

                Node {
                    position: cameraOrbit.position
                    eulerRotation: cameraOrbit.eulerRotation

                    PerspectiveCamera {
                        id: skeletonOverlayCamera
                        z: root.cameraDistance
                        clipNear: 1
                        clipFar: 10000
                        fieldOfView: root.orthographicView ? root.isometricFieldOfView : root.perspectiveFieldOfView
                    }
                }

                SkeletonOverlay {
                    visible: root.jointMode || root.autoPosingMode
                    joints: root.documentViewModel
                            ? (root.autoPosingMode
                               ? root.documentViewModel.autoPosingPreviewJoints
                               : root.documentViewModel.skeletonJoints)
                            : []
                    bones: root.documentViewModel
                           ? (root.autoPosingMode
                              ? root.documentViewModel.autoPosingPreviewBones
                              : root.documentViewModel.skeletonBones)
                           : []
                }
            }

            AutoPosingOverlay {
                id: autoPosingOverlay
                anchors.fill: sceneView
                sceneView: sceneView
                cameraOrbit: cameraOrbit
                cameraDistance: root.cameraDistance
                fieldOfView: root.orthographicView ? root.isometricFieldOfView : root.perspectiveFieldOfView
                horizontalFieldOfView: perspectiveCamera.fieldOfViewOrientation === PerspectiveCamera.Horizontal
                visibleWorldHeight: root.visibleHeightFor(
                                        root.cameraDistance,
                                        root.orthographicView ? root.isometricFieldOfView : root.perspectiveFieldOfView)
                documentViewModel: root.documentViewModel
            }

            ViewportCameraController {
                id: cameraController
                anchors.fill: sceneView
                origin: cameraOrbit
                camera: perspectiveCamera
                navigationEnabled: !root.navigationLocked
                focusTarget: Qt.vector3d(0, 110, 0)
                focusDistance: 520
                orbitSensitivity: 0.35
                panSensitivity: 1.0
                zoomStep: 0.12
                minDistance: 80
                maxDistance: 2400
                onAxisSnapRequested: (pitch, yaw) => root.applyViewAngles(pitch, yaw)
                onDistanceChanged: distance => root.cameraDistance = distance
            }

            OrientationGizmo {
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.margins: 20
                cameraOrbit: cameraOrbit
                orthographicView: root.orthographicView
                navigationLocked: root.navigationLocked
                onProjectionClicked: root.toggleProjectionMode()
                onXAxisClicked: root.toggleXAxisView()
                onYAxisClicked: root.toggleYAxisView()
                onZAxisClicked: root.toggleZAxisView()
                onLockClicked: root.navigationLocked = !root.navigationLocked
            }

            Rectangle {
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                height: 24
                color: "#1e1e1e"
                opacity: 0.9

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 12
                    anchors.rightMargin: 12

                    Label {
                        text: root.autoPosingMode
                              ? qsTr("AutoPosing mode active")
                              : (root.importedModelReady
                                 ? qsTr("Imported runtime model")
                                 : qsTr("Placeholder model active"))
                        color: "#cccccc"
                        font.pixelSize: 11
                    }

                    Item { Layout.fillWidth: true }

                    Label {
                        text: root.documentViewModel
                              ? (root.autoPosingMode
                                 ? root.documentViewModel.previewStatusMessage
                                 : root.documentViewModel.statusMessage)
                              : qsTr("Preparing assets...")
                        color: "#999999"
                        font.pixelSize: 11
                        elide: Text.ElideRight
                        Layout.maximumWidth: parent.width * 0.75
                    }
                }
            }
        }
    }
}
