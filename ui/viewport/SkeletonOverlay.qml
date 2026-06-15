import QtQuick
import QtQuick3D

Node {
    id: root

    property var joints: []
    property var bones: []
    property real primitiveBaseSize: 100.0
    property real jointDiameter: 3.0
    property real endJointDiameter: 4.4
    property real boneThickness: 0.7
    property real depthBias: -10.0

    visible: joints && joints.length > 0

    function vectorFromBoneStart(bone) {
        return Qt.vector3d(
            bone.endX - bone.startX,
            bone.endY - bone.startY,
            bone.endZ - bone.startZ
        )
    }

    function vectorLength(vector) {
        return Math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
    }

    function midpoint(bone) {
        return Qt.vector3d(
            (bone.startX + bone.endX) * 0.5,
            (bone.startY + bone.endY) * 0.5,
            (bone.startZ + bone.endZ) * 0.5
        )
    }

    function boneRotationQuaternion(bone) {
        const direction = vectorFromBoneStart(bone)
        const length = vectorLength(direction)
        if (length <= 0.0001)
            return Qt.quaternion(1, 0, 0, 0)

        const normalized = Qt.vector3d(direction.x / length, direction.y / length, direction.z / length)
        const up = Qt.vector3d(0, 1, 0)
        const dot = Math.max(-1, Math.min(1, up.x * normalized.x + up.y * normalized.y + up.z * normalized.z))

        if (dot >= 0.9999)
            return Qt.quaternion(1, 0, 0, 0)

        if (dot <= -0.9999)
            return Qt.quaternion(0, 0, 0, 1)

        const axis = Qt.vector3d(
            up.y * normalized.z - up.z * normalized.y,
            up.z * normalized.x - up.x * normalized.z,
            up.x * normalized.y - up.y * normalized.x
        )
        const axisLength = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z)
        if (axisLength <= 0.0001)
            return Qt.quaternion(1, 0, 0, 0)

        const normalizedAxis = Qt.vector3d(axis.x / axisLength, axis.y / axisLength, axis.z / axisLength)
        const angle = Math.acos(dot)
        const halfAngle = angle * 0.5
        const sine = Math.sin(halfAngle)
        return Qt.quaternion(
            Math.cos(halfAngle),
            normalizedAxis.x * sine,
            normalizedAxis.y * sine,
            normalizedAxis.z * sine
        )
    }

    Repeater3D {
        model: root.bones

        delegate: Node {
            required property var modelData

            position: root.midpoint(modelData)
            rotation: root.boneRotationQuaternion(modelData)

            Model {
                source: "#Cylinder"
                depthBias: root.depthBias
                scale: Qt.vector3d(
                    root.boneThickness / root.primitiveBaseSize,
                    Math.max(0.001, root.vectorLength(root.vectorFromBoneStart(modelData)) / root.primitiveBaseSize),
                    root.boneThickness / root.primitiveBaseSize
                )
                materials: [
                    DefaultMaterial {
                        lighting: DefaultMaterial.NoLighting
                        diffuseColor: modelData.color ? modelData.color : "#1b6f3a"
                    }
                ]
            }
        }
    }

    Repeater3D {
        model: root.joints

        delegate: Model {
            required property var modelData

            source: "#Sphere"
            depthBias: root.depthBias
            position: Qt.vector3d(modelData.x, modelData.y, modelData.z)

            scale: {
                const diameter = modelData.isTerminal ? root.endJointDiameter : root.jointDiameter
                const normalized = diameter / root.primitiveBaseSize
                return Qt.vector3d(normalized, normalized, normalized)
            }

            materials: [
                DefaultMaterial {
                    lighting: DefaultMaterial.NoLighting
                    diffuseColor: modelData.color ? modelData.color : "#17823d"
                }
            ]
        }
    }
}
