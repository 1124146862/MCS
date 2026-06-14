import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    color: "#252526"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 10

        TextField {
            Layout.fillWidth: true
            placeholderText: qsTr("Filter by name")
            color: "#ffffff"
            font.pixelSize: 12

            background: Rectangle {
                color: "#1e1e1e"
                border.color: "#3e3e42"
                radius: 4
                implicitHeight: 28
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#1e1e1e"
            border.color: "#303642"
            radius: 4

            Column {
                anchors.fill: parent
                anchors.margins: 12
                spacing: 8

                Label { text: qsTr("> scene_root"); color: "#cccccc"; font.pixelSize: 12 }
                Label { text: qsTr("  > viewport_camera"); color: "#999999"; font.pixelSize: 12 }
                Label { text: qsTr("  > directional_light"); color: "#999999"; font.pixelSize: 12 }
                Label { text: qsTr("  > default_avatar"); color: "#999999"; font.pixelSize: 12 }
                Label { text: qsTr("  > source_model"); color: "#999999"; font.pixelSize: 12 }
            }
        }
    }
}
