import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: root

    property var documentViewModel: null
    color: "#252526"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 10

        Label {
            text: qsTr("Viewport Properties")
            color: "#ffffff"
            font.pixelSize: 12
            font.bold: true
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 220
            color: "#1e1e1e"
            border.color: "#303642"
            radius: 4

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 12
                spacing: 8

                Label { text: qsTr("Camera: Perspective"); color: "#cccccc"; font.pixelSize: 12 }
                Label { text: qsTr("Ground grid: On"); color: "#cccccc"; font.pixelSize: 12 }
                Label { text: qsTr("Asset source: FBX"); color: "#cccccc"; font.pixelSize: 12 }
                Label {
                    text: root.documentViewModel && root.documentViewModel.hasGeneratedModel
                          ? qsTr("Runtime import: Ready")
                          : qsTr("Runtime import: Pending / Fallback")
                    color: "#cccccc"
                    font.pixelSize: 12
                }
            }
        }

        Item { Layout.fillHeight: true }
    }
}
