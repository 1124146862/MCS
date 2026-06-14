import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../components" as Components

Rectangle {
    color: "#252526"
    SplitView.preferredHeight: 140

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
                    text: qsTr("Timeline")
                    color: "#ffffff"
                    font.pixelSize: 12
                    font.bold: true
                }

                Item { Layout.fillWidth: true }

                Components.StyledToolButton { text: qsTr("Play"); implicitHeight: 24 }
                Components.StyledToolButton { text: qsTr("Pause"); implicitHeight: 24 }
                Components.StyledToolButton { text: qsTr("Stop"); implicitHeight: 24 }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#1e1e1e"
            radius: 2

            Repeater {
                model: 25

                Rectangle {
                    x: 12 + index * ((parent.width - 24) / 24)
                    y: 8
                    width: 1
                    height: parent.height - 16
                    color: index % 5 === 0 ? "#4b5361" : "#353b45"
                }
            }

            Rectangle {
                x: 140
                y: 0
                width: 2
                height: parent.height
                color: "#f85149"
            }
        }
    }
}
