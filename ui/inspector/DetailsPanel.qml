import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    color: "#252526"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 10

        Label {
            text: qsTr("Session Notes")
            color: "#ffffff"
            font.pixelSize: 12
            font.bold: true
        }

        TextArea {
            Layout.fillWidth: true
            Layout.fillHeight: true
            placeholderText: qsTr("Prototype notes...")
            color: "#cccccc"
            font.pixelSize: 12
            wrapMode: TextEdit.Wrap

            background: Rectangle {
                color: "#1e1e1e"
                border.color: "#303642"
                radius: 4
            }
        }
    }
}
