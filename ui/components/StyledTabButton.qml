import QtQuick
import QtQuick.Controls

TabButton {
    id: tabButton

    contentItem: Text {
        text: tabButton.text
        color: tabButton.checked ? "#ffffff" : "#999999"
        font.pixelSize: 12
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }

    background: Rectangle {
        color: tabButton.checked ? "#252526" : "transparent"
    }
}
