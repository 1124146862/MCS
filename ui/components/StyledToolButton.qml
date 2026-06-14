import QtQuick
import QtQuick.Controls

ToolButton {
    id: button

    contentItem: Text {
        text: button.text
        color: (button.checked || button.down) ? "#ffffff" : (button.hovered ? "#ffffff" : "#cccccc")
        font.pixelSize: 13
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }

    background: Rectangle {
        color: (button.checked || button.down) ? "#007acc" : (button.hovered ? "#3e3e42" : "transparent")
        radius: 4
    }
}
