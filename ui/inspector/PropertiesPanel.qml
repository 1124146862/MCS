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
            text: root.documentViewModel && root.documentViewModel.isAutoPosingMode
                  ? qsTr("AutoPosing Properties")
                  : qsTr("Viewport Properties")
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

                Label {
                    text: qsTr("Edit mode: ") + (root.documentViewModel ? root.documentViewModel.currentEditMode : qsTr("view"))
                    color: "#cccccc"
                    font.pixelSize: 12
                }

                Label {
                    visible: !(root.documentViewModel && root.documentViewModel.isAutoPosingMode)
                    text: qsTr("Camera: Perspective")
                    color: "#cccccc"
                    font.pixelSize: 12
                }

                Label {
                    visible: !(root.documentViewModel && root.documentViewModel.isAutoPosingMode)
                    text: qsTr("Ground grid: On")
                    color: "#cccccc"
                    font.pixelSize: 12
                }

                Label {
                    visible: !(root.documentViewModel && root.documentViewModel.isAutoPosingMode)
                    text: qsTr("Asset source: FBX")
                    color: "#cccccc"
                    font.pixelSize: 12
                }

                Label {
                    visible: !(root.documentViewModel && root.documentViewModel.isAutoPosingMode)
                    text: root.documentViewModel && root.documentViewModel.hasGeneratedModel
                          ? qsTr("Runtime import: Ready")
                          : qsTr("Runtime import: Pending / Fallback")
                    color: "#cccccc"
                    font.pixelSize: 12
                }

                Label {
                    visible: root.documentViewModel && root.documentViewModel.isAutoPosingMode
                    text: root.documentViewModel && root.documentViewModel.selectedAutoPosingControllerName !== ""
                          ? qsTr("Controller: ") + root.documentViewModel.selectedAutoPosingControllerName
                          : qsTr("Controller: None selected")
                    color: "#ffffff"
                    font.pixelSize: 12
                    font.bold: true
                }

                Label {
                    visible: root.documentViewModel && root.documentViewModel.isAutoPosingMode
                    text: qsTr("Type: ") + (root.documentViewModel ? root.documentViewModel.selectedAutoPosingControllerType : "")
                    color: "#cccccc"
                    font.pixelSize: 12
                }

                Label {
                    visible: root.documentViewModel && root.documentViewModel.isAutoPosingMode
                    text: qsTr("Joint: ") + (root.documentViewModel ? root.documentViewModel.selectedAutoPosingControllerJointName : "")
                    color: "#cccccc"
                    font.pixelSize: 12
                }

                Label {
                    visible: root.documentViewModel && root.documentViewModel.isAutoPosingMode
                    text: qsTr("State: ") + (root.documentViewModel ? root.documentViewModel.selectedAutoPosingControllerStatus : "")
                    color: "#cccccc"
                    font.pixelSize: 12
                }

                Label {
                    visible: root.documentViewModel && root.documentViewModel.isAutoPosingMode
                    text: qsTr("Position: ") + (root.documentViewModel ? root.documentViewModel.selectedAutoPosingControllerPosition : "")
                    color: "#cccccc"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                }

                RowLayout {
                    visible: root.documentViewModel && root.documentViewModel.isAutoPosingMode
                              && root.documentViewModel.selectedAutoPosingControllerName !== ""
                    spacing: 8

                    Button {
                        text: qsTr("Fix / Unfix")
                        onClicked: root.documentViewModel.toggleSelectedAutoPosingFixed()
                    }

                    Button {
                        text: qsTr("Reset (Shift+Z)")
                        onClicked: root.documentViewModel.resetSelectedAutoPosingController()
                    }
                }
            }
        }

        Item { Layout.fillHeight: true }
    }
}
