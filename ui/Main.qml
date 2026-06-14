pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts
import Qt5Compat.GraphicalEffects
import "components" as Components
import "inspector" as Inspector
import "outliner" as Outliner
import "timeline" as Timeline
import "viewport" as Viewport

ApplicationWindow {
    id: root

    required property var documentViewModel

    width: 1600
    height: 920
    minimumWidth: 1200
    minimumHeight: 720
    visible: true
    title: qsTr("MCS Python Prototype")
    color: "#1e1e1e"

    readonly property var documentState: documentViewModel
    readonly property bool hasDocumentData: documentState !== null
    property string launcherSection: "models"

    menuBar: MenuBar {
        background: Rectangle { color: "#252526" }

        delegate: MenuBarItem {
            id: menuBarItem

            contentItem: Text {
                text: menuBarItem.text
                color: menuBarItem.highlighted ? "#ffffff" : "#cccccc"
                font.pixelSize: 12
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
            }

            background: Rectangle {
                color: menuBarItem.highlighted ? "#3e3e42" : "transparent"
            }
        }

        Menu {
            title: qsTr("File")
            Action { text: qsTr("New Project") }
            Action { text: qsTr("Open Project") }
            MenuSeparator {}
            Action { text: qsTr("Exit") }
        }

        Menu {
            title: qsTr("Edit")
            Action { text: qsTr("Undo") }
            Action { text: qsTr("Redo") }
        }

        Menu {
            title: qsTr("View")
            Action { text: qsTr("Reset Layout") }
            Action { text: qsTr("Focus Viewport") }
        }

        Menu {
            title: qsTr("Playback")
            Action { text: qsTr("Play") }
            Action { text: qsTr("Stop") }
        }

        Menu {
            title: qsTr("Settings")
            Action { text: qsTr("Preferences") }
        }

        Menu {
            title: qsTr("Help")
            Action { text: qsTr("About") }
        }
    }

    header: ToolBar {
        padding: 6

        background: Rectangle {
            color: "#2d2d30"

            Rectangle {
                width: parent.width
                height: 1
                anchors.bottom: parent.bottom
                color: "#1e1e1e"
            }
        }

        RowLayout {
            anchors.fill: parent
            spacing: 4

            ToolButton {
                id: homeButton
                text: "\u2302"
                implicitWidth: 32
                implicitHeight: 28
                hoverEnabled: true
                onClicked: launcherPopup.open()

                contentItem: Text {
                    text: homeButton.text
                    color: homeButton.hovered ? "#ffffff" : "#d7d7d7"
                    font.pixelSize: 16
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                background: Rectangle {
                    color: homeButton.down ? "#007acc" : (homeButton.hovered ? "#3e3e42" : "transparent")
                    radius: 4
                }

                ToolTip.visible: homeButton.hovered
                ToolTip.text: qsTr("Home")
            }

            Rectangle {
                Layout.fillHeight: true
                Layout.margins: 4
                Layout.preferredWidth: 1
                color: "#454545"
            }

            Components.StyledToolButton { text: qsTr("Select") }
            Components.StyledToolButton { text: qsTr("Move") }
            Components.StyledToolButton { text: qsTr("Rotate") }
            Components.StyledToolButton { text: qsTr("Scale") }

            Rectangle {
                Layout.fillHeight: true
                Layout.margins: 4
                Layout.preferredWidth: 1
                color: "#454545"
            }

            Components.StyledToolButton { text: qsTr("Front") }
            Components.StyledToolButton { text: qsTr("Side") }
            Components.StyledToolButton { text: qsTr("Top") }
            Components.StyledToolButton { text: qsTr("Persp") }

            Item { Layout.fillWidth: true }

            Label {
                text: qsTr("MCS Viewport Prototype")
                color: "#999999"
                font.pixelSize: 13
                Layout.rightMargin: 10
            }
        }
    }

    FileDialog {
        id: importModelDialog
        title: qsTr("Import FBX Model")
        nameFilters: [qsTr("FBX Files (*.fbx)"), qsTr("All Files (*)")]

        onAccepted: {
            if (selectedFile) {
                root.documentState.importModelFromFile(selectedFile.toString())
                root.launcherSection = "models"
                launcherPopup.open()
            }
        }
    }

    Popup {
        id: launcherPopup

        x: Math.round((root.width - width) / 2)
        y: Math.round(Math.max(44, (root.height - height) / 2 - 26))
        width: Math.min(root.width - 96, 980)
        height: Math.min(root.height - 140, 620)
        modal: true
        focus: true
        padding: 0
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        Overlay.modal: Rectangle {
            color: "#000000"
            opacity: 0.42
        }

        background: Rectangle {
            color: "#1f2024"
            radius: 14
            border.color: "#069df3"
            border.width: 1
        }

        contentItem: ColumnLayout {
            spacing: 0

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 78
                color: "#069df3"
                topLeftRadius: 14
                topRightRadius: 14

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 28
                    anchors.rightMargin: 22
                    spacing: 22

                    Label {
                        text: qsTr("MCS Hub")
                        color: "#ffffff"
                        font.pixelSize: 24
                        font.bold: true
                    }

                    Item { Layout.fillWidth: true }

                    Repeater {
                        model: ["Website", "Upgrade", "Support", "Documentation"]

                        delegate: ToolButton {
                            id: launcherNavButton
                            required property string modelData
                            text: modelData
                            hoverEnabled: true

                            contentItem: Text {
                                text: launcherNavButton.text
                                color: "#ffffff"
                                font.pixelSize: 13
                                font.bold: launcherNavButton.hovered
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }

                            background: Rectangle {
                                color: launcherNavButton.hovered ? "#1587d0" : "transparent"
                                radius: 4
                            }

                            onClicked: {
                                if (text === "Website")
                                    Qt.openUrlExternally("https://www.noitom.com")
                                else if (text === "Support")
                                    Qt.openUrlExternally("https://support.noitom.com")
                                else if (text === "Documentation")
                                    Qt.openUrlExternally("https://docs.noitom.com")
                            }
                        }
                    }

                    Rectangle {
                        Layout.preferredWidth: 28
                        Layout.preferredHeight: 28
                        radius: 14
                        color: "#ffffff"
                        opacity: 0.92
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 0

                Rectangle {
                    Layout.preferredWidth: 220
                    Layout.fillHeight: true
                    color: "#222222"

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 0
                        anchors.rightMargin: 20
                        anchors.topMargin: 34
                        anchors.bottomMargin: 26
                        spacing: 6

                        ToolButton {
                            id: projectsButton
                            text: qsTr("PROJECTS")
                            Layout.fillWidth: true
                            Layout.preferredHeight: 36
                            hoverEnabled: true
                            onClicked: root.launcherSection = "projects"

                            contentItem: Text {
                                text: projectsButton.text
                                color: root.launcherSection === "projects" ? "#069df3" : "#ffffff"
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                anchors.left: parent.left; anchors.leftMargin: 28
                            }

                            background: Rectangle {
                                color: projectsButton.hovered ? "#2a2c31" : "transparent"
                                Rectangle {
                                    width: 4; height: parent.height
                                    color: "#069df3"
                                    visible: root.launcherSection === "projects"
                                    anchors.left: parent.left
                                }
                            }
                        }

                        ToolButton {
                            id: learnButton
                            text: qsTr("LEARN")
                            Layout.fillWidth: true
                            Layout.preferredHeight: 36
                            hoverEnabled: true

                            contentItem: Text {
                                text: learnButton.text
                                color: "#ffffff"
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                anchors.left: parent.left; anchors.leftMargin: 28
                            }

                            background: Rectangle {
                                color: learnButton.hovered ? "#2a2c31" : "transparent"
                            }
                        }

                        ToolButton {
                            id: modelsButton
                            text: qsTr("MODELS")
                            Layout.fillWidth: true
                            Layout.preferredHeight: 36
                            hoverEnabled: true
                            onClicked: root.launcherSection = "models"

                            contentItem: Text {
                                text: modelsButton.text
                                color: root.launcherSection === "models" ? "#069df3" : "#ffffff"
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                anchors.left: parent.left; anchors.leftMargin: 28
                            }

                            background: Rectangle {
                                color: modelsButton.hovered ? "#2a2c31" : "transparent"
                                Rectangle {
                                    width: 4; height: parent.height
                                    color: "#069df3"
                                    visible: root.launcherSection === "models"
                                    anchors.left: parent.left
                                }
                            }
                        }

                        ToolButton {
                            id: newsButton
                            text: qsTr("NEWS")
                            Layout.fillWidth: true
                            Layout.preferredHeight: 36
                            hoverEnabled: true

                            contentItem: Text {
                                text: newsButton.text
                                color: "#ffffff"
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                anchors.left: parent.left; anchors.leftMargin: 28
                            }

                            background: Rectangle {
                                color: newsButton.hovered ? "#2a2c31" : "transparent"
                            }
                        }

                        Item { Layout.fillHeight: true }

                        Button {
                            id: newSceneButton
                            text: qsTr("New scene")
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            Layout.leftMargin: 28
                            onClicked: {
                                root.documentState.newScene()
                                launcherPopup.close()
                            }

                            contentItem: Text {
                                text: newSceneButton.text
                                color: "#ffffff"
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }

                            background: Rectangle {
                                color: "#069df3"
                                radius: 6
                            }
                        }

                        Button {
                            id: openButton
                            text: qsTr("Open ...")
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            Layout.leftMargin: 28
                            Layout.topMargin: 4
                            onClicked: importModelDialog.open()

                            contentItem: Text {
                                text: openButton.text
                                color: "#ffffff"
                                font.pixelSize: 14
                                font.bold: true
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                anchors.left: parent.left
                                anchors.leftMargin: 12
                            }

                            background: Rectangle {
                                color: "transparent"
                                radius: 6
                                border.color: "#ffffff"
                                border.width: 1
                            }
                        }

                        Label {
                            text: qsTr("Version: Prototype")
                            color: "#4a4c52"
                            font.pixelSize: 11
                            Layout.leftMargin: 28
                            Layout.topMargin: 20
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: "#202225"

                    StackLayout {
                        anchors.fill: parent
                        anchors.margins: 26
                        currentIndex: root.launcherSection === "projects" ? 0 : 1

                        ColumnLayout {
                            spacing: 16

                            Label {
                                text: qsTr("PROJECTS")
                                color: "#ffffff"
                                font.pixelSize: 24
                                font.bold: true
                            }

                            Label {
                                text: qsTr("Use New scene to start with an empty viewport, or Open to import another FBX into the project.")
                                color: "#b7bcc4"
                                font.pixelSize: 13
                                wrapMode: Text.WordWrap
                                Layout.maximumWidth: 520
                            }

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                radius: 12
                                color: "#26282d"
                                border.color: "#333740"

                                Column {
                                    anchors.centerIn: parent
                                    spacing: 10

                                    Label {
                                        text: root.documentState && root.documentState.currentModelFileName !== ""
                                              ? qsTr("Current model: ") + root.documentState.currentModelDisplayName
                                              : qsTr("Current scene uses the placeholder model")
                                        color: "#ffffff"
                                        font.pixelSize: 18
                                    }

                                    Label {
                                        text: root.documentState ? root.documentState.statusMessage : qsTr("Preparing assets...")
                                        color: "#8e949e"
                                        font.pixelSize: 12
                                        horizontalAlignment: Text.AlignHCenter
                                    }
                                }
                            }
                        }

                        ColumnLayout {
                            spacing: 14

                            Column {
                                spacing: 4
                                Label {
                                    text: qsTr("MODELS")
                                    color: "#069df3"
                                    font.pixelSize: 15
                                    font.bold: true
                                    font.letterSpacing: 0.5
                                }
                                Row {
                                    Rectangle { width: 66; height: 2; color: "#069df3" }
                                    Rectangle { width: 620; height: 1; color: "#303030"; anchors.bottom: parent.bottom }
                                }
                            }

                            GridView {
                                id: modelGrid
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                Layout.topMargin: 8
                                cellWidth: width >= 720 ? 230 : 210
                                cellHeight: 180
                                clip: true
                                model: root.documentState ? root.documentState.availableModels : []

                                delegate: Item {
                                    id: modelCard
                                    required property var modelData

                                    width: GridView.view.cellWidth
                                    height: GridView.view.cellHeight

                                    Column {
                                        anchors.centerIn: parent
                                        spacing: 12

                                        Rectangle {
                                            width: 196
                                            height: 110
                                            radius: 8
                                            border.color: modelCard.modelData.selected ? "#ffffff" : "transparent"
                                            border.width: modelCard.modelData.selected ? 2 : 0

                                            gradient: Gradient {
                                                GradientStop { position: 0.0; color: "#f8f9fb" }
                                                GradientStop { position: 1.0; color: "#dbe0e6" }
                                            }

                                            Rectangle {
                                                id: maskRect
                                                anchors.fill: parent
                                                anchors.margins: parent.border.width
                                                radius: Math.max(1, 8 - parent.border.width)
                                                visible: false
                                            }

                                            Image {
                                                id: previewImg
                                                anchors.fill: parent
                                                anchors.margins: parent.border.width
                                                source: "../assets/preview.png"
                                                fillMode: Image.PreserveAspectCrop
                                                sourceSize.width: 196
                                                sourceSize.height: 110
                                                smooth: true
                                                antialiasing: true
                                                visible: false
                                            }

                                            OpacityMask {
                                                anchors.fill: previewImg
                                                source: previewImg
                                                maskSource: maskRect
                                            }

                                            Rectangle {
                                                anchors.left: parent.left
                                                anchors.right: parent.right
                                                anchors.bottom: parent.bottom
                                                anchors.margins: parent.border.width
                                                height: 24
                                                color: "#99000000"
                                                visible: modelCard.modelData.selected || cardMouseArea.containsMouse
                                                bottomLeftRadius: Math.max(1, 6 - parent.border.width)
                                                bottomRightRadius: Math.max(1, 6 - parent.border.width)

                                                Label {
                                                    anchors.centerIn: parent
                                                    text: modelCard.modelData.selected ? qsTr("Loaded") : qsTr("Click to load")
                                                    color: "#ffffff"
                                                    font.pixelSize: 10
                                                    font.bold: true
                                                }
                                            }
                                        }

                                        Label {
                                            text: modelCard.modelData.displayName.toUpperCase()
                                            color: "#d0d2d6"
                                            font.pixelSize: 11
                                            font.bold: true
                                            elide: Text.ElideRight
                                            horizontalAlignment: Text.AlignHCenter
                                            width: 196
                                        }
                                    }

                                    MouseArea {
                                        id: cardMouseArea
                                        anchors.fill: parent
                                        hoverEnabled: true
                                        cursorShape: Qt.PointingHandCursor
                                        onClicked: {
                                            root.documentState.selectModel(modelCard.modelData.fileName)
                                            launcherPopup.close()
                                        }
                                    }
                                }

                                ScrollBar.vertical: ScrollBar {
                                    policy: ScrollBar.AsNeeded
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    footer: Rectangle {
        height: 24
        color: "#007acc"

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 12
            anchors.rightMargin: 12

            Label {
                text: qsTr("PySide6 + Qt Quick 3D")
                color: "#ffffff"
                font.pixelSize: 11
            }

            Item { Layout.fillWidth: true }

            Label {
                text: root.hasDocumentData && root.documentState.sourceModelPath !== ""
                      ? qsTr("Source: ") + root.documentState.currentModelDisplayName
                      : qsTr("No source model")
                color: "#ffffff"
                font.pixelSize: 11
            }
        }
    }

    SplitView {
        anchors.fill: parent
        orientation: Qt.Horizontal

        handle: Rectangle {
            implicitWidth: 4
            color: "#1e1e1e"
        }

        SplitView {
            SplitView.fillWidth: true
            orientation: Qt.Vertical

            handle: Rectangle {
                implicitHeight: 4
                color: "#1e1e1e"
            }

            Viewport.ViewportPanel {
                SplitView.fillHeight: true
                documentViewModel: root.documentState
            }

            Timeline.TimelinePanel { }
        }

        Rectangle {
            SplitView.preferredWidth: 330
            SplitView.minimumWidth: 280
            color: "#252526"

            ColumnLayout {
                anchors.fill: parent
                spacing: 0

                TabBar {
                    id: panelTabs
                    Layout.fillWidth: true
                    background: Rectangle { color: "#2d2d30" }

                    Components.StyledTabButton { text: qsTr("Outliner") }
                    Components.StyledTabButton { text: qsTr("Properties") }
                    Components.StyledTabButton { text: qsTr("Details") }
                }

                StackLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    currentIndex: panelTabs.currentIndex

                    Outliner.OutlinerPanel { }
                    Inspector.PropertiesPanel { documentViewModel: root.documentState }
                    Inspector.DetailsPanel { }
                }
            }
        }
    }
}
