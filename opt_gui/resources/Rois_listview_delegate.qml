import QtQuick 2.0
import QtQuick.Controls 1.1

Component {
    id: imageDelegate
    Item {
        id: imagewraper
        width: show_rois.width/3
        height: show_rois.width/3+20
        Rectangle {//main rect
            id: image
            color: "black"
            anchors.top: parent.top
            width: show_rois.width/3
            height:show_rois.width/3+20
            Image {//image display
                id:image_
                anchors.top: parent.top
                width: show_rois.width/3
                height:show_rois.width/3
                cache : false
                fillMode: Image.Stretch
                source: roiSource
                Image {//icon for remove the current roi
                    id: delete_red
                    z:2
                    visible: showConfirmbutton
                    anchors.top:parent.top
                    anchors.right: parent.right
                    source: "/images/DeleteRed.png"
                    width: 30
                    height: 30
                    fillMode: Image.PreserveAspectFit
                    MouseArea{
                        anchors.fill: parent
                        onClicked:
                        {
                            var roi_no_=roi_visual_Model.get(index).roiNo
                            roi_visual_Model.remove(index)
                            roiModel.remove(roi_no_)
                            main_view.no_roi--
                        }
                    }
                }
            }
            TextInput {
                id:roi_name
                anchors.top: image_.bottom
                anchors.left: image_.left
                height: 20
                width: show_rois.width/3-50
                color: "lime"
                text:roiName
                font.pixelSize: 16

            }
            Button{
                id:confirm_button
                anchors.top: image_.bottom
                anchors.right: image_.right
                height: 20
                width: 50
                visible: showConfirmbutton
                z:1
                text: "confirm"
                property int roi_no_
                onClicked://add the current roi to the models
                {
                    roi_name.focus=false
                    main_view.focus=true
                    roi_visual_Model.setProperty(index, "roiName", roi_name.text)
                    roi_visual_Model.setProperty(index, "showConfirmbutton", false)
                    roi_no_=roi_visual_Model.get(index).roiNo
                    roiModel.setProperty(roi_no_, "showConfirmbutton", false)
                    roiModel.setProperty(roi_no_, "roiName", roi_name.text)
                    confirm_button.visible=false
                    delete_red.visible=false
                }
            }
        }
/*
        MouseArea {//strech the image
            id:imagearea
            z:0
            anchors.top: imagewraper.top
            anchors.bottom: imagewraper.bottom
            anchors.bottomMargin: 20
            anchors.left: imagewraper.left
            anchors.right: imagewraper.right
            onClicked: {
                switch(imagewraper.state)
                {
                case "":imagewraper.state="expanded";break;
                case "expanded":imagewraper.state="";break;
                default:imagewraper.state="";break;
                }
            }
        }
        states: [
            State {
                name: "expanded"
                PropertyChanges { target: imagewraper; height: roiblock.height/2 ;width:roiblock.width;z:100}
                PropertyChanges { target: image_; width: roiblock.width; height: roiblock.height/2}
                PropertyChanges { target: imagewraper.ListView.view; contentX: imagewraper.x;contentY: imagewraper.y; interactive: true }
            }
        ]


        transitions: [
            Transition {
                NumberAnimation {
                    duration: 100;
                    properties: "height,width,anchors.rightMargin,anchors.topMargin,opacity,contentX,contentY"
                }
            }
        ]
    */

    }
}
