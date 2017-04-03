import QtQuick 2.0
import QtQuick.Controls 1.1

Component {
    id: imageDelegate
    Item {
        id: imagewraper
        width: show_rois.width/3
        height: show_rois.width/3+20

        Rectangle {
            id: image
            color: "black"
//            anchors.left: parent.left
            anchors.top: parent.top
            width: show_rois.width/3
            height:show_rois.width/3+20
            Image {
                id:image_
                anchors.top: parent.top
                width: show_rois.width/3
                height:show_rois.width/3
                z:1
                fillMode: Image.Stretch
                source: roiSource
            }
            TextInput {
                id:roi_name
//                anchors.left: parent.left
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
                z:1
                text: qsTr("Confirm")
                onClicked:
                {
                    roi_name.focus=false
                    main_view.focus=true
                    roiblock.model.get(index).roiName=roi_name.text
                    switch(main_view.current_camera)
                    {
                    case 1:
                        videoStream1.set_roi_name(index,roi_name.text)
                        confirm_button.visible=false
                        break;
                    case 2:
                        videoStream2.set_roi_name(index,roi_name.text)
                        confirm_button.visible=false
                        break;
                    case 3:
                        videoStream3.set_roi_name(index,roi_name.text)
                        confirm_button.visible=false
                        break;
                    case 4:
                        videoStream4.set_roi_name(index,roi_name.text)
                        confirm_button.visible=false
                        break;
                    case 5:
                        videoStream5.set_roi_name(index,roi_name.text)
                        confirm_button.visible=false
                        break;
                    case 6:
                        videoStream6.set_roi_name(index,roi_name.text)
                        confirm_button.visible=false
                        break;
                    }
                }
            }
        }

        MouseArea {
            id:imagearea
            z:0
            anchors.top: imagewraper.top
            anchors.bottom: imagewraper.bottom
            anchors.bottomMargin: 20
            anchors.left: imagewraper.left
            anchors.right: imagewraper.right

            //            anchors.fill: imagewraper
            //            hoverEnabled: true
            //            drag.target: imagewraper
            onClicked: {
                switch(imagewraper.state)
                {
                case "":imagewraper.state="expanded";break;
                case "expanded":imagewraper.state="";break;
                    //                case "doubleexpanded":imagewraper.state="doubleexpanded";break;
                default:imagewraper.state="";break;
                }
            }
            //            onDoubleClicked: {
            //                switch(imagewraper.state)
            //                {
            //                case "expanded":imagewraper.state="expanded";break;
            //                case "":imagewraper.state="doubleexpanded";break
            //                case "doubleexpanded":imagewraper.state="expanded";break;
            //                default:imagewraper.state="expanded";break;
            //                }
            //            }
        }
        states: [
            State {
                name: "expanded"
                PropertyChanges { target: imagewraper; height: roiblock.height/2 ;width:roiblock.width;z:100}
                PropertyChanges { target: image_; width: roiblock.width; height: roiblock.height/2}
                PropertyChanges { target: imagewraper.ListView.view; contentX: imagewraper.x;contentY: imagewraper.y; interactive: true }
            }
            //            ,State {
            //                name: "doubleexpanded"
            //                PropertyChanges { target: imagewraper; height: roiblock.height*2 ;width:imageblock.width*2;z:100}
            //                PropertyChanges { target: image; width: imageblock.width*2; height: imageblock.height*2}
            //                PropertyChanges { target: imagewraper.GridView.view; contentX: imagewraper.x;contentY: imagewraper.y.mouseY; interactive: true }
            //            }
        ]


        transitions: [
            Transition {
                NumberAnimation {
                    duration: 100;
                    properties: "height,width,anchors.rightMargin,anchors.topMargin,opacity,contentX,contentY"
                }
            }
        ]
    }
}
