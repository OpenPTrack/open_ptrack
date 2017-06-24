import QtQuick 2.0
import QtQuick.Window 2.1
import OPT_Streaming_Component 1.0
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1
import Qt.labs.folderlistmodel 2.1
Window {
    id: main_window
    //visibility: Window.FullScreen
    width: Screen.width
    height: Screen.height
    title: "MOT"
    visible: true

    RowLayout{//Main row, 3 items: Multi-Streaming_listview; main_view; rois_view
        spacing: 10

        // Multi-Streaming_listview
        ListView {
            spacing: 20
            width: (Screen.height-40)*16/54
            height: (Screen.height-40)
            model: topic_model// this model comes from c++ in :"main_application.cpp"
            delegate: Streaming_listview_delegate{}
        }

        //Main_view
        ColumnLayout {//main_view
            spacing: 50
            Image {//logo
                id: openptrack_logo
                source: "/images/openptrack-logo.png"
                width: 244
                height: 116
                anchors.horizontalCenter: parent.horizontalCenter
                fillMode: Image.PreserveAspectFit
            }

            //main_view  where to select rois
            Rectangle {
                width: 960
                height:540
                color: "#c92c2c"
                focus: true
                id: main_view
                property int no_roi : 0
                property int rect_X
                property int rect_Y
                property int rect_width
                property int rect_height
                OptStreamingComponent {//main stream
                    id:main_videoStream
                    anchors.fill: parent
                    Component.onCompleted: main_videoStream.setup()
                }
                Text {//display the text of the topic name
                    id: main_topic
                    anchors.left: parent.left
                    anchors.top: parent.top
                    z:1
                    color: "lime"
                    text: main_videoStream.topic
                    font.pixelSize: 20
                    font.bold: true
                    font.italic: true
                }
                MouseArea {// area to draw the bounding boxes
                    id: selectArea;
                    property bool ready2add : false
                    z:0
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    onPressed: {
                        selectArea.focus=true
                        if (highlightItem !== null)
                        {
                            // if there is already a selection, delete it
                            highlightItem.destroy ();
                        }
                        // create a new rectangle at the wanted position
                        highlightItem = roiComponent.createObject (selectArea, {"x" : mouse.x, "y" : mouse.y ,"width" : 0, "height" :0});
                    }
                    onPositionChanged: {
                        highlightItem.width = (Math.abs (mouse.x - highlightItem.x));
                        highlightItem.height = (Math.abs (mouse.y - highlightItem.y));
                    }
                    onReleased: {
                        main_view.rect_X=highlightItem.x;
                        main_view.rect_Y=highlightItem.y;
                        main_view.rect_width=highlightItem.width;
                        main_view.rect_height=highlightItem.height;
                        ready2add=true
                    }
                    property Rectangle highlightItem : null;
                    Component {
                        id: roiComponent;
                        Rectangle {
                            id:roi_rec
                            border.color: "red"
                            border.width: 2
                            color: "transparent"
                            x: selectArea.x
                            y: selectArea.y
                            width: selectArea.width
                            height:selectArea.height
                        }
                    }
                }
                Keys.onSpacePressed:{//press space to confirm your BB, save the rois as images , add them to the model
                    if(selectArea.ready2add)
                    {
                        var topic_name_list = main_topic.text.split('/');
                        var sensor_name=topic_name_list[1];
                        var roi_name=sensor_name+"_roi_"+no_roi.toString();
                        var roiSourceUrl=main_videoStream.all_rois_dir_path+sensor_name+"_roi_"+no_roi.toString()+".png"
                        main_videoStream.save_ROI(sensor_name,no_roi,rect_X,rect_Y,rect_width,rect_height);//save the roi as image
                        roiModel.append({"roiNo": no_roi ,"sensorName":sensor_name, "roiName": roi_name,"rectX":rect_X,"rectY":rect_Y,"rectWidth":rect_width,"rectHeight":rect_height,"roiSource": roiSourceUrl ,"showConfirmbutton":true, "published":false});
                        roi_visual_Model.append({"roiNo": no_roi ,"sensorName":sensor_name, "roiName": roi_name,"rectX":rect_X,"rectY":rect_Y,"rectWidth":rect_width,"rectHeight":rect_height,"roiSource": roiSourceUrl ,"showConfirmbutton":true, "published":false});
                        selectArea.highlightItem.destroy ();
                        no_roi++;
                    }
                    selectArea.ready2add=false
                }
            }
        }

        //Rois_view
        Rectangle{
            id: show_rois
            width:Screen.width-(Screen.height-40)*16/54-980
            height: Screen.height
            RowLayout{//One Row who has two items:: show roi_from_gui; shwo roi_form_file
                anchors.fill: parent

                //show roi_from_gui
                ListView {
                    id: roiblock
                    width: parent.width/2
                    height: parent.height
                    spacing: 20
                    clip: true
                    model: roi_visual_Model
                    delegate: Rois_listview_delegate{}
                    header: header_roiblock
                    footer: footer_roiblock
                    ListModel{
                        id: roiModel//list model for all the rois in all the cameras
                    }
                    ListModel{
                        id: roi_visual_Model// list model for the rois of the current camera
                    }

                    Component {//header of the list view: Text and Button
                        id: header_roiblock
                        Rectangle
                        {
                            width: parent.width/2
                            height: 50
                            ColumnLayout{
                                anchors.fill: parent
                                spacing: 10
                                Text {
                                    id: header_roiblock_text
                                    width: parent.width
                                    height: 20
                                    color: "blue"
                                    font.pixelSize: 20
                                    text: "ROIs_from_ <b><i>marking</i></b>"
                                }
                                Button{
                                    id: publish_rois_from_gui
                                    anchors.top:header_roiblock_text.bottom
                                    height: 20
                                    width: parent.width
                                    text: qsTr("Publish_rois_from_gui")
                                    onClicked:
                                    {// add all the rois in the roi_visual_Model to the msg ,and publish the msg(all in c++)
                                        for( var i = 0; i< roi_visual_Model.count; i++ )
                                        {
                                            if(roi_visual_Model.get(i).published===false)
                                            {
                                                main_videoStream.add_roi(roi_visual_Model.get(i).roiNo,roi_visual_Model.get(i).roiName, roi_visual_Model.get(i).rectX,roi_visual_Model.get(i).rectY, roi_visual_Model.get(i).rectWidth, roi_visual_Model.get(i).rectHeight);//add roi to the msg
                                                roi_visual_Model.setProperty(i, "published", true)//set flag in the model
                                                var no_ROI_=roi_visual_Model.get(i).roiNo;
                                                roiModel.setProperty(no_ROI_, "published", true)//set flag in the model
                                            }
                                        }
                                        main_videoStream.publish_rois_from_gui();//publish the msg
                                    }
                                }
                            }
                        }
                    }
                    Component {//footer of the list view:empty rectangle
                        id: footer_roiblock
                        Rectangle
                        {
                            width: parent.width/2
                            height: 50
                        }
                    }
                }



                //show roi_form_file
                ListView {
                    id:roi_folder
                    width: parent.width/2
                    height: parent.height
                    spacing: 20
                    model: roi_model_file
                    delegate: roifileDelegate
                    header: roi_folder_header

                    FolderListModel {//model from folder file list
                        id:roi_model_file
                        nameFilters: ["*.png"]
                        folder: main_videoStream.rois_dir_path
                    }
                    Component {////header of the list view: Text and Button
                        id: roi_folder_header
                        Rectangle
                        {
                            width: parent.width/2
                            height: 50
                            ColumnLayout{
                                anchors.fill: parent
                                spacing: 10
                                Text {
                                    id: header_roiblock_text2
                                    height: 20
                                    text: "ROIs_from_ <b><i>folder</i></b>"
                                    color: "green"
                                    font.pixelSize: 20
                                }
                                Button{
                                    id: publish_rois_from_file
                                    anchors.top:header_roiblock_text2.bottom
                                    height: 20
                                    width: parent.width
                                    text: qsTr("publish_rois_from_file")
                                    onClicked:
                                    {
                                        main_videoStream.publish_rois_from_file();// call the c++ fuction
                                    }
                                }
                            }
                        }
                    }

                    Component {//delegate of the list view:show the image and text(roi file name)
                        id: roifileDelegate
                        Rectangle {
                            width: show_rois.width/3
                            height:show_rois.width/3+20
                            Image {
                                id:roi_folder_image
                                anchors.top: parent.top
                                width: show_rois.width/3
                                height:show_rois.width/3
                                z:1
                                fillMode: Image.Stretch
                                source: roi_model_file.folder+fileName
                            }
                            Text {
                                id:roi_name
                                anchors.top: roi_folder_image.bottom
                                anchors.left: roi_folder_image.left
                                height: 20
                                width: show_rois.width/3
                                color: "lime"
                                text:fileName.substring(0,fileName.indexOf('.'))
                                font.pixelSize: 16
                            }
                        }
                    }
                }
            }
        }
    }
}

