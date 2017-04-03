import QtQuick 2.0
import QtQuick.Window 2.1
import OPT_Streaming_Component 1.0
import QtQuick.Controls 1.1
import QtQuick.Layouts 1.1

Window {
    id: main_window
    visibility: Window.FullScreen
    width: Screen.width
    height: Screen.height
    title: "MOT"
    visible: true

//    property string sensor_topic1:"/kinect2_head/rgb_lowres/image"
//    property string sensor_topic2:"/kinect2_far/rgb_lowres/image"
//    property string sensor_topic3:"/kinect2_lenovo/rgb_lowres/image"
//    property string sensor_topic4:"/kinect2_head/rgb_lowres/image"
//    property string sensor_topic5:"/kinect2_head/rgb_lowres/image"
//    property string sensor_topic6:"/kinect2_head/rgb_lowres/image"

    RowLayout{
        spacing: 10
        ColumnLayout {
            spacing: 10
            Rectangle{
                id:sensor_01
                property int number_rois : 0
                width: (Screen.height-40)*16/54
                height: (Screen.height-40)/6
                color: "#c92c2c"
                OptStreamingComponent {
                    id: videoStream1
                    anchors.fill: parent
                    objectName: "videoStream1"
//                    topic: topic1.text
                }
                TextInput {
                    id: topic1
                    anchors.left: parent.left
                    anchors.top: parent.top
                    color: "lime"
                    text: videoStream1.topic
                    font.pixelSize: 16
                    font.bold: true
                }
                MouseArea{
                    anchors.top: parent.top
                    anchors.topMargin: 20
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    onClicked:
                    {
                        main_videoStream.topic= videoStream1.topic
                        main_view.current_camera=1
                        main_view.no_roi=sensor_01.number_rois
                        roiblock.model=roiModel1
                        topic1.font.bold=true
                        topic2.font.bold=false
                        topic3.font.bold=false
                        topic4.font.bold=false
                        topic5.font.bold=false
                        topic6.font.bold=false
                    }
                }
            }
            Rectangle{
                id:sensor_02
                property int number_rois : 0
                width: sensor_01.width
                height: sensor_01.height
                color: "#c92c2c"
                OptStreamingComponent {
                    anchors.fill: parent
                    objectName: "videoStream2"
                    id: videoStream2
//                    topic: topic2.text
                }
                TextInput {
                    id: topic2
                    anchors.left: parent.left
                    anchors.top: parent.top
                    color: "lime"
                    text: videoStream2.topic
                    font.pixelSize: 16
                    font.bold: false
                }
                MouseArea{
                    anchors.top: parent.top
                    anchors.topMargin: 20
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    onClicked:
                    {
                        main_videoStream.topic= videoStream2.topic
                        main_view.current_camera=2
                        main_view.no_roi=sensor_02.number_rois
                        roiblock.model=roiModel2
                        topic1.font.bold=false
                        topic2.font.bold=true
                        topic3.font.bold=false
                        topic4.font.bold=false
                        topic5.font.bold=false
                        topic6.font.bold=false
                    }
                }
            }
            Rectangle{
                id:sensor_03
                property int number_rois : 0

                width: sensor_01.width
                height: sensor_01.height
                color: "#c92c2c"
                OptStreamingComponent {
                    anchors.fill: parent
                    objectName: "videoStream3"
                    id: videoStream3
//                    topic: topic3.text
                }
                TextInput {
                    id: topic3
                    anchors.left: parent.left
                    anchors.top: parent.top
                    color: "lime"
                    text: videoStream3.topic
                    font.pixelSize: 16
                    font.bold: false
                }
                MouseArea{
                    anchors.top: parent.top
                    anchors.topMargin: 20
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    onClicked:
                    {
                        main_videoStream.topic= videoStream3.topic
                        main_view.current_camera=3
                        main_view.no_roi=sensor_03.number_rois
                        roiblock.model=roiModel3
                        topic1.font.bold=false
                        topic2.font.bold=false
                        topic3.font.bold=true
                        topic4.font.bold=false
                        topic5.font.bold=false
                        topic6.font.bold=false

                    }
                }
            }
            Rectangle{
                id:sensor_04
                property int number_rois : 0
                width: sensor_01.width
                height: sensor_01.height
                color: "#c92c2c"
                OptStreamingComponent {
                    anchors.fill: parent
                    objectName: "videoStream4"
                    id: videoStream4
//                    topic: topic4.text
                }
                TextInput {
                    id: topic4
                    anchors.left: parent.left
                    anchors.top: parent.top
                    color: "lime"
                    text: videoStream4.topic
                    font.pixelSize: 16
                    font.bold: false
                }
                MouseArea{
                    anchors.top: parent.top
                    anchors.topMargin: 20
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    onClicked:
                    {
                        main_videoStream.topic= videoStream4.topic
                        main_view.current_camera=4
                        main_view.no_roi=sensor_04.number_rois
                        roiblock.model=roiModel4
                        topic1.font.bold=false
                        topic2.font.bold=false
                        topic3.font.bold=false
                        topic4.font.bold=true
                        topic5.font.bold=false
                        topic6.font.bold=false

                    }
                }
            }
            Rectangle{
                id:sensor_05
                property int number_rois : 0
                width: sensor_01.width
                height: sensor_01.height
                color: "#c92c2c"
                OptStreamingComponent {
                    anchors.fill: parent
                    objectName: "videoStream5"
                    id: videoStream5
//                    topic: topic5.text
                }
                TextInput {
                    id: topic5
                    anchors.left: parent.left
                    anchors.top: parent.top
                    color: "lime"
                    text: videoStream5.topic
                    font.pixelSize: 16
                    font.bold: false
                }
                MouseArea{
                    anchors.top: parent.top
                    anchors.topMargin: 20
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    onClicked:
                    {
                        main_videoStream.topic= videoStream5.topic
                        main_view.current_camera=5
                        main_view.no_roi=sensor_05.number_rois
                        roiblock.model=roiModel5
                        topic1.font.bold=false
                        topic2.font.bold=false
                        topic3.font.bold=false
                        topic4.font.bold=false
                        topic5.font.bold=true
                        topic6.font.bold=false

                    }
                }
            }
            Rectangle{
                id:sensor_06
                property int number_rois : 0
                width: sensor_01.width
                height: sensor_01.height
                color: "#c92c2c"
                OptStreamingComponent {
                    anchors.fill: parent
                    objectName: "videoStream6"
                    id: videoStream6
//                    topic: topic6.text
                }
                TextInput {
                    id: topic6
                    anchors.left: parent.left
                    anchors.top: parent.top
                    color: "lime"
                    text: videoStream6.topic
                    font.pixelSize: 16
                    font.bold: false
                }
                MouseArea{
                    anchors.top: parent.top
                    anchors.topMargin: 20
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    onClicked:
                    {
                        main_videoStream.topic= videoStream6.topic
                        main_view.current_camera=6
                        main_view.no_roi=sensor_06.number_rois
                        roiblock.model=roiModel6
                        topic1.font.bold=false
                        topic2.font.bold=false
                        topic3.font.bold=false
                        topic4.font.bold=false
                        topic5.font.bold=false
                        topic6.font.bold=true
                    }
                }
            }
        }
        ColumnLayout {
            spacing: 50
            Image {
                id: openptrack_logo
                source: "/images/openptrack-logo.png"
                width: 244
                height: 116
                anchors.horizontalCenter: parent.horizontalCenter
                fillMode: Image.PreserveAspectFit
            }
            Rectangle{
                width: 960
                height:540
                color: "#c92c2c"
                focus: true
                id: main_view
                property int current_camera : 1
                property int no_roi : 0
                property int rect_X
                property int rect_Y
                property int rect_width
                property int rect_height

                //  property string roi_name_s
                OptStreamingComponent {
                    id:main_videoStream
                    anchors.fill: parent
                    objectName: "main_videoStream"
//                    topic: main_topic.text
                }
                TextInput {
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
                 MouseArea {
                    id: selectArea;
                    property bool ready2add : false
                    z:0
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    onPressed: {
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
                Keys.onSpacePressed:{
                    if(selectArea.ready2add)
                    {
                        no_roi++
                        var topic_name_list = main_topic.text.split('/');
                        var sensor_name=topic_name_list[1];
                        var roi_name=sensor_name+"_roi_"+no_roi.toString();
                        var roiSourceUrl="file:/tmp/"+sensor_name+"_roi_"+no_roi.toString()+".png"

                        switch(current_camera)
                        {
                        case 1:
                            videoStream1.save_ROI(sensor_name,no_roi,roi_name,rect_X,rect_Y,rect_width,rect_height)
                            roiModel1.append({"roiNo": no_roi ,"roiName": roi_name, "roiSource": roiSourceUrl });
                            (sensor_01.number_rois)++;//use the the number of the models
                            selectArea.highlightItem.destroy ();
                            break;
                        case 2:
                            videoStream2.save_ROI(sensor_name,no_roi,roi_name,rect_X,rect_Y,rect_width,rect_height)
                            roiModel2.append({"roiNo": no_roi ,"roiName": roi_name, "roiSource": roiSourceUrl });
                            (sensor_02.number_rois)++;
                            selectArea.highlightItem.destroy ();
                            break;
                        case 3:
                            videoStream3.save_ROI(sensor_name,no_roi,roi_name,rect_X,rect_Y,rect_width,rect_height)
                            roiModel3.append({"roiNo": no_roi ,"roiName": roi_name, "roiSource": roiSourceUrl });
                            (sensor_03.number_rois)++;
                            selectArea.highlightItem.destroy ();
                            break;
                        case 4:
                            videoStream4.save_ROI(sensor_name,no_roi,roi_name,rect_X,rect_Y,rect_width,rect_height)
                            roiModel4.append({"roiNo": no_roi ,"roiName": roi_name, "roiSource": roiSourceUrl });
                            (sensor_04.number_rois)++;
                            selectArea.highlightItem.destroy ();
                            break;
                        case 5:
                            videoStream5.save_ROI(sensor_name,no_roi,roi_name,rect_X,rect_Y,rect_width,rect_height)
                            roiModel5.append({"roiNo": no_roi ,"roiName": roi_name, "roiSource": roiSourceUrl });
                            (sensor_05.number_rois)++;
                            selectArea.highlightItem.destroy ();
                            break;
                        case 6:
                            videoStream6.save_ROI(sensor_name,no_roi,roi_name,rect_X,rect_Y,rect_width,rect_height)
                            roiModel6.append({"roiNo": no_roi ,"roiName": roi_name, "roiSource": roiSourceUrl });
                            (sensor_06.number_rois)++;
                            selectArea.highlightItem.destroy ();
                            break;
                        }
                    }
                    selectArea.ready2add=false
                }
            }
        }
        Rectangle{
            id: show_rois
            width:Screen.width-(Screen.height-40)*16/54-980
            height: Screen.height
            ListView {
                id: roiblock
                anchors.top: parent.top
                width: parent.width
                height: parent.height-50
                spacing: 20
                clip: true
                model: roiModel1
                delegate: Rois_listview_delegate{}
                ListModel{
                    id: roiModel1
                }
                ListModel{
                    id: roiModel2
                }
                ListModel{
                    id: roiModel3
                }
                ListModel{
                    id: roiModel4
                }
                ListModel{
                    id: roiModel5
                }
                ListModel{
                    id: roiModel6
                }
            }
            Button{
                id: publish_rois
                anchors.top: roiblock.bottom
                anchors.topMargin: 20
                anchors.left: roiblock.left
                anchors.leftMargin: 40
                height: 20
                width: 100
                text: qsTr("Start Detection")

                onClicked:
                {
                    switch(main_view.current_camera)
                    {
                    case 1: videoStream1.publish_rois();
                        break;
                    case 2: videoStream2.publish_rois();
                        break;
                    case 3: videoStream3.publish_rois();
                        break;
                    case 4: videoStream4.publish_rois();
                        break;
                    case 5: videoStream5.publish_rois();
                        break;
                    case 6: videoStream6.publish_rois();
                        break;
                    }
                }
            }
        }
    }
}

