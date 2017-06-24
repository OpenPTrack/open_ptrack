import QtQuick 2.0
import QtQuick.Controls 1.1
import OPT_Streaming_Component 1.0
import QtQuick.Window 2.1

Component{
    id:steaming_delegate
    Rectangle{
        width: (Screen.height-40)*16/54
        height: (Screen.height-40)/6
        color: "#c92c2c"
        OptStreamingComponent {//main component from c++
            id:opt_streaming
            anchors.fill: parent
            topic:modelData
        }
        Text {
            id: topic_text
            anchors.left: parent.left
            anchors.top: parent.top
            color: "lime"
            text: opt_streaming.topic
            font.pixelSize: 16
            font.bold: true
        }
        MouseArea{//click on the area to select this camera view as the main camera view in the "main_view"
            anchors.top: parent.top
            anchors.topMargin: 20
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            onClicked:
            {
                // set the camera topic in the "main_view",this change will activete the "set_topic" function in c++,
                // and modify the subscriber to subscribe another camera topic ,at the same time ,the advertiser of roi will
                // also be changed with the new sensor name from the new topic
                main_videoStream.topic= opt_streaming.topic;

                var current_topic_name_list = main_videoStream.topic.split('/');
                var current_sensor_name=current_topic_name_list[1];


                //clear the roi_visual_Model and add the rois from "roiModel" to roi_visual_Model with the current sensor name
                roi_visual_Model.clear();
                for( var i = 0; i< roiModel.count; i++ )
                {
                    if (roiModel.get(i).sensorName===current_sensor_name)
                        roi_visual_Model.append(roiModel.get(i))
                }

                // set the rois_dir_path in the main_videoStream with the sensor name
                main_videoStream.rois_dir_path=main_videoStream.all_rois_dir_path+current_sensor_name+"/"
            }
        }
        Component.onCompleted: opt_streaming.setup()// set up the subscriber to subscribe camera topic ,at the same time ,the advertiser of roi will
        // also be setted with the  sensor name from the topic
    }
}
