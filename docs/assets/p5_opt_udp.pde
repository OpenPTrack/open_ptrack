
// import UDP library
import hypermedia.net.*;


UDP udp;  // define the UDP object
JSONObject json;
float _x;
float _y;
float _id;
float _xOffset;
float _yOffset;
float _scaleFactor = 10;
/**
 * init
 */
void setup() {

  // create a new datagram connection on default OPT port 
  // (can be changed in ~open_ptrack/opt_utils/conf/udp.yaml)
  udp = new UDP( this, 21234 );
  
  // and wait for incomming message
  udp.listen( true );
  
  // initalize vars
  _x = 0;
  _y = 0;
  _id = 0;
  _xOffset = 10;
  _yOffset = 10;
}

//process events
void draw() {

   // we'll just draw in our json loop for example... see below
}


 void receive( byte[] data ) { 			
  
  // get the "real" message / forget the "\n" at the end 
  data = subset(data, 0, data.length-2);
  String message = new String( data );
  //println(message);
  json = JSONObject.parse(message);
  
  JSONArray opt = json.getJSONArray("tracks");

  try {
     int i;
     // for each track 
     for(i = 0; i < opt.size(); i++){
       
       JSONObject track = opt.getJSONObject(i); 
       //println("track "+i+" : "+track.getFloat("x")+", "+"+track.getFloat("y"));

       _x = track.getFloat("x");
       _y = track.getFloat("y");
       _id =track.getFloat("id");
       
       stroke(255/_id);  // change plot color based on id, never exceed 255
       
       // plot x,y on canvas
       //line(_x, _y, _x, _y); // this works, but is very small, thus:
       // scalefactor and _Offsets allow for displaying OPT data on processing canvas / avoids plotting off-screen 
       // (opt values can be negative, depending on final calibration step)
       line(_x*_scaleFactor + _xOffset, _y*_scaleFactor+ _yOffset, _x*_scaleFactor+ _xOffset, _y*_scaleFactor+ _yOffset);

     }
  } catch (Exception e) {
    println("no tracks");
  } 
}

