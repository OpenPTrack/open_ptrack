// Simple parser for OPT data in Max/MSP
// Sept 13, 2015
// jburke@ucla.edu
// Requires a non-zero padded string as input 
// Feed from the supplied net.udp.recvzeropad until this issue is fixed - https://github.com/OpenPTrack/open_ptrack/issues/52

// Output is a list:
//    id x y height age confidence seconds nanoseconds seq#
//    
// Note that the last three are per-packet, not per track, but we output with each track message  
// 

// We're just going to output one message per incoming track
outlets = 1

function anything() { 
	
	// concatenate message and arguments in case there are spaces in incoming JSON that caused net input object to split it
	var a = messagename;
	for (var i=0; i < arguments.length; i++) {
		a += arguments[i]; 
		}
		
	var obj=JSON.parse(a);

    // Only report incoming packets with tracks for now
    // Should provide a keep-alive?
    // 
	if (obj.hasOwnProperty("tracks")) {
 		for (var i=0; i < obj.tracks.length; i++) {
			outlet(0, [ obj.tracks[i].id, obj.tracks[i].x, obj.tracks[i].y, obj.tracks[i].height, obj.tracks[i].age, obj.tracks[i].confidence, obj.header.stamp.sec, obj.header.stamp.nsec, obj.header.seq ] );
		}
	}
}