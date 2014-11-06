/**	PTRACK CONNECTION ********************************************************\

	This code fragment can be incorporated within a NodeJS-based server
	instance. It reads from a UDP broadcast address and then writes to
	a TCP socket. A client browser using a websocket connection can then
	receive data as it arrives.

	This code will likely not run stand-alone, but is provided as a template
	for adding it to your own server code as a module.

	HOW UDP WORKS

	Multicast is one computer sending a UDP package to a special IP address,
	in the range 224.0.0.1 through 239.255.255.255. These "groups" are 
	assigned by IANA:
	http://www.iana.org/assignments/multicast-addresses/multicast-addresses.xhtml

	The magic of multicast is that the network hardware propagates packets
	across routers, and anyone can listen. To listen, you specify a host and a
	group address. If you want to listen to all UDP traffic, don't specify a
	host, and just use the group address.

	GENERAL APPROACH

	1. create socket for udp4
	2. enable multicast reception 
	3. set multicast TTL (not necessary I suspect for a client)
	4. join a channel/group (224.0.0.1). Add originating host if necessary
	5. bind 'message' event to handler to receive packets

	From what I understand:

	The UDP source has a host_address and a port. Together this identifies
	the source, which then puts all its traffic on a multicast_group_address.
	A listener would bind to the port, and instead of selecting host_address
	it would bind to the multicast_group_address (done by adding membership).
	If the multicast_address is a regular one, then one must also designate 
	the host_address to receive packets from it even if a member of the
	multicast_group_address.

	references:
	http://stackoverflow.com/questions/14130560/nodejs-udp-multicast-how-to

	developed as part of REMAP/STEP
	- report bugs to david@davidseah.com or ben@inquirium.net


///////////////////////////////////////////////////////////////////////////////
/** MODULE-WIDE VARIABLES ****************************************************/

//	required node modules
	var dgram = require('dgram');
	var WebSocketServer = require('ws').Server;

// 	udp socket connection
	var udp_socket;

// 	web socket connection
	var wss;					// web socket server
	var web_socket;				// web socket

// 	connection parameters for multicast (UDP)
	var mc = {
		port: 	21234,			// ptrack port
		group: 	'224.0.0.1',	// standard LAN multicast address
		host: 	undefined 		// undefined so node listens to all hosts
	}


///////////////////////////////////////////////////////////////////////////////
/** MODULE FUNCTIONS *********************************************************/

///	- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
/*/	Main exported function to effect UDP listening and TCP forwarding on
	websocket.
/*/	function API_ConnectTracker () {

	/**	1. create UDP listener to remote camera tracker **********************/

		udp_socket = dgram.createSocket('udp4');

		// connect to UDP group/host (host is optional)
		udp_socket.bind ( mc.port, mc.host, function () {
			var address = udp_socket.address();
			console.log("*** LISTENING TO "+address.address+" PORT "+address.port+" ("+address.family+") ***")
			// enable receiving multicast packets
			udp_socket.setMulticastLoopback ( true); 		
			// join multicast group
			udp_socket.addMembership( mc.group, mc.host ); 
		});

		// set up event handler for receving UDP data

		// NOTE: 
	 	// msg is a buffer object: http://nodejs.org/api/buffer.html
		// The conversion below is a little hacky, but works to
		// extract the JSON-encoded string.
		// msg.toString() grabs all the garbage after the end of the json
		// msg.toJSON() doesn't work because it grabs each byte as a number
		udp_socket.once('message',function( msg, rinfo ) {
			// convert msg to string, hack-style!
			var s = msg.toString();		
			s = s.substr(0,s.indexOf(']}')+2);
			// if TCP socket is available, forward string
			if (web_socket) {
		    	web_socket.send(s);
		    }
		});	

	/**	2. create TCP server on port 3030 for browser-based web app **********/

		// create a websocket server to listen to...
		// NOTE: TCP Port 3030 is an arbitrary port.
		wss = new WebSocketServer( { port: 3030 } );

		// connect to TCP port
		wss.on('connection', function ( wsocket ) {
			console.log("*** 3030 Browser Client Connect");
			web_socket = wsocket;
			web_socket.once('close',function() {
				console.log('remote socket BROWSER 3030 closed');
				web_socket = null;
			});
		});

		// report errors
		wss.on('error', function (err) {
			console.log("connectTracker socket server error: "+err);
			web_socket = null;
		});

	} // API_ConnectTracker

///	- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
/*/	Brute-force close all open sockets. This might not be the right way to
	do this.
/*/	function API_CloseTracker () {
		if (web_socket) web_socket.close();
		if (wss) wss.close();
		if (udp_socket) udp_socket.close();
		console.log("\n*** closing connections\n");

	} // API_ConnectTracker
	

///////////////////////////////////////////////////////////////////////////////
/** EXPORT MODULE API ********************************************************/

	exports.connectTracker = API_ConnectTracker;
	exports.closeTracker = API_CloseTracker;

	

