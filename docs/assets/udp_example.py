import socket
import json
import time

TRACKING_PORT = 21234
BIND_IP = ''
PACKET_SIZE = 2048

def setupUDP():
	print("Setting up UDP")
	global sock 
	global transform
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  #Added by damon attempting to overcome port in use issue
	sock.bind((BIND_IP,TRACKING_PORT))
	sock.setblocking(0)
	#sock.settimeout(0.0)


def closeUDP():
	global sock
	if(sock):
		print("closing socket")
		if(sock) : sock.close()
		sock = None
		print(sock)

def receiveOnce():
	global sock
	if(sock == None):
		# not sure how to get touch to call start so I'm checking here
		# and calling start if needed (lame!)
		setupUDP()
	try:

		#loop on the receive to ensure the last packet arrived is used
		#loops = 0
		data = ''
		while True:
			try:
				data, address = sock.recvfrom(PACKET_SIZE)
				#print(data)
				#loops += 1
			except:
				break

		#data, address = sock.recvfrom(PACKET_SIZE)

		if (data != ''):
			#print(loops)
			msg = str(data)

		#	print(msg)
			end = msg.find("]}") + 2
			start = msg.find('{"header')
	#		print(start)

			trackingData = json.loads(msg[start:end])
			tracks = trackingData['tracks']
			trackData = []
			for track in tracks:
				trackData.append([track['id'], track['x'], track['y'], track['height']])
			return trackData
		else:
			#print ("no data")
			return None
	except socket.error:
		return None

def start():
	pass

def create():
	return

def exit():
	return