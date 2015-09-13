
# OpenPTrack Sender Simulator
# Sept 13, 2015
# jburke@ucla.edu

import socket, time, json, time, random

UDP_IP = "127.0.0.1"
UDP_PORT = 21234
PERIOD = .100        # how often to publish in time

# For the random walk
MAXSTEP_X = 10         
MAXSTEP_Y = 10
WOBBLE_Z = 1
Z_NOMINAL = 40

# Increasing packet seq number
_SEQ = 0 

# Current message format
# https://github.com/OpenPTrack/open_ptrack/wiki/Using%20The%20Data
#
#MESSAGE = '{"header":{"seq":336988,"stamp":{"sec":1441244414,"nsec":266356327},"frame_id":"world"},"tracks":[{"id":170,"x":0.740519,"y":-3.21577,"height":1.01898,"age":79.4518,"confidence":0.491777},{"id":172,"x":0.843167,"y":-3.29433,"height":1.10497,"age":29.471,"confidence":0.500193}]}'

def track( id, x, y, height, age, confidence ) :
    return {"id":id, "x":x, "y":y, "height":height, "age": age, "confidence":confidence}

def packet( tracks ) : 
    global _SEQ
    _SEQ+=1
    now = float(time.time())
    sec = int(now)
    nsec = int((now-sec) * 1e9)
    header = { "seq":_SEQ, "stamp": {"sec":sec, "nsec":nsec}, "frame_id":"world" }
    return { "header":header, "tracks":tracks } 

# Provide two random walkers 
# More is exercise for reader ... 

def walk(W):
    for w in W: 
        w[0] += MAXSTEP_X * 2*(random.random() - 0.5)
        w[1] += MAXSTEP_Y * 2*(random.random() - 0.5)
        w[2] = Z_NOMINAL + WOBBLE_Z*2*(random.random()-0.5)

walkers = [ [random.randrange(200)-100, random.randrange(200)-100, Z_NOMINAL], 
            [random.randrange(200)-100, random.randrange(200)-100, Z_NOMINAL] ]
            
print("^C to stop")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP   
try:
    while True:
        
        walk(walkers)
        MESSAGE = json.dumps( packet( [ track(42, walkers[0][0], walkers[0][1], walkers[0][2], _SEQ+100+random.random(), random.random()),
                                        track(43, walkers[1][0], walkers[1][1], walkers[1][2], _SEQ+100+random.random(), random.random())] )  )
    
        # We throw some zeroes at the end to simulate OpenPTrack's current zero padding,
        # so parsers make sure to handle it. This padding should be removed soon.
        # https://github.com/OpenPTrack/open_ptrack/issues/52
        payload = bytes(MESSAGE.encode('utf-8')) + bytes(bytearray(100))
        
        sock.sendto(payload, (UDP_IP, UDP_PORT))
        print(payload)
        time.sleep(PERIOD)
        
except KeyboardInterrupt:
    pass # do cleanup here
