
# OpenPTrack and Max/MSP

These files provide an initial example of using OpenPTrack data with Cycling 74's Max/MSP.  It was tested in Max 7. 

Note that the recvzeropad Java object is only needed until this issue is fixed: https://github.com/OpenPTrack/open_ptrack/issues/52
After that, the existing net.udp.recv object should work. 

*Instructions*: 

1. Clone or unpack all the files in this folder. 
2. Copy or move recvzeropad.java into the Max Java classpath under the net.udp package.  The easiest location for this on the Mac is the existing  /Applications/Max.app/Contents/Resources/C74/packages/max-mxj/java-classes/classes/net/udp 
3. Open the example (OpenPTrackExample.maxpat) and click the viewsource message.  Choose "Java...Open Compile Window" from the editor, then click the compile button. Check that there are no errors. 
4. Start OpenPTrack on the same subnet as your machine (or with unicast pointed to your IP) or use a data simulator to test. 

- jburke@ucla.edu, Sept 13, 2015


