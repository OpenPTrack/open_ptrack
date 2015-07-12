# Install Mesa SwissRanger driver:
cd ~/Downloads
wget -U "Mozilla" http://downloads.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.14-748.amd64.deb
sudo dpkg -i libmesasr-dev-1.0.14-748.amd64.deb 

# or for 32 bit - if get dpkg: error processing libmesasr-dev-1.0.14-748.amd64.deb (--install):
# package architecture (amd64) does not match system (i386)

#http://downloads.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.14-747.i386.deb
#sudo dpkg -i libmesasr-dev-1.0.14-747.i386.deb
