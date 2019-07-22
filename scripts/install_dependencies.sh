#!/bin/bash



mkdir build
cd build

install_git_repo () {
	if [ -d "./$1" ] 
	then
		echo "Library $1 already installed" 
	else
		git clone $2
		cd $1
		if [ -z "$3" ]   # Is parameter #1 zero length?
		then
			git checkout "$3"
		fi
		
		mkdir build ; cd build
		cmake ..
		make -j4
		sudo make install 
		cd ../..
	fi
}

###################################################################
###########		INSTALL TINYXML2		###########
###################################################################
install_git_repo "tinyxml2" "https://github.com/leethomason/tinyxml2"

###################################################################
###########		INSTALL SERIAL			###########
###################################################################
install_git_repo "serial" "https://github.com/wjwwood/serial"

###################################################################
###########		INSTALL rapidjson		###########
###################################################################
install_git_repo "rapidjson" "https://github.com/Tencent/rapidjson"

###################################################################
###########		INSTALL collada			###########
###################################################################
install_git_repo "collada-dom" "https://github.com/rdiankov/collada-dom" "v2.5.0"

###################################################################
###########		INSTALL OpenSceneGraph		###########
###################################################################
install_git_repo "OpenSceneGraph" "https://github.com/openscenegraph/OpenSceneGraph" "OpenSceneGraph-3.4"

###################################################################
###########		INSTALL OPENRAVE		###########
###################################################################
sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy python-sympy qt4-dev-tools libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev libccd-dev

pip install sympy==0.7.1.0 numpy numpy-quaternion numba

git clone https://github.com/rdiankov/openrave.git
cd openrave
git checkout 2befaa1b842320633a2a93046ebd456051f16bdc
mkdir build ; cd build
cmake ..
make -j4
sudo make install




