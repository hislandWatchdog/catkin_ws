This repository includes all of the code that is running on the jetson nano except for webpage
server code. There are a couple of sub repositories added that were forked and then used as
submodules, for example rplidar.

Ater you make a "git pull" to the main repository you have to make sure that the submodules 
are also updated. For this in the main repository run the following command:
git submodule update --init --recursive

There are a couple things you have to download for some of the packages:

robot\_gui\_bridge: sudo apt-get install ros-melodic-rosbridge-server

my\_camera: sudo apt-get install ros-melodic-libuvc-camera

keyboard\_controller: when compiling the opencv might not detect your opencv instalation,
			you have to tell it the location of a specific file from your opencv
			instalation. First locate the "OpenCVConfig.cmake" file and then add
			the following command in the CMakeLists.txt file
			set(OpenCV_DIR /path/to/previous/file)

			if you don't know where it is installed you can try the following:
			find / -name "OpenCVConfig.cmake"

			Since you will be modifying this file for every user you must untrack
			the CMakeLists file with the following git command:
			git update-index --skip-worktree keyboard_controller/CMakeLists.txt

navigation: this submodule doesn't have a master branch instead 'git checkout melodic-devel'

robot\_pose\_publisher: this submodule does have a master but default is 'develop'




