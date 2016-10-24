#include <thread>
#include <iostream>
#include "frame.h"
#include "epipolar_error.h"
#include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;
using std::vector;

using vslam::GenericFisheyeCamera;
using vslam::AbstractFrame;
using vslam::AbstractCamera;


template <class CameraType>
int AbstractFrame<CameraType>::frame_counter_ = 0;

extern int QtDisplayThread();



int MainThread() {
    cout << "hello vslam" << endl;
	return 0;
}

int  main() {
	std::thread t1(MainThread);
    //std::thread t2(QtDisplayThread);

	t1.join();
    //t2.join();
	return 1;
}
