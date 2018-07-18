#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<System.h>


using namespace cv;

int main(int argc, char** argv) {
    VideoCapture cam;

    if(!cam.open(0))
        return 0;

    ORB_SLAM2::System SLAM("../config/ORBvoc.txt", "../config/webcam.yaml", ORB_SLAM2::System::MONOCULAR, true);

    while (true) {
        Mat frame;
        cam >> frame;

        if (frame.empty())
            break; 

        //imshow("Webcam", frame);

	#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

	double timestamp = time(NULL);

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame, timestamp);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

	usleep(ttrack*1e6);
    }
    
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}
