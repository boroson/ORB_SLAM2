//
// Created by lboroson on 1/25/19.
//

#include "SingleRobot.h"

// main function

namespace ORB_SLAM2
{

int main(int argc, char **argv)
{

    for(i = 0; i < nRobots; ++i)
    {
        // Initialize SLAM system for each robot
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        SingleRobot(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    }



    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    for (i = 0; i < nRobots; ++i) {
        // Kick off SLAM processing, to run asynchronously
        SingleRobot::RunMapping();

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl << endl;

        // Main loop
        cv::Mat imLeft, imRight;
        for (int ni = 0; ni < nImages; ni++) {
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
            imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];

            if (imLeft.empty()) {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[ni]) << endl;
                return 1;
            }

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft, imRight, tframe);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

            double ttrack = std::chrono::duration_cast < std::chrono::duration < double > > (t2 - t1).count();

            vTimesTrack[ni] = ttrack;

            // Wait to load the next frame
            double T = 0;
            if (ni < nImages - 1)
                T = vTimestamps[ni + 1] - tframe;
            else if (ni > 0)
                T = tframe - vTimestamps[ni - 1];

            if (ttrack < T)
                usleep((T - ttrack) * 1e6);
        }
    }

    for (i = 0; i < nRobots; ++i) {
        // Stop all threads
        SLAM.Shutdown();
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

}
