//
// Created by lboroson on 1/25/19.
//

#include "SingleRobot.h"
#include "../include/System.h"

namespace ORB_SLAM2 {

    // Initialize the robot: set up ORB-SLAM, loop closure, landmark DB
    SingleRobot::SingleRobot(const string &strImage, const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                             const bool bUseViewer = true)
    {
        // Load data
        // Retrieve paths to images
        vector<string> vstrImageLeft;
        vector<string> vstrImageRight;
        vector<double> vTimestamps;
        LoadImages(strImage, vstrImageLeft, vstrImageRight, vTimestamps);

        const int nImages = vstrImageLeft.size();

        // Initialize ORB-SLAM system (including VO and loop closure)
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        System SLAM(strVocFile,strSettingsFile,ORB_SLAM2::System::STEREO,false);

        //TODO: Set up local part of landmark DB
    }

    // Start in a thread to do all processing
    int SingleRobot::RunMapping(void)
    {
//        // Vector for tracking time statistics
//        vector<float> vTimesTrack;
//        vTimesTrack.resize(nImages);

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl << endl;

        // Main loop
        // For all data:
        cv::Mat imLeft, imRight;
        for(int ni=0; ni<nImages; ni++)
        {
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
            imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[ni]) << endl;
                return 1;
            }

//#ifdef COMPILEDWITHC11
//            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//#else
//            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//#endif

            // process new frames: VO, loop closure in ORB-SLAM
            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe);

//#ifdef COMPILEDWITHC11
//            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//#else
//            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//#endif
//
//            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//            vTimesTrack[ni]=ttrack;
//
//            // Wait to load the next frame
//            double T=0;
//            if(ni<nImages-1)
//                T = vTimestamps[ni+1]-tframe;
//            else if(ni>0)
//                T = tframe-vTimestamps[ni-1];
//
//            if(ttrack<T)
//                usleep((T-ttrack)*1e6);

            // TODO: Exchange features with map DB
            // if previous map DB match optimization done
            // TODO: apply corrections
            // if new map DB match:
            // TODO: start local optimization
            // For now, we're going to write all graph files and then call distributedMapper
            // Eventually, should construct graph and start thread for optimization

            // for each pose in robot's trajectory
            // using only KFs here (for speed?)

            vector<KeyFrame*> vpKFs = SLAM.mpMap->GetAllKeyFrames();
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
                // write vertex line
            // for each pair of poses in robot's trajectory
                // write edge line for frame-to-frame constraint (VO)

            // for each new feature match
                // write edge line for constraint

        }

        // Stop all threads
        SLAM.Shutdown();

//        // Tracking time statistics
//        sort(vTimesTrack.begin(),vTimesTrack.end());
//        float totaltime = 0;
//        for(int ni=0; ni<nImages; ni++)
//        {
//            totaltime+=vTimesTrack[ni];
//        }
//        cout << "-------" << endl << endl;
//        cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
//        cout << "mean tracking time: " << totaltime/nImages << endl;

        // Save camera trajectory
        SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

        return 0;
    }


}