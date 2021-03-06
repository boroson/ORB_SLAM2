//
// Created by lboroson on 1/25/19.
//

#ifndef MRSLAM_SINGLEROBOT_H
#define MRSLAM_SINGLEROBOT_H

namespace ORB_SLAM2
{
    class SLAM;

    class Viewer;
    class FrameDrawer;
    class Map;
    class Tracking;
    class LocalMapping;
    class LoopClosing;

    class SingleRobot
    {
    public:
        // Input sensor
        enum eSensor{
            MONOCULAR=0,
            STEREO=1,
            RGBD=2
        };

    public:

        // Initialize the robot: set up ORB-SLAM, loop closure, landmark DB
        SingleRobot(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

        // Start in a thread to do all processing
                // For all data:
                        // process new frames: VO, loop closure
                                //Exchange features with mab DB
                                        // if new loop closure

        void RunMapping(void);


        // This stops local mapping thread (map building) and performs only camera tracking.
        void ActivateLocalizationMode();
        // This resumes local mapping thread and performs SLAM again.
        void DeactivateLocalizationMode();

        // Returns true if there have been a big map change (loop closure, global BA)
        // since last call to this function
        bool MapChanged();

        // Reset the system (clear map)
        void Reset();

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown();

        // Save camera trajectory in the TUM RGB-D dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryTUM(const string &filename);

        // Save keyframe poses in the TUM RGB-D dataset format.
        // This method works for all sensor input.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveKeyFrameTrajectoryTUM(const string &filename);

        // Save camera trajectory in the KITTI dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
        void SaveTrajectoryKITTI(const string &filename);

        // TODO: Save/Load functions
        // SaveMap(const string &filename);
        // LoadMap(const string &filename);

        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        int GetTrackingState();
        std::vector<MapPoint*> GetTrackedMapPoints();
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    private:

        // SLAM System
        System* SLAM;

        // Input sensor
        eSensor mSensor;

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary* mpVocabulary;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase* mpKeyFrameDatabase;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map* mpMap;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking* mpTracker;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        LocalMapping* mpLocalMapper;

        // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
        // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
        LoopClosing* mpLoopCloser;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer* mpViewer;

        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;

        // System threads: Local Mapping, Loop Closing, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread* mptLocalMapping;
        std::thread* mptLoopClosing;
        std::thread* mptViewer;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Tracking state
        int mTrackingState;
        std::vector<MapPoint*> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
        std::mutex mMutexState;
    };

}// namespace ORB_SLAM

#endif //MRSLAM_SINGLEROBOT_H
