//
// Created by lboroson on 1/25/19.
//

#include "SingleRobot.h"

namespace ORB_SLAM2 {

    // Initialize the robot: set up ORB-SLAM, loop closure, landmark DB
    SingleRobot::SingleRobot(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                             const bool bUseViewer = true)
    {
        // Load data
        // Initialize ORB-SLAM system (including VO and loop closure)
        // Set up local part of landmark DB
    }

    // Start in a thread to do all processing
    void SingleRobot::RunMapping(void)
    {
        // For all data:
            // process new frames: VO, loop closure in ORB-SLAM
            // Exchange features with map DB
            // if previous map DB match optimization done
                // apply corrections
            // if new map DB match:
                // start local optimization
    }


}