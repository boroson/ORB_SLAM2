//
// Created by lboroson on 1/29/19.
//

#ifndef MRSLAM_MAPDATABASE_H
#define MRSLAM_MAPDATABASE_H

#endif //MRSLAM_MAPDATABASE_H


namespace ORB_SLAM2 {

    class MapDatabase {

    private:
        // Internal list of constraints to send to optimization
        List<constraints> ConstraintList;

    public:

        // Initialize the DB
        MapDatabase(void);

        // Look at a list of new features with descriptors and 3D locations
        // If any of them match with old features, add constraints to list of constraints
        void ProcessNewFeatures(list<features>);

        // Return list of constraints for optimization
        // Then clear ConstraintList so we don't apply them multiple times
        List<constraint> GetConstraints(void);

        // Not quite sure how this works, but this function should initiate exchange with other robots nearby
        // It should send new features (or all features?) and receive features from other robots
        // If there are any matches in newly received features, add constraints to the constraint list
        void ExchangeFeatureData(void);

    }

}