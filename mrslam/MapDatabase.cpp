//
// Created by lboroson on 1/29/19.
//

#include "MapDatabase.h"


namespace ORB_SLAM2 {

    // Initialize the DB
    MapDatabase::MapDatabase(void)
    {
        // Initialize DB structure
        // Initialize empty constraint list
    }

    // Look at a list of new features with descriptors and 3D locations
    // If any of them match with old features, add constraints to list of constraints
    void MapDatabase::ProcessNewFeatures(list < features > )
    {
        // For each new feature
            // If it's already in DB
                // Add to list of re-observed features
            // else
                // Add to DB

        // If enough features re-observed
            // Do consistency check
            // Create constraints for any possible image/feature pairs

    }

    // Return list of constraints for optimization
    // Then clear ConstraintList so we don't apply them multiple times
    List <constraint> MapDatabase::GetConstraints(void)
    {
        // copy ConstraintList
        // clear ConstraintList
        // return copy
    }

    // Not quite sure how this works, but this function should initiate exchange with other robots nearby
    // It should send new features (or all features?) and receive features from other robots
    // If there are any matches in newly received features, add constraints to the constraint list
    void ExchangeFeatureData(void);

}