#include "controller/finite_state_controller.hpp"

#include <cstddef>

FinStateCtrl::FinStateCtrl(std::vector<JointPose> phase_positions, int initialIdx)
{

    // Initializes the controller's state
    mPhasePositions = phase_positions;
    mInitialIdx = initialIdx;
}

void FinStateCtrl::next(JointPose& poseOut)
{
    const size_t numPoses = mPhasePositions.size();

    // Increment the pose index and output the referred joint pose
    mInitialIdx = (mInitialIdx + 1) % numPoses;
    poseOut = mPhasePositions[mInitialIdx];
}