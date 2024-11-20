#ifndef FINITE_STATE_CONTROLLER_HPP
#define FINITE_STATE_CONTROLLER_HPP

#include <vector>
#include <memory>

typedef std::vector<double> JointPose;

class FinStateCtrl
{
public:

    /**
     * @brief Construct a new Finite State Controller object
     * 
     * The initial index refers to the pose immediately before the next pose
     * returned by first call to the next() member function.
     * 
     * @param phase_positions ordered list of poses in joint-space
     * @param initialIdx index of the assumed starting joint pose from the given list of poses
     */
    FinStateCtrl(std::vector<JointPose> phase_positions, int initialIdx);

    /**
     * @brief Outputs the next joint pose in the sequence
     * 
     * @param poseOut destination to store the next joint position
     */
    void next(JointPose& poseOut);
private:

    /** Stores an ordered list of joint poses */
    std::vector<JointPose> mPhasePositions;

    /** Refers to the current joint pose from the stored list of joint poses */
    int mInitialIdx;
};

typedef std::shared_ptr<FinStateCtrl> FinStateCtrlPtr;

#endif // FINITE_STATE_CONTROLLER_HPP