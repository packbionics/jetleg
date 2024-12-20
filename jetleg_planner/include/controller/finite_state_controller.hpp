// Copyright 2024 Pack Bionics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#ifndef CONTROLLER__FINITE_STATE_CONTROLLER_HPP_
#define CONTROLLER__FINITE_STATE_CONTROLLER_HPP_

#include <vector>
#include <memory>

typedef std::vector<double> JointPose;

/**
 * @brief Handles controlled transitions between discrete, finite joint poses
 *
 */
class FinStateCtrl
{
public:
  /**
   * @brief Construct a new Finite State Controller object
   *
   * The initial index refers to the pose immediately before the next pose
   * returned by first call to the next() member function.
   *
   * If the provided initial index is negative or greater than or equal to the
   * number of poses, the index will be reduced modulo the number of poses
   * on the next call to FinStateCtrl::next()
   *
   * @param phase_positions ordered list of poses in joint-space
   * @param initialIdx index of the assumed starting joint pose from the given list of poses
   * @throws std::invalid_argument This exception is thrown
   *        if the constructor is provided an empty collection of joint poses
   */
  FinStateCtrl(std::vector<JointPose> phase_positions, int initialIdx);

  /**
   * @brief Outputs the next joint pose in the sequence
   *
   * @param poseOut destination to store the next joint position
   */
  void next(JointPose & poseOut);

private:
  /** Stores an ordered list of joint poses */
  std::vector<JointPose> mPhasePositions;

  /** Refers to the current joint pose from the stored list of joint poses */
  int mInitialIdx;
};

typedef std::shared_ptr<FinStateCtrl> FinStateCtrlPtr;

#endif  // CONTROLLER__FINITE_STATE_CONTROLLER_HPP_
