// ======================================================================
// \title  LeeCtrlImpl.cpp
// \author mereweth
// \brief  cpp file for LeeCtrl component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
//
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ======================================================================


#include <Gnc/quest_gnc/src/comp/LeeCtrl/LeeCtrlComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  LeeCtrlComponentImpl ::
#if FW_OBJECT_NAMES == 1
    LeeCtrlComponentImpl(
        const char *const compName
    ) :
      LeeCtrlComponentBase(compName),
#else
    LeeCtrlComponentImpl(void) :
      LeeCtrlComponentImpl(void),
#endif
      seq(0u),
      w_q_b(0.0f, 0.0f, 0.0f, 1.0f),
      leeControl()
  {

  }

  void LeeCtrlComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    LeeCtrlComponentBase::init(instance);
  }

  LeeCtrlComponentImpl ::
    ~LeeCtrlComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void LeeCtrlComponentImpl ::
    odometry_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::nav_msgs::Odometry &Odometry
    )
  {
      using namespace ROS::geometry_msgs;
      ROS::std_msgs::Header h = Odometry.getheader();
      this->seq = h.getseq();
      // TODO(mereweth) check h.getstamp()

      Point x_w__msg = Odometry.getpose().getpose().getposition();
      this->w_q_b = Odometry.getpose().getpose().getorientation();

      Vector3 v_w__msg = Odometry.gettwist().gettwist().getlinear();
      Vector3 omega_w__msg = Odometry.gettwist().gettwist().getangular();

      Eigen::Quaterniond w_q_b = Eigen::Quaterniond(this->w_q_b.getw(),
                                                    this->w_q_b.getx(),
                                                    this->w_q_b.gety(),
                                                    this->w_q_b.getz());

      Eigen::Vector3d v_b = w_q_b.inverse() * Eigen::Vector3d(v_w__msg.getx(),
                                                              v_w__msg.gety(),
                                                              v_w__msg.getz());
      Eigen::Vector3d omega_b = w_q_b.inverse()
                                * Eigen::Vector3d(omega_w__msg.getx(),
                                                  omega_w__msg.gety(),
                                                  omega_w__msg.getz());

      this->leeControl.SetOdometry(Eigen::Vector3d(x_w__msg.getx(),
                                                   x_w__msg.gety(),
                                                   x_w__msg.getz()),
                                   w_q_b, v_b, omega_b);
  }

  void LeeCtrlComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      if (portNum == 0) {
          // TODO(mereweth) check context == run controller context
          using namespace ROS::geometry_msgs;
          Eigen::Vector3d a_w__comm(0, 0, 0);
          Eigen::Vector3d alpha_b__comm(0, 0, 0);
          this->leeControl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
          ROS::std_msgs::Header h(this->seq, this->getTime(), "body");
          Eigen::Quaterniond w_q_b = Eigen::Quaterniond(this->w_q_b.getw(),
                                                        this->w_q_b.getx(),
                                                        this->w_q_b.gety(),
                                                        this->w_q_b.getz());
          Eigen::Vector3d a_b__comm = w_q_b.inverse() * a_w__comm;
          ROS::mav_msgs::TorqueThrust u_b__comm(h,
            Vector3(a_b__comm(0), a_b__comm(1), a_b__comm(2)),
            Vector3(alpha_b__comm(0), alpha_b__comm(1), alpha_b__comm(2)));
          this->controls_out(0, u_b__comm);
      }
      else if (portNum == 1) {
          //this->tlmWrite_
      }
  }

  void LeeCtrlComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    // TODO
  }

} // end namespace Gnc
