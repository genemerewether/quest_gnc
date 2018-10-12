#include <gtest/gtest.h>

#include "quest_gnc/ctrl/lee_control.h"
#include "quest_gnc/utils/common.h"

using namespace quest_gnc;
using namespace quest_gnc::multirotor;

class LeeControlTest : public testing::Test
{
protected:
  LeeControlTest() :
    ctrl()
  {
    int stat;

    stat = ctrl.SetGains(Vector3(1.0, 1.0, 1.0),
       Vector3(0.1, 0.1, 0.1),
       Vector3(1.0, 1.0, 1.0),
       Vector3(0.1, 0.1, 0.1));
    EXPECT_EQ(stat, 0);

    mrModel = {1.0,
         0.01, 0.01, 0.01,
         0.0, 0.0, 0.0};
    stat = ctrl.SetModel(mrModel);
    EXPECT_EQ(stat, 0);

    wParams = {9.80665, 1.2};
    stat = ctrl.SetWorldParams(wParams);
    EXPECT_EQ(stat, 0);
  }

  virtual ~LeeControlTest() {
  }

  LeeControl ctrl;

  MultirotorModel mrModel;
  WorldParams wParams;
};

class SmallMagnitudeLinear : public LeeControlTest { };
class SmallMagnitudeAngular : public LeeControlTest { };

TEST_F(SmallMagnitudeAngular, angZ)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(AngleAxis(0.1, Vector3::UnitZ()));

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.0, 0.0, 0.0),
                                     Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_GT(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeAngular, angX)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(AngleAxis(0.1, Vector3::UnitX()));

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.0, 0.0, 0.0),
                                     Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_GT(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeAngular, angY)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(AngleAxis(0.1, Vector3::UnitY()));

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.0, 0.0, 0.0),
                                     Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_GT(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeAngular, angVelZ)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(Matrix3::Identity());

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.0, 0.0, 0.1),
                                     Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_GT(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeAngular, angVelX)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(Matrix3::Identity());

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.1, 0.0, 0.0),
                                     Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_GT(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeAngular, angVelY)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(Matrix3::Identity());

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.0, 0.1, 0.0),
                                     Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_GT(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeAngular, angAccZ)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(Matrix3::Identity());

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.0, 0.0, 0.0),
                                     Vector3(0.0, 0.0, 0.1));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  // TODO(mereweth) - equality check using inertia
  EXPECT_GT(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeAngular, angAccX)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(Matrix3::Identity());

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.0, 0.0, 0.0),
                                     Vector3(0.1, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  // TODO(mereweth) - equality check using inertia
  EXPECT_GT(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeAngular, angAccY)
{
  Vector3 alpha_b__comm;
  int stat;
  Quaternion w_q_b__des(Matrix3::Identity());

  stat = ctrl.SetAttitudeAngAccelDes(w_q_b__des,
                                     Vector3(0.0, 0.0, 0.0),
                                     Vector3(0.0, 0.1, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAngAccelCommand(&alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  // TODO(mereweth) - equality check using inertia
  EXPECT_GT(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);
}

TEST_F(SmallMagnitudeLinear, posZ)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.0, 0.0, 0.1),
                             Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_FLOAT_EQ(a_w__comm(0), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(1), 0.0);
  EXPECT_GT(a_w__comm(2), mrModel.mass * wParams.gravityMag);
}

TEST_F(SmallMagnitudeLinear, posX)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.1, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_GT(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_GT(a_w__comm(0), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(1), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(2), mrModel.mass * wParams.gravityMag);
}

TEST_F(SmallMagnitudeLinear, posY)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.0, 0.1, 0.0),
                             Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_LT(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_FLOAT_EQ(a_w__comm(0), 0.0);
  EXPECT_GT(a_w__comm(1), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(2), mrModel.mass * wParams.gravityMag);
}

TEST_F(SmallMagnitudeLinear, velZ)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.1),
                             Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_FLOAT_EQ(a_w__comm(0), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(1), 0.0);
  EXPECT_GT(a_w__comm(2), mrModel.mass * wParams.gravityMag);
}

TEST_F(SmallMagnitudeLinear, velX)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.0, 0.0, 0.0),
                             Vector3(0.1, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_GT(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_GT(a_w__comm(0), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(1), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(2), mrModel.mass * wParams.gravityMag);
}

TEST_F(SmallMagnitudeLinear, velY)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.1, 0.0),
                             Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_LT(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_FLOAT_EQ(a_w__comm(0), 0.0);
  EXPECT_GT(a_w__comm(1), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(2), mrModel.mass * wParams.gravityMag);
}

TEST_F(SmallMagnitudeLinear, accZ)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.1));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_FLOAT_EQ(a_w__comm(0), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(1), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(2), 0.1 + mrModel.mass * wParams.gravityMag);
}

TEST_F(SmallMagnitudeLinear, accX)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.0),
                             Vector3(0.1, 0.0, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_FLOAT_EQ(alpha_b__comm(0), 0.0);
  EXPECT_GT(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_FLOAT_EQ(a_w__comm(0), 0.1);
  EXPECT_FLOAT_EQ(a_w__comm(1), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(2), mrModel.mass * wParams.gravityMag);
}

TEST_F(SmallMagnitudeLinear, accY)
{
  Vector3 a_w__comm;
  Vector3 alpha_b__comm;
  int stat;

  stat = ctrl.SetPositionDes(Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.0, 0.0),
                             Vector3(0.0, 0.1, 0.0));
  EXPECT_EQ(stat, 0);

  stat = ctrl.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
  EXPECT_EQ(stat, 0);

  EXPECT_LT(alpha_b__comm(0), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(1), 0.0);
  EXPECT_FLOAT_EQ(alpha_b__comm(2), 0.0);

  EXPECT_FLOAT_EQ(a_w__comm(0), 0.0);
  EXPECT_FLOAT_EQ(a_w__comm(1), 0.1);
  EXPECT_FLOAT_EQ(a_w__comm(2), mrModel.mass * wParams.gravityMag);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
