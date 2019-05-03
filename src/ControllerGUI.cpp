/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Stéphane Caron
 *
 * This file is part of lipm_walking_controller.
 *
 * lipm_walking_controller is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * lipm_walking_controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lipm_walking_controller. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <lipm_walking/Controller.h>

namespace lipm_walking
{
  void Controller::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
  {
    using namespace mc_rtc::gui;

    gui->addElement(
      {"Walking", "Controller"},
      Button(
        "# EMERGENCY STOP",
        [this]()
        {
          emergencyStop = true;
          this->interrupt();
        }),
      Button(
        "Reset",
        [this]() { this->resume("Initial"); }),
      NumberInput(
        "Torso pitch [rad]",
        [this]() { return torsoPitch_; },
        [this](double pitch)
        {
          pitch = clamp(pitch, MIN_CHEST_P, MAX_CHEST_P);
          defaultTorsoPitch_ = pitch;
          torsoPitch_ = pitch;
        }));

    gui->addElement(
      {"Walking", "Plan"},
      Label(
        "Name",
        [this]() { return plan.name; }),
      NumberInput(
        "CoM height",
        [this]() { return plan.comHeight(); },
        [this](double height) { plan.comHeight(height); }),
      NumberInput(
        "Initial DSP duration [s]",
        [this]() { return plan.initDSPDuration(); },
        [this](double duration) { plan.initDSPDuration(duration); }),
      NumberInput(
        "SSP duration [s]",
        [this]() { return plan.singleSupportDuration(); },
        [this](double duration)
        {
          constexpr double T = ModelPredictiveControl::SAMPLING_PERIOD;
          duration = std::round(duration / T) * T;
          plan.singleSupportDuration(duration);
        }),
      NumberInput(
        "DSP duration [s]",
        [this]() { return plan.doubleSupportDuration(); },
        [this](double duration)
        {
          constexpr double T = ModelPredictiveControl::SAMPLING_PERIOD;
          duration = std::round(duration / T) * T;
          plan.doubleSupportDuration(duration);
        }),
      NumberInput(
        "Final DSP duration [s]",
        [this]() { return plan.finalDSPDuration(); },
        [this](double duration) { plan.finalDSPDuration(duration); }),
      NumberInput(
        "Swing height [m]",
        [this]() { return plan.swingHeight(); },
        [this](double height) { plan.swingHeight(height); }),
      NumberInput(
        "Takeoff duration",
        [this]() { return plan.takeoffDuration(); },
        [this](double duration) { plan.takeoffDuration(duration); }),
      NumberInput(
        "Takeoff pitch [rad]",
        [this]() { return plan.takeoffPitch(); },
        [this](double pitch) { plan.takeoffPitch(pitch); }),
      NumberInput(
        "Landing duration",
        [this]() { return plan.landingDuration(); },
        [this](double duration) { plan.landingDuration(duration); }),
      NumberInput(
        "Landing pitch [rad]",
        [this]() { return plan.landingPitch(); },
        [this](double pitch) { plan.landingPitch(pitch); }));

    constexpr double ARROW_HEAD_DIAM = 0.015;
    constexpr double ARROW_HEAD_LEN = 0.05;
    constexpr double ARROW_SHAFT_DIAM = 0.015;
    constexpr double FORCE_SCALE = 0.0015;

    const std::map<char, Color> COLORS =
    {
      {'r', Color{1.0, 0.0, 0.0}},
      {'g', Color{0.0, 1.0, 0.0}},
      {'b', Color{0.0, 0.0, 1.0}},
      {'y', Color{1.0, 0.5, 0.0}},
      {'c', Color{0.0, 0.5, 1.0}},
      {'m', Color{1.0, 0.0, 0.5}}
    };

    ArrowConfig pendulumArrowConfig;
    pendulumArrowConfig.color = COLORS.at('y');
    pendulumArrowConfig.end_point_scale = 0.02;
    pendulumArrowConfig.head_diam = .1 * ARROW_HEAD_DIAM;
    pendulumArrowConfig.head_len = .1 * ARROW_HEAD_LEN;
    pendulumArrowConfig.scale = 1.;
    pendulumArrowConfig.shaft_diam = .1 * ARROW_SHAFT_DIAM;
    pendulumArrowConfig.start_point_scale = 0.02;

    ArrowConfig pendulumForceArrowConfig;
    pendulumForceArrowConfig.shaft_diam = 1 * ARROW_SHAFT_DIAM;
    pendulumForceArrowConfig.head_diam = 1 * ARROW_HEAD_DIAM;
    pendulumForceArrowConfig.head_len = 1 * ARROW_HEAD_LEN;
    pendulumForceArrowConfig.scale = 1.;
    pendulumForceArrowConfig.start_point_scale = 0.02;
    pendulumForceArrowConfig.end_point_scale = 0.;

    ArrowConfig netWrenchForceArrowConfig = pendulumForceArrowConfig;
    netWrenchForceArrowConfig.color = COLORS.at('r');

    ArrowConfig refPendulumForceArrowConfig = pendulumForceArrowConfig;
    refPendulumForceArrowConfig = COLORS.at('y');

    ForceConfig copForceConfig(COLORS.at('g'));
    copForceConfig.start_point_scale = 0.02;
    copForceConfig.end_point_scale = 0.;

    auto footStepPolygon = [](const Contact& contact)
    {
      std::vector<Eigen::Vector3d> polygon;
      polygon.push_back(contact.vertex0());
      polygon.push_back(contact.vertex1());
      polygon.push_back(contact.vertex2());
      polygon.push_back(contact.vertex3());
      return polygon;
    };

    gui->addElement(
      {"Walking", "Markers", "Contacts"},
      Polygon(
        "SupportContact",
        COLORS.at('g'),
        [this, footStepPolygon]()
        {
          return footStepPolygon(supportContact());
        }),
      Polygon(
        "TargetContact",
        COLORS.at('b'),
        [this, footStepPolygon]()
        {
          return footStepPolygon(targetContact());
        }),
      Polygon(
        "FootstepPlan",
        COLORS.at('b'),
        [this, footStepPolygon]()
        {
          std::vector<std::vector<Eigen::Vector3d>> polygons;
          const auto & contacts = plan.contacts();
          for (unsigned i = 0; i < contacts.size(); i++)
          {
            auto & contact = contacts[i];
            double supportDist = (contact.p() - supportContact().p()).norm();
            double targetDist = (contact.p() - targetContact().p()).norm();
            constexpr double SAME_CONTACT_DIST = 0.005;
            // only display contact if it is not the support or target contact
            if (supportDist > SAME_CONTACT_DIST && targetDist > SAME_CONTACT_DIST)
            {
              polygons.push_back(footStepPolygon(contact));
            }
          }
          return polygons;
        }));

    gui->addElement(
      {"Walking", "Markers", "Pendulum"},
      Arrow(
        "ControlCoMDCMArrow",
        COLORS.at('b'),
        [this]() -> Eigen::Vector3d
        {
          return controlCom_;
        },
        [this]() -> Eigen::Vector3d
        {
          return controlCom_ + controlComd_ / pendulum().omega();
        }),
      Point3D(
        "ControlCoM",
        PointConfig(COLORS.at('b')),
        [this]()
        {
          return controlCom_;
        }),
      Point3D(
        "ControlDCM",
        PointConfig(COLORS.at('b'), 0.01),
        [this]() -> Eigen::Vector3d
        {
          return controlCom_ + controlComd_ / pendulum().omega();
        }),
      Arrow(
        "RealCoMDCMArrow",
        COLORS.at('b'),
        [this]() -> Eigen::Vector3d
        {
          return realCom_;
        },
        [this]() -> Eigen::Vector3d
        {
          return realCom_ + realComd_ / pendulum().omega();
        }),
      Point3D(
        "RealCoM",
        PointConfig(COLORS.at('b')),
        [this]()
        {
          return realCom_;
        }),
      Point3D(
        "RealDCM",
        PointConfig(COLORS.at('b'), 0.01),
        [this]() -> Eigen::Vector3d
        {
          return realCom_ + realComd_ / pendulum().omega();
        }));

    gui->addElement(
      {"Walking", "Markers", "Force"},
      Force(
        "LeftCoPForce",
        copForceConfig,
        [this]()
        {
          return this->robot().surfaceWrench("LeftFootCenter");
        },
        [this]()
        {
          Eigen::Vector3d cop = this->robot().copW("LeftFootCenter");
          return sva::PTransformd(this->robot().surface("LeftFootCenter").X_0_s(this->robot()).rotation(), cop);
        }),
      Force(
        "RightCoPForce",
        copForceConfig,
        [this]()
        {
          return this->robot().surfaceWrench("RightFootCenter");
        },
        [this]()
        {
          Eigen::Vector3d cop = this->robot().copW("RightFootCenter");
          return sva::PTransformd(this->robot().surface("RightFootCenter").X_0_s(this->robot()).rotation(), cop);
        }),
      Point3D(
        "NetWrenchZMP",
        PointConfig(COLORS.at('r'), 0.01),
        [this]() -> Eigen::Vector3d
        {
          return netWrenchObs_.zmp();
        }),
      Arrow(
        "NetWrenchForce",
        netWrenchForceArrowConfig,
        [this]() -> Eigen::Vector3d
        {
          return this->netWrenchObs_.zmp();
        },
        [this, FORCE_SCALE]() -> Eigen::Vector3d
        {
          return netWrenchObs_.zmp() + FORCE_SCALE * netWrenchObs_.wrench().force();
        }),
      Arrow(
        "RefPendulum",
        pendulumArrowConfig,
        [this]() -> Eigen::Vector3d
        {
          return this->pendulum_.zmp();
        },
        [this]() -> Eigen::Vector3d
        {
          return this->pendulum_.com();
        }),
      Arrow(
        "RefPendulumForce",
        refPendulumForceArrowConfig,
        [this]() -> Eigen::Vector3d
        {
          return this->pendulum_.zmp();
        },
        [this, FORCE_SCALE]() -> Eigen::Vector3d
        {
          double lambda = std::pow(pendulum_.omega(), 2);
          Eigen::Vector3d contactForce = controlRobot().mass() * lambda * (pendulum_.com() - pendulum_.zmp());
          return pendulum_.zmp() + FORCE_SCALE * contactForce;
        }),
      Point3D(
        "StabilizerZMP",
        PointConfig(COLORS.at('m'), 0.02),
        [this]() { return stabilizer_.zmp(); }),
      Point3D(
        "StabilizerCoP_LeftFootCenter",
        PointConfig(COLORS.at('m'), 0.01),
        [this]() { return stabilizer_.leftFootTask->targetCoPW(); }),
      Point3D(
        "StabilizerCoP_RightFootCenter",
        PointConfig(COLORS.at('m'), 0.01),
        [this]() { return stabilizer_.rightFootTask->targetCoPW(); }));
  }
}
