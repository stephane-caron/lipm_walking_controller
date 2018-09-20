/* Copyright 2018 CNRS-UM LIRMM
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

#include <lipm_walking/HorizontalMPCProblem.h>

namespace lipm_walking
{
  using namespace HorizontalMPC;

  void HorizontalMPCProblem::writePython(const std::string & suffix)
  {
    auto t = std::time(nullptr);
    auto tm = std::localtime(&t);
    std::stringstream fileName;
    fileName << "/tmp/lcp_plot"
      << "_" << 10000 + (++nbWritePythonCalls_) << "_"
      << "ss" << nbInitSupportSteps_ << "-"
      << "ds" << nbDoubleSupportSteps_ << "-"
      << "ts" << nbTargetSupportSteps_ << "-"
      << "nds" << nbNextDoubleSupportSteps_
      << ((suffix.length() > 0) ? "_" : "") << suffix
      << ".py";

    Eigen::VectorXd xMax(NB_STEPS + 1);
    Eigen::VectorXd xMin(NB_STEPS + 1);
    Eigen::VectorXd yMax(NB_STEPS + 1);
    Eigen::VectorXd yMin(NB_STEPS + 1);
    for (unsigned i = 0; i <= NB_STEPS; i++)
    {
      // show DSP constraints from start of DSP, since drawstyle='steps-post'
      unsigned plotHrepIndex = (0 < i && i == nbInitSupportSteps_) ? 1 
        : (i == nbInitSupportSteps_ + nbDoubleSupportSteps_ + nbTargetSupportSteps_) ? 3
        : indexToHrep[i];
      auto hrep = hreps_[plotHrepIndex];
      Eigen::Polyhedron poly;
      xMax[i] = -4242;
      xMin[i] = +4242;
      yMax[i] = -4242;
      yMin[i] = +4242;
      if (!poly.setHrep(hrep.first, hrep.second))
      {
        LOG_WARNING("cddlib conversion error: " << poly.lastErrorMessage());
        continue;
      }
      auto vrep = poly.vrep();
      for (unsigned j = 0; j < vrep.first.rows(); j++)
      {
        xMax[i] = std::max(xMax[i], vrep.first(j, 0));
        xMin[i] = std::min(xMin[i], vrep.first(j, 0));
        yMax[i] = std::max(yMax[i], vrep.first(j, 1));
        yMin[i] = std::min(yMin[i], vrep.first(j, 1));
      }
    }

    pyScript_.open(fileName.str());
    pyScript_.precision(20);
    pyScript_ << "#!/usr/bin/env python" << std::endl << std::endl
      << "import IPython" << std::endl << std::endl
      << "from pylab import *" << std::endl << std::endl
      << "n = " << NB_STEPS + 1 << std::endl
      << "nb_init_support_steps = " << nbInitSupportSteps_ << std::endl
      << "nb_double_support_steps = " << nbDoubleSupportSteps_ << std::endl
      << "nb_target_support_steps = " << nbTargetSupportSteps_ << std::endl
      << "nb_next_double_support_steps = " << nbNextDoubleSupportSteps_ << std::endl
      << "t = [i * " << SAMPLING_PERIOD << " for i in xrange(" << NB_STEPS + 1 << ")]" << std::endl;
    writePythonContact(initContact_, "init_contact");
    writePythonContact(targetContact_, "target_contact");
    writePythonSolution();

    Eigen::Vector2d dcmRef = targetContact_.p().head<2>();
    pyScript_ << "zeta = " << zeta_ << std::endl
      << "dcm_ref = [" << dcmRef(0) << ", " << dcmRef(1) << "]"
      << std::endl;

    writePythonSerializedVector(xMax, "x_max", 0, NB_STEPS + 1);
    writePythonSerializedVector(xMin, "x_min", 0, NB_STEPS + 1);
    writePythonSerializedVector(yMax, "y_max", 0, NB_STEPS + 1);
    writePythonSerializedVector(yMin, "y_min", 0, NB_STEPS + 1);

    pyScript_ << R"(
assert len(com_x) == n
assert len(u_x) == n - 1

omegaInv = sqrt(zeta)
dcm_x = [com_x[i] + omegaInv * comd_x[i] for i in xrange(n)]
dcm_y = [com_y[i] + omegaInv * comd_y[i] for i in xrange(n)]
zmp_x = [com_x[i] - zeta * comdd_x[i] for i in xrange(n)]
zmp_y = [com_y[i] - zeta * comdd_y[i] for i in xrange(n)]

com = [array([com_x[i], com_y[i]]) for i in xrange(n)]
comd = [array([comd_x[i], comd_y[i]]) for i in xrange(n)]
comdd = [array([comdd_x[i], comdd_y[i]]) for i in xrange(n)]
comddd = [array([u_x[i], u_y[i]]) for i in xrange(n - 1)]
dcm = [array([dcm_x[i], dcm_y[i]]) for i in xrange(n)]
zmp = [array([zmp_x[i], zmp_y[i]]) for i in xrange(n)]
zmp_ref = [array([zmp_ref_x[i], zmp_ref_y[i]]) for i in xrange(n)]

zmp_error = [zmp[i] - zmp_ref[i] for i in xrange(n)]

def plot_contact(contact):
    plot([x for (x, y, z) in contact], [y for (x, y, z) in contact], 'k--')

def plot_x_y(fig_id):
    figure(fig_id)
    plot(com_x, com_y, 'b-')
    plot(dcm_x, dcm_y, 'g-')
    plot(zmp_x, zmp_y, 'r--')
    plot(zmp_ref_x, zmp_ref_y, 'k:')
    legend(('CoM', 'DCM', 'ZMP', 'ZMP_ref'))
    xlim(-0.1, 0.4)
    ylim(-0.2, 0.3)
    plot_contact(init_contact)
    plot_contact(target_contact)
    plot([com_x[0]], [com_y[0]], 'bo', markersize=15)
    plot([dcm_ref[0]], [dcm_ref[1]], 'g*', markersize=20)
    plot([dcm_x[0]], [dcm_y[0]], 'go', markersize=15)
    plot([zmp_ref_x[0]], [zmp_ref_y[0]], 'ko', markersize=15)
    plot([zmp_x[0]], [zmp_y[0]], 'ro', markersize=15)
    plot(com_x, com_y, 'bo')
    plot(dcm_x, dcm_y, 'go')
    plot(zmp_ref_x, zmp_ref_y, 'ko')
    plot(zmp_x, zmp_y, 'ro')

def plot_t_xy(fig_id):
    figure(fig_id)
    subplot(311)
    plot(t, com_x, 'b-')
    plot(t, dcm_x, 'g-')
    plot(t, zmp_x, 'r-')
    plot(t, zmp_ref_x, 'r--')
    legend(('COM_x', 'DCM_x', 'ZMP_x', 'ZMP_ref_x'))
    plot(t, x_min, 'k:', drawstyle='steps-post')
    plot(t, x_max, 'k:', drawstyle='steps-post')
    subplot(312)
    plot(t, com_y, 'b-')
    plot(t, dcm_y, 'g-')
    plot(t, zmp_y, 'r-')
    plot(t, zmp_ref_y, 'r--')
    legend(('COM_y', 'DCM_y', 'ZMP_y', 'ZMP_ref_y'))
    plot(t, y_min, 'k:', drawstyle='steps-post')
    plot(t, y_max, 'k:', drawstyle='steps-post')
    subplot(313)
    plot(t, vel_ref_x, 'r--')
    plot(t, vel_ref_y, 'g--')
    legend(('refVel_x', 'refVel_y'))

if __name__ == "__main__":
    ion()
    # close('all')
    clf()
    plot_t_xy(1)
    # plot_x_y(2)
    if IPython.get_ipython() is None:
        IPython.embed()

# File written: )";

    pyScript_ //<< (1900 + tm->tm_year) << "/"
      << (1 + tm->tm_mon) << "/" << tm->tm_mday
      << " " << tm->tm_hour << ":" << tm->tm_min << ":" << tm->tm_sec;
    pyScript_.close();
  }

  void HorizontalMPCProblem::writePythonContact(const Contact & contact, const std::string & label)
  {
    pyScript_ << label << " = ["
      << "(" << contact.vertex0()(0) << ", " << contact.vertex0()(1) << ", " << contact.vertex0()(2) << "), "
      << "(" << contact.vertex1()(0) << ", " << contact.vertex1()(1) << ", " << contact.vertex1()(2) << "), "
      << "(" << contact.vertex2()(0) << ", " << contact.vertex2()(1) << ", " << contact.vertex2()(2) << "), "
      << "(" << contact.vertex3()(0) << ", " << contact.vertex3()(1) << ", " << contact.vertex3()(2) << "),"
      << "(" << contact.vertex0()(0) << ", " << contact.vertex0()(1) << ", " << contact.vertex0()(2) << ")]"
      << std::endl;
  }

  void HorizontalMPCProblem::writePythonSolution()
  {
    const Eigen::VectorXd & stateTraj = solution_.stateTraj();
    const Eigen::VectorXd & jerkTraj = solution_.jerkTraj();
    writePythonSerializedVector(jerkTraj, "u_x", 0, NB_STEPS);
    writePythonSerializedVector(jerkTraj, "u_y", 1, NB_STEPS);
    writePythonSerializedVector(stateTraj, "com_x", 0, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "com_y", 1, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "comd_x", 2, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "comd_y", 3, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "comdd_x", 4, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "comdd_y", 5, NB_STEPS + 1);
    writePythonSerializedVector(zmpRef_, "zmp_ref_x", 0, NB_STEPS + 1);
    writePythonSerializedVector(zmpRef_, "zmp_ref_y", 1, NB_STEPS + 1);
    writePythonSerializedVector(velRef_, "vel_ref_x", 0, NB_STEPS + 1);
    writePythonSerializedVector(velRef_, "vel_ref_y", 1, NB_STEPS + 1);
  }

  void HorizontalMPCProblem::writePythonSerializedVector(const Eigen::VectorXd & vec, const std::string & label, unsigned index, unsigned nbChunks)
  {
    long chunkSize = vec.size() / nbChunks;
    pyScript_ << label << " = [";
    for (unsigned i = 0; i < nbChunks; i++)
    {
      pyScript_ << vec(chunkSize * i + index);
      if (i < nbChunks - 1)
      {
        pyScript_ << ", ";
      }
    }
    pyScript_ << "]" << std::endl;
  }
}
