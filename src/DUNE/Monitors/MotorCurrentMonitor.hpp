//***************************************************************************
// Copyright (C) 2007-2013 Laboratório de Sistemas e Tecnologia Subaquática *
// Departamento de Engenharia Electrotécnica e de Computadores              *
// Rua Dr. Roberto Frias, 4200-465 Porto, Portugal                          *
//***************************************************************************
// Author: Pedro Calado                                                     *
//***************************************************************************
// $Id:: MotorCurrentMonitor.hpp 12731 2013-01-25 11:30:03Z pdcalado      $:*
//***************************************************************************

#ifndef DUNE_MONITORS_MOTOR_CURRENT_MONITOR_HPP_INCLUDED_
#define DUNE_MONITORS_MOTOR_CURRENT_MONITOR_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <stdexcept>
#include <string>
#include <vector>

// DUNE headers.
#include <DUNE/Math.hpp>
#include <DUNE/Monitors/DelayedTrigger.hpp>

namespace DUNE
{
  namespace Monitors
  {
    template <typename Tc, typename Tr>
    class MotorCurrentMonitor
    {
    public:
      class Error: public std::runtime_error
      {
      public:
        Error(const std::string& msg):
          std::runtime_error(DTR("motor monitor error: ") + msg)
        { }
      };

      //! Constructor (assumes current and rpm values are of the same type)
      //! @param lin_points points of the motor model to be used in piecewise linear function
      //! @param threshold current error value above which a detection will trigger
      //! @param detection_delay delay in seconds before a detection is triggered
      //! @param average_samples number of samples to use in the delayed trigger
      MotorCurrentMonitor(const std::vector<Tc>& lin_points, const Tc threshold, const double detection_delay, const unsigned average_samples):
        c_lin_points(lin_points),
        c_threshold(threshold)
      {
        // validate the points provided for piecewise linear function
        validatePoints();

        m_trigger = new DelayedTrigger<Tc>(threshold, detection_delay, average_samples);
      }

      //! Destructor.
      ~MotorCurrentMonitor(void)
      {
        Memory::clear(m_trigger);
      }

      //! Update function.
      //! @param curr_value new current measurement
      //! @param rpm_value new rpm measurement
      //! @return will be -1.0 if no detection triggers and the threshold value otherwise
      Tc
      updateAndTest(const Tc& curr_value, const Tr& rpm_value)
      {
        Tc linvalue = interpolateValue(rpm_value);
        if (m_trigger->updateAndTest(curr_value - linvalue))
          return linvalue + c_threshold;
        else
          return -1.0;
      }

    private:
      //! Validate the points vector for linear interpolation
      inline void
      validatePoints(void)
      {
        if (c_lin_points.size() % 2)
          throw Error(DTR("number of linear points is not even"));
      }

      //! Curve value function
      //! @param rpms value of rpms read to estimate the value of the current
      //! @return linearized value of the current for the given rpms
      Tc
      interpolateValue(const Tr& value)
      {
        using Math::LinIntParam;

        if (!c_lin_points.size())
          return 0.0;

        if (value <= c_lin_points[0])
          return linearInterpolation(LinIntParam<Tc>((Tc)0.0, c_lin_points[1],
                                                     (Tc)0.0, c_lin_points[0],
                                                     (Tc)value));

        for (unsigned i = 1; i < c_lin_points.size() / 2; ++i)
          if (value <= c_lin_points[i * 2])
            return linearInterpolation(LinIntParam<Tc>(c_lin_points[(i - 1) * 2 + 1],
                                                       c_lin_points[i * 2 + 1],
                                                       c_lin_points[(i - 1) * 2],
                                                       c_lin_points[i * 2],
                                                       (Tc)value));

        size_t rows = c_lin_points.size() / 2;
        return linearInterpolation(LinIntParam<Tc>(c_lin_points[(rows - 2) * 2 + 1],
                                                   c_lin_points[(rows - 1) * 2 + 1],
                                                   c_lin_points[(rows - 2) * 2],
                                                   c_lin_points[(rows - 1) * 2],
                                                   (Tc)value));
      }

      //! Vector of linearization points for the current
      //! will be assumed to have current values in the
      //! first column and rpm values in the second one
      const std::vector<Tc> c_lin_points;
      //! Threshold for the fault detection
      const Tc c_threshold;
      //! DelayedTrigger object for the fault detection
      DelayedTrigger<Tc>* m_trigger;
    };
  }
}

#endif
