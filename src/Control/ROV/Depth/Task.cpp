//***************************************************************************
// Copyright (C) 2007-2013 Laboratório de Sistemas e Tecnologia Subaquática *
// Departamento de Engenharia Electrotécnica e de Computadores              *
// Rua Dr. Roberto Frias, 4200-465 Porto, Portugal                          *
//***************************************************************************
// Author: Jorge Estrela da Silva (original controller implementation)      *
// Author: Renato Caldas                                                    *
// Author: Pedro Calado (adapted to ROVs)                                   *
//***************************************************************************
// $Id:: Task.cpp 12695 2013-01-23 22:54:41Z rasm                         $:*
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace ROV
  {
    namespace Depth
    {
      using DUNE_NAMESPACES;

      //! Controllable loops
      static const uint32_t c_controllable = IMC::CL_DEPTH | IMC::CL_VERTICAL_RATE;
      //! Required loops
      static const uint32_t c_required = 0;

      //! Task arguments
      struct Arguments
      {
        unsigned vmotor_id;
        std::vector<float> depth_gains;
        float max_heave;
        std::vector<float> heave_gains;
        float max_thrust;
      };

      struct Task: public DUNE::Control::BasicAutopilot
      {
        //! Depth PID controller
        DiscretePID m_depth_pid;
        //! Heave speed PID controller
        DiscretePID m_heave_pid;
        //! Desired actuation torque vector.
        IMC::SetThrusterActuation m_thrust;
        //! Task Arguments
        Arguments m_args;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::BasicAutopilot(name, ctx, c_controllable, c_required)
        {
          param("Vertical Motor Id", m_args.vmotor_id)
          .defaultValue("4")
          .description("");

          param("Depth PID Gains", m_args.depth_gains)
          .defaultValue("")
          .size(3)
          .description("Proportional gain for depth controller");

          param("Maximum Heave Reference", m_args.max_heave)
          .defaultValue("1.0")
          .description("Maximum admissible heave reference for depth controller");

          param("Heave PID Gains", m_args.heave_gains)
          .defaultValue("")
          .size(3)
          .description("Proportional gain for heave controller");

          param("Maximum Thrust Reference", m_args.max_thrust)
          .defaultValue("1.0")
          .description("Maximum admissible thrust command for heave controller");
        }

        void
        reset(void)
        {
          BasicAutopilot::reset();

          m_depth_pid.reset();
          m_heave_pid.reset();

          m_thrust.id = m_args.vmotor_id;
          m_thrust.value = 0.0;
          dispatch(m_thrust);
        }

        void
        onResourceInitialization(void)
        {
          BasicAutopilot::onResourceInitialization();

          // Depth control parameters.
          m_depth_pid.setGains(m_args.depth_gains);
          m_depth_pid.setOutputLimits(-m_args.max_heave, m_args.max_heave);

          // Heave control parameters.
          m_heave_pid.setGains(m_args.heave_gains);
          m_heave_pid.setOutputLimits(-m_args.max_thrust, m_args.max_thrust);
        }

        void
        onEstimatedState(const double timestep, const IMC::EstimatedState* msg)
        {
          m_thrust.id = m_args.vmotor_id; // CHANGE TO VERTICAL MOTOR ID
          m_thrust.value = depthControl(timestep, msg);

          dispatch(m_thrust);
        }

        //! Compute thrust to comply with heave reference
        float
        depthControl(const double timestep, const IMC::EstimatedState* msg)
        {
          float cmd = 0;
          float z_error = 0;

          switch (getVerticalMode())
          {
            case VERTICAL_MODE_HEAVE:
              cmd = getVerticalRef();
              break;
            case VERTICAL_MODE_DEPTH:
              z_error = getVerticalRef() - msg->depth;
              cmd = m_depth_pid.step(timestep, z_error);
              break;
            case VERTICAL_MODE_ALTITUDE:
              z_error = getBottomFollowDepth() - msg->alt;
              cmd = m_depth_pid.step(timestep, z_error);
              break;
            default:
              signalBadVertical();
              return 0.0;
          }

          // Log desired heave
          if ((getVerticalMode() == VERTICAL_MODE_DEPTH) ||
              (getVerticalMode() == VERTICAL_MODE_ALTITUDE))
          {
            IMC::DesiredVelocity m_heave_ref;
            m_heave_ref.w = cmd;
            m_heave_ref.flags = IMC::DesiredVelocity::FL_HEAVE;
            dispatch(m_heave_ref);
          }

          // Positive depth implies positive heave
          float heave_error = cmd - msg->w;
          cmd = -m_heave_pid.step(timestep, heave_error);

          return cmd;
        }
      };
    }
  }
}

DUNE_TASK
