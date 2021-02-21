//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "SBG_Ellipse.h"


namespace KTH
{

  namespace SBG_ELLIPSE2D
  {
    using DUNE_NAMESPACES;

    //Static stuff

    struct Arguments
    {
      //TODO arguments
    };

    struct Task: public Tasks::Task
    {
      //Local variables
      SBG_Ellipse ahrs;
      Stream* serialPort;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx)
      {
        // Define configuration parameters.


        //Start sbg etc
        serialPort = new Stream();
        ahrs.begin(serialPort);


        //bind<IMC::DevDataText>(this);
        //bind<IMC::IoEvent>(this);
      }

      void
      onResourceAcquisition(void)
      {

      }

      void
      onResourceRelease(void)
      {
      }

      void
      onResourceInitialization(void)
      {
      }

      void
      onMain(void)
      {
        while (!stopping())
        {
          //waitForMessages(0.0001);
          SBG_DATA sbg_data_type = ahrs.ReadData();
          switch (sbg_data_type) {
            case UTC_TIME_DATA:
            {

            }
            break;
            case IMU_DATA:
            {
              IMC::AngularVelocity msg_gyro;
              msg_gyro.x = ahrs.imu.gyro_x;
              msg_gyro.y = ahrs.imu.gyro_y;
              msg_gyro.z = ahrs.imu.gyro_z;
              msg_gyro.setSourceEntity(getEntityId());
              dispatch(msg_gyro);

              IMC::Acceleration msg_acc;
              msg_acc.x = ahrs.imu.accel_x;
              msg_acc.y = ahrs.imu.accel_y;
              msg_acc.z = ahrs.imu.accel_z;
              msg_acc.setSourceEntity(getEntityId());
              dispatch(msg_acc);
            }
            break;
            case MAGNETOMETER_DATA:
            {
              IMC::MagneticField msg;
              msg.x = ahrs.mag.mx;
              msg.y = ahrs.mag.my;
              msg.z = ahrs.mag.mz;
              msg.setSourceEntity(getEntityId());
              dispatch(msg);
            }
            break;
            case EKF_EULER_DATA:
            {
              IMC::EulerAngles msg;
              msg.phi = ahrs.euler.roll;
              msg.theta = ahrs.euler.pitch;
              msg.psi = ahrs.euler.yaw;
              msg.setSourceEntity(getEntityId());
              dispatch(msg);
            }
            break;
            case EKF_QUAT_DATA:
            {

            }
            break;
            case EKF_NAV_POS_VEL_DATA:
            {

            }
            break;
            case SHIP_MOTION_DATA:
            {

            }
            break;
            case GPS_VEL_DATA:
            {

            }
            break;
            case GPS_POS_DATA:
            {
              IMC::GpsFix msg;
              msg.lat = ahrs.GPSPos.lat;
              msg.lon = ahrs.GPSPos.lon;
              msg.satellites = ahrs.GPSPos.num_sat;
              //TODO add the rest
              msg.setSourceEntity(getEntityId());
              dispatch(msg);


            }
            break;
            case GPS_TRUE_HEADING_DATA:
            {

            }
            break;
          }
        }
      }
    };
  }
}

DUNE_TASK
