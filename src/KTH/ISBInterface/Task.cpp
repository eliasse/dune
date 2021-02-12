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
// Author: Niklas                                                           *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <stdio.h>

//Libraries
#include <SerialInterface/SerialInterface.h>

// Local Headers.
#include <KTH/ISBInterface/message_id.h>

namespace KTH
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Niklas
  namespace ISBInterface
  {
    using DUNE_NAMESPACES;

    SerialInterface* isb = NULL;
    //RosInterFace rosInterface;
    //UTM utmConverter_toutm;
    //UTM utmConverter_fromutm;

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      // Serial port device.
      std::string uart_dev;
      // Serial port baud rate.
      int uart_baud;

      // Motor Entity ID's
      unsigned int id_port;
      unsigned int id_starboard;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
          param("Serial Port - Device", uart_dev)
                  .defaultValue("")
                  .description("Serial port device used to communicate with the sensor");

          param("Serial Port - Baud Rate", uart_baud)
                  .defaultValue("9600")
                  .description("Serial port baud rate");

        isb = new SerialInterface();
        isb->setup(uart_dev, uart_baud);
        //isb->setCallback(&this->ISB_CALLBACK);
        auto fp = std::bind(&Task::ISB_CALLBACK, this);
        isb->setCallback(fp);

        bind<SetThrusterActuation>(this);

        id_port = reserveEntity("Port Motor");
        id_starboard = reserveEntity("Starboard Motor");
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
        resolveEntity(getEntityId());
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          waitForMessages(0.001);
          //std::cout << "Hello! ISBInterface is alive" << std::endl;
          if(isb != NULL) isb->readData();
        }
      }

      void ISB_CALLBACK() {
        //std::cout << "Callback called. Message id=" << (int) isb->messageID() << std::endl;

        //if((int) isb->messageID() == ID_HEARTBEAT) {}

        switch (isb->messageID()) {
          case ID_VOLTAGE: {
            IMC::Voltage msg;
            msg.value = isb->parse_float();
            msg.setSourceEntity(getEntityId());
            dispatch(msg);
            //std::cout << "dispatched Voltage message" << std::endl;
          } break;
          case ID_CURRENT: {
            IMC::Current msg;
            msg.value = isb->parse_float();
            msg.setSourceEntity(getEntityId());
            dispatch(msg);
            //std::cout << "dispatched Current message" << std::endl;
          } break;
          case ID_TEMPERATURE: {
            IMC::Temperature msg;
            msg.value = isb->parse_float();
            msg.setSourceEntity(getEntityId());
            dispatch(msg);
            //std::cout << "dispatched Temperature message" << std::endl;
          } break;

          case ID_THRUSTER_PORT: {
            std::cout << "Port thruster data received" << std::endl;
            float rpm       = isb->parse_float();
            //float current   = isb->parse_float();
            //float torque    = isb->parse_float();
            //float tempC     = isb->parse_float();
            //float voltage   = isb->parse_float();
            IMC::Rpm msg;
            msg.value = (int16_t)rpm;
            msg.setSourceEntity(id_port); //Crashes here
            dispatch(msg);
          } break;

          case ID_THRUSTER_STRB: {
            std::cout << "Strb thruster data received" << std::endl;
            float rpm       = isb->parse_float();
            //float current   = isb->parse_float();
            //float torque    = isb->parse_float();
            //float tempC     = isb->parse_float();
            //float voltage   = isb->parse_float();
            IMC::Rpm msg;
            msg.value = (int16_t)rpm;
            msg.setSourceEntity(id_starboard);
            dispatch(msg);
          } break;

          case ID_ECHOSOUNDER: {
            std::cout << "Echosounder data received" << std::endl;
            float dbt                   = isb->parse_float();
            float offset                = isb->parse_float();
            float max_range_scale       = isb->parse_float();
            //float sea_water_temperature = isb->parse_float();
            uint16_t depth_scaled = offset + dbt*10;
            //uint8_t* depth_bytes = (uint8_t*) &depth_scaled;

            /*IMC::SonarData sonar_msg;
            sonar_msg.type = 1; //ECHOSOUNDER
            sonar_msg.frequency = 200000;
            sonar_msg.min_range = 0.5;
            sonar_msg.max_range = max_range_scale;
            sonar_msg.bits_per_point = 16;
            sonar_msg.scale_factor = 0.1;
            //sonar_msg.beam_config =
            sonar_msg.data.push_back(depth_bytes[0]);
            sonar_msg.data.push_back(depth_bytes[1]);
            sonar_msg.setSourceEntity(getEntityId());
            dispatch(sonar_msg);
            */
            IMC::BeamConfig beam;
            beam.beam_width = 9*3.1415/180.0;
            beam.beam_height = 9*3.1415/180.0;
            IMC::Distance sonar_msg;
            sonar_msg.validity = 1;
            sonar_msg.beam_config.push_back(beam);
            sonar_msg.value = depth_scaled;
            sonar_msg.setSourceEntity(getEntityId());
            dispatch(sonar_msg);

          } break;
        }
      }

      void
      consume(const IMC::SetThrusterActuation* msg)
      {
        if(msg->id == 0) {
          isb->new_package(DI_SET_THRUSTER_PORT);
          isb->add_float(msg->value); //between -1 and 1
          isb->send_package();
          //std::cout << "Thruster port: " << msg->value << std::endl;
        }
        if(msg->id == 1) {
          isb->new_package(DI_SET_THRUSTER_STRB);
          isb->add_float(msg->value); //between -1 and 1
          isb->send_package();
          //std::cout << "Thruster strb: " << msg->value << std::endl;
        }
        else {
          std::cerr << "unknown thruster ID" << std::endl;
        }
      }
    };
  }
}

DUNE_TASK
