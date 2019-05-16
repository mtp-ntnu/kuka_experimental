/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#ifndef KUKA_RSI_HW_INTERFACE_RSI_STATE_
#define KUKA_RSI_HW_INTERFACE_RSI_STATE_

#include <string>
#include <tinyxml.h>

namespace kuka_rsi_hw_interface
{
class RSIState
{
private:
  std::string xml_doc_;

public:
  RSIState()
    : position(6, 0.0)
    , initial_position(6, 0.0)
    , cart_position(6, 0.0)
    , initial_cart_position(6, 0.0)
    , velocity(6, 0.0)
    , effort(6, 0.0)
  {
    xml_doc_.resize(1024);
  }

  RSIState(std::string xml_doc);
  // AIPOS
  std::vector<double> position;
  // ASPos
  std::vector<double> initial_position;
  // RIst
  std::vector<double> cart_position;
  // RSol
  std::vector<double> initial_cart_position;
  // IPOC
  unsigned long long ipoc;
  // Vel
  std::vector<double> velocity;
  // Eff
  std::vector<double> effort;
};

RSIState::RSIState(std::string xml_doc)
  : xml_doc_(xml_doc)
  , position(6, 0.0)
  , initial_position(6, 0.0)
  , cart_position(6, 0.0)
  , initial_cart_position(6, 0.0)
  , velocity(6, 0.0)
  , effort(6, 0.0)
{
  // Parse message from robot
  TiXmlDocument bufferdoc;
  bufferdoc.Parse(xml_doc_.c_str());
  // Get the Rob node:
  TiXmlElement* rob = bufferdoc.FirstChildElement("Rob");
  // Extract axis specific actual position
  TiXmlElement* position_element = rob->FirstChildElement("Pos");
  position_element->Attribute("A1", &position[0]);
  position_element->Attribute("A2", &position[1]);
  position_element->Attribute("A3", &position[2]);
  position_element->Attribute("A4", &position[3]);
  position_element->Attribute("A5", &position[4]);
  position_element->Attribute("A6", &position[5]);
  // Extract velocities computed on the robot side
  TiXmlElement* AIPos_el = rob->FirstChildElement("Vel");
  AIPos_el->Attribute("A1", &velocity[0]);
  AIPos_el->Attribute("A2", &velocity[1]);
  AIPos_el->Attribute("A3", &velocity[2]);
  AIPos_el->Attribute("A4", &velocity[3]);
  AIPos_el->Attribute("A5", &velocity[4]);
  AIPos_el->Attribute("A6", &velocity[5]);
  // Extract gear torque (effort)
  TiXmlElement* Eff_el = rob->FirstChildElement("Eff");
  AIPos_el->Attribute("A1", &effort[0]);
  AIPos_el->Attribute("A2", &effort[1]);
  AIPos_el->Attribute("A3", &effort[2]);
  AIPos_el->Attribute("A4", &effort[3]);
  AIPos_el->Attribute("A5", &effort[4]);
  AIPos_el->Attribute("A6", &effort[5]);
  // Extract axis specific setpoint position
  TiXmlElement* ASPos_el = rob->FirstChildElement("ASPos");
  ASPos_el->Attribute("A1", &initial_position[0]);
  ASPos_el->Attribute("A2", &initial_position[1]);
  ASPos_el->Attribute("A3", &initial_position[2]);
  ASPos_el->Attribute("A4", &initial_position[3]);
  ASPos_el->Attribute("A5", &initial_position[4]);
  ASPos_el->Attribute("A6", &initial_position[5]);
  // Extract cartesian actual position
  TiXmlElement* RIst_el = rob->FirstChildElement("RIst");
  RIst_el->Attribute("X", &cart_position[0]);
  RIst_el->Attribute("Y", &cart_position[1]);
  RIst_el->Attribute("Z", &cart_position[2]);
  RIst_el->Attribute("A", &cart_position[3]);
  RIst_el->Attribute("B", &cart_position[4]);
  RIst_el->Attribute("C", &cart_position[5]);
  // Extract cartesian actual position
  TiXmlElement* RSol_el = rob->FirstChildElement("RSol");
  RSol_el->Attribute("X", &initial_cart_position[0]);
  RSol_el->Attribute("Y", &initial_cart_position[1]);
  RSol_el->Attribute("Z", &initial_cart_position[2]);
  RSol_el->Attribute("A", &initial_cart_position[3]);
  RSol_el->Attribute("B", &initial_cart_position[4]);
  RSol_el->Attribute("C", &initial_cart_position[5]);
  // Get the IPOC timestamp
  TiXmlElement* ipoc_el = rob->FirstChildElement("IPOC");
  ipoc = std::stoull(ipoc_el->FirstChild()->Value());
}

}  // namespace kuka_rsi_hw_interface

#endif
