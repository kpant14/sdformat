/*
 * Copyright 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "sdf/NavSatMultipath.hh"

using namespace sdf;
using namespace gz;

/// \brief Private navsat data.
class sdf::NavSatMultipath::Implementation
{
  /// \brief Noise values for the horizontal positioning sensor
  public: Noise horizontalPositionNoise;

  /// \brief Noise values for the vertical positioning sensor
  public: Noise verticalPositionNoise;

  /// \brief Noise values for the horizontal velocity sensor
  public: Noise horizontalVelocityNoise;

  /// \brief Noise values for the verical velocity sensor
  public: Noise verticalVelocityNoise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

//////////////////////////////////////////////////
NavSatMultipath::NavSatMultipath()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Errors NavSatMultipath::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load NAVSAT, but the provided SDF "
        "element is null."});
    return errors;
  }
  // Check that the provided SDF element is a <navsat> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "navsat_multipath" && _sdf->GetName() != "gps")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load NAVSAT, but the provided SDF element is "
        "not a <navsat>."});
    return errors;
  }

  // Load navsat sensor properties
  if (_sdf->HasElement("position_sensing"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("position_sensing");
    if (elem->HasElement("horizontal"))
    {
      sdf::ElementPtr horiz = elem->GetElement("horizontal");
      if (horiz->HasElement("noise"))
      {
        this->dataPtr->horizontalPositionNoise.Load(horiz->GetElement("noise"));
      }
    }
    if (elem->HasElement("vertical"))
    {
      sdf::ElementPtr vert = elem->GetElement("vertical");
      if (vert->HasElement("noise"))
      {
        this->dataPtr->verticalPositionNoise.Load(vert->GetElement("noise"));
      }
    }
  }
  if (_sdf->HasElement("velocity_sensing"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("velocity_sensing");
    if (elem->HasElement("horizontal"))
    {
      sdf::ElementPtr horiz = elem->GetElement("horizontal");
      if (horiz->HasElement("noise"))
      {
        this->dataPtr->horizontalVelocityNoise.Load(horiz->GetElement("noise"));
      }
    }
    if (elem->HasElement("vertical"))
    {
      sdf::ElementPtr vert = elem->GetElement("vertical");
      if (vert->HasElement("noise"))
      {
        this->dataPtr->verticalVelocityNoise.Load(vert->GetElement("noise"));
      }
    }
  }

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr NavSatMultipath::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
const Noise &NavSatMultipath::HorizontalPositionNoise() const
{
  return this->dataPtr->horizontalPositionNoise;
}

//////////////////////////////////////////////////
void NavSatMultipath::SetHorizontalPositionNoise(const Noise &_noise)
{
  this->dataPtr->horizontalPositionNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &NavSatMultipath::HorizontalVelocityNoise() const
{
  return this->dataPtr->horizontalVelocityNoise;
}

//////////////////////////////////////////////////
void NavSatMultipath::SetHorizontalVelocityNoise(const Noise &_noise)
{
  this->dataPtr->horizontalVelocityNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &NavSatMultipath::VerticalPositionNoise() const
{
  return this->dataPtr->verticalPositionNoise;
}

//////////////////////////////////////////////////
void NavSatMultipath::SetVerticalPositionNoise(const Noise &_noise)
{
  this->dataPtr->verticalPositionNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &NavSatMultipath::VerticalVelocityNoise() const
{
  return this->dataPtr->verticalVelocityNoise;
}

//////////////////////////////////////////////////
void NavSatMultipath::SetVerticalVelocityNoise(const Noise &_noise)
{
  this->dataPtr->verticalVelocityNoise = _noise;
}

//////////////////////////////////////////////////
bool NavSatMultipath::operator==(const NavSatMultipath &_navsat_multipath) const
{
  if (this->dataPtr->verticalPositionNoise != _navsat_multipath.VerticalPositionNoise())
    return false;
  if (this->dataPtr->horizontalPositionNoise !=
      _navsat_multipath.HorizontalPositionNoise())
    return false;
  if (this->dataPtr->verticalVelocityNoise != _navsat_multipath.VerticalVelocityNoise())
    return false;
  if (this->dataPtr->horizontalVelocityNoise !=
      _navsat_multipath.HorizontalVelocityNoise())
    return false;

  return true;
}

//////////////////////////////////////////////////
bool NavSatMultipath::operator!=(const NavSatMultipath &_navsat_multipath) const
{
  return !(*this == _navsat_multipath);
}
