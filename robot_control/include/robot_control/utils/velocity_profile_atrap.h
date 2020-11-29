/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VELOCITY_PROFILE_ATRAP_H
#define VELOCITY_PROFILE_ATRAP_H

#include "kdl/velocityprofile.hpp"
#include <iostream>

namespace rc {

class VelocityProfile_ATrap : public KDL::VelocityProfile
{
 public:
  VelocityProfile_ATrap(double max_vel = 0, double max_acc = 0, double max_dec = 0);

  virtual void SetProfile(double pos1, double pos2) override;

  virtual void SetProfileDuration(double pos1, double pos2, double duration) override;

  bool setProfileAllDurations(double pos1, double pos2, double duration1, double duration2, double duration3);


  bool setProfileStartVelocity(double pos1, double pos2, double vel1);

  double FirstPhaseDuration() const {return t_a_;}
  double SecondPhaseDuration() const {return t_b_;}
  double ThirdPhaseDuration() const  {return t_c_;}

  bool operator==(const VelocityProfile_ATrap& other) const;

  virtual double Duration() const override;
  virtual double Pos(double time) const override;
  virtual double Vel(double time) const override;
  virtual double Acc(double time) const override;
  virtual void Write(std::ostream& os) const override;
  virtual KDL::VelocityProfile* Clone() const override;

  friend std::ostream &operator<<(std::ostream& os, const VelocityProfile_ATrap& p); //LCOV_EXCL_LINE

  virtual ~VelocityProfile_ATrap();

 private:

  void setEmptyProfile();

 private:

  const double max_vel_;
  const double max_acc_;
  const double max_dec_;
  double start_pos_;
  double end_pos_;

  double start_vel_;

  double a1_,a2_,a3_;
  double b1_,b2_,b3_;
  double c1_,c2_,c3_;

  double t_a_;
  double t_b_;
  double t_c_;
};

std::ostream &operator<<(std::ostream& os, const VelocityProfile_ATrap& p);//LCOV_EXCL_LINE

}

#endif // VELOCITY_PROFILE_ATRAP_H