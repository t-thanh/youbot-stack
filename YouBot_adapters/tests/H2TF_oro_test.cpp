/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:21:07 CET 2010  HelloRobot.cpp

                        HelloRobot.cpp -  description
                           -------------------
    begin                : Tue November 16 2010
    copyright            : (C) 2010 Ruben Smits
    email                : first.last@mech.kuleuven.be

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/


#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <rtt/Component.hpp>
#include "LinearMath/btMatrix3x3.h"

using namespace RTT;
typedef std_msgs::Float64 XXDouble ;
typedef std_msgs::Float64MultiArray XXArray;

class H_matrix_publisher : public RTT::TaskContext{
private:  
  XXDouble x,y,z,yaw,pitch,roll;

  OutputPort<XXArray> Hmatrix;

  
public:
  H_matrix_publisher(const std::string& name):
    TaskContext(name),
    Hmatrix("Hmatrix")
  {
    
    this->addPort(Hmatrix);

    this->addProperty("x",x);
    this->addProperty("y",y);
    this->addProperty("z",z);
    
    this->addProperty("yaw",yaw);
    this->addProperty("pitch",pitch);
    this->addProperty("roll",roll);
   }
  ~H_matrix_publisher(){}
private:
  std::vector<double> XYZYRPtoHMatrix(double x, double y,double z,double yaw,double roll, double pitch)
  {
    btMatrix3x3 rotMatrix;
    rotMatrix.setEulerYPR(btScalar(yaw), btScalar(pitch), btScalar(roll));
    std::vector<double> temp;
    temp.resize(16);
    for(int j=0;j<3;j++)
      for(int i=0;i<3;i++)
        temp[i+j*4]=rotMatrix[i][j];
    temp[3]=x;
    temp[7]=y;
    temp[11]=z;
    temp[15]=1;
    return temp;

   }
  void updateHook(){
    std_msgs::Float64MultiArray msg;
      msg.data= XYZYRPtoHMatrix(x.data, y.data, z.data, yaw.data, roll.data,  pitch.data);

    Hmatrix.write(msg);
  }
};
ORO_CREATE_COMPONENT(H_matrix_publisher)
