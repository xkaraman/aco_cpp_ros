/* iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. 

 * Website: http://www.iroboapp.org/index.php?title=IPath
 * Contact: 
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include"pheromone.h"
using namespace std;


pheromone::pheromone(void)
{
}
pheromone::pheromone(int cellS, int cellG,  long double PHEROMONE)
{
  fromCell=cellS;
  toCell=cellG;
  Pheromone=PHEROMONE;
}
pheromone::~pheromone(void)
{
}
/**********************************************************/
//Function: Mutators
/**********************************************************/
void pheromone::setFromCell(int cell)
{
  fromCell=cell;
}
void pheromone::setToCell(int cell)
{
  toCell=cell;
}
void pheromone::setPheromone( long double PHEROMONE)
{
  Pheromone=PHEROMONE;
}
/**********************************************************/
//Function: Accessors
/**********************************************************/
int pheromone::getFromCell()
{
  return fromCell;
}
int pheromone::getToCell()
{
  return toCell;
}
 long double pheromone::getPheromone()
{
  return Pheromone;
}
