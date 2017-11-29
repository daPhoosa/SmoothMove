/*
   SmoothMove
   Phillip Schmidt
   v0.1

         This program is free software: you can redistribute it and/or modify
         it under the terms of the GNU General Public License as published by
         the Free Software Foundation, either version 3 of the License, or
         (at your option) any later version.

         This program is distributed in the hope that it will be useful,
         but WITHOUT ANY WARRANTY; without even the implied warranty of
         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
         GNU General Public License for more details.

         You should have received a copy of the GNU General Public License
         along with this program.  If not, see <http://www.gnu.org/licenses/>
*/


#include "SmoothMove.h"
#include "motion.h"
#include "blockBuffer.h"


SmoothMove::SmoothMove( float _accel, float _velMax, float _cornerRounding )
{
   currentBlockIndex = 0;
   newBlockIndex = 1;
   blockCount    = 0; // dummy first block

   maxAccel     = _accel;
   accelInverse = 1.0f / _accel;
   accelInverseHalf = 0.5f * accelInverse;
   accelDouble  = 2.0f * _accel;

   maxVel       = abs( _velMax );

   cornerRoundDist = max( _cornerRounding, 0.001f );
   cornerRoundDistSq = cornerRoundDist * cornerRoundDist;
   cornerRoundDistHalf = cornerRoundDist * 0.5f;

   motionFeedOverride  = 1.0f;
   extrudeRateOverride = 1.0f;

   motionStopped = true;
}


SmoothMove::~SmoothMove()
{

}












// End