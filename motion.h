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


void SmoothMove::startMoving( float _x, float _y, float _z ) //
{
   //Serial.println(blockCount);

   if(blockCount == 0)
   {
      X_end = _x; // no queued blocks, so end point equals start point
      Y_end = _y;
      Z_end = _z;

      addLinear_Block(0, _x, _y, _z, 1.0f ); // add dummy block
      addDelay(1);
   }
   else
   {
      // Update first block start position
      float dx, dy, dz;


      int B_0 = currentBlockIndex;       // dummy block
      int B_1 = nextBlockIndex(B_0);     // first real block
      int B_2 = nextBlockIndex(B_1);     // second real block

      //Serial.print(B_0);Serial.print(" ");Serial.print(B_1);Serial.print(" ");Serial.println(B_2);

      moveBuffer[B_0].X_start  = _x; // set start to current position
      moveBuffer[B_0].Y_start  = _y;
      moveBuffer[B_0].Z_start  = _z;

      moveBuffer[B_0].accelTime     = 0;
      moveBuffer[B_0].accelEndPoint = 0.0f;

      moveBuffer[B_0].velTime       = 0;
      moveBuffer[B_0].velEndPoint   = 0.0f;

      moveBuffer[B_0].decelTime     = 0;
      moveBuffer[B_0].decelLength   = 0.0f;

      moveBuffer[B_0].length    = 0.0f;
      moveBuffer[B_0].targetVel = 0.0f;

      moveBuffer[B_0].exactStopDelay = 20000; // 20ms delay on start


      moveBuffer[B_1].X_start  = _x; // set start to current position
      moveBuffer[B_1].Y_start  = _y;
      moveBuffer[B_1].Z_start  = _z;

      if(blockCount > 2)
      {
         dx = moveBuffer[B_2].X_start - moveBuffer[B_1].X_start;
         dy = moveBuffer[B_2].Y_start - moveBuffer[B_1].Y_start;
         dz = moveBuffer[B_2].Z_start - moveBuffer[B_1].Z_start;
      }
      else
      {
         dx = X_end - _x;
         dy = Y_end - _y;
         dz = Z_end - _z;
      }

      moveBuffer[B_1].length = sqrt(dx * dx + dy * dy + dz * dz);

      if(moveBuffer[B_1].length > 0.0001f)
      {
         float inverseLength = 1.0f / moveBuffer[B_1].length;
         moveBuffer[B_1].X_vector = dx * inverseLength;  // line unit vector
         moveBuffer[B_1].Y_vector = dy * inverseLength;
         moveBuffer[B_1].Z_vector = dz * inverseLength;
      }
      else
      {
         moveBuffer[B_1].X_vector = 0.0f;  // line unit vector
         moveBuffer[B_1].Y_vector = 0.0f;
         moveBuffer[B_1].Z_vector = 0.0f;
      }

      constAccelTrajectory();
   }

   totalDistance = 0.0f;

   motionPaused  = false;
   motionStopped = false;
   segmentIndex = 0;
   segmentTime = 0.0f;
   segmentStartTime = startOffset = micros();
}


void SmoothMove::abortMotion() //
{
   motionStopped = true;
}


void SmoothMove::pause() //
{
   motionPaused = true;
}


void SmoothMove::resume() //
{
   // add minimum delay to allow for smooth start
   addDelay(1);
   startExactStop(currentBlockIndex);

   motionPaused = false;
}


void SmoothMove::advancePostion() // this moves forward along the acc/dec trajectory
{
   uint32_t timeNow;

   if( blockCount == 0 || motionStopped || checkExactStop() )
   {
      // no blocks ready to be executed, or on exact stop
      //Serial.print(blockCount);Serial.println("");
      velocityNow = 0.0f;
      segmentStartTime = micros();
   }
   else
   {
      timeNow = micros();
      uint32_t deltaTime = timeNow - segmentStartTime;

      //  check if the next segment has been entered  -- while loop is used to cross multiple zero length segments
      while( deltaTime > segmentTime && blockCount > 0 && !exactStopActive )
      {
         //Serial.println(blockCount);
         //Serial.println(currentBlockIndex);
         //Serial.println(segmentTime);
         //Serial.println(deltaTime);

         segmentStartTime += segmentTime; // advance start time by previous segment time
         timeNow = micros();
         deltaTime = timeNow - segmentStartTime;

         switch( segmentIndex )
         {
            case 2 : // switch to ACCELERATION
               totalDistance += moveBuffer[currentBlockIndex].length;
               removeOldBlock(); // previous block complete, index to next block

               segmentTime    = moveBuffer[currentBlockIndex].accelTime;
               lookAheadTime -= moveBuffer[currentBlockIndex].accelTime; // segment time is removed as soon as the segment is started
               segmentIndex = 0;
               break;

            case 0 : // switch to CONST VELOCITY
               segmentTime    = moveBuffer[currentBlockIndex].velTime;
               lookAheadTime -= moveBuffer[currentBlockIndex].velTime;
               segmentIndex = 1;
               break;

            case 1 : // switch to DECELERATION
               segmentTime    = moveBuffer[currentBlockIndex].decelTime;
               lookAheadTime -= moveBuffer[currentBlockIndex].decelTime;
               segmentIndex = 2;
               break;
         }
      }

      if(exactStopActive)
      {
         blockPosition = 0.0f; // during exact stop, position is forced to the beginning of the next block
         velocityNow   = 0.0f;
      }
      else
      {
         float dt = float(deltaTime) * 0.000001f;
         float dt_Sq;
         int start;

         switch(segmentIndex)  // compute current position in the block
         {
            case 0 : // state: Accel
               dt_Sq = dt * dt;
               start = previousBlockIndex(currentBlockIndex);
               blockPosition = 0.5f * maxAccel * dt_Sq + xVel[start] * dt;
               velocityNow = maxAccel * dt + xVel[start];
               break;

            case 1 : // state: Const Vel
               blockPosition = moveBuffer[currentBlockIndex].targetVel * dt + moveBuffer[currentBlockIndex].accelEndPoint;
               velocityNow = moveBuffer[currentBlockIndex].targetVel;
               break;

            case 2 : // state: Decel
               dt_Sq = dt * dt;
               blockPosition = -0.5f * maxAccel * dt_Sq + moveBuffer[currentBlockIndex].peakVel * dt + moveBuffer[currentBlockIndex].velEndPoint;
               velocityNow = -maxAccel * dt + moveBuffer[currentBlockIndex].peakVel;
               break;
         }
      }
   }

   // debug output
   if(blockCount > 0) //
   {
      //Serial.print(float(timeNow - startOffset) / 1000000.0f, 3); Serial.print("\t");

      //Serial.print(blockPosition, 3); Serial.print("\t");
      //Serial.print(totalDistance + blockPosition, 3); Serial.print("\t");

      //Serial.print(velocityNow, 1);

      //Serial.println("");
   }

}


void SmoothMove::startExactStop(int index)
{
   if( moveBuffer[index].exactStopDelay )
   {
      exactStopStartTime = micros();
      exactStopDelay = moveBuffer[index].exactStopDelay;
      exactStopActive  = true;
   }
   else
   {
      exactStopDelay  = 0;
      exactStopActive = false;
   }
}


bool SmoothMove::checkExactStop()
{
   if(exactStopActive)
   {
      if(micros() - exactStopStartTime  < exactStopDelay)
      {
         return true;
      }
      else
      {
         exactStopActive = false; // past delay time, turn off flag
         segmentStartTime = micros(); // set the start time of the next (current) segment to now
      }
   }
   return false;
}


void SmoothMove::setMaxStartVel(const int & index)
{
   int prevBlock = previousBlockIndex(index);

   if(blockCount > 1 && !moveBuffer[prevBlock].exactStopDelay)
   {
      float prevBlockDist = moveBuffer[prevBlock].length - cornerRoundDist;

      float x1, y1, z1;
      float x2, y2, z2;
      getPos( x1, y1, z1, index, cornerRoundDist );
      getPos( x2, y2, z2, prevBlock, prevBlockDist );

      x1 -= x2; // difference in positions
      y1 -= y2;
      z1 -= z2;
      float pointDistSq = x1 * x1 + y1 * y1 + z1 * z1;

      float radius = sqrt( pointDistSq * cornerRoundDistSq / ( 4.00001f * cornerRoundDistSq - pointDistSq ));

      float vel = sqrt(maxAccel * radius);

      moveBuffer[index].maxStartVel = min( vel, min( moveBuffer[index].targetVel, moveBuffer[prevBlock].targetVel ));

      //Serial.println(radius);
      //Serial.println(moveBuffer[index].maxStartVel);
   }
   else
   {
      moveBuffer[index].maxStartVel = 0.0f;  // first block always starts at zero vel and blocks after an exact stop
   }
}


void SmoothMove::constAccelTrajectory()
{
   int exit  = newBlockIndex;
   int start = exit + 1;
   if(start == bufferCount) start = 0;  // wrap buffer pointer

   xVel[exit]    = 0.0f;   // newest block always ends at zero
   xVel_Sq[exit] = 0.0f;

   //Serial.println("");

   for(int i = blockCount - 1; i > 0 ; i--)
   {
      // iterate through the active blocks backwards (newest to oldest)
      //    On the first pass, only border velocities are changed
      //    These can only be made slower, never faster

      xVel[start] = moveBuffer[exit].maxStartVel;
      xVel_Sq[start] = xVel[start] * xVel[start];

      float distToDeltaVel = (xVel_Sq[start] - xVel_Sq[exit]) * accelInverseHalf;

      if(distToDeltaVel > moveBuffer[exit].length)
      {
         // not enough room to decel from startVel to endVel
         xVel_Sq[start] = xVel_Sq[exit] + accelDouble * moveBuffer[exit].length; // set startVel lower
         xVel[start]    = sqrt(xVel_Sq[start]);
         //Serial.print("1");
      }
      else if(distToDeltaVel < -moveBuffer[exit].length)
      {
         // not enough room to accel from startVel to endVel
         xVel_Sq[exit] = xVel_Sq[start] + accelDouble * moveBuffer[exit].length; // set exitVel lower
         xVel[exit]    = sqrt(xVel_Sq[exit]);
         //Serial.print("2");
      }
      else
      {
         //Serial.print("3");
      }

      // if neither of the above cases are trigered, then the start and end velocities are "in reach" of each other

      //Serial.print(start);Serial.print("\t");Serial.println(exit);
      //Serial.print("<=B= ");
      //displayBlock(exit);

      // increment pointers
      exit++;
      start++;

      if(exit  == bufferCount) exit  = 0;  // wrap buffer pointer
      if(start == bufferCount) start = 0;  // wrap buffer pointer
   }

   // Reminder: the current (oldest) block should not be adjusted

   for(int i = blockCount; i > 0 ; i--)
   {
      // iterate forward
      //    check and set boundary velocities
      //    set position and time variables

      int index = exit;  // for reading clarity, index points to current block

      float distToDeltaVel = (xVel_Sq[exit] - xVel_Sq[start]) * accelInverseHalf;

      if(distToDeltaVel > moveBuffer[index].length)
      {
         // not enough room to accel from startVel to endVel

         xVel_Sq[exit] = xVel_Sq[start] + accelDouble * moveBuffer[index].length; // set exitVel lower
         xVel[exit]    = sqrt(xVel_Sq[exit]);

         moveBuffer[index].peakVel = xVel[start];  // shouldn't need this...

         moveBuffer[index].accelTime     = uint32_t(( xVel[exit] - xVel[start] ) * accelInverse * 1000000.0f);
         moveBuffer[index].accelEndPoint = moveBuffer[index].length;

         moveBuffer[index].velTime       = 0;
         moveBuffer[index].velEndPoint   = moveBuffer[index].accelEndPoint;

         moveBuffer[index].decelTime     = 0;
         moveBuffer[index].decelLength   = 0.0f;

         //Serial.print("1");

      }
      else
      {
         // Compute accel and decel

         moveBuffer[index].decelLength   = ( moveBuffer[index].targetVel_Sq - xVel_Sq[exit]  ) * accelInverseHalf;

         moveBuffer[index].accelEndPoint = ( moveBuffer[index].targetVel_Sq - xVel_Sq[start] ) * accelInverseHalf;

         float constVelLength = moveBuffer[index].length - moveBuffer[index].decelLength - moveBuffer[index].accelEndPoint;

         // Check for enough room to execute both
         if(constVelLength > 0.0f)  // accel should end before const vel
         {
            // enough room for both accel to and decel from targetVel

            moveBuffer[index].peakVel = moveBuffer[index].targetVel;

            moveBuffer[index].velEndPoint = constVelLength + moveBuffer[index].accelEndPoint;

            moveBuffer[index].accelTime = uint32_t(( moveBuffer[index].targetVel - xVel[start] ) * accelInverse * 1000000.0f);
            moveBuffer[index].velTime   = uint32_t(( moveBuffer[index].velEndPoint - moveBuffer[index].accelEndPoint) / moveBuffer[index].targetVel * 1000000.0f);
            moveBuffer[index].decelTime = uint32_t(( moveBuffer[index].targetVel - xVel[exit]  ) * accelInverse * 1000000.0f);

            //Serial.print("2");
         }
         else
         {
            // peaked acceleration, targetVel not reached

            float halfExcessLength = constVelLength * 0.5f;  // negative

            moveBuffer[index].accelEndPoint += halfExcessLength;
            moveBuffer[index].velEndPoint    = moveBuffer[index].accelEndPoint; // zero length
            moveBuffer[index].decelLength   += halfExcessLength;

            moveBuffer[index].peakVel = sqrt( xVel_Sq[start] + accelDouble * moveBuffer[index].accelEndPoint );

            moveBuffer[index].accelTime = uint32_t(( moveBuffer[index].peakVel - xVel[start]) * accelInverse * 1000000.0f);
            moveBuffer[index].velTime   = 0;
            moveBuffer[index].decelTime = uint32_t(( moveBuffer[index].peakVel - xVel[exit] ) * accelInverse * 1000000.0f);

            //Serial.print("3");
         }
      }

      //Serial.print("=F=> ");
      //displayBlock(exit);

      // decrement pointers
      exit--;
      start--;

      if(exit  < 0) exit  = bufferCount - 1;  // wrap buffer pointer
      if(start < 0) start = bufferCount - 1;  // wrap buffer pointer
   }

}


void SmoothMove::getTargetLocation(float & x, float & y, float & z)
{

   if(blockCount == 0) // if no blocks are queued up, return current end point
   {
      x = X_end;
      y = Y_end;
      z = Z_end;
      return;
   }

   // symetric smoothing
   float smoothingRadius = min(cornerRoundDist, velocityNow * velocityNow * accelInverseHalf);

   if(smoothingRadius < 0.010f)
   {
      getPos( x, y, z, currentBlockIndex, blockPosition); // return position without smoothing if velocity is very low
      return;
   }

   float smoothingPosStart = blockPosition - smoothingRadius;
   float smoothingPosEnd   = blockPosition + smoothingRadius;
   float x2, y2, z2;

   // get trailing smoothing position
   if(smoothingPosStart >= 0.0f)
   {
      getPos( x, y, z, currentBlockIndex, smoothingPosStart); // smoothing start position
   }
   else // start is in a previous block (velocity is high enough that there must be a previous block)
   {
      int smoothingIndexStart = previousBlockIndex(currentBlockIndex);
      float position = max(0.0f, moveBuffer[smoothingIndexStart].length + smoothingPosStart);
      getPos( x, y, z, smoothingIndexStart, position); // smoothing start position
   }

   // get leading smoothing position
   if(smoothingPosEnd <= moveBuffer[currentBlockIndex].length || blockCount < 2)
   {
      getPos( x2, y2, z2, currentBlockIndex, smoothingPosEnd); // smoothing end position is in this block
   }
   else // end position projects into the next block 
   {
      int smoothingIndexEnd = nextBlockIndex(currentBlockIndex);
      float position = min( moveBuffer[smoothingIndexEnd].length , smoothingPosEnd - moveBuffer[currentBlockIndex].length ); // don't go beyond the end of the next block
      getPos( x2, y2, z2, smoothingIndexEnd, position); // smoothing end position is in the next block
   }

   x = ( x + x2 ) * 0.5f; // average the two smoothing points
   y = ( y + y2 ) * 0.5f;
   z = ( z + z2 ) * 0.5f;



   /*
   // forward smoothing
   getPos( x, y, z, currentBlockIndex, blockPosition); // position of the primary (trailing) smoothing point

   //return;
   
   int index = currentBlockIndex;
   float x2, y2, z2;
   float lookAheadPos;  // the position in the current block that the look ahead points to

   if( exactStopActive )
   {
      uint32_t timeToEnd = exactStopStartTime + exactStopDelay - micros();
      if( timeToEnd < exactStopSmoothingDelay )  // allow early release of look ahead for smoothing
      {
         float t = float( exactStopSmoothingDelay - timeToEnd ) * 0.000001f;
         lookAheadPos = 0.5f * maxAccel * t * t;
      }
      else
      {
         lookAheadPos = 0.0f;
      }
   }
   else
   {
      lookAheadPos = 2.0f * cornerRoundDist + blockPosition;  // normal look ahead
   }

   int thisBlockCount = blockCount;

   while( lookAheadPos > moveBuffer[index].length && moveBuffer[index].exactStopDelay == 0 && thisBlockCount > 1 )
   {
      lookAheadPos -= moveBuffer[index].length;
      index = nextBlockIndex(index);
      thisBlockCount--;
      //Serial.print("*");
   }

   lookAheadPos = min(lookAheadPos, moveBuffer[index].length); // don't look past the end of the next/last segment

   //Serial.print(lookAheadPos);Serial.print("\t");

   getPos( x2, y2, z2, index, lookAheadPos); // position of look ahead (leading) smoothing point

   x = ( x + x2 ) * 0.5f; // average the two smoothing points
   y = ( y + y2 ) * 0.5f;
   z = ( z + z2 ) * 0.5f;
   */

}


void SmoothMove::getPos(float & x, float & y, float & z, const int & index, const float & position)  // maps from accel trajectory to 3D space
{
   float angle;

   switch(moveBuffer[index].moveType)
   {
      case Rapid  :
      case Linear :
         x = moveBuffer[index].X_vector * position + moveBuffer[index].X_start;
         y = moveBuffer[index].Y_vector * position + moveBuffer[index].Y_start;
         z = moveBuffer[index].Z_vector * position + moveBuffer[index].Z_start;
         break;

      case ArcCW  :
      case ArcCCW :
         if(moveBuffer[index].moveType == ArcCW)
         {
            angle = -position / moveBuffer[index].radius;
         }
         else
         {
            angle = position / moveBuffer[index].radius;
         }
         angle += moveBuffer[index].startAngle;
         x = moveBuffer[index].X_vector + moveBuffer[index].radius * cos(angle);
         y = moveBuffer[index].Y_vector + moveBuffer[index].radius * sin(angle);
         z = moveBuffer[index].Z_vector * position + moveBuffer[index].Z_start;
         break;
   }
}


uint32_t SmoothMove::getExtrudeLocationSteps()
{
   return moveBuffer[previousBlockIndex(currentBlockIndex)].extrudePosition + uint32_t( moveBuffer[currentBlockIndex].extrudeScaleFactor * blockPosition );
}


float SmoothMove::setMotionRateOverride(  float scale )
{
   motionFeedOverride = constrain( scale, 0.1f, 2.0f );
   return motionFeedOverride;
}


float SmoothMove::setExtrudeRateOverride( float scale )
{
   extrudeRateOverride = constrain( scale, 0.1f, 2.0f );
   return extrudeRateOverride;
}

