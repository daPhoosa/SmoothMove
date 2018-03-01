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


void SmoothMove::startMoving() //
{
   // SetPosition(...) must be used before this is run

   blockCount    = 0; // "forget" all previous blocks
   lookAheadTime = 0;
   segmentIndex  = 0;
   segmentTime   = 0;

   addRapid_Block( X_end, Y_end, Z_end ); // add dummy block at current position, zero length
   addDelay( 100 );   // give time for more blocks to be added to buffer before starting to move

   currentBlockIndex = newBlockIndex; // execute the block that was just added

   moveBuffer[currentBlockIndex].accelTime = 0; // manually set these to zero since trajectory planning doesn't touch the curent block
   moveBuffer[currentBlockIndex].velTime   = 0;
   moveBuffer[currentBlockIndex].decelTime = 0;

   motionStopped    = false;
   segmentStartTime = micros();
}


void SmoothMove::abortMotion() // all blocks in queue will be lost on restart
{
   motionStopped = true;
}


void SmoothMove::advancePostion() // this moves forward along the acc/dec trajectory
{

   if( blockCount == 0 || motionStopped )
   {
      // no blocks ready to be executed
      velocityNow = 0.0f;
      segmentStartTime = micros();
   }
   else
   {
      uint32_t deltaTime = micros() - segmentStartTime;

      //  check if the next segment has been entered  -- while loop is used to cross multiple zero length segments
      while( deltaTime > segmentTime )
      {

         segmentStartTime += segmentTime; // advance start time by previous segment time
         deltaTime -= segmentTime;

         switch( segmentIndex )
         {
            case 4 : // switch to ACCELERATION
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
               segmentIndex = 2; // move to next block
               break;

            case 2 : // switch to DWELL
               segmentTime  = moveBuffer[currentBlockIndex].dwell;
               segmentIndex = 3;
               break;

            case 3 : // wait for next block
               if( blockCount > 1 && !moveBuffer[currentBlockIndex].staticExtrude )
               {
                  segmentIndex = 4; // only advance to next block if one exists
                  segmentTime  = 0;
               }
               else
               {
                  segmentTime = 10000UL; // force 10ms of dwell before checking again
               }
               break;
         }
      }

      float dt, dt_Sq;
      int start;

      switch(segmentIndex)  // compute current position in the block
      {
         case 0 : // state: Accel
            dt = float(deltaTime) * 0.000001f;
            dt_Sq = dt * dt;
            start = previousBlockIndex(currentBlockIndex);
            blockPosition = 0.5f * moveBuffer[currentBlockIndex].maxAccel * dt_Sq + xVel[start] * dt;
            velocityNow = moveBuffer[currentBlockIndex].maxAccel * dt + xVel[start];
            break;

         case 1 : // state: Const Vel
            dt = float(deltaTime) * 0.000001f;
            blockPosition = moveBuffer[currentBlockIndex].targetVel * dt + moveBuffer[currentBlockIndex].accelEndPoint;
            velocityNow = moveBuffer[currentBlockIndex].targetVel;
            break;

         case 2 : // state: Decel
            dt = float(deltaTime) * 0.000001f;
            dt_Sq = dt * dt;
            blockPosition = -0.5f * moveBuffer[currentBlockIndex].maxAccel * dt_Sq + moveBuffer[currentBlockIndex].peakVel * dt + moveBuffer[currentBlockIndex].velEndPoint;
            velocityNow = -moveBuffer[currentBlockIndex].maxAccel * dt + moveBuffer[currentBlockIndex].peakVel;
            break;

         case 3 : // state: Dwell
         case 4 : // state: Wait for next block
            blockPosition = moveBuffer[currentBlockIndex].length; // stop at end of current block
            velocityNow = 0.0f;
            break;
      }
   }
}


void SmoothMove::setMaxStartVel(const int & index)  // Junction Velocity
{
   int prevBlock = previousBlockIndex(index);

   if( blockCount > 1 && !moveBuffer[prevBlock].dwell )
   {
      float prevBlockDist = moveBuffer[prevBlock].length - cornerRoundDist;

      if( prevBlockDist < 0.0f && blockCount > 2 ) // look "past" very short blocks 
      {
         prevBlock = previousBlockIndex(prevBlock);
         prevBlockDist = moveBuffer[prevBlock].length + prevBlockDist;
         SERIAL_PORT.println("Very short block bridged for junction speed calc");
      }

      float x1, y1, z1;
      float x2, y2, z2;
      getPos( x1, y1, z1, index, cornerRoundDist );
      getPos( x2, y2, z2, prevBlock, prevBlockDist );
      float maxAccel = min( moveBuffer[index].maxAccel, moveBuffer[prevBlock].maxAccel ); // use lower acceleration rate

      x1 -= x2; // difference in positions
      y1 -= y2;
      z1 -= z2;
      float pointDistSq = x1 * x1 + y1 * y1 + z1 * z1;

      float radius = sqrtf( pointDistSq * cornerRoundDistSq / ( 4.00001f * cornerRoundDistSq - pointDistSq ));

      float junctionVelSq = maxAccel * radius;

      float minBlockVel = min( moveBuffer[index].targetVel, moveBuffer[prevBlock].targetVel );

      if( junctionVelSq < minBlockVel * minBlockVel )
      {
         moveBuffer[index].maxStartVel = sqrtf(junctionVelSq);
         moveBuffer[index].fastJunction = false;
      }
      else
      {
         moveBuffer[index].maxStartVel = minBlockVel;
         moveBuffer[index].fastJunction = true;
      }
   }
   else
   {
      moveBuffer[index].maxStartVel = 0.0f;  // first block always starts at zero vel and blocks after an exact stop
      moveBuffer[index].fastJunction = true; // no point in smoothing if coming from a dead stop
   }
}


void SmoothMove::constAccelTrajectory()
{
   int exit  = newBlockIndex;
   int start = previousBlockIndex(exit);

   xVel[exit]    = 0.0f;   // newest block always ends at zero
   xVel_Sq[exit] = 0.0f;

   for( int i = blockCount - 2; i > 0 ; i-- )
   {

      // iterate through the active blocks backwards (newest to oldest)
      //    On the first pass, only border velocities are changed
      //    These can only be made slower, never faster
      //    Reminder: the current (oldest) block should not be adjusted

      xVel[start] = moveBuffer[exit].maxStartVel;
      xVel_Sq[start] = xVel[start] * xVel[start];

      float distToDeltaVel = (xVel_Sq[start] - xVel_Sq[exit]) * moveBuffer[exit].accelInverseHalf;

      if(distToDeltaVel > moveBuffer[exit].length)
      {
         // not enough room to decel from startVel to endVel
         xVel_Sq[start] = xVel_Sq[exit] + moveBuffer[exit].accelDouble * moveBuffer[exit].length; // set startVel lower
         xVel[start]    = sqrtf(xVel_Sq[start]);
      }
      else if(distToDeltaVel < -moveBuffer[exit].length)
      {
         // not enough room to accel from startVel to endVel
         xVel_Sq[exit] = xVel_Sq[start] + moveBuffer[exit].accelDouble * moveBuffer[exit].length; // set exitVel lower
         xVel[exit]    = sqrtf(xVel_Sq[exit]);
      }

      // move backwards through block queue
      exit  = previousBlockIndex(exit);
      start = previousBlockIndex(start);
   }

   lookAheadTime = 0; // zero before re-summing total
   if( segmentIndex < 1 ) lookAheadTime += moveBuffer[currentBlockIndex].velTime;    // include const vel time
   if( segmentIndex < 2 ) lookAheadTime += moveBuffer[currentBlockIndex].decelTime;  // include decel time

   for( int i = blockCount - 1; i > 0 ; i-- )
   {
      //Serial.print( exit );Serial.print(" ");
      // iterate forward
      //    check and set boundary velocities
      //    set position and time variables

      int index = exit;  // for reading clarity, index points to current block

      float distToDeltaVel = (xVel_Sq[exit] - xVel_Sq[start]) * moveBuffer[index].accelInverseHalf;

      if(distToDeltaVel > moveBuffer[index].length)
      {
         // not enough room to accel from startVel to endVel

         xVel_Sq[exit] = xVel_Sq[start] + moveBuffer[index].accelDouble * moveBuffer[index].length; // set exitVel lower
         xVel[exit]    = sqrtf(xVel_Sq[exit]);

         moveBuffer[index].peakVel = xVel[start];  // shouldn't need this...

         moveBuffer[index].accelTime     = uint32_t(( xVel[exit] - xVel[start] ) * moveBuffer[index].accelInverse * 1000000.0f);
         moveBuffer[index].accelEndPoint = moveBuffer[index].length;

         moveBuffer[index].velTime       = 0;
         moveBuffer[index].velEndPoint   = moveBuffer[index].accelEndPoint;

         moveBuffer[index].decelTime     = 0;
         moveBuffer[index].decelLength   = 0.0f;
      }
      else
      {
         // Compute accel and decel

         moveBuffer[index].decelLength   = ( moveBuffer[index].targetVel_Sq - xVel_Sq[exit]  ) * moveBuffer[index].accelInverseHalf;

         moveBuffer[index].accelEndPoint = ( moveBuffer[index].targetVel_Sq - xVel_Sq[start] ) * moveBuffer[index].accelInverseHalf;

         float constVelLength = moveBuffer[index].length - moveBuffer[index].decelLength - moveBuffer[index].accelEndPoint;

         // Check for enough room to execute both
         if(constVelLength > 0.0f)  // accel should end before const vel
         {
            // enough room for both accel to and decel from targetVel

            moveBuffer[index].peakVel = moveBuffer[index].targetVel;

            moveBuffer[index].velEndPoint = constVelLength + moveBuffer[index].accelEndPoint;

            moveBuffer[index].accelTime = uint32_t(( moveBuffer[index].targetVel - xVel[start] ) * moveBuffer[index].accelInverse * 1000000.0f);
            moveBuffer[index].velTime   = uint32_t(( moveBuffer[index].velEndPoint - moveBuffer[index].accelEndPoint) / moveBuffer[index].targetVel * 1000000.0f);
            moveBuffer[index].decelTime = uint32_t(( moveBuffer[index].targetVel - xVel[exit]  ) * moveBuffer[index].accelInverse * 1000000.0f);
         }
         else
         {
            // peaked acceleration, targetVel not reached

            float halfExcessLength = constVelLength * 0.5f;  // negative

            moveBuffer[index].accelEndPoint += halfExcessLength;
            moveBuffer[index].velEndPoint    = moveBuffer[index].accelEndPoint; // zero length
            moveBuffer[index].decelLength   += halfExcessLength;

            moveBuffer[index].peakVel   = sqrtf( xVel_Sq[start] + moveBuffer[index].accelDouble * moveBuffer[index].accelEndPoint );

            moveBuffer[index].accelTime = uint32_t(( moveBuffer[index].peakVel - xVel[start]) * moveBuffer[index].accelInverse * 1000000.0f);
            moveBuffer[index].velTime   = 0;
            moveBuffer[index].decelTime = uint32_t(( moveBuffer[index].peakVel - xVel[exit] ) * moveBuffer[index].accelInverse * 1000000.0f);
         }
      }

      lookAheadTime += moveBuffer[index].accelTime + moveBuffer[index].velTime + moveBuffer[index].decelTime;

      // move forward in block queue
      exit  = nextBlockIndex(exit);
      start = nextBlockIndex(start);
   }
}


void SmoothMove::getTargetLocation(float & x, float & y, float & z) // call to get current cartesian position
{
   /*
      TIME TESTS
                     Teensy 3.2     Teensy 3.5
      Min               7us            2us         (middle of a line segment when no smoothing is happening)

      Avg (linear)      13us           2us         (only linear moves, geometry dependent)
      Max (linear)      34us           6us         (time during line to line smoothing)

      Avg (mixed)       30us           15us        (mixed linear and arc moves, geometry dependent)
      Max (Arc)         300us          200us       (time during arc to arc transitions)

   */

   advancePostion();

   if(blockCount == 0) // if no blocks are queued up, return current end point
   {
      x = X_end;
      y = Y_end;
      z = Z_end;
      return;
   }

   // symetric smoothing
   float smoothingRadius = min( cornerRoundDistHalf, velocityNow * velocityNow * moveBuffer[currentBlockIndex].accelInverseHalf );

   if( pathSmoothingOff ||          // smoothing turned off
       smoothingRadius < 0.003f )   // return current position without smoothing if velocity is very low
   {
      getPos( x, y, z, currentBlockIndex, blockPosition ); // get current position
      return;
   }

   float smoothingPosStart = blockPosition - smoothingRadius;
   float smoothingPosEnd   = blockPosition + smoothingRadius;

   bool startInBlock, endInBlock;
   int smoothingIndexStart, smoothingIndexEnd;

   if( smoothingPosStart >= 0.0f ) // check if start point is in current block
   {
      startInBlock = true;
      smoothingIndexStart = currentBlockIndex;
   }
   else
   {
      startInBlock = false;
      smoothingIndexStart = previousBlockIndex(currentBlockIndex);
   }

   if( smoothingPosEnd <= moveBuffer[currentBlockIndex].length || blockCount < 2 ) // check if end point is in current block
   {
      endInBlock = true;
      smoothingIndexEnd = currentBlockIndex;
   }
   else
   {
      endInBlock   = false;
      smoothingIndexEnd = nextBlockIndex(currentBlockIndex);
   }

   if( ( startInBlock && endInBlock ) ||                 // do not smooth if "far" from junction (both points lie in the current block)
         moveBuffer[smoothingIndexEnd].fastJunction )    // do not smooth if junction does not force deceleration
   {
      getPos( x, y, z, currentBlockIndex, blockPosition ); // get current position
      return;
   }

   float x1, y1, z1;
   float x2, y2, z2;

   // get trailing smoothing position
   if( startInBlock )
   {
      getPos( x1, y1, z1, smoothingIndexStart, smoothingPosStart); // smoothing start position
   }
   else // start is in a previous block (velocity is high enough that there must be a previous block)
   {
      //float position = max(0.0f, moveBuffer[smoothingIndexStart].length + smoothingPosStart);
      float position = moveBuffer[smoothingIndexStart].length + smoothingPosStart;
      getPos( x1, y1, z1, smoothingIndexStart, position); // smoothing start position
   }

   // get leading smoothing position
   if( endInBlock )
   {
      getPos( x2, y2, z2, smoothingIndexEnd, smoothingPosEnd); // smoothing end position is in this block
   }
   else // end position projects into the next block
   {
      //float position = min( moveBuffer[smoothingIndexEnd].length , smoothingPosEnd - moveBuffer[currentBlockIndex].length ); // don't go beyond the end of the next block
      float position = smoothingPosEnd - moveBuffer[currentBlockIndex].length;
      getPos( x2, y2, z2, smoothingIndexEnd, position); // smoothing end position is in the next block
   }

   x = ( x1 + x2 ) * 0.5f; // average two smoothing points
   y = ( y1 + y2 ) * 0.5f;
   z = ( z1 + z2 ) * 0.5f;

   // two point smoothing creates a straight "chamfer" across the corner
}


void SmoothMove::getPos(float & x, float & y, float & z, const int & index, const float & position)  // maps from accel trajectory to 3D space
{
   float angle;

   switch(moveBuffer[index].moveType)
   {
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
         x = moveBuffer[index].X_vector + moveBuffer[index].radius * cosf(angle);
         y = moveBuffer[index].Y_vector + moveBuffer[index].radius * sinf(angle);
         z = moveBuffer[index].Z_vector * position + moveBuffer[index].Z_start;
         break;
   }
}


float SmoothMove::getExtrudeLocationMM()
{
   static uint32_t lastTime;

   uint32_t timeNow = micros();
   float  deltaTime = float(timeNow - lastTime) * (1.0f / 1000000.0f);
   lastTime = timeNow;

   if( moveBuffer[currentBlockIndex].staticExtrude ) // extrude with no head movement
   {
      static float extrudePos = 0.0f;
      static float velocity = 0.0f;
      bool endFound = false;

      if( moveBuffer[currentBlockIndex].extrudeDist > 0.0f )
      {
         float decelVel = min( sqrtf( 2.0f * (moveBuffer[currentBlockIndex].extrudeDist - extrudePos) * extrudeAccel ), extrudeVel );
         velocity = min( velocity + extrudeAccel * deltaTime, decelVel );
         //Serial.println(velocity);
         extrudePos += velocity * deltaTime;
         if( extrudePos > moveBuffer[currentBlockIndex].extrudeDist ) endFound = true;
      }
      else  // negative extrude
      {
         float decelVel = min( sqrtf( 2.0f * (extrudePos - moveBuffer[currentBlockIndex].extrudeDist) * extrudeAccel ), extrudeVel );
         //Serial.println(decelVel);
         velocity = min( velocity + extrudeAccel * deltaTime, decelVel );
         //Serial.println(velocity);
         extrudePos -= velocity * deltaTime;
         if( extrudePos < moveBuffer[currentBlockIndex].extrudeDist ) endFound = true;
      }

      //Serial.println(extrudePos);

      if( endFound ) // end of extrude reached
      {
         extrudePos = 0.0f;
         velocity = 0.0f;
         moveBuffer[currentBlockIndex].staticExtrude = false;

         extrudeMachPos += moveBuffer[currentBlockIndex].extrudeDist; // force final position to be exact
         return extrudeMachPos;
      }
      return extrudeMachPos + extrudePos; // middle of extrude
   }
   else  // extrude while moving
   {
      return moveBuffer[currentBlockIndex].extrudeScaleFactor * blockPosition + extrudeMachPos;
   }
}
