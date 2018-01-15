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


bool SmoothMove::bufferVacancy() // always call this to check for room before adding a new block
{
   //if( motionStopped ) return false; // Don't accept blocks when not running

   if( blockCount < 3 ) return true; // try to maintain 3 block look ahead minimum
   
   if( blockCount == bufferCount - 1 ) return false; // don't exceed max buffer size

   uint32_t lookAheadTimeMin = uint32_t(( velocityNow * accelInverse + 0.050f ) * 1000000.0f ); // time to decel to zero plus 50ms in [us]
   if( lookAheadTime < lookAheadTimeMin ) return true; // try to insure adequate blocks to prevent velocity throttling

   return false;
}


void SmoothMove::addDwell_Block( int delayMS )  // add block with no movement to block que, then append delay
{
   if( blockCount < bufferCount - 1 )
   {
      addLinear_Block( X_end, Y_end, Z_end, maxVel );
   }
   addDelay( delayMS );
}


void SmoothMove::addRapid_Block( float _x, float _y, float _z )
{
   addLinear_Block( _x, _y, _z, maxVel );
}


void SmoothMove::addLinear_Block( float _x, float _y, float _z, float _feed )
{
   int index = addBaseBlock( _x, _y, _z );

   moveBuffer[index].moveType = Linear;  // feed move G0/G1

   moveBuffer[index].targetVel = constrain( _feed * motionFeedOverride, 0.01f, maxVel );  // constrain to reasonable limits

   moveBuffer[index].targetVel_Sq = moveBuffer[index].targetVel * moveBuffer[index].targetVel;

   float dx = _x - moveBuffer[index].X_start;
   float dy = _y - moveBuffer[index].Y_start;
   float dz = _z - moveBuffer[index].Z_start;
   moveBuffer[index].length = sqrt(dx * dx + dy * dy + dz * dz);

   if(moveBuffer[index].length > 0.001f)
   {
      float inverseLength = 1.0f / moveBuffer[index].length;
      moveBuffer[index].X_vector = dx * inverseLength;  // line unit vector
      moveBuffer[index].Y_vector = dy * inverseLength;
      moveBuffer[index].Z_vector = dz * inverseLength;
   }
   else
   {
      moveBuffer[index].X_vector = 0.0f;  // line unit vector
      moveBuffer[index].Y_vector = 0.0f;
      moveBuffer[index].Z_vector = 0.0f;
   }

   setMaxStartVel(index);  // set cornering/start speed

   constAccelTrajectory();
}


void SmoothMove::addArc_Block(int type, float _x, float _y, float _feed, float centerX, float centerY)
{

   int index = addBaseBlock( _x, _y, Z_end );

   if(type == 2)
   {
      moveBuffer[index].moveType = ArcCW; // Clockwise Arc G2
   }
   else
   {
      moveBuffer[index].moveType = ArcCCW; // Counter-Clockwise Arc G3
   }

   float dXstart = moveBuffer[index].X_start - centerX;
   float dYstart = moveBuffer[index].Y_start - centerY;
   float startRadiusSq = dXstart * dXstart + dYstart * dYstart;  // used later to compare start/end radii
   moveBuffer[index].radius = sqrt(startRadiusSq);
   
   moveBuffer[index].startAngle = atan2(dYstart, dXstart);
   if(moveBuffer[index].startAngle < 0.0f) moveBuffer[index].startAngle += 6.2831853f; // force positive

   float dXend = X_end - centerX;
   float dYend = Y_end - centerY;

   float endAngle = atan2(dYend, dXend);
   if(endAngle < 0.0f) endAngle += 6.2831853f; // force positive

   float arcAngle;

   if(moveBuffer[index].moveType == ArcCW)
   {
      arcAngle = moveBuffer[index].startAngle - endAngle;
   }
   else
   {
      arcAngle = endAngle - moveBuffer[index].startAngle;
   }

   if(arcAngle < 0.0001f)     // must be positive also matching start and stop locations indicates full circle
   {
      arcAngle += 6.2831853f;
   }

   moveBuffer[index].length = arcAngle * moveBuffer[index].radius;

   float endRadiusSq = dXend * dXend + dYend * dYend;

   // check start and end point consistency
   if( abs( startRadiusSq - endRadiusSq ) > 0.000645f )  // both radii should match within .025mm (.001in)
   {
      SERIAL_PORT.println("ARC ERROR - Start-Center-End Radius Mismatch"); // length of the two radii are too different
      while(true); // hang - probably a better way to do this
   }

   moveBuffer[index].X_vector = centerX;
   moveBuffer[index].Y_vector = centerY;
   moveBuffer[index].Z_vector = 0.0f;

   _feed = constrain( _feed * motionFeedOverride, 0.01f, sqrt(maxAccel * moveBuffer[index].radius) );  // limit feed rate to prevent excessive radial acceleration
   moveBuffer[index].targetVel    = min( _feed, maxVel );
   moveBuffer[index].targetVel_Sq = moveBuffer[index].targetVel * moveBuffer[index].targetVel;

   setMaxStartVel(index);  // set cornering/start speed

   constAccelTrajectory();

}


int SmoothMove::addBaseBlock( const float & _x, const float & _y, const float & _z )  // called to perform operations shared by all addBlock functions
{
   int index = AddNewBlockIndex();

   moveBuffer[index].X_start = X_end; // set start point to previous blocks end point
   moveBuffer[index].Y_start = Y_end;
   moveBuffer[index].Z_start = Z_end;

   X_end = _x; // save this block's end point
   Y_end = _y;
   Z_end = _z;

   moveBuffer[index].dwell = 0;  // assume continuous motion
   moveBuffer[index].extrudeDist = 0.0f; // assume no extrude
   moveBuffer[index].extrudeScaleFactor = 0.0f;

   return index;
}


void SmoothMove::addDelay(int delayMS)
{
   if(delayMS > 0)
   {
      moveBuffer[newBlockIndex].dwell = delayMS * 1000UL;
   }
   else
   {
      moveBuffer[newBlockIndex].dwell = 0;
   }
}


void SmoothMove::addExtrudeMM( float positionMM )
{
   
   moveBuffer[newBlockIndex].extrudeDist = ( positionMM - extrudeProgPos ) * extrudeRateOverride;

   computeExtrudeFactors( newBlockIndex );

   extrudeProgPos = positionMM;
}


void SmoothMove::computeExtrudeFactors( int index )
{
   if( moveBuffer[index].extrudeDist > 0.001f )
   {
      if( moveBuffer[index].length > 0.001f )
      {
         moveBuffer[index].extrudeScaleFactor = moveBuffer[index].extrudeDist / moveBuffer[index].length;
         moveBuffer[index].staticExtrude = false;
      }
      else
      {
         moveBuffer[index].staticExtrude = true;
      }
   }
   else
   {
      moveBuffer[index].extrudeScaleFactor = 0.0f;
      moveBuffer[index].staticExtrude = false;
   }
}


int SmoothMove::getBlockCount()
{
   return blockCount;
}


void SmoothMove::removeOldBlock()
{
   if(blockCount > 0) // don't allow negative block counts
   {
      blockPosition -= moveBuffer[currentBlockIndex].length;
      moveBuffer[currentBlockIndex].targetVel = 0.0f;

      if( abs(moveBuffer[currentBlockIndex].extrudeDist) > 0.0001f )
      {
         extrudeMachPos += moveBuffer[currentBlockIndex].extrudeDist;
      }

      currentBlockIndex = nextBlockIndex(currentBlockIndex);
      blockCount--;
   }
   else
   {
      // something screwed up if this is ever executed...
      currentBlockIndex = blockCount = 0; 
      newBlockIndex = 1;
   }
}


int SmoothMove::AddNewBlockIndex()
{
   newBlockIndex = nextBlockIndex( newBlockIndex );
   blockCount++;

   return newBlockIndex;
}


int SmoothMove::nextBlockIndex( int currentIndex )  // direction of travel
{
   if(currentIndex > 0)
      return currentIndex - 1;

   return bufferCount - 1;
}


int SmoothMove::previousBlockIndex( int currentIndex ) // against direction of travel
{
   if(currentIndex < bufferCount - 1)
      return currentIndex + 1;

   return 0;
}


bool SmoothMove::blockQueueComplete()
{
   if( blockCount == 0 || (blockCount == 1 && segmentIndex > 2) ) // no blocks or must be on last block and waiting for next block
   {
      return true;
   }
   return false;
}


void SmoothMove::displayBlock( int i )
{

   Serial.print(i);Serial.print("\t");

   //Serial.print(moveBuffer[i].accelEndPoint, 2); Serial.print("\t");
   //Serial.print(moveBuffer[i].velEndPoint,   2); Serial.print("\t");
   //Serial.print(moveBuffer[i].length,        2); Serial.print("\t");

   //Serial.print(moveBuffer[i].accelTime); Serial.print("\t");
   //Serial.print(moveBuffer[i].velTime  ); Serial.print("\t");
   //Serial.print(moveBuffer[i].decelTime); Serial.print("\t");

   Serial.print(moveBuffer[i].accelEndPoint, 2);                             Serial.print("\t");
   Serial.print(moveBuffer[i].velEndPoint - moveBuffer[i].accelEndPoint, 2); Serial.print("\t");
   Serial.print(moveBuffer[i].length - moveBuffer[i].velEndPoint,        2); Serial.print("\t");

   Serial.print(moveBuffer[i].targetVel, 1);     Serial.print("\t");
   Serial.print(xVel[previousBlockIndex(i)], 1); Serial.print("\t");  // start vel
   Serial.print(moveBuffer[i].peakVel, 1);       Serial.print("\t");  // peak vel
   Serial.print(xVel[i], 1);                     Serial.print("\t");  // exit vel

   Serial.print(moveBuffer[i].X_start); Serial.print(" ");
   Serial.print(moveBuffer[i].Y_start); Serial.print(" ");
   Serial.print(moveBuffer[i].Z_start); Serial.print("\t");

   //Serial.print(X_end); Serial.print(" ");
   //Serial.print(Y_end); Serial.print(" ");
   //Serial.print(Z_end); Serial.print("\t");

   Serial.print(moveBuffer[i].dwell);

   Serial.println("");
}






