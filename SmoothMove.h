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

#ifndef SmoothMove_h
   #define SmoothMove_h

   #include <arduino.h>

   class SmoothMove
   {
      public:
         SmoothMove(float _accel, float _velMax, float _cornerRounding);
         ~SmoothMove();

         bool bufferVacancy();

         void addRapid_Block(  float _x, float _y, float _z );
         void addLinear_Block( float _x, float _y, float _z, float _feed );
         void addArc_Block( int type, float _x, float _y, float _feed, float centerX, float centerY );
         void addDelay( int delayMS );
         void addExtrude( uint32_t positionSteps );

         void startMoving( float _x, float _y, float _z );
         void abortMotion();
         void pause();
         void resume();

         float setMotionRateOverride(  float scale );
         float setExtrudeRateOverride( float scale );

         void junctionSmoothingOff();
         void junctionSmoothingOn();

         void getTargetLocation( float & x, float & y, float & z );
         float getSpeed();
         bool blockQueueComplete();
         uint32_t getExtrudeLocationSteps();

         int  getBlockCount();


      private:

         // DATA STRUCTURES AND VARIABLES

         /*
            Each Block (line/arc) is divided into 3 segments:

                             Accelerate          Velocity Const         Decelerate
            - - - - -> 0---------------------->|---------------->|---------------------->0 - - -> direction of travel
                       |         s_0           |      s_1        |         s_2           |
                       |                       |                 |                       |
                       |<- Velocity Increase ->|                 |<- Velocity Decrease ->|

            * For a single G-code block, Velocity Increase will always lead Velocity Decrease (never Decrease before Increase on a single block)
            * Some segments may have zero length
            * A single semgment may bridge multiple blocks, in which case the other segments have zero length
            * Arcs are treated as a straight line and then curved on output

         */


         /*
            Ring buffer topology

            | Block Max | Block n   |         | Block 1  | Block 0 | Block null |
            |---------->|---------->|--/.../->|--------->|-------->|            |
            | Run now   | Run next  |         |          | newest  | dead stop  |
            |<----------------------- Ring Buffer ---------------->|            |

            * the null block is hard coded to force the machine to a stop if there is buffer starvation
            * completing a block decrements the "oldBlock" index
            * adding a block decrements the "newBlock" index
            * roll over is handled as needed
            * as each block is added, block acc/dec/vel segements are recomputed starting from null Block to Run Next Block.

         */

         enum moveType_t {
            Linear,
            ArcCW,
            ArcCCW
         };

         const static int bufferCount = 21;

         struct block_t
         {
            float length;
            float radius, startAngle;

            float X_start,  Y_start,  Z_start;
            float X_vector, Y_vector, Z_vector;

            float targetVel, targetVel_Sq, peakVel, maxStartVel;

            float accelEndPoint, velEndPoint, decelLength; // length of each segement
            uint32_t accelTime,  velTime,     decelTime,  dwell;   // time to complete each segement

            uint32_t extrudePosition, extrudePositionProgram;
            float extrudeScaleFactor;

            moveType_t moveType;

            bool fastJunction;  // true if the transition from the prev block does not reduce vel
         } moveBuffer[bufferCount];

         float xVel[bufferCount];      // velocity at block boundarys
         float xVel_Sq[bufferCount];   // boundary velocities squared

         float motionFeedOverride, extrudeRateOverride;

         float blockPosition, velocityNow;

         int currentBlockIndex, newBlockIndex, blockCount, segmentIndex;

         float X_end, Y_end, Z_end;

         float cornerRoundDist, cornerRoundDistSq, cornerRoundDistHalf;

         float maxAccel, accelInverse, accelInverseHalf, accelDouble;
         float maxVel;

         uint32_t exactStopStartTime, exactStopDelay;
         bool exactStopActive;

         bool motionStopped, motionPaused;

         float totalDistance, totalTime;
         uint32_t lookAheadTime;

         uint32_t segmentStartTime, segmentTime, startOffset;

         bool pathSmoothingOff;


         // *** PRIVATE FUNCTIONS ***
         void advancePostion();
         void constAccelTrajectory();

         int addBaseBlock( const float & _x, const float & _y, const float & _z );
         int AddNewBlockIndex();
         void removeOldBlock();

         int previousBlockIndex( int currentIndex );
         int previousBlockIndex();

         int previousSegmentIndex( int currentIndex );

         int nextBlockIndex( int currentIndex );
         int nextBlockIndex();

         void setMaxStartVel( const int & index );
         void getPos( float & x, float & y, float & z, const int & index, const float & position );

         void displayBlock( int i );

   };

#endif