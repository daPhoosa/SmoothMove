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

void SmoothMove::setPosition( float t_x, float t_y, float t_z )
{
   setPosX( t_x );
   setPosY( t_y );
   setPosZ( t_z );   
}


void SmoothMove::setPosition( float t_x, float t_y, float t_z, float t_e )
{
   setPosX( t_x );
   setPosY( t_y );
   setPosZ( t_z );   
   setPosE( t_e ); 
}


void SmoothMove::setPosX( float t_x )
{
   if( !motionStopped ) motionStopped = true;

   X_end = t_x; // no queued blocks, so end point equals start point
}


void SmoothMove::setPosY( float t_y )
{
   if( !motionStopped ) motionStopped = true;

   Y_end = t_y; // no queued blocks, so end point equals start point
}


void SmoothMove::setPosZ( float t_z )
{
   if( !motionStopped ) motionStopped = true;

   Z_end = t_z; // no queued blocks, so end point equals start point
}


void SmoothMove::setPosE( float t_e )
{
   if( !motionStopped ) motionStopped = true;

   extrudeProgPos = t_e;
   extrudeMachPos = t_e;
}