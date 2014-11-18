/*
 * Copyright (C) 2014, Jason S. McMullan <jason.mcmullan@gmail.com>
 * All right reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS\'\' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef AXISALLY_H
#define AXISALLY_H

#include <stdio.h>
#include <limits.h>
#include <math.h>

#define AXISALLY_MIN    INT_MIN
#define AXISALLY_MAX    INT_MAX

#define BUG(f,args...) do { if (0) printf(f ,##args ); } while (0)

class AxisAlly {
    public:
        AxisAlly() {
            _location_min = AXISALLY_MIN;
            _location_max = AXISALLY_MAX;

            _velocity_max = 100.0;        /* 100 units per 1s */
            _acceleration_max = 10.0;      /* +10units/sec per sec */
        }
        virtual ~AxisAlly() { }

        /* Perform the initial motor activation and homing
         */
        virtual void begin() = 0;

        /* Process location updates
         *   Returns false if no movements are pending
         */
        virtual bool update(long delta_ms) = 0;

        /* Set the current location (for homing) */
        virtual void setLocation(int location) = 0;
        virtual int getLocation() = 0;

        /* Get the current velocity */
        virtual float getVelocity() = 0;

        /* Move to new location */
        virtual void moveLocation(int location) {
            if (location > _location_max)
                location = _location_max;
            else if (location < _location_min)
                location = _location_min;
           _moveto = location;
        }

        /* Set the location limits
         *   Initially, these are AXISALLY_MIN and AXISALLY_MAX
         */
        virtual void setLocationRange(int min_location, int max_location) {
            _location_min = min_location;
            _location_max = max_location;
        }

        /* Set maximum acceleration pointer
         *   Acceleration is |delta_velocity/delta_time|
         */
        virtual void setAccelerationMax(float acceleration_max) {
            _acceleration_max = acceleration_max;
        }

        virtual float getAccelerationMax() {
            return _acceleration_max;
        }

        /* Set maximum velocity
         *   Velocity is |delta_location/delta_time|
         */
        virtual void setVelocityMax(float velocity_max) {
            _velocity_max = velocity_max;
        }

        virtual float getVelocityMax() {
            return _velocity_max;
        }


protected:
        int _moveto;    /* Desired location */
        int _location_min;      /* Min allowed location */
        int _location_max;      /* Max allowed location */

        float _acceleration_max;
        float _velocity_max;
};

class AxisAlly_Sim : public AxisAlly {
    public:
        AxisAlly_Sim() : AxisAlly() {
            _millis = 0;
            _last_location = 0.0;
            _last_velocity = 0.0;
        }

        virtual void begin() {
        }

        /* Process location updates
         *   Returns false if no movements are pending
         */
        virtual bool update(long delta_ms) {
            float location, moveto_delta;
            float velocity, location_delta, b_distance;
            float sec;
            bool moving = true;

            sec = update_milli_stats(delta_ms);
            if (sec == 0.0)
                return false;

            /* SIMULATION! */
            location_delta = _last_velocity * sec;
BUG("dl %f, dv %f, dt %f\n", location_delta, _last_velocity, sec);

            location = _last_location + location_delta;
            velocity = location_delta / sec;

            moveto_delta = (_moveto - location);

            if (location > _location_max) {
                location = _location_max;
                moving = false;
            } else if (location <  _location_min) {
                location = _location_min;
                moving = false;
            }

            int moveto_dir = moveto_delta < 0 ? -1 : 1;
            int velocity_dir = velocity < 0 ? -1 : 1;
            if (moveto_dir * moveto_delta > velocity_dir * velocity * _loop_sec_avg ) {

                /* Are we within the braking distance at this velocity? */
                b_distance = velocity * velocity / _acceleration_max;

                /* See if we need to slow down */
                if (b_distance * 1.2 > fabsf(moveto_delta)) {
                    float delta = -moveto_dir * _acceleration_max * _loop_sec_avg;
    BUG("SLOW DOWN- v %f, dv %f (braking distance %f)\n", velocity, delta, b_distance);
                    /* Time to slow down! */
                    velocity += delta;
                    if (moveto_dir * velocity < 0)
                        velocity = 0;
                } else if (moveto_dir * velocity < _velocity_max) {
                    float delta = moveto_dir * _acceleration_max * _loop_sec_avg;
    BUG("SPEED UP - v %f, dv %f (braking distance %f)\n", velocity, delta, b_distance);
                    /* Speed up! */
                    velocity += moveto_dir * _acceleration_max * _loop_sec_avg;
                    if (moveto_dir * velocity >  _velocity_max)
                        velocity = moveto_dir * _velocity_max;
                }
            } else {
BUG("STOP\n");
                moving = false;
            }
BUG("dt %f, @%d -> %d, v=%f, a=%f (%f-%f)/%f\n", sec, (int)location, (int)_moveto, velocity,  (velocity - _last_velocity)/sec, velocity, _last_velocity, sec);


            _last_location = location;
            _last_velocity = velocity;

            return moving;
        }

        virtual float getVelocity() {
            return _last_velocity;
        }

    private:

        /* Get the average # of seconds between each call to loop
         */
        float update_milli_stats(long delta_ms) {
            if (_millis == 0)
                _milli_total = 0;
            if (_millis >= 7)
                _milli_total -= _milli[_millis & 7];
            _millis++;
            _milli[_millis & 7] = delta_ms;
            _milli_total += delta_ms;
          
            if (_millis < 8)
                _loop_sec_avg = _milli_total / 1000.0 / _millis;
            else
                _loop_sec_avg = _milli_total / 1000.0 / 8.0;

            return delta_ms / 1000.0;
        }


        /* Set the current location (for homing) */
        virtual void setLocation(int location) {
            _last_location = location;
        }
        virtual int getLocation() {
            return (int)_last_location;
        }

    protected:
        long _last_millis;   /* Last time */
        float _last_location;  /* Last known location */
        float _last_velocity;  /* Last velocity */

        long  _milli[8];
        int   _millis;
        long  _milli_total;
        float _loop_sec_avg;
};
          

#endif /* AXISALLY_H */
/* vim: set shiftwidth=4 expandtab:  */
