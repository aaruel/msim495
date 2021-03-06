//
//  flightsim.h
//  MSIM495
//

#ifndef __MSIM495__flightsim__
#define __MSIM495__flightsim__

#include <stdio.h>
#include "physics.h"
#include <math.h>
#include <assert.h>

namespace Physics {
    class Aero : public ForceGenerator {
    protected:
        Matrix3 tensor;
        Vector3 position;
        Vector3 * windspeed;
        
    public:
        Aero(
            Matrix3 tensor,
            Vector3 position,
            Vector3 * windspeed
        ) : tensor(tensor), position(position), windspeed(windspeed) {}
        
        virtual void update_force(RigidBody * body, real duration) {
            update_force_from_tensor(body, duration, tensor);
        }
        
    protected:
        void update_force_from_tensor(
            RigidBody * body,
            real duration,
            Matrix3 &tensor
        ) {
            Vector3 velocity = body->get_velocity();
            velocity += *windspeed;
            
            Vector3 body_velo =
                body->get_transform().transform_inverse_direction(velocity);
            
            Vector3 body_force = tensor.transform(body_velo);
            Vector3 force = body->get_transform().transform_direction(body_force);
            
            body->add_force_at_body_point(force, position);
        }
    };
    
    class AeroControl : public Aero {
    protected:
        Matrix3 max_tensor;
        Matrix3 min_tensor;
        real control_setting;
    
    private:
        Matrix3 get_tensor() {
            if (control_setting <= -1.f) return min_tensor;
            else if (control_setting >= 1.f) return max_tensor;
            else if (control_setting < 0) {
                return Matrix3::linear_interpolate(
                    tensor,
                    max_tensor,
                    control_setting
                );
            }
            else return tensor;
        }
        
    public:
        AeroControl(
            Matrix3 base,
            Matrix3 min, Matrix3 max,
            Vector3 position, Vector3 * windspeed
        ) : Aero(base, position, windspeed) {
            min_tensor = min;
            max_tensor = max;
            control_setting = 0.0f;
        }
        
        void set_control(real value) { control_setting = value; }
        
        virtual void update_force(RigidBody * body, real duration) {
            Matrix3 tensor = get_tensor();
            Aero::update_force_from_tensor(body, duration, tensor);
        }
    };
    
    
    /**
     * Engine Force
     */
    class PropulsionForce : public ForceGenerator {
        constexpr static real max = 10.f;
        real propel;
        
        // 0 degrees  - 100% forward
        // 90 degrees - 100% downward
        real thrust_angle;
        
    public:
        PropulsionForce(real p = max) : propel(p), thrust_angle(0) {}
    
        void increment_propel(real inc) {
            real oper = propel + inc;
            
            if (oper >= 0 && oper <= max) {
                propel = oper;
            }
        }
        
        void increment_thrust_angle(real inc) {
            real oper = thrust_angle + inc;
            
            if (oper >= 0 && oper <= 90) {
                thrust_angle = oper;
            }
        }
        
        real get_propel() { return propel; }
        void set_propel(real p) { propel = p; }
        
        real get_thrust_angle() { return thrust_angle; }
        void set_thrust_angle(real angle) {
            assert(angle >= 0 && angle <= 90);
            thrust_angle = angle;
        }
    
        virtual void update_force(RigidBody * body, real duration) {
            real down_p = propel * sinf(degs_to_rads(thrust_angle));
            real forward_p = propel * cosf(degs_to_rads(thrust_angle));
            Vector3 propulsion(-forward_p, down_p, 0);
            propulsion = body->get_transform().transform_direction(propulsion);
            body->add_force(propulsion);
        }
    };
}

#endif /* defined(__MSIM495__flightsim__) */
