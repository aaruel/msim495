//
//  core.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 1/17/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "core.h"
#include <math.h>
#include <assert.h>

#define real_pow powf
#define real_sqrt sqrtf

namespace Physics {
    /* 
     * Namespace Functions
     */
    
    real real_sqrt(real x) {
        return static_cast<real>(sqrt(x));
    }
    
    void makeOrthonormalBasis(Vector3 * a, Vector3 * b, Vector3 * c) {
        a->normalize();
        (*c) = a->vector_product(*b);
        
        if (c->magnitude_squared() == 0.0) return;
        
        c->normalize();
        (*b) = c->vector_product(*a);
    }
    
    
    
    /*
     * Class Implementations 
     */
    
    // Vector 3 //
    //////////////
    
    void Vector3::invert() {
        x = -x;
        y = -y;
        z = -z;
    }
    
    void Vector3::print() {
        printf("<%f, %f, %f>\n", x, y, z);
    }
    
    void Vector3::clear() {
        (*this) = Vector3();
    }
    
    real Vector3::magnitude_squared() {
        // Avoids sqrt and square
        return x*x + y*y + z*z;
    }
    
    real Vector3::magnitude() {
        return real_sqrt( magnitude_squared() );
    }
    
    void Vector3::normalize() {
        real length = magnitude();
        if (length > 0) {
            (*this) *= static_cast<real>(1) / length;
        }
    }
    
    void Vector3::scale_vector_and_add(Vector3 &v, const real scale) {
        (*this) += (v * scale);
    }
    
    Vector3 Vector3::component_product(Vector3 &v) {
        return Vector3(x * v.x, y * v.y, z * v.z);
    }
    
    void Vector3::set_component_product(Vector3 &v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
    }
    
    real Vector3::scalar_product(Vector3 &v) const {
        return x*v.x + y*v.y + z*v.z;
    }
    
    Vector3 Vector3::vector_product(Vector3 &v) {
        return Vector3(
            y*v.z - z*v.y,
            z*v.x - x*v.z,
            x*v.y - y*v.x
        );
    }

    real Vector3::distance(Vector3 b) {
        return real_sqrt(
            real_pow(x - b.x, 2)
            + real_pow(y - b.y, 2)
            + real_pow(z - b.z, 2)
        );
    }
    
    Vector3 Vector3::midpoint(Vector3 b) {
        Vector3 n = (*this) + b;
        return Vector3(n.x / 2, n.y / 2, n.z /2);
    }
    
    Vector3 Vector3::direction(Vector3 b) {
        Vector3 n = b - (*this);
        n.normalize();
        return n;
    }
    
    real Vector3::angle_2d(Vector3 b) {
        real mag = x * b.x - z * b.z;
        int sign = (mag < 0 ? -1 : 1);
        return sign * acos((scalar_product(b)) / (magnitude() * b.magnitude()));
    }
    
    real Vector3::angle(Vector3 b) {
        real dot = scalar_product(b);
        real mag = magnitude() * b.magnitude();
        return acosf(dot/mag);
    }
    
    
    
    // Particle //
    //////////////
    
    void Particle::set_mass(real mass) {
        if (mass <= 0.0) inverse_mass = 0.0;
        else inverse_mass = 1.0 / mass;
    }
    
    void Particle::integrate(real time) {
        if (inverse_mass <= 0.0f) return;
        assert(time > 0.0);
        // update position
        position += (velocity * time);
        // update velocity with time adjusted damping factor
        Vector3 adjusted_acc = acceleration;
        adjusted_acc += force_accumulator * inverse_mass;
        velocity = (velocity * real_pow(damping, time)) + (adjusted_acc * time);
        clear_impulse();
    }
    
}