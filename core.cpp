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
    
    
    
    // RigidBody //
    ///////////////
    
    static inline void _calculate_transform_matrix(
        Matrix4 &transformMatrix,
        Vector3 &position,
        Quaternion &orientation
    ) {
        transformMatrix.data[ 0] =
            1-2*orientation.j*orientation.j-
            2*orientation.k*orientation.k;
        transformMatrix.data[ 1] =
            2*orientation.i*orientation.j -
            2*orientation.r*orientation.k;
        transformMatrix.data[ 2] =
            2*orientation.i*orientation.k +
            2*orientation.r*orientation.j;
        transformMatrix.data[ 3] = position.x;

        transformMatrix.data[ 4] =
            2*orientation.i*orientation.j +
            2*orientation.r*orientation.k;
        transformMatrix.data[ 5] =
            1-2*orientation.i*orientation.i-
            2*orientation.k*orientation.k;
        transformMatrix.data[ 6] =
            2*orientation.j*orientation.k -
            2*orientation.r*orientation.i;
        transformMatrix.data[ 7] = position.y;

        transformMatrix.data[ 8] =
            2*orientation.i*orientation.k -
            2*orientation.r*orientation.j;
        transformMatrix.data[ 9] =
            2*orientation.j*orientation.k +
            2*orientation.r*orientation.i;
        transformMatrix.data[10] =
            1-2*orientation.i*orientation.i-
            2*orientation.j*orientation.j;
        transformMatrix.data[11] = position.z;
    }
    
    static inline void _transform_inertia_tensor(
        Matrix3 &iitWorld,
        const Quaternion &q,
        const Matrix3 &iitBody,
        const Matrix4 &rotmat
    ) {
        real t4 = rotmat.data[0]*iitBody.data[0]+
            rotmat.data[1]*iitBody.data[3]+
            rotmat.data[2]*iitBody.data[6];
        real t9 = rotmat.data[0]*iitBody.data[1]+
            rotmat.data[1]*iitBody.data[4]+
            rotmat.data[2]*iitBody.data[7];
        real t14 = rotmat.data[0]*iitBody.data[2]+
            rotmat.data[1]*iitBody.data[5]+
            rotmat.data[2]*iitBody.data[8];
        real t28 = rotmat.data[4]*iitBody.data[0]+
            rotmat.data[5]*iitBody.data[3]+
            rotmat.data[6]*iitBody.data[6];
        real t33 = rotmat.data[4]*iitBody.data[1]+
            rotmat.data[5]*iitBody.data[4]+
            rotmat.data[6]*iitBody.data[7];
        real t38 = rotmat.data[4]*iitBody.data[2]+
            rotmat.data[5]*iitBody.data[5]+
            rotmat.data[6]*iitBody.data[8];
        real t52 = rotmat.data[8]*iitBody.data[0]+
            rotmat.data[9]*iitBody.data[3]+
            rotmat.data[10]*iitBody.data[6];
        real t57 = rotmat.data[8]*iitBody.data[1]+
            rotmat.data[9]*iitBody.data[4]+
            rotmat.data[10]*iitBody.data[7];
        real t62 = rotmat.data[8]*iitBody.data[2]+
            rotmat.data[9]*iitBody.data[5]+
            rotmat.data[10]*iitBody.data[8];

        iitWorld.data[0] = t4*rotmat.data[0]+
            t9*rotmat.data[1]+
            t14*rotmat.data[2];
        iitWorld.data[1] = t4*rotmat.data[4]+
            t9*rotmat.data[5]+
            t14*rotmat.data[6];
        iitWorld.data[2] = t4*rotmat.data[8]+
            t9*rotmat.data[9]+
            t14*rotmat.data[10];
        iitWorld.data[3] = t28*rotmat.data[0]+
            t33*rotmat.data[1]+
            t38*rotmat.data[2];
        iitWorld.data[4] = t28*rotmat.data[4]+
            t33*rotmat.data[5]+
            t38*rotmat.data[6];
        iitWorld.data[5] = t28*rotmat.data[8]+
            t33*rotmat.data[9]+
            t38*rotmat.data[10];
        iitWorld.data[6] = t52*rotmat.data[0]+
            t57*rotmat.data[1]+
            t62*rotmat.data[2];
        iitWorld.data[7] = t52*rotmat.data[4]+
            t57*rotmat.data[5]+
            t62*rotmat.data[6];
        iitWorld.data[8] = t52*rotmat.data[8]+
            t57*rotmat.data[9]+
            t62*rotmat.data[10];
    }
    
    void RigidBody::calculate_derived_data() {
        orientation.normalize();
        
        _calculate_transform_matrix(
            transform_matrix,
            position,
            orientation
        );
        
        _transform_inertia_tensor(
            inverse_inertia_tensor_world,
            orientation,
            inverse_inertia_tensor,
            transform_matrix
        );
    }
}