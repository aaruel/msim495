//
//  core.h
//  MSIM495
//
//  Created by Aaron Ruel on 1/17/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#ifndef __MSIM495__core__
#define __MSIM495__core__

#include <stdio.h>
#include <vector>
#include <math.h>
#include <functional>

namespace Util {
    /**
     * More functional version of for_each
     */
    template<typename T, typename F>
    void map(std::vector<T> iteratable, F function) {
        auto i = iteratable.begin();
        for (; i != iteratable.end(); ++i) {
            function(*i);
        }
    }
}

namespace Physics {
    
    /**
     * Represents : Float
     */
    typedef float real;
    
    /**
     * PI constant
     */
    constexpr real pi = 3.141592654;
    
    /**
     * Return sign of real input
     */
    constexpr int sign(real n) {
        return (0 < n) - (n < 0);
    }
    
    /**
     * Radians to Degrees
     */
    constexpr real rads_to_degs(real rads) {
        return rads * (180.0 / pi);
    }
    
    class Vector3 {
    public:
        /*
         * Vector Components
         */
        real x;
        real y;
        real z;
        
    private:
        /* 
         * 2^n optimization
         */
        real pad;
        
    public:
    
        /* 
         * Constructors 
         */
        Vector3():
            x(0), y(0), z(0) {}
        
        Vector3(const real x, const real y, const real z):
            x(x), y(y), z(z) {}
        
        /**
         * (Vector * -1)
         */
        void invert();
        
        /**
         * Formatted print of vector components
         */
        void print();
        
        /**
         * Zero vector components
         */
        void clear();
        
        /**
         * Avoids redundant calculation
         * Calculates summed square of component vectors
         */
        real magnitude_squared();
        
        /**
         * Returns total length of vector
         */
        real magnitude();
        
        /**
         * Normalizing a vector makes its magnitude == 1
         * Makes vector calculations easier
         */
        void normalize();
        
        /**
         * Example usage: p' = p + (dp)t ---> position += velocity * time;
         */
        void scale_vector_and_add(Vector3 &v, real scale);
        
        /**
         * Resulting vector from component multiplication of
         * this vector and another
         */
        Vector3 component_product(Vector3 &v);
        
        /**
         * Above operation applies product to this vector
         */
        void set_component_product(Vector3 &v);
        
        /**
         * Equal to |a||b|cos(theta) where theta is angle between two vectors
         */
        real scalar_product(Vector3 &v) const;
        
        /**
         * Dot Product
         * Equal to |a||b|sin(theta) where theta is angle between two vectors
         * Difference is sin vs cos
         */
        Vector3 vector_product(Vector3 &v);
        
        /**
         * Get distance between this vector and another
         */
        real distance(Vector3 b);
        Vector3 midpoint(Vector3 b);
        Vector3 direction(Vector3 b);
        real angle(Vector3 b);
        
        /**
         * Return angle between current and reference vector on xz plane
         */
        real angle_2d(Vector3 b);
        
        /* 
         * Operators 
         */
        // Products
        void operator*=(real value) {
            x *= value;
            y *= value;
            z *= value;
        };
        
        Vector3 operator*(const real value) const {
            return Vector3(
                x * value,
                y * value,
                z * value
            );
        };
        
        real operator*(Vector3 &v) const {
            return scalar_product(v);
        }
        
        // Addition
        void operator+=(const Vector3 &v) {
            x += v.x;
            y += v.y;
            z += v.z;
        };
        
        Vector3 operator+(const Vector3 &v) const {
            return Vector3(
                x + v.x,
                y + v.y,
                z + v.z
            );
        };
        
        // Subtraction
        void operator-=(const Vector3 &v) {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        };
        
        Vector3 operator-(const Vector3 &v) const {
            return Vector3(
                x - v.x,
                y - v.y,
                z - v.z
            );
        };
    };
    
    
    
    class Particle {
        /*
         * Info:
         * Newton's Laws
         * Law 1: An object's velocity is only affected when external forces act upon it
         * Law 2: An object's acceleration is affected by the external force and the mass of the object (a = f/m)
         *
         * Force between two objects - law of universal gravitation
         * f = G * ((m1 * m2) / (r * r))
         * G is a constant, earth's radius is kept consant, earth's mass is constant
         * this simplifies the equation to f = g * mobject where g = G * (mearth / (r * r))
         */
    protected:
        /* 
         * Position and derivative attributes of a particle in world space
         */
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        Vector3 force_accumulator;
        
        /*
         * Factor to remove any inaccuracy in the integrator stage
         * range 0..1
         * Value of 0.999 for example will be enough to remove any excess energy
         */
        real damping = 0.999;
        
        /*
         * This solves two problems, ease calculation of (a = f/m) to instead (a = (im)*f),
         * also prevents divide by zero errors and instead making immovable object with an input of zero
         */
        real inverse_mass;
    public:
        constexpr static real normal_gravity = -9.8;
    
        /*
         * Constructors
         */
        Particle(): position(Vector3(0,0,0)){}
        Particle(Vector3 v): position(v) {}
        Particle(real x, real y, real z): position(Vector3(x, y, z)) {}
        
        /*
         * Getters / Setters
         */
        Vector3 get_position() const { return position; }
        Vector3 get_velocity() { return velocity; }
        Vector3 get_acceleration() { return acceleration; }
        Vector3 get_force() { return force_accumulator; }
        real get_damping() { return damping; }
        real get_mass() { return inverse_mass <= 0.0 ? 0.0 : 1.f/inverse_mass; }
        real get_inverse_mass() { return inverse_mass; }
        void set_mass(real mass);
        void set_position(Vector3 v) { position = v; }
        void set_velocity(Vector3 v) { velocity = v; }
        void set_acceleration(Vector3 v) { acceleration = v; }
        void set_damping(real d) { damping = d; }
        
        /**
         * Summation of all forces equals resultant force
         */
        void add_impulse(Vector3 v) { force_accumulator += v; }
        
        /**
         * Zero the force accumulator
         */
        void clear_impulse() { force_accumulator = Vector3(); }
        
        /**
         * Zero everything
         */
        void clear() { acceleration = Vector3(); velocity = Vector3(); position = Vector3(); }
        
        /**
         * Handle particles physics at
         */
        void integrate(real time);
    };

    /**
     * Right Handed Coordinate Algorithm
     * Orthonormal set: All vectors in set are orthogonal (right angles)
     */
    void makeOrthonormalBasis(Vector3 * a, Vector3 * b, Vector3 * c);
};

#endif /* defined(__MSIM495__core__) */
