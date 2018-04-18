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
#include <float.h>

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
    
    constexpr real degs_to_rads(real degs) {
        return degs * (pi / 180);
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
     * 4 element spacial rotation structure
     */
    class Quaternion {
    public:
    
        /**
         * 4 * 32bit data declaration
         */
        union {
            struct {
                union {real r; real w;}; // real
                union {real i; real x;}; // complex
                union {real j; real y;}; // complex
                union {real k; real z;}; // complex
            };
            
            real data[4];
        };
        
        /**
         * Constructors
         */
        Quaternion() : r(1), i(0), j(0), k(0) {}
        
        Quaternion(
            real r, real i,
            real j, real k
        ) : r(r), i(i), j(j), k(k) {}
        
        /**
         * Print quaternion to console
         */
        void print() {
            printf("%%{r: %f, i: %f, j: %f, k: %f}\n", r, i, j, k);
        }
        
        /**
         * Multiply all components by the magnitude
         * Component magnitude becomes 1
         */
        void normalize() {
            real d = r*r+i*i+j*j+k*k;

            // Check for zero length quaternion, and use the no-rotation
            // quaternion in that case.
            if (d < FLT_EPSILON) {
                r = 1;
                return;
            }

            d = ((real)1.0) / sqrtf(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }
        
        /**
         * Scale XYZ components of input vector
         * Rotate input by (this)
         * Add input to (this)
         */
        void add_scaled_vector(Vector3 vector, real scale) {
            Quaternion q(
                0,
                vector.x * scale,
                vector.y * scale,
                vector.z * scale
            );
            
            q *= *this;
            r += q.r * ((real)0.5);
            i += q.i * ((real)0.5);
            j += q.j * ((real)0.5);
            k += q.k * ((real)0.5);
        }

        void rotate_by_vector(Vector3& vector) {
            Quaternion q(0, vector.x, vector.y, vector.z);
            (*this) *= q;
        }
        
        /**
         * Operators
         */
        void operator *=(Quaternion &multiplier)
        {
            Quaternion q = *this;
            
            r = (
                q.r*multiplier.r - q.i*multiplier.i
                - q.j*multiplier.j - q.k*multiplier.k
            );
            
            i = (
                q.r*multiplier.i + q.i*multiplier.r
                + q.j*multiplier.k - q.k*multiplier.j
            );
            
            j = (
                q.r*multiplier.j + q.j*multiplier.r
                + q.k*multiplier.i - q.i*multiplier.k
            );
            
            k = (
                q.r*multiplier.k + q.k*multiplier.r
                + q.i*multiplier.j - q.j*multiplier.i
            );
        }
    };
    
    
    
    /**
     * 3x3 Matrix implementation
     * inherently inelegant :(
     */
    class Matrix3 {
    public:
        typedef real mat3x3[9];
        mat3x3 data;
        
        
        
    public:
        Matrix3() {
            data[0] = data[1] = data[2] = 0;
            data[3] = data[4] = data[5] = 0;
            data[6] = data[7] = data[8] = 0;
        }
        
        Matrix3(Matrix3 * m) {
            for (unsigned i = 0; i < 9; ++i) data[i] = m->data[i];
        }
    
        Matrix3(
            real c0, real c1, real c2,
            real c3, real c4, real c5,
            real c6, real c7, real c8
        ) {
            data[0] = c0; data[1] = c1; data[2] = c2;
            data[3] = c3; data[4] = c4; data[5] = c5;
            data[6] = c6; data[7] = c7; data[8] = c8;
        }
        
        Matrix3 operator*(Matrix3 &o)
        {
            return Matrix3(
                data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6],
                data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7],
                data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8],

                data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6],
                data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7],
                data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8],

                data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6],
                data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7],
                data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8]
            );
        }
        
        static Matrix3 linear_interpolate(Matrix3 &a, Matrix3 &b, real prop) {
            Matrix3 result;
            
            for (unsigned i = 0; i < 9; ++i) {
                result.data[i] = a.data[i] * (1-prop) + b.data[i] * prop;
            }
            
            return result;
        }
    
        Vector3 operator*(Vector3 &v) {
            return Vector3(
                v.x * data[0] + v.y * data[1] + v.z * data[2],
                v.x * data[3] + v.y * data[4] + v.z * data[5],
                v.x * data[6] + v.y * data[7] + v.z * data[8]
            );
        }
        
        void set_inverse(Matrix3 &m)
        {
            real t4  = m.data[0]*m.data[4];
            real t6  = m.data[0]*m.data[5];
            real t8  = m.data[1]*m.data[3];
            real t10 = m.data[2]*m.data[3];
            real t12 = m.data[1]*m.data[6];
            real t14 = m.data[2]*m.data[6];

            // Calculate the determinant
            real t16 = (
                t4*m.data[8] - t6*m.data[7] - t8*m.data[8]
                + t10*m.data[7] + t12*m.data[5] - t14*m.data[4]
            );

            // Make sure the determinant is non-zero.
            if (t16 == (real)0.0f) return;
            real t17 = 1 / t16;

            data[0] =  (m.data[4]*m.data[8]-m.data[5]*m.data[7])*t17;
            data[1] = -(m.data[1]*m.data[8]-m.data[2]*m.data[7])*t17;
            data[2] =  (m.data[1]*m.data[5]-m.data[2]*m.data[4])*t17;
            data[3] = -(m.data[3]*m.data[8]-m.data[5]*m.data[6])*t17;
            data[4] =  (m.data[0]*m.data[8]-t14)*t17;
            data[5] = -(t6-t10)*t17;
            data[6] =  (m.data[3]*m.data[7]-m.data[4]*m.data[6])*t17;
            data[7] = -(m.data[0]*m.data[7]-t12)*t17;
            data[8] =  (t4-t8)*t17;
        }
        
        Matrix3 inverse() {
            Matrix3 result;
            result.set_inverse(*this);
            return result;
        }
        
        void set_transpose(Matrix3 &m) {
            data[0] = m.data[0];
            data[1] = m.data[3];
            data[2] = m.data[6];
            data[3] = m.data[1];
            data[4] = m.data[4];
            data[5] = m.data[7];
            data[6] = m.data[2];
            data[7] = m.data[5];
            data[8] = m.data[8];
        }
        
        void set_orientation(Quaternion &q) {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = 2*q.i*q.j - 2*q.k*q.r;
            data[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[5] = 2*q.j*q.k + 2*q.i*q.r;
            data[6] = 2*q.i*q.k + 2*q.j*q.r;
            data[7] = 2*q.j*q.k - 2*q.i*q.r;
            data[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
        }
        
        void set_inertia_tensor_coeffs(
            real ix, real iy, real iz,
            real ixy=0, real ixz=0, real iyz=0
        ) {
            data[0] = ix;
            data[1] = data[3] = -ixy;
            data[2] = data[6] = -ixz;
            data[4] = iy;
            data[5] = data[7] = -iyz;
            data[8] = iz;
        }
        
        void set_block_inertia_tensor(Vector3 &halfSizes, real mass) {
            Vector3 squares = halfSizes.component_product(halfSizes);
            set_inertia_tensor_coeffs(
                0.3f*mass*(squares.y + squares.z),
                0.3f*mass*(squares.x + squares.z),
                0.3f*mass*(squares.x + squares.y)
            );
        }

        Matrix3 transpose() {
            Matrix3 result;
            result.set_transpose(*this);
            return result;
        }
        
        Vector3 transform(Vector3 &v) {
            return (*this) * v;
        }
    };
    
    
    
    /**
     * 4x4 Matrix implementation
     * inherently inelegant :(
     */
    class Matrix4 {
    public:
        typedef real mat4x4[12];
        mat4x4 data;
        
        /**
         * Not for use, pads data to 512 bits for alignment
         */
        real padding[4];
        
    public:
        Vector3 operator*(Vector3 &v) {
            return Vector3(
                v.x * data[0] + v.y * data[1] + v.z * data[ 2] + data[ 3],
                v.x * data[4] + v.y * data[5] + v.z * data[ 6] + data[ 7],
                v.x * data[8] + v.y * data[9] + v.z * data[10] + data[11]
            );
        }
        
        Matrix4 operator*(const Matrix4 &o) const
        {
            Matrix4 result;
            result.data[ 0] = (o.data[0]*data[0]) + (o.data[4]*data[1]) + (o.data[ 8]*data[ 2]);
            result.data[ 4] = (o.data[0]*data[4]) + (o.data[4]*data[5]) + (o.data[ 8]*data[ 6]);
            result.data[ 8] = (o.data[0]*data[8]) + (o.data[4]*data[9]) + (o.data[ 8]*data[10]);

            result.data[ 1] = (o.data[1]*data[0]) + (o.data[5]*data[1]) + (o.data[ 9]*data[ 2]);
            result.data[ 5] = (o.data[1]*data[4]) + (o.data[5]*data[5]) + (o.data[ 9]*data[ 6]);
            result.data[ 9] = (o.data[1]*data[8]) + (o.data[5]*data[9]) + (o.data[ 9]*data[10]);

            result.data[ 2] = (o.data[2]*data[0]) + (o.data[6]*data[1]) + (o.data[10]*data[ 2]);
            result.data[ 6] = (o.data[2]*data[4]) + (o.data[6]*data[5]) + (o.data[10]*data[ 6]);
            result.data[10] = (o.data[2]*data[8]) + (o.data[6]*data[9]) + (o.data[10]*data[10]);

            result.data[ 3] = (o.data[3]*data[0]) + (o.data[7]*data[1]) + (o.data[11]*data[ 2]) + data[ 3];
            result.data[ 7] = (o.data[3]*data[4]) + (o.data[7]*data[5]) + (o.data[11]*data[ 6]) + data[ 7];
            result.data[11] = (o.data[3]*data[8]) + (o.data[7]*data[9]) + (o.data[11]*data[10]) + data[11];

            return result;
        }
        
        real get_determinant() {
            return (
                - data[8]*data[5]*data[ 2]
                + data[4]*data[9]*data[ 2]
                + data[8]*data[1]*data[ 6]
                - data[0]*data[9]*data[ 6]
                - data[4]*data[1]*data[10]
                + data[0]*data[5]*data[10]
            );
        }
        
        void set_inverse(Matrix4 &m) {
            // Make sure the determinant is non-zero.
            real det = get_determinant();
            if (det == 0) return;
            det = ((real)1.0)/det;

            data[ 0] = (-m.data[9]*m.data[6]+m.data[5]*m.data[10])*det;
            data[ 4] = (+m.data[8]*m.data[6]-m.data[4]*m.data[10])*det;
            data[ 8] = (-m.data[8]*m.data[5]+m.data[4]*m.data[ 9])*det;

            data[ 1] = (+m.data[9]*m.data[2]-m.data[1]*m.data[10])*det;
            data[ 5] = (-m.data[8]*m.data[2]+m.data[0]*m.data[10])*det;
            data[ 9] = (+m.data[8]*m.data[1]-m.data[0]*m.data[ 9])*det;

            data[ 2] = (-m.data[5]*m.data[2]+m.data[1]*m.data[ 6])*det;
            data[ 6] = (+m.data[4]*m.data[2]-m.data[0]*m.data[ 6])*det;
            data[10] = (-m.data[4]*m.data[1]+m.data[0]*m.data[ 5])*det;

            data[3] = (
                +m.data[9]*m.data[ 6]*m.data[ 3]
                -m.data[5]*m.data[10]*m.data[ 3]
                -m.data[9]*m.data[ 2]*m.data[ 7]
                +m.data[1]*m.data[10]*m.data[ 7]
                +m.data[5]*m.data[ 2]*m.data[11]
                -m.data[1]*m.data[ 6]*m.data[11]
            ) * det;
            
            data[7] = (
                -m.data[8]*m.data[ 6]*m.data[ 3]
                +m.data[4]*m.data[10]*m.data[ 3]
                +m.data[8]*m.data[ 2]*m.data[ 7]
                -m.data[0]*m.data[10]*m.data[ 7]
                -m.data[4]*m.data[ 2]*m.data[11]
                +m.data[0]*m.data[ 6]*m.data[11]
            ) * det;
            
            data[11] = (
                +m.data[8]*m.data[ 5]*m.data[ 3]
                -m.data[4]*m.data[ 9]*m.data[ 3]
                -m.data[8]*m.data[ 1]*m.data[ 7]
                +m.data[0]*m.data[ 9]*m.data[ 7]
                +m.data[4]*m.data[ 1]*m.data[11]
                -m.data[0]*m.data[ 5]*m.data[11]
            ) * det;
        }
        
        void set_orientation_and_pos(const Quaternion &q, const Vector3 &pos) {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = pos.x;

            data[4] = 2*q.i*q.j - 2*q.k*q.r;
            data[5] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[6] = 2*q.j*q.k + 2*q.i*q.r;
            data[7] = pos.y;

            data[8] = 2*q.i*q.k + 2*q.j*q.r;
            data[9] = 2*q.j*q.k - 2*q.i*q.r;
            data[10] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
            data[11] = pos.z;
        }
        
        Vector3 transform_inverse(Vector3 &vector) {
            Vector3 tmp = vector;
            tmp.x -= data[3];
            tmp.y -= data[7];
            tmp.z -= data[11];
            return Vector3(
                tmp.x * data[0]
                + tmp.y * data[4]
                + tmp.z * data[8],

                tmp.x * data[1]
                + tmp.y * data[5]
                + tmp.z * data[9],

                tmp.x * data[2]
                + tmp.y * data[6]
                + tmp.z * data[10]
            );
        }
        
        Vector3 transform_direction(Vector3 &vector) {
            return Vector3(
                vector.x * data[0] +
                vector.y * data[1] +
                vector.z * data[2],

                vector.x * data[4] +
                vector.y * data[5] +
                vector.z * data[6],

                vector.x * data[8] +
                vector.y * data[9] +
                vector.z * data[10]
            );
        }
        
        Vector3 transform_inverse_direction(Vector3 &vector) {
            return Vector3(
                vector.x * data[0] +
                vector.y * data[4] +
                vector.z * data[8],

                vector.x * data[1] +
                vector.y * data[5] +
                vector.z * data[9],

                vector.x * data[2] +
                vector.y * data[6] +
                vector.z * data[10]
            );
        }
        
        void fill_GL_array(float array[16]) {
            array[0] = (float)data[0];
            array[1] = (float)data[4];
            array[2] = (float)data[8];
            array[3] = (float)0;

            array[4] = (float)data[1];
            array[5] = (float)data[5];
            array[6] = (float)data[9];
            array[7] = (float)0;

            array[8] = (float)data[2];
            array[9] = (float)data[6];
            array[10] = (float)data[10];
            array[11] = (float)0;

            array[12] = (float)data[3];
            array[13] = (float)data[7];
            array[14] = (float)data[11];
            array[15] = (float)1;
        }
        
        void print() {
            printf(
                "-                     -\n"
                "| %.2f %.2f %.2f %.2f |\n"
                "| %.2f %.2f %.2f %.2f |\n"
                "| %.2f %.2f %.2f %.2f |\n"
                "| %.2f %.2f %.2f %.2f |\n"
                "-                     -\n",
                data[0], data[1], data[2], data[3],
                data[4], data[5], data[6], data[7],
                data[8], data[9], data[10], data[11],
                padding[0], padding[1], padding[2], padding[3]
            );
        }
        
        void print_gl() {
            float datagl[16];
            fill_GL_array(datagl);
            printf(
                "-                     -\n"
                "| %.2f %.2f %.2f %.2f |\n"
                "| %.2f %.2f %.2f %.2f |\n"
                "| %.2f %.2f %.2f %.2f |\n"
                "| %.2f %.2f %.2f %.2f |\n"
                "-                     -\n",
                datagl[ 0], datagl[ 1], datagl[ 2], datagl[ 3],
                datagl[ 4], datagl[ 5], datagl[ 6], datagl[ 7],
                datagl[ 8], datagl[ 9], datagl[10], datagl[11],
                datagl[12], datagl[13], datagl[14], datagl[15]
            );
        }
        
        static Vector3 world_to_local(Vector3 &world, Matrix4 transform) {
            return transform.transform_inverse(world);
        }
        
        static Vector3 local_to_world_direction(Vector3 &local, Matrix4 &transform) {
            return transform.transform_direction(local);
        }
        
        static Vector3 world_to_local_direction(Vector3 &world, Matrix4 &transform) {
            return transform.transform_inverse_direction(world);
        }
        
        Vector3 transform(Vector3 &v) {
            return (*this) * v;
        }
    };
    
    
    
    /**
     * Similar to the Particle class
     */
    class RigidBody {
    protected:
        real inverse_mass;
        real linear_damping;
        real angular_damping;
        
        Quaternion orientation;
        
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        Vector3 last_frame_accerlation;
        Vector3 rotation;
        Vector3 force_accumulator;
        Vector3 torque_accumulator;
        
        Matrix3 inverse_inertia_tensor;
        Matrix3 inverse_inertia_tensor_world;
        Matrix4 transform_matrix;
        
        bool is_awake;
        bool can_sleep;
        
    public:
        void set_mass(real mass) {
            if (mass <= 0.0) inverse_mass = 0.0;
            else inverse_mass = 1.0 / mass;
        }
        void set_damping(real linear, real angular) {
            linear_damping = linear;
            angular_damping = angular;
        }
        void set_acceleration(Vector3 acc) { acceleration = acc; }
        void set_velocity(Vector3 vel) { velocity = vel; }
        void set_position(Vector3 pos) { position = pos; }
        void set_rotation(Vector3 r) { rotation = r; }
        void set_orientation(Quaternion o) { orientation = o; }
        void set_awake(bool a) { is_awake = a; }
        void set_can_sleep(bool cs) { can_sleep = cs; if (!can_sleep && !is_awake) set_awake(true); }
        bool has_finite_mass() { return inverse_mass > 0; }
        real get_mass() { return inverse_mass > 0 ? 1.f/inverse_mass : 0; }
        Vector3 get_position() { return position; }
        Vector3 get_velocity() { return velocity; }
        Vector3 get_acceleration() { return acceleration; }
        Matrix4 get_transform() { return transform_matrix; }
        Quaternion get_orientation() { return orientation; }
        
        void calculate_derived_data();
        
        Vector3 get_point_in_local_space(Vector3 &point) {
            return transform_matrix.transform_inverse(point);
        }
        
        Vector3 get_point_in_world_space(Vector3 &point) {
            return transform_matrix.transform(point);
        }
        
        Vector3 get_direction_in_local_space(Vector3 &direction) {
            return transform_matrix.transform_inverse_direction(direction);
        }
        
        Vector3 get_direction_in_world_space(Vector3 &direction) {
            return transform_matrix.transform_direction(direction);
        }
        
        void set_inertia_tensor(Matrix3 &inertia_tensor) {
            inverse_inertia_tensor.set_inverse(inertia_tensor);
        }
        
        void add_force(Vector3 &force) {
            force_accumulator += force;
            is_awake = true;
        }
        
        void clear_accumulator() {
            force_accumulator = Vector3();
            torque_accumulator = Vector3();
        }
        
        // Breakpoints don't work if this function is named integrate...????
        void intergrate(real duration) {
            // Calculate linear acceleration
            last_frame_accerlation = acceleration;
            last_frame_accerlation.scale_vector_and_add(force_accumulator, inverse_mass);
            
            // Calculate angular acceleration
            Vector3 angular_acceleration =
                inverse_inertia_tensor_world.transform(torque_accumulator);
            
            // Update velocity
            velocity.scale_vector_and_add(last_frame_accerlation, duration);
            
            // Update angular velocity
            rotation.scale_vector_and_add(angular_acceleration, duration);
            
            // Calculate drag
            velocity *= powf(linear_damping, duration);
            rotation *= powf(angular_damping, duration);
            
            // Update positions
            position.scale_vector_and_add(velocity, duration);
                        
            // Update angular positions
            orientation.add_scaled_vector(rotation, duration);
            
            // Normalize orientation
            calculate_derived_data();
            
            clear_accumulator();
        }
        
        void add_force_at_point(
            Vector3 &force,
            Vector3 &point
        ) {
            Vector3 point_copy = point;
            point_copy -= position;
            
            force_accumulator += force;
            torque_accumulator += point_copy.vector_product(force);
            
            is_awake = true;
        }
        
        void add_force_at_body_point(
            Vector3 &force,
            Vector3 &point
        ) {
            Vector3 piws = get_point_in_world_space(point);
            add_force_at_point(force, piws);
            
            is_awake = true;
        }
        
        void get_gl_transform(float matrix[16]) {
            matrix[0] = (float)transform_matrix.data[0];
            matrix[1] = (float)transform_matrix.data[4];
            matrix[2] = (float)transform_matrix.data[8];
            matrix[3] = 0;

            matrix[4] = (float)transform_matrix.data[1];
            matrix[5] = (float)transform_matrix.data[5];
            matrix[6] = (float)transform_matrix.data[9];
            matrix[7] = 0;

            matrix[8] = (float)transform_matrix.data[2];
            matrix[9] = (float)transform_matrix.data[6];
            matrix[10] = (float)transform_matrix.data[10];
            matrix[11] = 0;

            matrix[12] = (float)transform_matrix.data[3];
            matrix[13] = (float)transform_matrix.data[7];
            matrix[14] = (float)transform_matrix.data[11];
            matrix[15] = 1;
        }
    };
    
    
    
    /**
     * AngleAxis implementation just for testing
     */
    class AngleAxis {
    public:
        real angle;
        real x;
        real y;
        real z;
        
        AngleAxis() : x(0), y(0), z(0), angle(0) {}
        AngleAxis(real angle, real x, real y, real z) : x(x), y(y), z(z), angle(angle) {}
        AngleAxis(Quaternion &q) { from_quaternion(q); }
        
        void from_quaternion(Quaternion &q) {
            angle = 2 * acosf(q.w);
            x = q.x / sqrtf(1 - q.w * q.w);
            y = q.y / sqrtf(1 - q.w * q.w);
            z = q.z / sqrtf(1 - q.w * q.w);
        }
        
        void print() {
            printf("%%{angle: %f, x: %f, y: %f, z: %f}\n", angle, x, y, z);
        }
    };

    /**
     * Right Handed Coordinate Algorithm
     * Orthonormal set: All vectors in set are orthogonal (right angles)
     */
    void makeOrthonormalBasis(Vector3 * a, Vector3 * b, Vector3 * c);
};

#endif /* defined(__MSIM495__core__) */
