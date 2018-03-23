//
//  Forces.h
//  MSIM495
//
//  Created by Aaron Ruel on 2/13/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#ifndef __MSIM495__Forces__
#define __MSIM495__Forces__

#include <stdio.h>
#include "core.h"

namespace Physics {
    /**
     * Interface for derived classes to apply forces
     * to particles in a more constructed manner
     */
    class ParticleForceGenerator {
    public:
        virtual void update_force(Particle *p, real time) = 0;
    };
    
    /*
     * Container and manager for link between particle 
     * and force generators
     */
    class ParticleForceRegistrar {
    protected:
        /**
         * Bundles particle with force generator
         */
        struct ParticleForceLink {
            Particle * particle;
            ParticleForceGenerator * fg;
        };
    
        /*
         * Link Container
         */
        typedef std::vector<ParticleForceLink> Registry;
        Registry links;
    
    public:
        /*
         * Methods
         */
    
        /**
         * Add link between particle and force generator
         */
        void add(Particle * particle, ParticleForceGenerator * fg);
    
        /**
         * Remove link between particle and force generator
         */
        void remove(Particle * particle, ParticleForceGenerator * fg);
    
        /**
         * Clear all connections
         */
        void clear();
    
        /**
         * Check if link between particle and force generator
         * exists
         */
        bool check_force_registered(
            Particle * particle,
            ParticleForceGenerator * fg
        );
    
        /**
         * Updates all connections for one time step
         */
        void update_forces(real duration);
    };
    
    
    
    /**
     * General gravitational force generator
     */
    class ParticleGravity : public ParticleForceGenerator {
        /*
         * force of gravity constant
         */
        Vector3 gravity;
        
    public:
        /*
         * Constructors
         */
        ParticleGravity(Vector3 v) : gravity(v) {}
        
        /*
         * Getters / Setters
         */
        void set_gravity(Vector3 g) { gravity = g; }
        
        /**
         * Particle force generator implementation
         */
        void update_force(Particle * particle, real duration);
    };
    
    
    
    /*
     * Attached spring force generator
     */
    class ParticleSpring : public ParticleForceGenerator {
        /* 
         * particle at one end of the spring
         */
        Particle * end;
        
        /* 
         * Spring Constant k
         */
        real spring_constant;
        
        /**
         * Resting length of spring
         */
        real rest_length;
    public:
        
        /*
         * Constructors
         */
        ParticleSpring(Particle * e, real s_c, real r_l) :
            end(e), spring_constant(s_c), rest_length(r_l){}
        
        /**
         * Particle force generator implementation
         */
        void update_force(Particle * particle, real duration);
    };
    
    
    
    /*
     * Similar to spring with damping factor
     */
    class ParticleStiffSpring : public ParticleForceGenerator {
        /* 
         * particle at one end of the spring
         */
        Particle * end;
        
        /* 
         * Spring Constant k
         */
        real spring_constant;
        
        /*
         * Resting length of spring
         */
        real damping;
    public:
    
        /*
         * Constructors
         */
        ParticleStiffSpring(Particle * e, real s_c, real d) :
            end(e), spring_constant(s_c), damping(d){}
        
        /**
         * Particle force generator implementation
         */
        void update_force(Particle * particle, real duration);
    };
    
    
    
    ///////////////////////// RigidBody /////////////////////////
    
    
    
    class ForceGenerator {
    public:
        virtual void update_force(RigidBody * body, real duration) = 0;
    };
    
    class Gravity : public ForceGenerator {
        Vector3 gravity;
        
    public:
        Gravity(Vector3 &gravity);
        
        virtual void update_force(RigidBody * body, real duration);
    };
    
    class Spring : public ForceGenerator {
        Vector3 connection_point_left;
        Vector3 connection_point_right;
        
        RigidBody * other;
        
        real spring_constant;
        real rest_length;
        
    public:
        Spring(
            Vector3 &left_connection_point,
            Vector3 &right_connection_point,
            RigidBody * other,
            real spring_constant,
            real rest_length
        );
        
        virtual void update_force(RigidBody * body, real duration);
    };
    
    
    
    class ForceRegistry {
    protected:
        /**
         * Bundles RigidBody with force generator
         */
        struct ForceRegistration {
            RigidBody * body;
            ForceGenerator * fg;
        };
    
        /*
         * Link Container
         */
        typedef std::vector<ForceRegistration> Registry;
        Registry links;
    
    public:
        /*
         * Methods
         */
    
        /**
         * Add link between RigidBody and force generator
         */
        void add(RigidBody * RigidBody, ForceGenerator * fg);
    
        /**
         * Remove link between RigidBody and force generator
         */
        void remove(RigidBody * RigidBody, ForceGenerator * fg);
    
        /**
         * Clear all connections
         */
        void clear();
    
        /**
         * Updates all connections for one time step
         */
        void update_forces(real duration);
    };
};

#endif /* defined(__MSIM495__Forces__) */
