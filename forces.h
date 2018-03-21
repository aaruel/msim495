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
    
};

#endif /* defined(__MSIM495__Forces__) */
