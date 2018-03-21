//
//  collision.h
//  MSIM495
//
//  Created by Aaron Ruel on 2/13/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#ifndef __MSIM495__collision__
#define __MSIM495__collision__

#include <stdio.h>
#include "core.h"

namespace Physics {
    /*
     * Forward declaration
     */
    class ParticleContactResolver;
    
    /**
     * Data and resolution for contact event
     */
    class ParticleContact {
        friend class ParticleContactResolver;
    
    public:
        /*
         * Two particles involved in a contact
         */
        Particle * left;
        Vector3 left_movement;
        Particle * right;
        Vector3 right_movement;
        
        /*
         * Value 0 - 1
         * How much velocity is retained after collision
         */
        real restitution;
        
        /*
         * Direction of the contact
         */
        Vector3 contact_normal;
        
        /*
         * On contact, depth of penetration between two objects
         */
        real penetration = 0;
        
    protected:
        /*
         * Resolves the velocity calculation
         */
        void resolve(real duration);
        
        /*
         * Calculates resulting velocity at contact
         */
        real calculate_separating_velocity();
        
    private:
        /**
         * Turns velocity to impulse resolution
         */
        void resolve_velocity(real duration);
        
        /**
         * Handles inter-penetration between contact points
         * Idea is to get a delta position relative to mass
         * in the direction of the contact_normal
         */
         void resolve_interpenetration(real duration);
    };
    
    /**
     * Simulation wide contact resolution
     */
    class ParticleContactResolver {
    protected:
        /*
         * Iteration limit and tracker
         */
        unsigned iterations;
        unsigned used_iterations;

    public:
        /*
         * Constructors
         */
        ParticleContactResolver(unsigned max_iterations)
            : iterations(max_iterations) {}
        
        /*
         * Getters / Setters
         */
        void set_iterations(unsigned max_iterations);
        
        /**
         * Resolve contact for both inter-penetration and 
         * velocity
         */
        void resolve_contacts(
            ParticleContact * contact_array,
            unsigned num_contacts,
            real duration
        );
    };
    
    /**
     * Base class for contact based generators
     */
    class ParticleContactGenerator {
    public:
    
        /**
         *
         */
        virtual unsigned add_contact(
            ParticleContact * particle,
            unsigned limit
        ) = 0;
    };
    
    
    /**
     * Base class for cable and rods
     */
    class ParticleLink : public ParticleContactGenerator {
    public:
        /*
         * Ends of the linkage
         */
        Particle * left;
        Particle * right;
    
    protected:
        /**
         * Returns length of the linkage
         */
        real current_length();
    
    public:
        /**
         * Passthrough add_contact
         */
        virtual unsigned add_contact(
            ParticleContact * contact,
            unsigned limit
        ) = 0;
    };
    
    
    /**
     * Cable linkage simulation
     */
    class ParticleCable : public ParticleLink {
    public:
        /**
         * Max length of cable
         */
        real max_length;
        
        /**
         * Bounciness
         */
        real restitution;
        
    public:
        virtual unsigned add_contact(
            ParticleContact * contact,
            unsigned limit
        );
        
    };
    
    
    /**
     * Cable linkage simulation
     */
    class ParticleRod : public ParticleLink {
    public:
        /**
         * Length of rod
         */
        real max_length;
        
    public:
        virtual unsigned add_contact(
            ParticleContact * contact,
            unsigned limit
        );
        
    };
};

#endif /* defined(__MSIM495__collision__) */
