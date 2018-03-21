//
//  Forces.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 2/13/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "forces.h"
#include "core.h"

namespace Physics {
    // Force Generator //
    /////////////////////
    
    void ParticleForceRegistrar::add(
        Particle * particle,
        ParticleForceGenerator * fg
    ) {
        links.push_back(ParticleForceLink{particle, fg});
    }
    
    void ParticleForceRegistrar::remove(
        Particle * particle,
        ParticleForceGenerator * fg
    ) {
        int i = 0;
        for (ParticleForceLink pfl : links) {
            if (pfl.particle == particle && pfl.fg == fg) {
                links.erase(links.begin() + i);
            }
            ++i;
        }
    }
    
    void ParticleForceRegistrar::clear() {
        links.clear();
    }
    
    bool ParticleForceRegistrar::check_force_registered(
        Particle * particle,
        ParticleForceGenerator * fg
    ) {
        auto i = links.begin();
        for (; i != links.end(); ++i) {
            if (i->particle == particle && i->fg == fg) {
                return true;
            }
        }
        return false;
    }
    
    void ParticleForceRegistrar::update_forces(real duration) {
        Registry::iterator i = links.begin();
        for (; i != links.end(); ++i) {
            i->fg->update_force(i->particle, duration);
        }
    }
    
    
    
    // ParticleGravity //
    /////////////////////
    
    void ParticleGravity::update_force(Particle * particle, real duration) {
        if (particle->get_mass() == 0) return;
        
        // F = m * g
        particle->add_impulse(gravity * particle->get_mass());
    }
    
    
    
    // ParticleSpring //
    ////////////////////
    
    void ParticleSpring::update_force(Particle * particle, real duration) {
        // Calculate vector of spring
        Vector3 force = particle->get_position();
        force -= end->get_position();
        
        // Calculate magnitude of spring
        real magnitude = force.magnitude();
        magnitude = fabsf(magnitude - rest_length);
        magnitude *= spring_constant;
        
        // Calculate final force
        force.normalize();
        force *= -magnitude;
        particle->add_impulse(force);
    }
    
    
    
    // ParticleStiffSpring //
    /////////////////////////
    
    void ParticleStiffSpring::update_force(Particle * particle, real duration) {
        if (particle->get_mass() <= 0.0) return;
        
        // Calculate vector of spring
        Vector3 position = particle->get_position();
        position -= end->get_position();
        
        // Calculate constants
        real gamma = 0.5f * sqrtf(4 * spring_constant - damping * damping);
        if (gamma == 0.f) return;
        Vector3 constant = position * (damping / (2.0f * gamma))
            + particle->get_velocity() * (1.0f / gamma);
        
        // Calculate target position
        Vector3 target = position * cosf(gamma * duration)
            + constant * sinf(gamma * duration);
        target *= expf(-0.5f * duration * damping);
        
        // calculate resulting acceleration
        Vector3 acceleration = (target - position) * (1.0f / duration * duration)
            - particle->get_velocity() * duration;
        particle->add_impulse(acceleration * particle->get_mass());
    }
};