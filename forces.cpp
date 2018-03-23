//
//  Forces.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 2/13/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "forces.h"
#include "core.h"
#include <cmath>

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
    
    
    
    ///////////////////////// RigidBody /////////////////////////
    
    
    
    // Gravity //
    /////////////
    
    void Gravity::update_force(RigidBody * body, real duration) {
        if (!body->has_finite_mass()) return;
        
        Vector3 c_gravity = gravity * body->get_mass();
        body->add_force(c_gravity);
    }
    
    
    
    // Spring //
    ////////////
    
    Spring::Spring(
        Vector3 &left_connection_point,
        Vector3 &right_connection_point,
        RigidBody * other,
        real spring_constant,
        real rest_length
    ) {
        
    };
    
    void Spring::update_force(RigidBody * body, real duration) {
        Vector3 left_piws = body->get_point_in_world_space(connection_point_left);
        Vector3 right_piws = body->get_point_in_world_space(connection_point_right);
        
        Vector3 force = left_piws - right_piws;
        
        real magnitude = force.magnitude();
        magnitude = std::abs(magnitude);
        magnitude *= spring_constant;
        
        force.normalize();
        force *= -magnitude;
        body->add_force_at_point(force, left_piws);
    }
    
    // Force Registry //
    ////////////////////
    
        void ForceRegistry::add(
        RigidBody * RigidBody,
        ForceGenerator * fg
    ) {
        links.push_back(ForceRegistration{RigidBody, fg});
    }
    
    void ForceRegistry::remove(
        RigidBody * RigidBody,
        ForceGenerator * fg
    ) {
        int i = 0;
        for (ForceRegistration pfl : links) {
            if (pfl.body == RigidBody && pfl.fg == fg) {
                links.erase(links.begin() + i);
            }
            ++i;
        }
    }
    
    void ForceRegistry::clear() {
        links.clear();
    }
    
    void ForceRegistry::update_forces(real duration) {
        Registry::iterator i = links.begin();
        for (; i != links.end(); ++i) {
            i->fg->update_force(i->body, duration);
        }
    }
};