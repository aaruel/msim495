//
//  collision.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 2/13/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "collision.h"
#include <limits.h>

namespace Physics {

    // ParticleContact //
    /////////////////////
    
    void ParticleContact::resolve(real duration) {
        resolve_velocity(duration);
        resolve_interpenetration(duration);
    }
    
    real ParticleContact::calculate_separating_velocity() {
        // Get the difference of the two colliding velocities
        Vector3 relative_velocity = left->get_velocity();
        if (right) relative_velocity -= right->get_velocity();
        
        // Point velocity towards contact_normal
        return relative_velocity * contact_normal;
    }
    
    void ParticleContact::resolve_velocity(real duration) {
        real separating_velocity = calculate_separating_velocity();
        if (separating_velocity > 0) {
            // Non-negative value means before or after separation
            return;
        }
        
        real new_separation_velo = -separating_velocity * restitution;
        
        // For inter-penetration checking
        // Get velocity due to acceleration
        Vector3 acceleration_from_velo = left->get_acceleration();
        if (right) acceleration_from_velo -= right->get_acceleration();
        
        real acceleration_causality =
            acceleration_from_velo
            * contact_normal
            * duration;
        
        // remove excess velocity from acceleration build up
        if (acceleration_causality < 0) {
            new_separation_velo +=
                restitution
                * acceleration_causality;
            
            // Make sure we don't end up with a negative
            // initial separation velocity
            if (new_separation_velo < 0) new_separation_velo = 0;
        }
        
        // Calculate new separation velocity
        real delta_velocity =
            new_separation_velo
            - separating_velocity;
        
        // process total inverse mass
        real total_inverse_mass = left->get_inverse_mass();
        if (right) total_inverse_mass += right->get_inverse_mass();
        if (total_inverse_mass == 0.f) return;
        
        // derive impulse from velocity
        real impulse = delta_velocity / total_inverse_mass;
        Vector3 impulse_per_inverse_mass = contact_normal * impulse;
        
        // Apply the impulse in direction of contact and
        // proportional to inverse mass
        left->set_velocity(
            left->get_velocity()
            + impulse_per_inverse_mass
            * left->get_inverse_mass()
        );
        
        if (right) {
            right->set_velocity(
                right->get_velocity()
                + impulse_per_inverse_mass
                * -right->get_inverse_mass()
            );
        }
    }
    
    void ParticleContact::resolve_interpenetration(real duration){
        // Penetration 0 or negative if no penetration
        if (penetration <= 0) return;
        
        real total_inverse_mass = left->get_inverse_mass();
        if (right) total_inverse_mass += right->get_inverse_mass();
        
        // Ignore if infinite mass
        if (total_inverse_mass <= 0) return;
        
        // Find delta position relative to mass
        Vector3 move_per_inverse_mass =
            contact_normal
            * (penetration / total_inverse_mass);
        
        // Calculate delta movements
        left_movement =
            move_per_inverse_mass
            * left->get_inverse_mass();
        
        if (right) {
            right_movement =
                move_per_inverse_mass
                * -right->get_inverse_mass();
        }
        else {
            right_movement.clear();
        }
        
        // Apply delta movements
        left->set_position(
            left->get_position() + left_movement
        );
        
        if (right) {
            right->set_position(
                right->get_position() + right_movement
            );
        }
        
    }
    
    
    
    // ParticleContactResolver //
    /////////////////////////////
    
    void ParticleContactResolver::resolve_contacts(
        ParticleContact * contact_array,
        unsigned num_contacts,
        real duration
    ) {
        used_iterations = 0;
        
        // Inefficient!
        while (used_iterations < iterations) {
            real max = MAXFLOAT;
            unsigned max_index = num_contacts;
            for (unsigned i = 0; i < num_contacts; ++i) {
                // Find contact with largest separating velocity
                real separating_velo = contact_array[i].calculate_separating_velocity();
                
                bool separation_condition = (
                    separating_velo < 0
                    || contact_array[i].penetration > 0
                );
                
                if (
                    separating_velo < max
                    && separation_condition
                ) {
                    max = separating_velo;
                    max_index = i;
                }
            }
            
            // if nothing to resolve, break
            if (max_index == num_contacts) break;
            
            // resolve contact
            contact_array[max_index].resolve(duration);
            
            // update interpenetration for all particles
            Vector3 move_left = contact_array[max_index].left_movement;
            Vector3 move_right = contact_array[max_index].right_movement;
            for (unsigned i = 0; i < num_contacts; ++i) {
                if (contact_array[i].left == contact_array[max_index].left) {
                    contact_array[i].penetration -= move_left * contact_array[i].contact_normal;
                }
                else if (contact_array[i].left == contact_array[max_index].right) {
                    contact_array[i].penetration -= move_right * contact_array[i].contact_normal;
                }
                if (contact_array[i].right) {
                    if (contact_array[i].right == contact_array[max_index].left) {
                        contact_array[i].penetration += move_left * contact_array[i].contact_normal;
                    }
                    else if (contact_array[i].right == contact_array[max_index].right) {
                        contact_array[i].penetration += move_right * contact_array[i].contact_normal;
                    }
                }
            }
            
            // track used iterations
            ++used_iterations;
        };
    }
    
    void ParticleContactResolver::set_iterations(unsigned max_iterations) {
        iterations = max_iterations;
    }
    
    
    
    // ParticleLink //
    //////////////////
    
    real ParticleLink::current_length() {
        return left->get_position().distance(right->get_position());
    }
    
    
    
    // ParticleCable //
    ///////////////////
    
    unsigned ParticleCable::add_contact(
        ParticleContact * contact,
        unsigned limit
    ) {
        real length = current_length();
        
        // Check if cable overextended
        // ?
        if (length < max_length) {
            return 0;
        }
        
        // assign contact
        contact->left = left;
        contact->right = right;
        
        // Calculate normal vector
        Vector3 normal = right->get_position() - left->get_position();
        normal.normalize();
        
        contact->contact_normal = normal;
        contact->penetration = length - max_length;
        contact->restitution = restitution;
        
        return 1;
    }
    
    
    
    // ParticleRod //
    /////////////////
    
    unsigned ParticleRod::add_contact(
        ParticleContact * contact,
        unsigned limit
    ) {
        real length = current_length();
        
        // Check if overextended
        if (length == max_length) {
            return 0;
        }
        
        contact->left = left;
        contact->right = right;
        
        Vector3 normal = right->get_position() - left->get_position();
        normal.normalize();
        
        // Direction of normal depends on compression or expansion
        if (length > max_length) {
            contact->contact_normal = normal;
            contact->penetration = length - max_length;
        }
        else {
            contact->contact_normal = normal * -1;
            contact->penetration = max_length - length;
        }
        
        // zero resitution
        contact->restitution = 0;
        
        return 1;
    }
};