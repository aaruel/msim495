//
//  groundforce.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 2/27/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "groundforce.h"
#include "playground.h"
#include "physics.h"

namespace GroundForce {
    /**
     * Ground contact generator
     */
    class ParticleGround : public Physics::ParticleLink {
    public:
        /**
         * GROUND IS LEFT -- PARTICLE IS RIGHT
         */
        virtual unsigned add_contact(
            Physics::ParticleContact * contact,
            unsigned limit
        ) {
            Physics::real ground_height = left->get_position().y;
            Physics::real particle_height = right->get_position().y;
            Physics::real height = particle_height - ground_height;
        
            // Check if height is in bounds
            if (height > 0) {
                return 0;
            }
            
            // assign contact
            contact->left = left;
            contact->right = right;
            
            // Calculate normal vector
            Physics::Vector3 normal = Physics::Vector3(0, -1, 0);
            normal.normalize();
            
            contact->contact_normal = normal;
            contact->penetration = 0 - height;
            contact->restitution = 0.8;
            
            return 1;
        }
        
    };

    Physics::ParticleWorld world(1);
    Physics::Particle particle = Physics::Vector3(1, 1, 0);
    Physics::ParticleGravity gravity(Physics::Vector3(0,-9.8,0));
    Physics::Particle ground = Physics::Vector3();
    ParticleGround ground_contact;
    bool physics_paused = true;
    std::vector<Physics::Particle*> particles;
    
    void draw_particle() {
        Graphics::draw_sphere(particle, 0.2);
    }
    
    void step_physics() {
        if (!physics_paused) {
            Physics::real frame_duration = Graphics::get_seconds_per_frame();
            world.run_physics(frame_duration);
        }
    }
    
    int main(int argc, char ** argv) {
        Graphics::Graphics(800, 600, argc, argv);
        
        particle.set_mass(1);
        particles.push_back(&particle);
        ground_contact.left = &ground;
        ground_contact.right = &particle;
        world.pass_particles(&particles);
        world.registry.add(&particle, &gravity);
        world.contact_generators.push_back(&ground_contact);
        
        Graphics::register_fire([](){
            physics_paused = !physics_paused;
        }, ENTER_KEY);
        Graphics::push_draw_pipeline(Graphics::draw_ground);
        Graphics::push_draw_pipeline(Graphics::draw_reference_points);
        Graphics::push_draw_pipeline(draw_particle);
        Graphics::push_draw_pipeline(step_physics);
        
        Graphics::start();
        return 0;
    }
}