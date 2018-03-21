//
//  assignment3.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 2/6/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "assignment3.h"
#include "playground.h"
#include "physics.h"
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <math.h>

namespace A3q1 {
    Physics::ParticleForceRegistrar particle_force_registrar;
    Physics::Particle target;
    Physics::Particle attraction_point;
    Physics::ParticleGravity gravity(Physics::Vector3{0, -1, 0});
    Physics::real frame_time = 1.0/30.0;

    void draw_reference_points() {
        for (int i = -3; i < 3; ++i) {
            for (int j = -3; j < 3; ++j) {
                Graphics::draw_sphere(Physics::Particle(
                    Physics::Vector3{
                        static_cast<Physics::real>(i*10.0),
                        0,
                        static_cast<Physics::real>(j*10.0)
                    }
                ), 0.1);
            }
        }
    }
    
    void step_physics() {
        printf("%f\n", Graphics::get_seconds_per_frame());
        
        Physics::Vector3 new_g = attraction_point.get_position() - target.get_position();
        Physics::real distance = target.get_position().distance(attraction_point.get_position());
        gravity.set_gravity(new_g * ( 1 / powf(distance, 2) ));
        particle_force_registrar.update_forces(frame_time);
        target.integrate(frame_time);
    }
    
    void draw_target() {
        Graphics::draw_pyramid(target);
        Graphics::draw_sphere(attraction_point, 0.1);
    }

    int main(int argc, char ** argv) {
        Graphics::Graphics(800, 600, argc, argv);
        
        // Physics
        target.set_position(Physics::Vector3(0, 1, 0));
        target.set_mass(1);
        target.set_velocity(Physics::Vector3(-1, 0, 0));
        attraction_point.set_position(Physics::Vector3(2, 2, -4));
        
        particle_force_registrar.add(
            &target,
            (Physics::ParticleForceGenerator*)&gravity
        );
        
        // Graphics
        Graphics::push_draw_pipeline(Graphics::draw_ground);
        Graphics::push_draw_pipeline(draw_reference_points);
        Graphics::push_draw_pipeline(draw_target);
        Graphics::push_draw_pipeline(step_physics);
        
        Graphics::start();
        return 0;
    };
}






namespace A3q2 {
    Physics::ParticleForceRegistrar particle_force_registrar;
    Physics::Particle target;
    Physics::Particle suspend_point;
    Physics::ParticleStiffSpring buoyancy(
        &suspend_point,
        2000,
        2
    );
    Physics::ParticleGravity gravity(Physics::Vector3{0, -2.5, 0});
    Physics::real frame_time = 1.0/30.0;

    void draw_reference_points() {
        for (int i = -3; i < 3; ++i) {
            for (int j = -3; j < 3; ++j) {
                Graphics::draw_sphere(Physics::Particle(
                    Physics::Vector3{
                        static_cast<Physics::real>(i*10.0),
                        0,
                        static_cast<Physics::real>(j*10.0)
                    }
                ), 0.1);
            }
        }
    }
    
    void step_physics() {
        particle_force_registrar.update_forces(frame_time);
        target.integrate(frame_time);
    }
    
    void draw_target() {
        Graphics::draw_sphere(target, 0.5);
        Graphics::draw_sphere(suspend_point, 0.1);
    }

    int main(int argc, char ** argv) {
        Graphics::Graphics(800, 600, argc, argv);
        
        // Physics
        target.set_position(Physics::Vector3(0, 0, 0));
        target.set_mass(1);
        target.set_damping(0.6);
        suspend_point.set_position(Physics::Vector3(0, 5, 0));
        
        particle_force_registrar.add(
            &target,
            (Physics::ParticleForceGenerator*)&buoyancy
        );
        
        particle_force_registrar.add(
            &target,
            (Physics::ParticleForceGenerator*)&gravity
        );
        
        // Graphics
        Graphics::push_draw_pipeline(Graphics::draw_ground);
        Graphics::push_draw_pipeline(draw_reference_points);
        Graphics::push_draw_pipeline(draw_target);
        Graphics::push_draw_pipeline(step_physics);
        
        Graphics::start();
        return 0;
    };
}






namespace A3q3 {
    class Claustrophobe : public Physics::Particle {
        Physics::real personal_space = 2;
        
    public:
        Physics::ParticleGravity gravity;
        Physics::ParticleSpring spring;
    
        Claustrophobe(Physics::Vector3 position)
            : gravity(Physics::Vector3{0,0,0}), spring(nullptr, 0, 0) {
            set_mass(10);
            set_position(position);
        }
        
        Physics::ParticleSpring * get_spring() {
            return &spring;
        }
        
        void register_forces(
            std::vector<Claustrophobe> * c_list,
            Physics::ParticleForceRegistrar * pfr
        ) {
            // this pointer doesn't assign correctly in constructor
            spring = Physics::ParticleSpring(this, 10, 10);
            
            // register force towards middle
            Physics::Vector3 vec = get_position();
            vec.invert();
            vec.normalize();
            gravity.set_gravity(vec);
            pfr->add(this, &this->gravity);
            
            // register personal space spring forces
            auto i = c_list->begin();
            for (; i != c_list->end(); ++i) {
                if (&(*i) != this) {
                    Claustrophobe * other = &(*i);
                    Physics::real distance = get_position().distance(i->get_position());
                    if (
                        distance < personal_space
                        && !pfr->check_force_registered(this, other->get_spring())
                    ) {
                        pfr->add(other, &this->spring);
                    }
                }
            }
        };
        
        void update(Physics::real duration) {
            integrate(duration);
        }
    };

    Physics::ParticleForceRegistrar particle_force_registrar;
    Physics::Particle target;
    Physics::Particle attraction_point;
    std::vector<Claustrophobe> c_list;
    Physics::real frame_time = 1.0/60.0;
    
    void load_claustrophobes() {
        for (int i = -3; i < 3; ++i) {
            for (int j = -3; j < 3; ++j) {
                c_list.push_back(Claustrophobe(
                    Physics::Vector3{
                        static_cast<Physics::real>(i*10.0),
                        0,
                        static_cast<Physics::real>(j*10.0)
                    }
                ));
            }
        }
    }
    
    void draw_claustrophobes() {
        auto i = c_list.begin();
        for (; i != c_list.end(); ++i) {
            Graphics::draw_sphere(*i, 0.1);
        }
    }
    
    void process_claustrophobes() {
        auto i = c_list.begin();
        for (; i != c_list.end(); ++i) {
            i->register_forces(&c_list, &particle_force_registrar);
        }
    }
    
    void step_physics() {
        particle_force_registrar.update_forces(frame_time);
        
        auto i = c_list.begin();
        for (; i != c_list.end(); ++i) {
            i->integrate(frame_time);
        }
        
        particle_force_registrar.clear();
    }
    
    void draw_target() {
    }

    int main(int argc, char ** argv) {
        Graphics::Graphics(800, 600, argc, argv);
        
        load_claustrophobes();
        
        // Physics
        
        // Graphics
        Graphics::push_draw_pipeline(Graphics::draw_ground);
        Graphics::push_draw_pipeline(draw_claustrophobes);
        Graphics::push_draw_pipeline(draw_target);
        Graphics::push_draw_pipeline(process_claustrophobes);
        Graphics::push_draw_pipeline(step_physics);
        
        Graphics::start();
        return 0;
    };
}