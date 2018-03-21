//
//  trebuchet.cpp
//  MSIM495
//

#include "trebuchet.h"
#include "playground.h"
#include "physics.h"
#include <stdlib.h>

#define CONTACT_OBJECTS 10
#define $(contents) [](){contents}

namespace Trebuchet {
    
    Physics::ParticleWorld world(
        CONTACT_OBJECTS
    );
    
    std::vector<Physics::Particle*> particles;
    bool physics_enabled = false;
    bool projectile_released = false;
    Physics::ParticleRod rod;
    Physics::ParticleRod rod2;
    Physics::ParticleCable arm;
    Physics::ParticleRod strength;
    Physics::real height_offset = 2.f;
    Physics::real counterweight = 1000;
    Physics::real score = 0.f;
    Physics::ParticleGravity gravity(Physics::Vector3(0,-10,0));
    Physics::ParticleGravity inverse(Physics::Vector3(0, 10,0));
    
    Physics::Particle * anchor = new Physics::Particle();
    Physics::Particle * pendulum = new Physics::Particle();
    Physics::Particle * hook = new Physics::Particle();
    Physics::Particle * projectile = new Physics::Particle();
    
    void initialize() {
        world.pass_particles(&particles);
        
        // Set positions
        anchor->set_position(Physics::Vector3(0, 2, 0));
        pendulum->set_position(Physics::Vector3(2, 2, 0));
        hook->set_position(Physics::Vector3(-2, 2, 0));
        projectile->set_position(Physics::Vector3(-2, 1, 0));
        
        // Set mass
        anchor->set_mass(0);
        pendulum->set_mass(counterweight);
        hook->set_mass(1);
        projectile->set_mass(1);
        
        // Make available to renderer
        particles.push_back(anchor);
        particles.push_back(pendulum);
        particles.push_back(hook);
        particles.push_back(projectile);
        
        // Force aggregators
        world.registry.add(pendulum, &gravity);
        world.registry.add(hook, &gravity);
        world.registry.add(projectile, &gravity);
        
        // Particle collision constructs
        // - Will aggregate mass with two end rods
        // connecting to the anchor and one rod
        // connecting each end of the bar
        rod.left = anchor;
        rod.right = pendulum;
        rod.max_length = 2;
        
        rod2.left = anchor;
        rod2.right = hook;
        rod2.max_length = 2;
        
        strength.left = hook;
        strength.right = pendulum;
        strength.max_length = 4;
        
        arm.left = hook;
        arm.right = projectile;
        arm.max_length = 1;
        arm.restitution = 0.5;
        
        // Push collision constructs
        world.contact_generators.push_back(&rod);
        world.contact_generators.push_back(&rod2);
        world.contact_generators.push_back(&strength);
        world.contact_generators.push_back(&arm);
    }
    
    void release_projectile() {
        auto i = world.contact_generators.begin();
        for (; i != world.contact_generators.end(); ++i) {
            if (*i == &arm) {
                world.contact_generators.erase(i);
                projectile_released = true;
                break;
            }
        }
    }
    
    void draw_objects() {
        Util::map(particles, [](Physics::Particle * particle){
            Graphics::draw_sphere(*particle, 0.2);
        });
        
        // Draw Base
        Graphics::draw_3d_rect(
            Physics::Particle(),
            1, 0.3, 1
        );
        
        // Draw stem
        Graphics::draw_3d_rect(
            Physics::Particle(),
            0.2, 4, 0.2
        );
        
        // Draw bar
        Physics::Vector3 pen_pos = pendulum->get_position();
        Physics::Vector3 pen_dir = anchor->get_position().direction(pen_pos);
        Physics::real angle = pen_dir.angle(Physics::Vector3(1, 0, 0));
        Graphics::draw_3d_rect(
            Physics::Particle(0, 2, 0),
            4, 0.2, 0.2,
            0, 0, -1,
            Physics::rads_to_degs(angle)
        );
    }
    
    void draw_score() {
        Physics::Vector3 proj_pos = projectile->get_position();
        if (proj_pos.x > 0 && proj_pos.y > 0) score += 1;
    
        Graphics::orthographic_render([](unsigned window_width, unsigned window_height){
            char * score_text = (char *)malloc(255);
            char * counterweight_text = (char*)malloc(255);
            sprintf(score_text, "Score: %d", (int)score);
            sprintf(counterweight_text, "Counterweight: %d g", (int)counterweight);
            Graphics::render_text((const char *)score_text, Physics::Vector3(50, window_height - 60, 0));
            Graphics::render_text((const char *)counterweight_text, Physics::Vector3(50, window_height-80, 0));
            Graphics::render_text("Press Enter to Start", Physics::Vector3(50, window_height-100, 0));
            Graphics::render_text("'r': Release 'e': Reset", Physics::Vector3(50, window_height-120, 0));
            Graphics::render_text("'i': Increase Weight", Physics::Vector3(50, window_height-140, 0));
            Graphics::render_text("'u': Decrease Weight", Physics::Vector3(50, window_height-160, 0));
            free((void*)score_text);
            free((void*)counterweight_text);
        });
    }
    
    void calculate_physics() {
        Physics::real duration = Graphics::get_seconds_per_frame();
        if (physics_enabled) world.run_physics(duration);
    }
    
    void reset() {
        physics_enabled = false;
        projectile_released = false;
        particles.clear();
        world.registry.clear();
        world.contact_generators.clear();
        score = 0.f;
        Util::map(particles, [](Physics::Particle * particle){
            particle->clear();
        });
        initialize();
    }
    
    void debug() {
    }
    
    void destruct() {
        Util::map(particles, [](Physics::Particle * particle){
            delete particle;
        });
    }

    int main(int argc, char ** argv) {
        Graphics::Graphics(800, 600, argc, argv);
        
        Graphics::register_fire($(physics_enabled = !physics_enabled;), ENTER_KEY);
        Graphics::register_fire(release_projectile, 'r');
        Graphics::register_fire(reset, 'e');
        Graphics::register_fire($(
            counterweight += 10;
            pendulum->set_mass(counterweight);
        ), 'i');
        Graphics::register_fire($(
            if (counterweight - 10 == 0) return;
            counterweight -= 10;
            pendulum->set_mass(counterweight);
        ), 'u');
        
        initialize();
        
        Graphics::push_draw_pipeline(draw_score);
        Graphics::push_draw_pipeline(Graphics::draw_ground);
        Graphics::push_draw_pipeline(Graphics::draw_reference_points);
        Graphics::push_draw_pipeline(draw_objects);
        Graphics::push_draw_pipeline(calculate_physics);
        Graphics::push_draw_pipeline(debug);
        
        Graphics::start();
        
        destruct();
        
        return 0;
    }
}