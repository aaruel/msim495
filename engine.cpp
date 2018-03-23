//
//  engine.cpp
//  MSIM495
//

#include "engine.h"

namespace Physics {
    ParticleWorld::ParticleWorld(
        unsigned max_contacts,
        unsigned iterations
    ) : resolver(iterations),
        max_contacts(max_contacts)
    {
        contacts = new ParticleContact[max_contacts];
        calculate_iterations = (iterations == 0);
    }
    
    ParticleWorld::~ParticleWorld() {
        delete [] contacts;
    }

    unsigned ParticleWorld::generate_contacts() {
        unsigned limit = max_contacts;
        ParticleContact * next_contact = contacts;
        
        ContactGenerators::iterator g = contact_generators.begin();
        for (; g != contact_generators.end(); ++g) {
            unsigned used = (*g)->add_contact(next_contact, limit);
            limit -= used;
            next_contact += used;
            if (limit <= 0) break;
        }
        
        return max_contacts - limit;
    }
    
    void ParticleWorld::integrate(real duration) {
        Particles::iterator p = particles->begin();
        for (; p != particles->end(); ++p) {
            (*p)->integrate(duration);
        }
    }
    
    void ParticleWorld::run_physics(real duration){
        registry.update_forces(duration);
        integrate(duration);
        unsigned used_contacts = generate_contacts();
        if (used_contacts) {
            if (calculate_iterations) resolver.set_iterations(used_contacts * 2);
            resolver.resolve_contacts(contacts, used_contacts, duration);
        }
    }
    
    
    
    // World //
    ///////////
    
    void World::start_frame() {
        RigidBodies::iterator b = bodies.begin();
        for (; b != bodies.end(); ++b) {
            b->clear_accumulator();
            b->calculate_derived_data();
        }
    }
    
    void World::integrate(real duration) {
        RigidBodies::iterator b = bodies.begin();
        for (; b != bodies.end(); ++b) {
            b->integrate(duration);
        }
    }
    
    void World::run_physics(real duration) {
        
        integrate(duration);
    }
}