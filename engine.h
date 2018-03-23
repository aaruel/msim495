//
//  engine.h
//  MSIM495
//

#ifndef __MSIM495__engine__
#define __MSIM495__engine__

#include "core.h"
#include "collision.h"
#include "forces.h"

namespace Physics {
    class ParticleWorld {
    public:
        typedef std::vector<Particle*> Particles;
        typedef std::vector<ParticleContactGenerator*> ContactGenerators;
        ParticleForceRegistrar registry;
        ParticleContactResolver resolver;
        ContactGenerators contact_generators;
        ParticleContact * contacts;
        unsigned max_contacts;
        bool calculate_iterations;
        
    protected:
        Particles * particles;
        
    public:
        ParticleWorld(
            unsigned max_contacts,
            unsigned iterations = 0
        );
        
        ~ParticleWorld();
        
        void start_frame();
        unsigned generate_contacts();
        void integrate(real duration);
        void run_physics(real duration);
        void pass_particles(Particles * p) { particles = p; }
        
    };
    
    
    
    class World {
    public:
        typedef std::vector<RigidBody> RigidBodies;
        
    protected:
        RigidBodies bodies;
        
    public:
        void start_frame();
        void run_physics(real duration);
        void integrate(real duration);
    };
}

#endif /* defined(__MSIM495__engine__) */
