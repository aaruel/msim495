//
//  collisionassignment.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 4/4/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "collisionassignment.h"
#include "physics.h"
#include "playground.h"
#include <random>

namespace CollisionAssignment {


    int main(int argc, char ** argv) {
        srand((unsigned)time(nullptr));
        auto r = [](){return (float)(rand() & 1) - (float)(rand() & 1);};
        
        Graphics::Graphics(300, 300, argc, argv);
        
        Physics::Vector3 map_size = Physics::Vector3(300, 300, 0);
        Physics::Object objects[40];
        Physics::BSPObjects a;
        Physics::BSPPlanes p;
        
        // objects
        for (int i = 0; i < 40; ++i) {
            objects[i] = Physics::Object(map_size);
            a.push_back(&objects[i]);
            a[i]->set_velocity(Physics::Vector3(r(), r(), 0));
            a[i]->set_mass(1);
        }
        
        // walls
        p.push_back(Physics::Plane( Physics::Vector3(150, 30, 0), Physics::Plane::NORTH() ));
        p.push_back(Physics::Plane( Physics::Vector3(30, 150, 0), Physics::Plane::EAST() ));
        p.push_back(Physics::Plane( Physics::Vector3(150,270, 0), Physics::Plane::SOUTH() ));
        p.push_back(Physics::Plane( Physics::Vector3(270,150, 0), Physics::Plane::WEST() ));
        
        Physics::BSPTree t(&p, &a);
        
        bool physics = false;
        auto plane_colors = Graphics::random_color();
        auto object_colors = Graphics::random_color();
        
        Graphics::register_fire([&](){
            physics = !physics;
        }, 's');
        
        Graphics::push_draw_pipeline([&]() {
            Graphics::orthographic_render([&](int x, int y){
            
                // Draw objects
                object_colors();
                auto it = a.begin();
                for (; it != a.end(); ++it) {
                    Graphics::draw_sphere_no_color(**it, 3);
                    if (physics) (*it)->integrate(0.33);
                }
                
                // Draw planes
                plane_colors();
                auto it2 = p.begin();
                for (; it2 != p.end(); ++it2) {
                    Graphics::draw_2d_plane(*it2, true);
                }
                
                // perform collision detection
                t.collision_detection();
                
            });
        });
        
        Graphics::start();
        return 0;
    }
}