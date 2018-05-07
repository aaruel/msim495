//
//  sailboat.cpp
//  MSIM495
//

#include "sailboat.h"

/*
 * The sailboat demo.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include "physics.h"
#include "flightsim.h"
#include "playground.h"
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <stdio.h>
#include <cassert>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace Physics {
    class Buoyancy : public ForceGenerator {
        /**
         * The maximum submersion depth of the object before
         * it generates its maximum buoyancy force.
         */
        real maxDepth;

        /**
         * The volume of the object.
         */
        real volume;

        /**
         * The height of the water plane above y=0. The plane will be
         * parallel to the XZ plane.
         */
        real waterHeight;

        /**
         * The density of the liquid. Pure water has a density of
         * 1000kg per cubic meter.
         */
        real liquidDensity;

        /**
         * The centre of buoyancy of the rigid body, in body coordinates.
         */
        Vector3 centreOfBuoyancy;

    public:

        /** Creates a new buoyancy force with the given parameters. */
        Buoyancy(const Vector3 &cOfB,
            real maxDepth, real volume, real waterHeight,
            real liquidDensity = 1000.0f)
        {
            centreOfBuoyancy = cOfB;
            Buoyancy::liquidDensity = liquidDensity;
            Buoyancy::maxDepth = maxDepth;
            Buoyancy::volume = volume;
            Buoyancy::waterHeight = waterHeight;
        }
        
        Physics::real get_sailboat_height(Physics::RigidBody &sailboat) {
            return sailboat.get_point_in_world_space(centreOfBuoyancy).y;
        }

        /**
         * Applies the force to the given rigid body.
         */
        virtual void update_force(RigidBody *body, real duration) {
            // Calculate the submersion depth
            Vector3 pointInWorld = body->get_point_in_world_space(centreOfBuoyancy);
            real depth = pointInWorld.y;

            // Check if we're out of the water
            if (depth >= waterHeight + maxDepth) return;
            Vector3 force(0,0,0);

            // Check if we're at maximum depth
            if (depth <= waterHeight - maxDepth)
            {
                force.y = liquidDensity * volume;
                body->add_force_at_body_point(force, centreOfBuoyancy);
                return;
            }

            // Otherwise we are partly submerged
            force.y = liquidDensity * volume *
                (depth - maxDepth - waterHeight) / 2 * maxDepth;
            body->add_force_at_body_point(force, centreOfBuoyancy);
        }
    };
}

/**
 * The main demo class definition.
 */
class SailboatDemo {
    Physics::Buoyancy buoyancy;

    Physics::Aero sail;
    Physics::RigidBody sailboat;
    Physics::ForceRegistry registry;

    Physics::Vector3 windspeed;
    Physics::Vector3 propulsion;

    float sail_control;

public:
    /** Creates a new demo object. */
    SailboatDemo();
    
    /** Returns the window title for the demo. */
    const char* getTitle();

    /** Display the particles. */
    void display();
    void text();

    /** Update the particle positions. */
    void update();

    /** Handle a key press. */
    void key(unsigned char key);
};

// Method definitions
SailboatDemo::SailboatDemo() :

sail(Physics::Matrix3(0,0,0, 0,0,0, 0,0,-1.0f),
     Physics::Vector3(2.0f, 0, 0), &windspeed),

buoyancy(Physics::Vector3(0.0f, 0.5f, 0.0f), 1.0f, 3.0f, 1.6f),

sail_control(0),

windspeed(0,0,0)
{
    // Set up the boat's rigid body.
    sailboat.set_position(Physics::Vector3(0, 1.6f, 0));
    sailboat.set_orientation(Physics::Quaternion(1,0,0,0));

    sailboat.set_velocity(Physics::Vector3());
    sailboat.set_rotation(Physics::Vector3());

    sailboat.set_mass(200.0f);
    Physics::Matrix3 it;
    Physics::Vector3 dd = Physics::Vector3(2,1,1);
    it.set_block_inertia_tensor(dd, 100.0f);
    sailboat.set_inertia_tensor(it);

    sailboat.set_damping(0.8f, 0.8f);

    sailboat.set_acceleration(Physics::Vector3(0, -9.8, 0));
    sailboat.calculate_derived_data();

    sailboat.set_awake(true);
    sailboat.set_can_sleep(false);

    registry.add(&sailboat, &sail);
    registry.add(&sailboat, &buoyancy);
}

static void drawBoat()
{
    // Left Hull
    glPushMatrix();
    glTranslatef(0, 0, -1.0f);
    glScalef(2.0f, 0.4f, 0.4f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Right Hull
    glPushMatrix();
    glTranslatef(0, 0, 1.0f);
    glScalef(2.0f, 0.4f, 0.4f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Deck
    glPushMatrix();
    glTranslatef(0, 0.3f, 0);
    glScalef(1.0f, 0.1f, 2.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Mast
    glPushMatrix();
    glTranslatef(0, 1.8f, 0);
    glScalef(0.1f, 3.0f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();

}

void SailboatDemo::display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    Physics::Vector3 pos = sailboat.get_position();
    Physics::Vector3 offset(5.0f, 0, 0);
    offset = sailboat.get_transform().transform_direction(offset);
    gluLookAt(pos.x+offset.x, pos.y+7.0f, pos.z+offset.z,
              pos.x, pos.y, pos.z,
              0.0, 1.0, 0.0);

    glColor3f(0.6f,0.6f,0.6f);
    int bx = int(pos.x);
    int bz = int(pos.z);
    glBegin(GL_QUADS);
    for (int x = -20; x <= 20; x++) for (int z = -20; z <= 20; z++)
    {
        glVertex3f(bx+x-0.1f, 0, bz+z-0.1f);
        glVertex3f(bx+x-0.1f, 0, bz+z+0.1f);
        glVertex3f(bx+x+0.1f, 0, bz+z+0.1f);
        glVertex3f(bx+x+0.1f, 0, bz+z-0.1f);
    }
    glEnd();

    // Set the transform matrix for the aircraft
    Physics::Matrix4 transform = sailboat.get_transform();
    GLfloat gl_transform[16];
    transform.fill_GL_array(gl_transform);
    glPushMatrix();
    glMultMatrixf(gl_transform);

    // Draw the boat
    glColor3f(0,0,0);
    drawBoat();
    glPopMatrix();
}

void SailboatDemo::text() {
    char buffer[256];
    sprintf(
        buffer,
        "Speed %.1f",
        sailboat.get_velocity().magnitude()
        );
    glColor3f(0,0,0);
    Graphics::render_text(buffer, Physics::Vector3(10.0f, 24.0f, 0.f));
    
    sprintf(
        buffer,
        "Propulsion: X: %.1f Y: %.1f Z: %.1f",
        propulsion.x, propulsion.y, propulsion.z
        );
    Graphics::render_text(buffer, Physics::Vector3(10.0f, 10.0f, 0.f));
}

void SailboatDemo::update()
{
    // Find the duration of the last frame in seconds
    float duration = Graphics::get_seconds_per_frame();
    if (duration <= 0.0f) return;

    // Start with no forces or acceleration.
    sailboat.clear_accumulator();
    
    // Make sure propulsion force only works when submerged
    if (buoyancy.get_sailboat_height(sailboat) <= 0.5f) {
        Physics::Vector3 d_propulsion(-500.0f, 0, 0);
        propulsion = sailboat.get_transform().transform_direction(d_propulsion);
        sailboat.add_force(propulsion);
    }
    else {
        propulsion = Physics::Vector3();
    }

    // Add the forces acting on the boat.
    registry.update_forces(duration);

    // Update the boat's physics.
    sailboat.intergrate(duration);
}

void SailboatDemo::key(unsigned char key)
{
    switch(key)
    {
    case 'q': case 'Q':
        sail_control -= 0.1f;
        break;

    case 'e': case 'E':
        sail_control += 0.1f;
        break;

    case 'w': case 'W':
        sail_control = 0.0f;
        break;

    default: ;
    }

    // Make sure the controls are in range
    if (sail_control < -1.0f) sail_control = -1.0f;
    else if (sail_control > 1.0f) sail_control = 1.0f;

    // Update the control surfaces
    //sail.set_control(sail_control);
}

namespace Sailboat {
    int main(int argc, char ** argv) {
        Graphics::Graphics(800, 600, argc, argv);
    
        SailboatDemo sb;
        
        Graphics::push_draw_pipeline([&sb]() { sb.display(); });
        
        Graphics::push_draw_pipeline([&sb]() {
            Graphics::orthographic_render([&sb](unsigned wh, unsigned ww) {
                sb.text();
            });
        });
        
        Graphics::push_draw_pipeline([&sb]() { sb.update(); });
        
        Graphics::ext_key_callback([&sb](unsigned char key, int x, int y) {
            sb.key(key);
        });
        
        Graphics::start();
        return 0;
    }
}