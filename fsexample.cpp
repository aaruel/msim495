#include "fsexample.h"
#include "physics.h"

/*
 * The flightsim demo.
 *
 * Part of the Physics physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include "playground.h"
#include "flightsim.h"
#include <OpenGL/gl.h>
#include <GLUT/glut.h>

#include <stdio.h>
#include <cassert>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/**
 * The main demo class definition.
 */
class FlightSimDemo
{
    Physics::AeroControl left_wing;
    Physics::AeroControl right_wing;
    Physics::AeroControl rudder;
    Physics::Aero tail;
    Physics::RigidBody aircraft;
    Physics::PropulsionForce propel;
    Physics::ForceRegistry registry;

    Physics::Vector3 windspeed;

    float left_wing_control;
    float right_wing_control;
    float rudder_control;

    void resetPlane();

public:
    /** Creates a new demo object. */
    FlightSimDemo();
    virtual ~FlightSimDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particles. */
    virtual void display();

    /** Update the particle positions. */
    virtual void update();

    /** Handle a key press. */
    virtual void key(unsigned char key);
    
    virtual void text();
};

// Method definitions
FlightSimDemo::FlightSimDemo()
:
right_wing(
    Physics::Matrix3(0,0,0, -1,-0.5f,0, 0,0,0),
    Physics::Matrix3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
    Physics::Matrix3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
    Physics::Vector3(-1.0f, 0.0f, 2.0f), &windspeed
),

left_wing(
    Physics::Matrix3(0,0,0, -1,-0.5f,0, 0,0,0),
    Physics::Matrix3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
    Physics::Matrix3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
    Physics::Vector3(-1.0f, 0.0f, -2.0f), &windspeed
),

rudder(
    Physics::Matrix3(0,0,0, 0,0,0, 0,0,0),
    Physics::Matrix3(0,0,0, 0,0,0, 0.01f,0,0),
    Physics::Matrix3(0,0,0, 0,0,0, -0.01f,0,0),
    Physics::Vector3(2.0f, 0.5f, 0), &windspeed),

tail(
    Physics::Matrix3(0,0,0, -1,-0.5f,0, 0,0,-0.1f),
    Physics::Vector3(2.0f, 0, 0), &windspeed
),

left_wing_control(0), right_wing_control(0), rudder_control(0),

windspeed(0,0,0)
{
    // Set up the aircraft rigid body.
    resetPlane();

    aircraft.set_mass(2.5f);
    Physics::Matrix3 it;
    Physics::Vector3 t = Physics::Vector3(2,1,1);
    it.set_block_inertia_tensor(t, 1);
    aircraft.set_inertia_tensor(it);

    aircraft.set_damping(0.8f, 0.8f);

    aircraft.set_acceleration(Physics::Vector3(0,-9.8, 0));
    aircraft.calculate_derived_data();

    aircraft.set_awake(true);
    aircraft.set_can_sleep(false);

    registry.add(&aircraft, &left_wing);
    registry.add(&aircraft, &right_wing);
    registry.add(&aircraft, &rudder);
    registry.add(&aircraft, &tail);
    registry.add(&aircraft, &propel);
}

FlightSimDemo::~FlightSimDemo()
{
}

void FlightSimDemo::resetPlane()
{
    aircraft.set_position(Physics::Vector3(0, 0, 0));
    aircraft.set_orientation(Physics::Quaternion(1,0,0,0));

    aircraft.set_velocity(Physics::Vector3(0,0,0));
    aircraft.set_rotation(Physics::Vector3(0,0,0));
}

static void drawAircraft()
{
    // Fuselage
    glPushMatrix();
    glTranslatef(-0.5f, 0, 0);
    glScalef(2.0f, 0.8f, 1.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rear Fuselage
    glPushMatrix();
    glTranslatef(1.0f, 0.15f, 0);
    glScalef(2.75f, 0.5f, 0.5f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Wings
    glPushMatrix();
    glTranslatef(0, 0.3f, 0);
    glScalef(0.8f, 0.1f, 6.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rudder
    glPushMatrix();
    glTranslatef(2.0f, 0.775f, 0);
    glScalef(0.75f, 1.15f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Tail-plane
    glPushMatrix();
    glTranslatef(1.9f, 0, 0);
    glScalef(0.85f, 0.1f, 2.0f);
    glutSolidCube(1.0f);
    glPopMatrix();
}

void FlightSimDemo::display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    Physics::Vector3 pos = aircraft.get_position();
    Physics::Vector3 offset(4.0f+aircraft.get_velocity().magnitude(), 0, 0);
    offset = aircraft.get_transform().transform_direction(offset);
    gluLookAt(pos.x+offset.x, pos.y+5.0f, pos.z+offset.z,
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
    Physics::Matrix4 transform = aircraft.get_transform();
    GLfloat gl_transform[16];
    transform.fill_GL_array(gl_transform);
    glPushMatrix();
    glMultMatrixf(gl_transform);

    // Draw the aircraft
    glColor3f(0,0,0);
    drawAircraft();
    glPopMatrix();

    glColor3f(0.8f, 0.8f, 0.8f);
    glPushMatrix();
    glTranslatef(0, -1.0f - pos.y, 0);
    glScalef(1.0f, 0.001f, 1.0f);
    glMultMatrixf(gl_transform);
    drawAircraft();
    glPopMatrix();
}

void FlightSimDemo::text() {
    char buffer[256];
    sprintf(
        buffer,
        "Altitude: %.1f | Speed %.1f | Throttle %.1f | Thrust Angle %.1f",
        aircraft.get_position().y,
        aircraft.get_velocity().magnitude(),
        propel.get_propel(), propel.get_thrust_angle()
    );
    glColor3f(0,0,0);
    Graphics::render_text(buffer, Physics::Vector3(10.f, 24.f, 0));

    sprintf(
        buffer,
        "Left Wing: %.1f | Right Wing: %.1f | Rudder %.1f",
        left_wing_control, right_wing_control, rudder_control
        );
    Graphics::render_text(buffer, Physics::Vector3(10, 10, 0));
}

void FlightSimDemo::update()
{
    // Find the duration of the last frame in seconds
    float duration = (float)Graphics::get_seconds_per_frame();
    if (duration <= 0.0f) return;

    // Start with no forces or acceleration.
    aircraft.clear_accumulator();

    // Add the forces acting on the aircraft.
    registry.update_forces(duration);

    // Update the aircraft's physics.
    aircraft.intergrate(duration);

    // Do a very basic collision detection and response with the ground.
    Physics::Vector3 pos = aircraft.get_position();
    if (pos.y < 0.0f)
    {
        pos.y = 0.0f;
        aircraft.set_position(pos);

        if (aircraft.get_velocity().y < -10.0f)
        {
            resetPlane();
        }
    }

//    Application::update();
}

const char* FlightSimDemo::getTitle()
{
    return "Physics > Flight Sim Demo";
}

void FlightSimDemo::key(unsigned char key)
{
    switch(key)
    {
    case 'q': case 'Q':
        rudder_control += 0.1f;
        break;

    case 'e': case 'E':
        rudder_control -= 0.1f;
        break;

    case 'w': case 'W':
        left_wing_control -= 0.1f;
        right_wing_control -= 0.1f;
        break;

    case 's': case 'S':
        left_wing_control += 0.1f;
        right_wing_control += 0.1f;
        break;

    case 'd': case 'D':
        left_wing_control -= 0.1f;
        right_wing_control += 0.1f;
        break;

    case 'a': case 'A':
        left_wing_control += 0.1f;
        right_wing_control -= 0.1f;
        break;

    case 'x': case 'X':
        left_wing_control = 0.0f;
        right_wing_control = 0.0f;
        rudder_control = 0.0f;
        break;

    case 'r': case 'R':
        resetPlane();
        break;
        
    case 'f': case 'F':
        propel.increment_propel(0.5);
        break;
        
    case 'v': case 'V':
        propel.increment_propel(-0.5);
        break;
        
    case 'c': case 'C':
        propel.set_propel(0);
        break;
        
    case 'b': case 'B':
        propel.increment_thrust_angle(5.f);
        break;
        
    case 'g': case 'G':
        propel.increment_thrust_angle(-5.f);
        break;

    default: ;
//        Application::key(key);
    }

    // Make sure the controls are in range
    if (left_wing_control < -1.0f) left_wing_control = -1.0f;
    else if (left_wing_control > 1.0f) left_wing_control = 1.0f;
    if (right_wing_control < -1.0f) right_wing_control = -1.0f;
    else if (right_wing_control > 1.0f) right_wing_control = 1.0f;
    if (rudder_control < -1.0f) rudder_control = -1.0f;
    else if (rudder_control > 1.0f) rudder_control = 1.0f;

    // Update the control surfaces
    left_wing.set_control(left_wing_control);
    right_wing.set_control(right_wing_control);
    rudder.set_control(rudder_control);
}

namespace FlightSimExample {
    int main(int argc, char ** argv) {
        Graphics::Graphics(800, 600, argc, argv);
    
        FlightSimDemo fsd;
        
        Graphics::push_draw_pipeline([&fsd]() { fsd.display(); });
        
        Graphics::push_draw_pipeline([&fsd]() {
            Graphics::orthographic_render([&fsd](unsigned wh, unsigned ww) {
                fsd.text();
            });
        });
        
        Graphics::push_draw_pipeline([&fsd]() { fsd.update(); });
        
        Graphics::ext_key_callback([&fsd](unsigned char key, int x, int y) {
            printf("key: %c\n", key);
            fsd.key(key);
        });
        
        Graphics::start();
        return 0;
    }
}