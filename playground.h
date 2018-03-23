//
//  playground.h
//  MSIM495
//
//  Created by Aaron Ruel on 1/17/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#ifndef __MSIM495__playground__
#define __MSIM495__playground__

#include <stdio.h>
#include <functional>
#define ESC_KEY 27
#define ENTER_KEY 13

namespace Physics { typedef float real; class Vector3; class Particle; }

namespace Graphics {
    /**
     * Scale camera zoom by n
     */
    void set_projection(unsigned int zoom = 1);
    
    /*
     * Return camera direction and position vectors
     */
    Physics::Vector3 get_camera_direction();
    Physics::Vector3 get_camera_position();
    
    /**
     * return window_width / window_height
     */
    float get_window_aspect();
    
    /*
     * Return FPS and inverse
     */
    Physics::real get_fps();
    Physics::real get_seconds_per_frame();
    
    /**
     * Register callback to fire on keypress
     */
    void register_fire(std::function<void(void)> cb, int key);
    void ext_key_callback(std::function<void(unsigned char, int, int)> cb);
    
    /**
     * Push back function to be called during display loop.
     * Functions called in order of registration every frame
     */
    void push_draw_pipeline(std::function<void(void)> func);
    
    /**
     * Graphics initialization function
     */
    void Graphics(int window_w, int window_h, int argc, char ** argv);
    
    /*
     * Primitive draw functions
     */
    void draw_pyramid(Physics::Particle position);
    void draw_sphere(Physics::Particle p, Physics::real scale = 1);
    void draw_ground();
    void draw_3d_rect(
        Physics::Particle p,
        Physics::real x = 1.f,
        Physics::real y = 1.f,
        Physics::real z = 1.f,
        Physics::real rotation_axis_x = 0.f,
        Physics::real rotation_axis_y = 0.f,
        Physics::real rotation_axis_z = 0.f,
        Physics::real rotation_degs = 0.f
    );
    void draw_reference_points();
    void orthographic_render(std::function<void(unsigned, unsigned)> f);
    void render_text(const char * text, Physics::Vector3 p);
    
    /**
     * Call to execute initialized callbacks
     */
    void start();
};

#endif /* defined(__MSIM495__playground__) */
