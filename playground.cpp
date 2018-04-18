//
//  playground.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 1/17/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "playground.h"

#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include "core.h"
#include "collisionengine.h"

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#define vertex(v) glVertex3f(v.x, v.y, v.z)

// los == line of sight
#define lookAt(position, los) gluLookAt( \
    position.x, position.y, position.z,  \
    position.x + los.x, position.y + los.y, position.z + los.z, \
    0.0f, 1.0f, 0.0f                     \
)

#define KEY_CACHE_SIZE 255
#define COLOR_SKY 0.196078f, 0.6f, 0.8f, 1.0f
#define COLOR_GROUND 0.35f, 0.5f, 0.28f, 1.0f
#define COLOR_OBJECT 0.75f, 0.75f, 0.75f
#define COLOR_BLACK 0.0f, 0.0f, 0.0f, 1.0f
#define COLOR_WHITE 1.0f, 1.0f, 1.0f, 1.0f

// Pseudo-class to deal with C-API
namespace Graphics {
    std::vector<std::function<void(void)>> draw_pipeline;
    // correspond with key_cache to programmatically register key callbacks
    std::function<void(void)> fire_callback[KEY_CACHE_SIZE];
    std::function<void(unsigned char, int, int)> ext_key_function;
    int main_window;
    int mouse_old_x = 0;
    int mouse_old_y = 0;
    int mouse_click_state = 0;
    int key_cache[KEY_CACHE_SIZE] = {0};
    GLfloat window_aspect;
    int window_width;
    int window_height;
    int frame_time = 0;
    Physics::real frames_per_second = 0;
    const Physics::real theoretical_framerate = 60.f;
    Physics::real angle = 0.0;
    Physics::real v_angle = 0.0;
    Physics::Vector3 camera_position(0.0, 1.0, 5.0);
    Physics::Vector3 camera_direction(0.0, 0.0, -1.0);
    
    bool in_range(float l, float in, float r) {
        return (in >= l) && (in <= r);
    }
    
    void register_fire(std::function<void(void)> cb, int key) {
        fire_callback[key] = cb;
    }
    
    void ext_key_callback(std::function<void(unsigned char, int, int)> cb) {
        ext_key_function = cb;
    }
    
    Physics::Vector3 get_camera_direction() {
        return camera_direction;
    };
    
    Physics::Vector3 get_camera_position() {
        return camera_position;
    };
    
    float get_window_aspect() {
        return window_aspect;
    }
    
    Physics::real get_fps() {
        return frames_per_second;
    }
    
    Physics::real get_seconds_per_frame() {
        if (frames_per_second == 0.f) {
            return 1.f / theoretical_framerate;
        }
        
        return 1.f / frames_per_second;
    }
    
    void push_draw_pipeline(std::function<void(void)> func) {
        draw_pipeline.push_back(func);
    }
    
    void gl_init() {
        glClearColor(COLOR_BLACK); // Set background color to black and opaque
        glClearDepth(1.0f);                   // Set background depth to farthest
        glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
        glEnable(GL_CULL_FACE);
        glEnable(GL_MULTISAMPLE);
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
        glShadeModel(GL_SMOOTH);   // Enable smooth shading
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
    }

    void set_projection(unsigned int zoom) {
        // Set the viewport to cover the new window
        glViewport(0, 0, window_width, window_height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        
        // fovy, aspect, zNear and zFar
        gluPerspective(45.0f/zoom, window_aspect, 0.1f, 100.0f);
        
        glMatrixMode(GL_MODELVIEW);
    }

    void reshape(GLsizei width, GLsizei height) {
        window_width = width;
        window_height = height;
        // Compute aspect ratio of the new window
        if (height == 0) height = 1;
        window_aspect = (GLfloat)width / (GLfloat)height;

        set_projection();
    }
    
    void mouse_motion(int x, int y) {
        const float y_limit_high = 0.90;
        const float y_limit_low  = -0.90;
        const float scale = 0.001;
    
        if (mouse_click_state) {
            mouse_old_x = x;
            mouse_old_y = y;
            mouse_click_state = 0;
        }
        
        int diff_x = x - mouse_old_x;
        int diff_y = y - mouse_old_y;
    
        angle   += diff_x * scale;
        v_angle += diff_y * scale;
        
        camera_direction.x = sin(angle);
        camera_direction.z = -cos(angle);
        
        if (in_range(y_limit_low, camera_direction.y, y_limit_high)) {
            camera_direction.y = -sin(v_angle);
        }
        if (camera_direction.y > y_limit_high) {
            camera_direction.y = y_limit_high;
        }
        else if (camera_direction.y < y_limit_low) {
            camera_direction.y = y_limit_low;
        }
    
        mouse_old_x = x;
        mouse_old_y = y;
    }
    
    void mouse_callback(int button, int state, int x, int y) {
        mouse_click_state = !state;
    }
    
    void orthographic_render(std::function<void(unsigned, unsigned)> f) {
        unsigned int window_height = glutGet(GLUT_WINDOW_HEIGHT);
        unsigned int window_width = glutGet(GLUT_WINDOW_WIDTH);
        
        // Ortho render space
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        gluOrtho2D(0.0, window_width, 0.0, window_height);
        
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        
            f(window_width, window_height);
        
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
        
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        
        set_projection();
    }
    
    void render_text(const char * text, Physics::Vector3 p) {
        // render text in white
        glColor3f(1.f, 1.f, 1.f);
        glRasterPos2i(p.x, p.y);
        
        for (char * c = (char *)text; *c != '\0'; ++c) {
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *c);
        }
    }
    
    std::function<void(void)> random_color() {
        auto r = [](){return (Physics::real)((rand() % 128) + 128) / 255.f;};
        Physics::real x = r(), y = r(), z = r();
        return [=](){
            glColor3f(x, y, z);
        };
    }

    void draw_pyramid(Physics::Particle p) {
        Physics::Vector3 position = p.get_position();
        // Render a pyramid consists of 4 triangles
        glPushMatrix();
        glTranslatef(position.x, position.y, position.z);
        Physics::real scale = 0.5;
        glBegin(GL_TRIANGLES);
            Physics::Vector3 top_v   = Physics::Vector3{0.0f, 1.0f, 0.0f};
            Physics::Vector3 front_l = Physics::Vector3{-1.0f, -1.0f, 1.0f} * scale;
            Physics::Vector3 front_r = Physics::Vector3{1.0f, -1.0f, 1.0f} * scale;
            Physics::Vector3 back_l  = Physics::Vector3{-1.0f, -1.0f, -1.0f} * scale;
            Physics::Vector3 back_r  = Physics::Vector3{1.0f, -1.0f, -1.0f} * scale;
        
            auto triangle = [](Physics::Vector3 v1, Physics::Vector3 v2, Physics::Vector3 v3) {
                glColor3f(1.0f, 0.0f, 0.0f); vertex(v1); // Red
                glColor3f(0.0f, 1.0f, 0.0f); vertex(v2); // Green
                glColor3f(0.0f, 0.0f, 1.0f); vertex(v3); // Blue
            };
        
            triangle(top_v, front_l, front_r); // Front
            triangle(top_v, front_r, back_r);  // Right
            triangle(top_v, back_r, back_l);   // Back
            triangle(top_v, back_l, front_l);  // Left
            triangle(front_l, back_r, front_r);
            triangle(front_l, back_l, back_r);
        glEnd();   // Done drawing the pyramid
        glPopMatrix();
    }
    
    void _draw_sphere(Physics::Particle p, Physics::real scale, bool override_color) {
        Physics::Vector3 position = p.get_position();
        glPushMatrix();
        glTranslatef(position.x, position.y, position.z);
        if (!override_color) glColor3f(COLOR_OBJECT);
        glutSolidSphere(scale, 50, 50);
        glPopMatrix();
    }
    
    void draw_sphere(Physics::Particle p, Physics::real scale) {
        _draw_sphere(p, scale, false);
    }
    
    void draw_sphere_no_color(Physics::Particle p, Physics::real scale) {
        _draw_sphere(p, scale, true);
    }
    
    void draw_2d_plane(Physics::Plane p, bool override_color) {
        // perpendicular
        Physics::Vector3 left(p.direction.y, -p.direction.x, 0);
        // invert
        Physics::Vector3 right = left;
        right.invert();
        
        left *= (Physics::real)(window_height * window_width);
        right *= (Physics::real)(window_height * window_width);
        
        glPushMatrix();
        glTranslatef(p.position.x, p.position.y, 0);
        if (!override_color) glColor4f(COLOR_WHITE);
        glBegin(GL_LINES);
            glVertex2f(left.x, left.y);
            glVertex2f(right.x, right.y);
        glEnd();
        glPopMatrix();
    }
    
    void draw_3d_rect(
        Physics::Particle p,
        Physics::real x,
        Physics::real y,
        Physics::real z,
        Physics::real rotation_axis_x,
        Physics::real rotation_axis_y,
        Physics::real rotation_axis_z,
        Physics::real rotation_degs
    ) {
        // Dimensions
        Physics::real height = y / 2.f;
        Physics::real length = z / 2.f;
        Physics::real width =  x / 2.f;
        
        Physics::Vector3 position = p.get_position();
        
        glPushMatrix();
        glTranslatef(position.x, position.y, position.z);
        glRotatef(
            rotation_degs,
            rotation_axis_x,
            rotation_axis_y,
            rotation_axis_z
        );
        glColor4f(COLOR_WHITE);
        glBegin(GL_QUADS);
            Physics::Vector3 top_back_left
                = Physics::Vector3{-1.f*width,  1.f*height, -1.f*length};
            Physics::Vector3 top_front_left
                = Physics::Vector3{-1.f*width,  1.f*height,  1.f*length};
            Physics::Vector3 top_back_right
                = Physics::Vector3{ 1.f*width,  1.f*height, -1.f*length};
            Physics::Vector3 top_front_right
                = Physics::Vector3{ 1.f*width,  1.f*height,  1.f*length};
        
            Physics::Vector3 bot_back_left
                = Physics::Vector3{-1.f*width, -1.f*height, -1.f*length};
            Physics::Vector3 bot_front_left
                = Physics::Vector3{-1.f*width, -1.f*height,  1.f*length};
            Physics::Vector3 bot_back_right
                = Physics::Vector3{ 1.f*width, -1.f*height, -1.f*length};
            Physics::Vector3 bot_front_right
                = Physics::Vector3{ 1.f*width, -1.f*height,  1.f*length};
        
            auto rect = [](
                Physics::Vector3 v1,
                Physics::Vector3 v2,
                Physics::Vector3 v3,
                Physics::Vector3 v4
            ) {
                vertex(v1);
                vertex(v2);
                vertex(v3);
                vertex(v4);
            };
        
            // top
            rect(
                top_back_left,
                top_front_left,
                top_front_right,
                top_back_right
            );
        
            // back
            rect(
                bot_back_left,
                top_back_left,
                top_back_right,
                bot_back_right
            );
        
            // front
            rect(
                top_front_left,
                bot_front_left,
                bot_front_right,
                top_front_right
            );
        
            // bottom
            rect(
                bot_front_left,
                bot_back_left,
                bot_back_right,
                bot_front_right
            );
        
            // left
            rect(
                top_front_left,
                top_back_left,
                bot_back_left,
                bot_front_left
            );
        
            // right
            rect(
                top_back_right,
                top_front_right,
                bot_front_right,
                bot_back_right
            );
        
        glEnd();
        glPopMatrix();
    }

    void draw_ground() {
        glColor4f(COLOR_GROUND);
        
        glBegin(GL_QUADS);
            glVertex3f(-100.0f, 0.0f, -100.0f);
            glVertex3f(-100.0f, 0.0f,  100.0f);
            glVertex3f( 100.0f, 0.0f,  100.0f);
            glVertex3f( 100.0f, 0.0f, -100.0f);
        glEnd();
    }
    
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

    void handle_keys() {
        if (key_cache['w']) {
            camera_position.x += camera_direction.x * 0.1;
            camera_position.z += camera_direction.z * 0.1;
        }
    
        if (key_cache['a']) {
            camera_position.x += camera_direction.z * 0.1;
            camera_position.z -= camera_direction.x * 0.1;
        }
        
        if (key_cache['s']) {
            camera_position.x -= camera_direction.x * 0.1;
            camera_position.z -= camera_direction.z * 0.1;
        }
        
        if (key_cache['d']) {
            camera_position.x -= camera_direction.z * 0.1;
            camera_position.z += camera_direction.x * 0.1;
        }
    }

    void track_fps() {
        const int one_second = 1000;
        static int n_frame = 0;
        int new_time_elapsed = glutGet(GLUT_ELAPSED_TIME);
        int frame_diff = new_time_elapsed - frame_time;
        ++n_frame;
        
        // if frame diff greater than 1000 milliseconds
        if (frame_diff > one_second) {
            frames_per_second =
                static_cast<float>(n_frame * one_second)
                / static_cast<float>(frame_diff);
            
            frame_time = new_time_elapsed;
            n_frame = 0;
        }
    }

    void display_loop() {
        track_fps();
    
        glClearColor(COLOR_SKY);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        handle_keys();
        lookAt(camera_position, camera_direction);
        
        // Dynamic Draw Pipeline
        std::for_each(
            draw_pipeline.begin(),
            draw_pipeline.end(),
            [](std::function<void(void)> draw){ draw(); }
        );
        
        glutSwapBuffers();
    }
    
    void keyboard_callback(unsigned char key, int x, int y) {
        key_cache[key] = 1;
        
        // Key registry at keypress
        if (fire_callback[key] != nullptr) {
            fire_callback[key]();
        }
        
        // outward facing callback
        if (ext_key_function != nullptr) {
            ext_key_function(key, x, y);
        }

        if (key == ESC_KEY) { // ESC key
            exit(0);
        }
    }
    
    void keyboard_release_callback(unsigned char key, int x, int y) {
        key_cache[key] = 0;
    }

    // Init
    void Graphics(int window_w, int window_h, int argc, char ** argv) {
        window_width = window_w;
        window_height = window_h;
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
        glutInitWindowSize(window_w, window_h);
        glutInitWindowPosition(200, 200);
        main_window = glutCreateWindow("Physics Playground");
        glutKeyboardFunc(keyboard_callback);
        glutKeyboardUpFunc(keyboard_release_callback);
        glutDisplayFunc(display_loop);
        glutIdleFunc(display_loop); // render constantly
        glutReshapeFunc(reshape);
        glutMotionFunc(mouse_motion);
        glutMouseFunc(mouse_callback);
        gl_init();
    }

    // Fire callbacks
    void start() {
        glutMainLoop();
    }
}