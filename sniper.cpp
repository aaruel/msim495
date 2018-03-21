//
//  sniper.cpp
//  MSIM495
//
//  Created by Aaron Ruel on 1/29/18.
//  Copyright (c) 2018 AAR. All rights reserved.
//

#include "sniper.h"
#include <iostream>
#include <vector>
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include "playground.h"
#include "physics.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace Sniper {
    struct Image {
        int width;
        int height;
        int bpp;
        unsigned int texture_buffer;
        unsigned char * data;
    };
    
    Physics::Vector3 wind_direction;
    unsigned wind_power;

    class Bullet : public Physics::Particle {
    private:
        // life before being taken out of render queue in seconds
        Physics::real lifetime = 10.f;
        
        // Properties
        void set_props(Physics::Vector3 pos) {
            origin = pos;
            set_position(pos);
            set_mass(1);
        }
        
        // Originating position
        Physics::Vector3 origin;
        
    public:
        constexpr static Physics::real velocity_scale_factor = 25;
        
        Bullet(){
            set_props(Physics::Vector3(0,0,0));
        }
        Bullet(Physics::Vector3 v) {
            set_props(v);
        }
        Bullet(Physics::real x, Physics::real y, Physics::real z) {
            set_props(Physics::Vector3(x, y, z));
        }
        
        void update(Physics::real time) {
            float gravity_scaled = normal_gravity / velocity_scale_factor;
            add_impulse(Physics::Vector3(0,gravity_scaled,0));
            add_impulse(wind_direction * wind_power);
            integrate(time);
            lifetime -= time;
        }
        
        Physics::Vector3 get_origin() {
            return origin;
        }
        
        Physics::real get_lifetime() { return lifetime; }
    };
    
    // Game parameters
    bool zoomed = false;
    std::vector<Bullet> bullets;
    std::vector<Physics::Particle> targets;
    Physics::real score = 0;
    Image scope;
    Image wind_arrow;
    
    void render_text(const char * text, Physics::Vector3 p) {
        // render text in white
        glColor3f(1.f, 1.f, 1.f);
        glRasterPos2i(p.x, p.y);
        
        for (char * c = (char *)text; *c != '\0'; ++c) {
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *c);
        }
    }
    
    void generate_wind() {
        auto r = [](){
            return ((float)(rand() % 1000 + 1) / 500.f) - 1.f;
        };
        wind_direction = Physics::Vector3(r(), 0, r());
        wind_direction.normalize();
        wind_power = rand() % 5 + 1;
    }

    void push_bullet() {
        Bullet bullet(Graphics::get_camera_position());
        bullet.set_velocity(Graphics::get_camera_direction() * Bullet::velocity_scale_factor);
        bullets.push_back(bullet);
        generate_wind();
    }
    
    void load_texture(Image * i) {
        glEnable(GL_TEXTURE_2D);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        
        if (i->texture_buffer != 0) {
            glBindTexture(GL_TEXTURE_2D, i->texture_buffer);
        }
        else {
            glGenTextures(1, &i->texture_buffer);
            glBindTexture(GL_TEXTURE_2D, i->texture_buffer);
            
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            
            glTexImage2D(
                GL_TEXTURE_2D,
                0,
                GL_RGBA,
                (GLsizei) i->width,
                (GLsizei) i->height,
                0,
                GL_RGBA,
                GL_UNSIGNED_BYTE,
                i->data
            );
        }
    };
    
    void scaled_texture_box(float scale) {
        glBegin(GL_POLYGON);
            glTexCoord2f(0, 1); glVertex3f(-1.0 * scale, -1.0 * scale, 0.0);
            glTexCoord2f(1, 1); glVertex3f(1.0 * scale, -1.0 * scale, 0.0);
            glTexCoord2f(1, 0); glVertex3f(1.0 * scale, 1.0 * scale, 0.0);
            glTexCoord2f(0, 0); glVertex3f(-1.0 * scale, 1.0 * scale, 0.0);
        glEnd();
    }
    
    void render_scope_overlay() {
        glPushMatrix();
    
        unsigned int window_height = glutGet(GLUT_WINDOW_HEIGHT);
        unsigned int window_width = glutGet(GLUT_WINDOW_WIDTH);
        float scale = 500;
        load_texture(&scope);
        glColor4f(1.f, 1.f, 1.f, 1.f);
        glTranslatef(window_width/2, window_height/2, 1);
        scaled_texture_box(scale);
        glDisable(GL_TEXTURE_2D);
        
        glPopMatrix();
    }
    
    void render_wind_arrow() {
        glPushMatrix();
        
        unsigned int window_height = glutGet(GLUT_WINDOW_HEIGHT);
        float scale = 50;
        load_texture(&wind_arrow);
        glColor4f(1.f, 1.f, 1.f, 1.f);
        glTranslatef(100, window_height-60, 1);
        Physics::Vector3 camera_direction = Graphics::get_camera_direction();
        camera_direction.normalize();
        Physics::real angle = wind_direction
            .angle_2d(Physics::Vector3(camera_direction.x, 0, camera_direction.z));
        
        glRotatef(
            (angle/Physics::pi*180),
            0, 0, -1
        );
        scaled_texture_box(scale);
        glDisable(GL_TEXTURE_2D);
    
        glPopMatrix();
        
        char * wp = (char*)malloc(255);
        sprintf(wp, "Wind Power: %d m/s", wind_power);
        render_text((const char *)wp, Physics::Vector3(50, window_height - 140, 0));
        free((void*)wp);
    }

    void render_bullets() {
        Graphics::register_fire(push_bullet, ENTER_KEY);
        
        Graphics::push_draw_pipeline([]() {
            // Operate on all bullets in queue
            for (int i = 0; i < bullets.size(); ++i) {
                Bullet current_bullet = bullets[i];
                Graphics::draw_sphere(current_bullet.get_position());
                
                // get distances between all targets
                int index = 0;
                std::for_each(
                    targets.begin(),
                    targets.end(),
                    [&current_bullet, &index](Physics::Particle t){
                    // Branch if register a hit
                    if (current_bullet.get_position().distance(t.get_position()) < 1.f) {
                        // Kill target
                        targets.erase(targets.begin() + index);
                        // Mark score as distance of shot
                        score += current_bullet.get_origin().distance(t.get_position());
                    }
                    ++index;
                });
                
                bullets[i].update(0.033);
                if (bullets[i].get_lifetime() <= 0.f
                    || current_bullet.get_position().y <= 0.f) {
                    bullets.erase(bullets.begin() + i);
                }
            }
        });
    }
    
    void zoom_projection() {
        if (zoomed) Graphics::set_projection(2);
        else Graphics::set_projection();
    }
    
    void render_score(unsigned int window_height) {
        char * score_text = (char *)malloc(255);
        sprintf(score_text, "Score: %d", (int)score);
        render_text((const char *)score_text, Physics::Vector3(50, window_height - 160, 0));
        free((void*)score_text);
    }
    
    void orthographic_stage() {
        Graphics::push_draw_pipeline([](){
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
            
                render_score(window_height);
                render_wind_arrow();
                render_scope_overlay();
                
            
            glMatrixMode(GL_MODELVIEW);
            glPopMatrix();
            
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();
            
            zoom_projection();
        });
    }
    
    void zoom() {
        Graphics::register_fire([](){
            zoomed = !zoomed;
        }, 'z');
    }
    
    void draw_targets() {
        std::for_each(targets.begin(), targets.end(), Graphics::draw_pyramid);
    }
    
    void generate_targets() {
        for (int i = -3; i < 3; ++i) {
            for (int j = -3; j < 3; ++j) {
                if (i != 0) {
                
                    targets.push_back(Physics::Particle(
                        Physics::Vector3{
                            static_cast<Physics::real>(i*10.0),
                            0,
                            static_cast<Physics::real>(j*10.0)
                        }
                    ));
                    
                }
            }
        }
    }

    int main(int argc, char ** argv) {
        srand((unsigned)time(NULL));
        generate_wind();
        
        Graphics::Graphics(1280, 800, argc, argv);
        
        scope.data = stbi_load(
            "scope1.png",
            &scope.width,
            &scope.height,
            &scope.bpp, 4
        );
        
        wind_arrow.data = stbi_load(
            "arrow1.png",
            &wind_arrow.width,
            &wind_arrow.height,
            &wind_arrow.bpp, 4
        );
        
        generate_targets();
        
        Graphics::push_draw_pipeline(zoom_projection);
        Graphics::push_draw_pipeline(Graphics::draw_ground);
        Graphics::push_draw_pipeline(draw_targets);
        render_bullets();
        orthographic_stage();
        zoom();
        
        Graphics::start();
        
        stbi_image_free(scope.data);
        stbi_image_free(wind_arrow.data);
        
        return 0;
    }
}