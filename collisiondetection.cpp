//
//  collisiondetection.cpp
//  MSIM495
//

#include "collisiondetection.h"
#include "physics.h"
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <SOLID.h>
#include "playground.h"

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#define array_2_vector(a, v) v.x = a[0]; v.y = a[1]; v.z = a[2];

namespace CollisionDetection {
    // Scene globals
    Physics::Vector3 gravity(0,-9.8,0);
    DT_SceneHandle scene;
    DT_RespTableHandle rth;
    DT_ResponseClass response_class;
    bool gravity_on = false;
    
    class Cube : public Physics::RigidBody {
        Physics::real size;
        DT_ShapeHandle shape;
        DT_ObjectHandle object;
        
        void collision() {
            // Apply transform matrix to collision detection object matrix for position + orientation
            Physics::real matrix[16];
            get_gl_transform(matrix);
            DT_SetMatrixf(object, matrix);
        }
    
        void render() {
            // Apply transform matrix to modelview matrix for position + orientation
            Physics::real matrix[16];
            get_gl_transform(matrix);
            
            glColor3f(0.7f, 0.7f, 0.7f);
            
            glPushMatrix();
                glMultMatrixf(matrix);
                glutSolidCube(size);
            glPopMatrix();
        }
        
    public:
        Physics::Vector3 points[8];
    
        Cube(Physics::Vector3 position, Physics::real size = 1, Physics::real mass = 1) : size(size) {
            // Set up RigidBody for basic interactivity
            Physics::Vector3 half(size/2.f, size/2.f, size/2.f);
            set_position(position);
            set_mass(mass);
            set_damping(0.95, 0.8);
            Physics::Matrix3 tensor;
            tensor.set_block_inertia_tensor(half, get_mass()*8);
            set_inertia_tensor(tensor);
            
            // Keep track of cube edges
            // top counterclockwise -> bottom counterclockwise
            points[0] = Physics::Vector3(-size,  size,  size);
            points[1] = Physics::Vector3(-size,  size, -size);
            points[2] = Physics::Vector3( size,  size, -size);
            points[3] = Physics::Vector3( size,  size,  size);
            points[4] = Physics::Vector3(-size, -size,  size);
            points[5] = Physics::Vector3(-size, -size, -size);
            points[6] = Physics::Vector3( size, -size, -size);
            points[7] = Physics::Vector3( size, -size,  size);
            
            // create the shape in the collision detector
            shape = DT_NewBox(size, size, size);
            object = DT_CreateObject(this, shape);
            
            // register object to scene
            DT_AddObject(scene, object);
            DT_SetResponseClass(rth, object, response_class);
        }
        
        ~Cube() {
            DT_DeleteShape(shape);
        }
        
        void run() {
            intergrate(Graphics::get_seconds_per_frame());
            collision();
            render();
        }
    };
    
    // COLLISION CALLBACK
    DT_Bool collision_response(
        void * client_data,
        void * client_object1,
        void * client_object2,
        const DT_CollData * coll_data
    ) {
        Cube * c1 = (Cube*)client_object1;
        Cube * c2 = (Cube*)client_object2;
        
        /// Simple contact resolution ///
        Physics::Vector3 f;
        array_2_vector(coll_data->normal, f);
        f.invert();
        f *= 100;
        
        Physics::Vector3 p1;
        array_2_vector(coll_data->point1, p1);
        
        Physics::Vector3 p2;
        array_2_vector(coll_data->point2, p2);
        
        c1->add_force_at_point(f, p1);
        c2->add_force_at_point(f, p2);
        
        // Resume contact detection
        return DT_CONTINUE;
    }

    int main(int argc, char ** argv) {
        // Init collision detection
        scene = DT_CreateScene();
        rth = DT_CreateRespTable();
        response_class = DT_GenResponseClass(rth);
        DT_AddDefaultResponse(rth, &collision_response, DT_DEPTH_RESPONSE, stdout);
        
        // Init world
        Cube c1(Physics::Vector3(0, 3, 0), 1);
        Cube c2(Physics::Vector3(0, 0, 0), 1, 0);
        
        // Init graphics
        Graphics::Graphics(800, 600, argc, argv);
        
        // Graphics pipeline
        Graphics::register_fire([](){ gravity_on = !gravity_on; }, ' ');
        Graphics::push_draw_pipeline(Graphics::draw_ground);
        Graphics::push_draw_pipeline(Graphics::draw_reference_points);
        Graphics::push_draw_pipeline([&](){
            // Apply gravity
            if (gravity_on) c1.add_force(gravity);
            
            // integrate:
            // - Physics
            // - Collision Detection
            // - Render
            c1.run();
            c2.run();
            
            // Test if any points are in contact
            DT_Test(scene, rth);
        });
        
        // Run graphics callbacks
        Graphics::start();
        
        // Clean up collision detection
        DT_DestroyScene(scene);
        DT_DestroyRespTable(rth);
        
        return 0;
    }
}