//
//  collisionengine.h
//  MSIM495
//

#ifndef __MSIM495__collisionengine__
#define __MSIM495__collisionengine__

#include <stdio.h>
#include <assert.h>
#include "core.h"

/**
 * Binary Space Partitioning tree implementation
 */

namespace Physics {
    struct BoundingSphere {
        Vector3 center;
        real radius;
    };
    
    

    /**
     * Plane representation
     * Direction of plane indicates orthogonal vector to surface 
     */
    struct Plane {
        Vector3 position;
        Vector3 direction;
        
        static Vector3 NORTH() {return Vector3(0, 1, 0);}
        static Vector3 SOUTH() {return Vector3(0, -1, 0);}
        static Vector3 EAST() {return Vector3(1, 0, 0);}
        static Vector3 WEST() {return Vector3(-1, 0, 0);}
        
        Plane() {}
        Plane(Plane const &p) : position(p.position), direction(p.direction) {}
        Plane(Vector3 position, Vector3 direction) : position(position), direction(direction) {}
        
        Plane(Vector3 bounds) {
            position = Vector3(
                rand() % (unsigned)bounds.x,
                rand() % (unsigned)bounds.y,
                0
            );
            
            unsigned selector = rand() & 0b11;
            switch (selector) {
                case 0: direction = NORTH(); break;
                case 1: direction = SOUTH(); break;
                case 2: direction = EAST() ; break;
                case 3: direction = WEST() ; break;
            };
        }
        
        void print() {
            printf(
                "{direction: (%.2f, %.2f), position: (%.2f, %.2f)}\n",
                direction.x, direction.y,
                position.x, position.y
            );
        }
        
        real side_of_plane(Vector3 object) {
            return (object - position) * direction;
        }
        
        bool positive_side(Vector3 object) {
            return side_of_plane(object) > 0;
        }
    };
    
    /**
     * Object interface
     */
    struct Object : public Particle {
        Object() {}
        Object(Vector3 bounds) {
            set_position(Vector3(
                rand() % (unsigned)bounds.x,
                rand() % (unsigned)bounds.y,
                0
            ));
        }
    };

    typedef std::vector<Object *> BSPObjects;
    typedef std::vector<Plane> BSPPlanes;
    
    enum BSPChildType {
        NODE,
        OBJECTS
    };
    
    /**
     * PARTICLE CHILD
     */
    struct BSPNode;
    class BSPChild {
    public:
        BSPChildType type;
        // keeps track of where objects should be
        bool front;
        union {
            BSPNode * node;
            // set of objects in front of plane
            BSPObjects * objects;
        };
        
    public:
        BSPChild() : type(NODE), node(nullptr) {}
        BSPChild(BSPNode * n): type(NODE), node(n) {}
        BSPChild(BSPObjects * o, bool front): type(OBJECTS), objects(o), front(front) {}
        void destroy() {
            if (type == OBJECTS) {
                delete objects;
                objects = nullptr;
            }
            if (type == NODE) {
                delete node;
                node = nullptr;
            }
        }
        
        void set_node(BSPNode * n) { assert(type == NODE); node = n; }
        void set_objects(BSPObjects * o) { assert(type == OBJECTS); objects = o; }
    };
    
    /**
     * PARTICLE NODE
     */
    struct BSPNode {
        Plane plane;
        BSPChild front;
        BSPChild back;
        
        BSPNode() {}
    };
    
    /**
     * :: FOR COLLISION ::
     * Traverse down to each object node
     * check members for collision based on distance
     */
    class BSPTree {
        BSPNode root;
        // cache for rebuilding
        BSPPlanes walls_cache;
        BSPObjects objects_cache;
        unsigned rebuild_count = 0;
        
        void add_partitions(BSPNode * n, BSPPlanes walls, BSPObjects objects) {
            if (walls.size()) {
                // get first wall in set
                Plane c = walls.back();
                n->plane = c;
                walls.pop_back();
                
                BSPObjects front_objects;
                BSPObjects back_objects;
                BSPPlanes front_walls;
                BSPPlanes back_walls;
                
                auto objects_it = objects.begin();
                for (; objects_it != objects.end(); ++objects_it) {
                    // sort objects
                    real indicator = n->plane.positive_side((*objects_it)->get_position());
            
                    if (indicator) front_objects.push_back(*objects_it);
                    else back_objects.push_back(*objects_it);
                }
                
                auto walls_it = walls.begin();
                for (; walls_it != walls.end(); ++walls_it) {
                    // sort walls
                    real indicator = n->plane.side_of_plane(walls_it->position);
            
                    if (indicator > 0) front_walls.push_back(*walls_it);
                    else back_walls.push_back(*walls_it);
                }
                
                if (front_walls.size()) {
                    n->front = BSPChild(new BSPNode());
                    add_partitions(n->front.node, front_walls, front_objects);
                }
                else {
                    n->front = BSPChild(new BSPObjects(front_objects), true);
                }
                if (back_walls.size()) {
                    n->back = BSPChild(new BSPNode());
                    add_partitions(n->back.node, back_walls, back_objects);
                }
                else {
                    n->back = BSPChild(new BSPObjects(back_objects), false);
                }
            }
        }
        
        void kill() {
            std::function<void(BSPNode *)> recur = [&](BSPNode * n){
                // recurse all points
                if (n->back.type == NODE && n->back.node != nullptr) {
                    recur(n->back.node);
                }
                if (n->front.type == NODE && n->front.node != nullptr) {
                    recur(n->front.node);
                }
                // delete from bottom up
                if (n) n->back.destroy();
                if (n) n->front.destroy();
                return;
            };
            recur(&root);
        }
        
        void rebuild() {
            kill();
            add_partitions(&root, walls_cache, objects_cache);
        }
        
    public:
        /**
         * Build tree method:
         *
         * Define first plane
         * Separate out objects from each side
         * recurse each side
         */
        BSPTree(BSPPlanes * walls, BSPObjects * objects) {
            walls_cache = BSPPlanes(*walls);
            objects_cache = BSPObjects(*objects);
            add_partitions(&root, walls_cache, objects_cache);
        }
        
        void each_object_node(std::function<void(BSPNode)> f) {
            std::function<void(BSPNode *)> recur = [&](BSPNode * n){
                if (n->back.type == OBJECTS || n->front.type == OBJECTS) {
                    f(*n);
                }
                if (n->back.type == NODE && n->back.node != nullptr) {
                    recur(n->back.node);
                }
                if (n->front.type == NODE && n->front.node != nullptr) {
                    recur(n->front.node);
                }
                
                return;
            };
            recur(&root);
        }
        
        void collision_detection() {
            // Take each node with objects in it
            // Test if they lie on the right side of the plane
            // Only rebuild if object not on right side
            bool rebuild = false;
            // return whether to rebuild
            auto check_bound = [](BSPObjects * os, Plane &p, bool front){
                auto it = os->begin();
                for (; it != os->end(); ++it) {
                    Vector3 pos = (*it)->get_position();
                    // rebuild if different
                    if (p.positive_side(pos) ^ front)
                        return true;
                }
                return false;
            };
            each_object_node([&rebuild, &check_bound](BSPNode n){
                if (n.front.type == OBJECTS) if (check_bound(n.front.objects, n.plane, true)) rebuild = true;
                if (n.back.type == OBJECTS) if (check_bound(n.back.objects, n.plane, false)) rebuild = true;
            });
            if (rebuild) {
                this->rebuild();
                printf("- rebuild tree count: %d\n", rebuild_count++);
            }
        }
    };
    
    // Imported
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    namespace BVH {
        /**
         * Represents a bounding sphere that can be tested for overlap.
         */
        struct BoundingSphere
        {
            Vector3 centre;
            real radius;

        public:
            /**
             * Creates a new bounding sphere at the given centre and radius.
             */
            BoundingSphere(const Vector3 &centre, real radius);

            /**
             * Creates a bounding sphere to enclose the two given bounding
             * spheres.
             */
            BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);

            /**
             * Checks if the bounding sphere overlaps with the other given
             * bounding sphere.
             */
            bool overlaps(const BoundingSphere *other) const;

            /**
             * Reports how much this bounding sphere would have to grow
             * by to incorporate the given bounding sphere. Note that this
             * calculation returns a value not in any particular units (i.e.
             * its not a volume growth). In fact the best implementation
             * takes into account the growth in surface area (after the
             * Goldsmith-Salmon algorithm for tree construction).
             */
            real getGrowth(const BoundingSphere &other) const;

            /**
             * Returns the volume of this bounding volume. This is used
             * to calculate how to recurse into the bounding volume tree.
             * For a bounding sphere it is a simple calculation.
             */
            real getSize() const
            {
                return ((real)1.333333) * pi * radius * radius * radius;
            }
        };

        /**
         * Stores a potential contact to check later.
         */
        struct PotentialContact
        {
            /**
             * Holds the bodies that might be in contact.
             */
            RigidBody* body[2];
        };

        /**
         * A base class for nodes in a bounding volume hierarchy.
         *
         * This class uses a binary tree to store the bounding
         * volumes.
         */
        template<class BoundingVolumeClass>
        class BVHNode
        {
        public:
            /**
             * Holds the child nodes of this node.
             */
            BVHNode * children[2];

            /**
             * Holds a single bounding volume encompassing all the
             * descendents of this node.
             */
            BoundingVolumeClass volume;

            /**
             * Holds the rigid body at this node of the hierarchy.
             * Only leaf nodes can have a rigid body defined (see isLeaf).
             * Note that it is possible to rewrite the algorithms in this
             * class to handle objects at all levels of the hierarchy,
             * but the code provided ignores this vector unless firstChild
             * is NULL.
             */
            RigidBody * body;

            // ... other BVHNode code as before ...

            /**
             * Holds the node immediately above us in the tree.
             */
            BVHNode * parent;

            /**
             * Creates a new node in the hierarchy with the given parameters.
             */
            BVHNode(BVHNode *parent, const BoundingVolumeClass &volume,
                RigidBody* body=NULL)
                : parent(parent), volume(volume), body(body)
            {
                children[0] = children[1] = NULL;
            }

            /**
             * Checks if this node is at the bottom of the hierarchy.
             */
            bool isLeaf() const
            {
                return (body != NULL);
            }

            /**
             * Checks the potential contacts from this node downwards in
             * the hierarchy, writing them to the given array (up to the
             * given limit). Returns the number of potential contacts it
             * found.
             */
            unsigned getPotentialContacts(PotentialContact* contacts,
                                          unsigned limit) const;

            /**
             * Inserts the given rigid body, with the given bounding volume,
             * into the hierarchy. This may involve the creation of
             * further bounding volume nodes.
             */
            void insert(RigidBody* body, const BoundingVolumeClass &volume);

            /**
             * Deltes this node, removing it first from the hierarchy, along
             * with its associated
             * rigid body and child nodes. This method deletes the node
             * and all its children (but obviously not the rigid bodies). This
             * also has the effect of deleting the sibling of this node, and
             * changing the parent node so that it contains the data currently
             * in that sibling. Finally it forces the hierarchy above the
             * current node to reconsider its bounding volume.
             */
            ~BVHNode();

        protected:

            /**
             * Checks for overlapping between nodes in the hierarchy. Note
             * that any bounding volume should have an overlaps method implemented
             * that checks for overlapping with another object of its own type.
             */
            bool overlaps(const BVHNode<BoundingVolumeClass> *other) const;

            /**
             * Checks the potential contacts between this node and the given
             * other node, writing them to the given array (up to the
             * given limit). Returns the number of potential contacts it
             * found.
             */
            unsigned getPotentialContactsWith(
                const BVHNode<BoundingVolumeClass> *other,
                PotentialContact* contacts,
                unsigned limit) const;

            /**
             * For non-leaf nodes, this method recalculates the bounding volume
             * based on the bounding volumes of its children.
             */
            void recalculateBoundingVolume(bool recurse = true);
        };

        // Note that, because we're dealing with a template here, we
        // need to have the implementations accessible to anything that
        // imports this header.

        template<class BoundingVolumeClass>
        bool BVHNode<BoundingVolumeClass>::overlaps(
            const BVHNode<BoundingVolumeClass> * other
            ) const
        {
            return volume->overlaps(other->volume);
        }

        template<class BoundingVolumeClass>
        void BVHNode<BoundingVolumeClass>::insert(
            RigidBody* newBody, const BoundingVolumeClass &newVolume
            )
        {
            // If we are a leaf, then the only option is to spawn two
            // new children and place the new body in one.
            if (isLeaf())
            {
                // Child one is a copy of us.
                children[0] = new BVHNode<BoundingVolumeClass>(
                    this, volume, body
                    );

                // Child two holds the new body
                children[1] = new BVHNode<BoundingVolumeClass>(
                    this, newVolume, newBody
                    );

                // And we now loose the body (we're no longer a leaf)
                this->body = NULL;

                // We need to recalculate our bounding volume
                recalculateBoundingVolume();
            }

            // Otherwise we need to work out which child gets to keep
            // the inserted body. We give it to whoever would grow the
            // least to incorporate it.
            else
            {
                if (children[0]->volume.getGrowth(newVolume) <
                    children[1]->volume.getGrowth(newVolume))
                {
                    children[0]->insert(newBody, newVolume);
                }
                else
                {
                    children[1]->insert(newBody, newVolume);
                }
            }
        }

        template<class BoundingVolumeClass>
        BVHNode<BoundingVolumeClass>::~BVHNode()
        {
            // If we don't have a parent, then we ignore the sibling
            // processing
            if (parent)
            {
                // Find our sibling
                BVHNode<BoundingVolumeClass> *sibling;
                if (parent->children[0] == this) sibling = parent->children[1];
                else sibling = parent->children[0];

                // Write its data to our parent
                parent->volume = sibling->volume;
                parent->body = sibling->body;
                parent->children[0] = sibling->children[0];
                parent->children[1] = sibling->children[1];

                // Delete the sibling (we blank its parent and
                // children to avoid processing/deleting them)
                sibling->parent = NULL;
                sibling->body = NULL;
                sibling->children[0] = NULL;
                sibling->children[1] = NULL;
                delete sibling;

                // Recalculate the parent's bounding volume
                parent->recalculateBoundingVolume();
            }

            // Delete our children (again we remove their
            // parent data so we don't try to process their siblings
            // as they are deleted).
            if (children[0]) {
                children[0]->parent = NULL;
                delete children[0];
            }
            if (children[1]) {
                children[1]->parent = NULL;
                delete children[1];
            }
        }

        template<class BoundingVolumeClass>
            void BVHNode<BoundingVolumeClass>::recalculateBoundingVolume(
            bool recurse
            )
        {
            if (isLeaf()) return;

            // Use the bounding volume combining constructor.
            volume = BoundingVolumeClass(
                children[0]->volume,
                children[1]->volume
                );

            // Recurse up the tree
            if (parent) parent->recalculateBoundingVolume(true);
        }

        template<class BoundingVolumeClass>
        unsigned BVHNode<BoundingVolumeClass>::getPotentialContacts(
            PotentialContact* contacts, unsigned limit
            ) const
        {
            // Early out if we don't have the room for contacts, or
            // if we're a leaf node.
            if (isLeaf() || limit == 0) return 0;

            // Get the potential contacts of one of our children with
            // the other
            return children[0]->getPotentialContactsWith(
                children[1], contacts, limit
                );
        }

        template<class BoundingVolumeClass>
        unsigned BVHNode<BoundingVolumeClass>::getPotentialContactsWith(
            const BVHNode<BoundingVolumeClass> *other,
            PotentialContact* contacts,
            unsigned limit
            ) const
        {
            // Early out if we don't overlap or if we have no room
            // to report contacts
            if (!overlaps(other) || limit == 0) return 0;

            // If we're both at leaf nodes, then we have a potential contact
            if (isLeaf() && other->isLeaf())
            {
                contacts->body[0] = body;
                contacts->body[1] = other->body;
                return 1;
            }

            // Determine which node to descend into. If either is
            // a leaf, then we descend the other. If both are branches,
            // then we use the one with the largest size.
            if (other->isLeaf() ||
                (!isLeaf() && volume->getSize() >= other->volume->getSize()))
            {
                // Recurse into ourself
                unsigned count = children[0]->getPotentialContactsWith(
                    other, contacts, limit
                    );

                // Check we have enough slots to do the other side too
                if (limit > count) {
                    return count + children[1]->getPotentialContactsWith(
                        other, contacts+count, limit-count
                        );
                } else {
                    return count;
                }
            }
            else
            {
                // Recurse into the other node
                unsigned count = getPotentialContactsWith(
                    other->children[0], contacts, limit
                    );

                // Check we have enough slots to do the other side too
                if (limit > count) {
                    return count + getPotentialContactsWith(
                        other->children[1], contacts+count, limit-count
                        );
                } else {
                    return count;
                }
            }
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /**
     * RIGIDBODY Object Interface
     */
    struct R_Object : public RigidBody {
        R_Object() {}
        R_Object(Vector3 bounds) {
            set_position(Vector3(
                rand() % (unsigned)bounds.x,
                rand() % (unsigned)bounds.y,
                0
            ));
        }
    };
    
    typedef std::vector<R_Object *> R_BSPObjects;
    
    struct BoundingSphereHierarchy {
        BVH::BVHNode<BVH::BoundingSphere> root;
        
        Vector3 get_centroid(std::vector<R_Object*> *rbs) {
            Vector3 sum;
            real ratio = rbs->size();
            auto it = rbs->begin();
            for (; it != rbs->end(); ++it) {
                sum += (*it)->get_position();
            }
            return sum * ratio;
        }
        
        real get_radius(std::vector<R_Object*> *rbs) {
            Vector3 center = get_centroid(rbs);
            real longest = 0.0;
            auto it = rbs->begin();
            for (; it != rbs->end(); ++it) {
                real distance = center.distance((*it)->get_position());
                if (distance > longest) longest = distance;
            }
            return longest;
        }
        
        BoundingSphereHierarchy(
            std::vector<R_Object*> rbs
        ) : root(NULL, BVH::BoundingSphere(get_centroid(&rbs), get_radius(&rbs))) {
            auto it = rbs.begin();
            for (; it != rbs.end(); ++it) {
                const BVH::BoundingSphere bs((*it)->get_position(), 3.f);
                root.insert(*it, bs);
            }
        }
    };
    
    /**
     * RIGIDBODY CHILD
     */
    struct R_BSPNode;
    class R_BSPChild {
    public:
        BSPChildType type;
        // keeps track of where objects should be
        bool front;
        union {
            R_BSPNode * node;
            // set of objects in front of plane
            R_BSPObjects * objects;
        };
        
        // Loaded when type == OBJECTS
        BoundingSphereHierarchy * BSH;
        
    public:
        R_BSPChild() : type(NODE), node(nullptr) {}
        R_BSPChild(R_BSPNode * n): type(NODE), node(n) {}
        R_BSPChild(
            R_BSPObjects * o,
            bool front
        ) : type(OBJECTS), objects(o), front(front), BSH(new BoundingSphereHierarchy(*o)) {}
        
        void destroy() {
            if (type == OBJECTS) {
                delete objects;
                delete BSH;
                objects = nullptr;
            }
            if (type == NODE) {
                delete node;
                node = nullptr;
            }
        }
        
        void set_node(R_BSPNode * n) { assert(type == NODE); node = n; }
        void set_objects(R_BSPObjects * o) { assert(type == OBJECTS); objects = o; }
    };
    
    /**
     * RIGIDBODY NODE
     */
    struct R_BSPNode {
        Plane plane;
        R_BSPChild front;
        R_BSPChild back;
        
        R_BSPNode() {}
    };
    
    class BVH_BSPTree {
        R_BSPNode root;
        // cache for rebuilding
        BSPPlanes walls_cache;
        R_BSPObjects objects_cache;
        unsigned rebuild_count = 0;
        
        void add_partitions(R_BSPNode * n, BSPPlanes walls, R_BSPObjects objects) {
            if (walls.size()) {
                // get first wall in set
                Plane c = walls.back();
                n->plane = c;
                walls.pop_back();
                
                R_BSPObjects front_objects;
                R_BSPObjects back_objects;
                BSPPlanes front_walls;
                BSPPlanes back_walls;
                
                auto objects_it = objects.begin();
                for (; objects_it != objects.end(); ++objects_it) {
                    // sort objects
                    real indicator = n->plane.positive_side((*objects_it)->get_position());
            
                    if (indicator) front_objects.push_back(*objects_it);
                    else back_objects.push_back(*objects_it);
                }
                
                auto walls_it = walls.begin();
                for (; walls_it != walls.end(); ++walls_it) {
                    // sort walls
                    real indicator = n->plane.side_of_plane(walls_it->position);
            
                    if (indicator > 0) front_walls.push_back(*walls_it);
                    else back_walls.push_back(*walls_it);
                }
                
                if (front_walls.size()) {
                    n->front = R_BSPChild(new R_BSPNode());
                    add_partitions(n->front.node, front_walls, front_objects);
                }
                else {
                    n->front = R_BSPChild(new R_BSPObjects(front_objects), true);
                }
                if (back_walls.size()) {
                    n->back = R_BSPChild(new R_BSPNode());
                    add_partitions(n->back.node, back_walls, back_objects);
                }
                else {
                    n->back = R_BSPChild(new R_BSPObjects(back_objects), false);
                }
            }
        }
        
        void kill() {
            std::function<void(R_BSPNode *)> recur = [&](R_BSPNode * n){
                // recurse all points
                if (n->back.type == NODE && n->back.node != nullptr) {
                    recur(n->back.node);
                }
                if (n->front.type == NODE && n->front.node != nullptr) {
                    recur(n->front.node);
                }
                // delete from bottom up
                if (n) n->back.destroy();
                if (n) n->front.destroy();
                return;
            };
            recur(&root);
        }
        
        void rebuild() {
            kill();
            add_partitions(&root, walls_cache, objects_cache);
        }
        
    public:
        /**
         * Build tree method:
         *
         * Define first plane
         * Separate out objects from each side
         * recurse each side
         */
        BVH_BSPTree(BSPPlanes * walls, R_BSPObjects * objects) {
            walls_cache = BSPPlanes(*walls);
            objects_cache = R_BSPObjects(*objects);
            add_partitions(&root, walls_cache, objects_cache);
        }
        
        void each_object_node(std::function<void(R_BSPNode)> f) {
            std::function<void(R_BSPNode *)> recur = [&](R_BSPNode * n){
                if (n->back.type == OBJECTS || n->front.type == OBJECTS) {
                    f(*n);
                }
                if (n->back.type == NODE && n->back.node != nullptr) {
                    recur(n->back.node);
                }
                if (n->front.type == NODE && n->front.node != nullptr) {
                    recur(n->front.node);
                }
                
                return;
            };
            recur(&root);
        }
        
        void collision_detection() {
            // Take each node with objects in it
            // Test if they lie on the right side of the plane
            // Only rebuild if object not on right side
            bool rebuild = false;
            // return whether to rebuild
            auto check_bound = [](R_BSPObjects * os, Plane &p, bool front){
                auto it = os->begin();
                for (; it != os->end(); ++it) {
                    Vector3 pos = (*it)->get_position();
                    // rebuild if different
                    if (p.positive_side(pos) ^ front)
                        return true;
                }
                return false;
            };
            each_object_node([&rebuild, &check_bound](R_BSPNode n){
                if (n.front.type == OBJECTS) if (check_bound(n.front.objects, n.plane, true)) rebuild = true;
                if (n.back.type == OBJECTS) if (check_bound(n.back.objects, n.plane, false)) rebuild = true;
            });
            if (rebuild) {
                this->rebuild();
                printf("- rebuild tree count: %d\n", rebuild_count++);
            }
        }
    };
}

#endif /* defined(__MSIM495__collisionengine__) */
