/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/console.h>
#include <ros/ros.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/selection_manager.h>

#include "flag_tool_3d.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


namespace rviz_waypoint_plugin
{

// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
FlagTool3D::FlagTool3D()
    : moving_flag_node_( NULL )
    , current_flag_property_( NULL )
{
    shortcut_key_ = 'm';
    pub = node.advertise<geometry_msgs::PoseArray>("/flag_poses",1000);
    //sub = node.subscribe<PointCloud>("/cloud_pcd",1,&PlantFlagTool::callback,this);
    //sub = node.subscribe<PointCloud>("/static_point_cloud",1,&PlantFlagTool::callback,this);
    sub = node.subscribe<PointCloud>("/point_map",1,&FlagTool3D::callback,this);


}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
FlagTool3D::~FlagTool3D()
{
    for( unsigned i = 0; i < flag_nodes_.size(); i++ )
    {
        scene_manager_->destroySceneNode( flag_nodes_[ i ]);
    }
}

void FlagTool3D::callback(const PointCloud::ConstPtr& msg)
{
    pcl::copyPointCloud(*msg,pcd);
    //printf (">>Point Cloud: width = %d, height = %d\n", pcd.width, pcd.height);

}

double FlagTool3D::computeDistance( Ogre::Vector2& a, Ogre::Vector2& b ){
    return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.
void FlagTool3D::onInitialize()
{
    //flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";
    flag_resource_ = "package://rviz_waypoint_plugin/mesh/barrier_scaled.dae";

    if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
    {
        ROS_ERROR( "PlantFlagTool: failed to load model resource '%s'.", flag_resource_.c_str() );
        return;
    }

    moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
    moving_flag_node_->attachObject( entity );
    moving_flag_node_->setVisible( false );
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the flag.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_flag_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void FlagTool3D::activate()
{
    if( moving_flag_node_ )
    {
        //moving_flag_node_->setVisible( true );

        current_flag_property_ = new rviz::VectorProperty( "Flag " + QString::number( flag_nodes_.size() ));
        current_flag_property_->setReadOnly( true );
        getPropertyContainer()->addChild( current_flag_property_ );
    }

}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of flags when
// we switch to another tool.
void FlagTool3D::deactivate()
{
    if( moving_flag_node_ )
    {
        moving_flag_node_->setVisible( false );
        delete current_flag_property_;
        current_flag_property_ = NULL;
    }
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing, then
// move the moving flag to that point and update the VectorProperty.
//
// If this mouse event was a left button press, we want to save the
// current flag location.  Therefore we make a new flag at the same
// place and drop the pointer to the VectorProperty.  Dropping the
// pointer means when the tool is deactivated the VectorProperty won't
// be deleted, which is what we want.
int FlagTool3D::processMouseEvent( rviz::ViewportMouseEvent& event )
{

    if( !moving_flag_node_ )
    {
        return Render;
    }

    Ogre::Vector3 intersection,intersection1;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );



    if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                           ground_plane,
                                           event.x, event.y, intersection ))
    {
        //moving_flag_node_->setVisible( true );
        moving_flag_node_->setPosition( intersection );
        current_flag_property_->setVector( intersection );

        if( event.leftDown() )
        {
            std::cout << "Computing projection for " << pcd.size() << " points.\n";
            for( size_t i = 0; i < pcd.size(); i++ ) {
                Ogre::Vector3 temp3D(pcd.points[i].x,pcd.points[i].y,pcd.points[i].z);
                Ogre::Vector2 temp2D = rviz::project3DPointToViewportXY(event.viewport,temp3D);
                //std::cout << "\t3D - x: " << temp3D.x << " y: " << temp3D.y << " z: " << temp3D.z << "\t";
                //std::cout << "2D - x: " << temp2D.x << " y: " << temp2D.y << "\n";
                correspondences.push_back(std::make_pair(temp3D,temp2D));
            }

            Ogre::Vector2 mouse_pos(event.x,event.y),closest;
            std::cout << "Mouse position - x: " << event.x << " y: " << event.y << "\n";

            std::cout << "Computing distances for " << correspondences.size() << " points:\n";
            double min_dist = 1000000;
            for(std::vector<std::pair<Ogre::Vector3,Ogre::Vector2> >::iterator it=correspondences.begin(); it != correspondences.end(); ++it){
                std::pair<Ogre::Vector3,Ogre::Vector2> pair = *it;
                double dist = FlagTool3D::computeDistance(mouse_pos,pair.second);
                //std::cout << "\t2D Point - x: " << pair.second.x << " y: " << pair.second.y << "\tdistance: " << dist << "\n";
                if(dist < min_dist) {
                    min_dist = dist;
                    intersection1 = pair.first;
                    closest = pair.second;
                }
            }
            std::cout << "Closest point - x: " << closest.x << " y: " << closest.y << "\n";
            std::cout << "Distance: " << min_dist << "\n";
            std::cout << "3D point - x: " << intersection1.x << " y: " << intersection1.y << " z: " << intersection1.z << "\n";

            moving_flag_node_->setVisible( true );

            makeFlag( intersection1 );

            //std::cout << "Ogre 3D point - x: " << intersection.x << " y: " << intersection.y << " z: " << intersection.z << "\n";
            //std::cout << "--------------------------------------------------------------------------------------------------\n";

            //Adding flag pose to the list
            geometry_msgs::Pose wp_pose;
            wp_pose.position.x = intersection1.x;
            wp_pose.position.y = intersection1.y;
            wp_pose.position.z = intersection1.z;
            wplist.poses.push_back(wp_pose);

            current_flag_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
            return Render | Finished;
        }

        if( event.rightDown() ){
            //Publishing the flag pose
            pub.publish(wplist);

            return Render | Finished;
        }
    }
    else
    {
        moving_flag_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
    }
    correspondences.clear();
    return Render;

}

// This is a helper function to create a new flag in the Ogre scene and save its scene node in a list.
void FlagTool3D::makeFlag( const Ogre::Vector3& position )
{
    Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
    node->attachObject( entity );
    node->setVisible( true );
    node->setPosition( position );
    flag_nodes_.push_back( node );
}

// Loading and saving the flags
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named flag positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.
void FlagTool3D::save( rviz::Config config ) const
{
    config.mapSetValue( "Class", getClassId() );

    // The top level of this tool's Config is a map, but our flags
    // should go in a list, since they may or may not have unique keys.
    // Therefore we make a child of the map (``flags_config``) to store
    // the list.
    rviz::Config flags_config = config.mapMakeChild( "Flags" );

    // To read the positions and names of the flags, we loop over the
    // the children of our Property container:
    rviz::Property* container = getPropertyContainer();
    int num_children = container->numChildren();
    for( int i = 0; i < num_children; i++ )
    {
        rviz::Property* position_prop = container->childAt( i );
        // For each Property, we create a new Config object representing a
        // single flag and append it to the Config list.
        rviz::Config flag_config = flags_config.listAppendNew();
        // Into the flag's config we store its name:
        flag_config.mapSetValue( "Name", position_prop->getName() );
        // ... and its position.
        position_prop->save( flag_config );
    }
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void FlagTool3D::load( const rviz::Config& config )
{
    // Here we get the "Flags" sub-config from the tool config and loop over its entries:
    rviz::Config flags_config = config.mapGetChild( "Flags" );
    int num_flags = flags_config.listLength();
    for( int i = 0; i < num_flags; i++ )
    {
        rviz::Config flag_config = flags_config.listChildAt( i );
        // At this point each ``flag_config`` represents a single flag.
        //
        // Here we provide a default name in case the name is not in the config file for some reason:
        QString name = "Flag " + QString::number( i + 1 );
        // Then we use the convenience function mapGetString() to read the
        // name from ``flag_config`` if it is there.  (If no "Name" entry
        // were present it would return false, but we don't care about
        // that because we have already set a default.)
        flag_config.mapGetString( "Name", &name );
        // Given the name we can create an rviz::VectorProperty to display the position:
        rviz::VectorProperty* prop = new rviz::VectorProperty( name );
        // Then we just tell the property to read its contents from the config, and we've read all the data.
        prop->load( flag_config );
        // We finish each flag by marking it read-only (as discussed
        // above), adding it to the property container, and finally making
        // an actual visible flag object in the 3D scene at the correct
        // position.
        prop->setReadOnly( true );
        getPropertyContainer()->addChild( prop );
        makeFlag( prop->getVector() );
    }
}




// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_waypoint_plugin::FlagTool3D,rviz::Tool )
// END_TUTORIAL
