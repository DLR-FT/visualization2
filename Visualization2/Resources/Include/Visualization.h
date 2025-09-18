/*!
 * @file     Visualization.h
 * @brief    DLR Visualization Library API
 * @mainpage Visualization Library API Reference
 * ############################################################################
 *
 *                                                              ▄█             
 *                                                            ▄█▀█             
 *                                                          ▄█▀  █             
 *                                                        ▄█▀    █             
 *                                                      ▄█▀      █             
 *                                                    ▄█▀        █             
 *                                         ▄█████████████████████████████▀     
 *                                       ▄█▀      ▄█▀        ▄█▀      ▄█▀      
 *                                     ▄█▀      ▄█▀        ▄█▀      ▄█▀        
 *                                   ▄█▀      ▄█▀        ▄█▀      ▄█▀          
 *                                 ▄██████████████████████████████▀            
 *                                          █        ▄█▀                       
 *                                          █      ▄█▀                         
 *                                          █    ▄█▀                           
 *                                          █  ▄█▀                             
 *                                          █▄█▀      █▀▀▀▄  █    █▀▀▀▄        
 *                                          █▀        █    █ █    █   █        
 *                                                    █    █ █    █▀▀█         
 *                                                    █▄▄▄▀  █▄▄▄ █   █        
 *                                                                             
 *     Deutsches Zentrum für Luft- und Raumfahrt e.V. (DLR)                    
 *     Robotik- und Mechatronikzentrum (RMC)                                   
 *     Institut für Systemdynamik und Regelungstechnik (SR)                    
 *                                                                             
 *     German Aerospace Center (DLR)                                           
 *     Robotics and Mechatronic Center (RMC)                                   
 *     Institute of System dynamics and Control (SR)                           
 *                                                                             
 * Web: <http://www.dlr.de/rm/en>                                            \n
 * Email: <mailto:rm-modelica@dlr.de>                                        \n
 * Address:                                                                  \n
 * >          Postfach 1116                                                  \n
 * >          82230 Wessling                                                 \n
 * >          Germany                                                        \n
 * Responsible: Dr.-Ing. Tobias Bellmann                                     \n
 * Authors: Matthias Hellerer, Sebastian Kuemper                             \n
 * Copyright: All rights reserved! &copy; 2018                               \n
 *                                                                           \n
 * ____________________________________________________________________________
 *
 * Visualization API
 * =================
 *
 * List of all exported functions:
 * <a href="Visualization_8h.html">Visualization API</a> 
 *
 * This Library ...
 *
 * On Linux this library must be linked with pthread (if possible use the
 * '-pthread' switch)
 *
 * The main loop should look something like this:
 *     vis_log_setLogger
 *     vis_startSimVis
 *     vis_init
 *     create objects
 *     time = 0
 *     loop:
 *         update object states
 *         time = time + step
 *         vis_advance(time)
 *         if time >= endTime:
 *             break
 *     vis_terminate
 *
 * ############################################################################
 */

#pragma once
#ifndef VISUALIZATION
#define VISUALIZATION

#ifdef MODELICA
#define USE_VOID_INTERFACE
#endif

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _MSC_VER
#pragma comment( lib, "ws2_32.lib" )
#endif

#ifdef _MSC_VER
#ifdef EXTERNAL_FUNCTION_EXPORT
    #define EXPORT __declspec( dllexport )
#else
    #define EXPORT
#endif
#elif __GNUC__ >= 4
#define EXPORT __attribute__ ((visibility("default")))
#else
#define EXPORT extern
#endif

// define USE_VOID_INTERFACE to use void* for all internal data structures
// on the interface. Primarily for interfacing with Dymola.
#ifdef USE_VOID_INTERFACE
typedef void vis_connection;
typedef void vis_object;
typedef void vis_overlay_buttonEvents;
typedef void vis_CADFile;
typedef void vis_CADArray;
typedef void vis_riggedCADFile;
typedef void vis_primitive;
typedef void vis_gearwheel;
typedef void vis_spring;
typedef void vis_extrudedshape;
typedef void vis_pipe;
typedef void vis_flexibleSurface;
typedef void vis_flexibleBody;
typedef void vis_camera;
typedef void vis_sync;
typedef void vis_windowSettings;
typedef void vis_environment;
typedef void vis_overlay;
typedef void vis_streamSource;
typedef void vis_texturedSurface;
typedef void vis_light;
typedef void vis_terrain;
typedef void vis_lineIntersector;
typedef void vis_billboard;
typedef void vis_text3d;
typedef void vis_depthMesh;
typedef void vis_depthStreamSource;
typedef void vis_pointCloud;
typedef void vis_line3d;
typedef void vis_pathFlow;
typedef void vis_gridParticleFlow;
typedef void vis_gridVectorField;
typedef void vis_cesiumTile;
typedef void vis_cloudLayer;
typedef void vis_coordSystem;

typedef void vis_geodeticConverter;
#else
struct vis_object_t;
typedef struct vis_object_t vis_object;
struct vis_connection_t;
typedef struct vis_connection_t vis_connection;
struct vis_primitive_t;
typedef struct vis_primitive_t vis_primitive;
struct vis_gearwheel_t;
typedef struct vis_gearwheel_t vis_gearwheel;
struct vis_spring_t;
typedef struct vis_spring_t vis_spring;
struct vis_extrudedshape_t;
typedef struct vis_extrudedshape_t vis_extrudedshape;
struct vis_pipe_t;
typedef struct vis_pipe_t vis_pipe;
struct vis_CADFile_t;
typedef struct vis_CADFile_t vis_CADFile;
struct vis_CADArray_t;
typedef struct vis_CADArray_t vis_CADArray;
struct vis_riggedCADFile_t;
typedef struct vis_riggedCADFile_t vis_riggedCADFile;
struct vis_flexibleSurface_t;
typedef struct vis_flexibleSurface_t vis_flexibleSurface;
struct vis_flexibleBody_t;
typedef struct vis_flexibleBody_t vis_flexibleBody;
struct vis_camera_t;
typedef struct vis_camera_t vis_camera;
struct vis_sync_t;
typedef struct vis_sync_t vis_sync;
struct vis_windowSettings_t;
typedef struct vis_windowSettings_t vis_windowSettings;
struct vis_environment_t;
typedef struct vis_environment_t vis_environment;
struct vis_overlay_t;
typedef struct vis_overlay_t vis_overlay;
struct vis_streamSource_t;
typedef struct vis_streamSource_t vis_streamSource;
struct vis_texturedSurface_t;
typedef struct vis_texturedSurface_t vis_texturedSurface;
struct vis_light_t;
typedef struct vis_light_t vis_light;
struct vis_terrain_t;
typedef struct vis_terrain_t vis_terrain;
struct vis_lineIntersector_t;
typedef struct vis_lineIntersector_t vis_lineIntersector;
struct vis_billboard_t;
typedef struct vis_billboard_t vis_billboard;
struct vis_text3d_t;
typedef struct vis_text3d_t vis_text3d;
struct vis_depthMesh_t;
typedef struct vis_depthMesh_t vis_depthMesh;
struct vis_depthStreamSource_t;
typedef struct vis_depthStreamSource_t vis_depthStreamSource;
struct vis_pointCloud_t;
typedef struct vis_pointCloud_t vis_pointCloud;
struct vis_line3d_t;
typedef struct vis_line3d_t vis_line3d;
struct vis_pathFlow_t;
typedef struct vis_pathFlow_t vis_pathFlow;
struct vis_gridParticleFlow_t;
typedef struct vis_gridParticleFlow_t vis_gridParticleFlow;
struct vis_gridVectorField_t;
typedef struct vis_gridVectorField_t vis_gridVectorField;
struct vis_cesiumTile_t;
typedef struct vis_cesiumTile_t vis_cesiumTile;
struct vis_cloudLayer_t;
typedef struct vis_cloudLayer_t vis_cloudLayer;
struct vis_coordSystem_t;
typedef struct vis_coordSystem_t vis_coordSystem;
typedef struct
{
    int inside;
    int pressed;
    int clicked;
    int mousePosX;
    int mousePosY;
} vis_overlay_buttonEvents;
struct vis_geodeticConverter_t;
typedef struct vis_geodeticConverter_t vis_geodeticConverter;
#endif

#ifdef MODELICA
void
vis_setModelicaFunctions();
#endif

/*! Start a new instance of SimVis
 * Will check for an already running instance on the given port and only start
 * a new instance if no existing one is found.
 * @param path the path to the binary
 * @param port TCP communication port for SimVis to open (default: 2345)
 */
EXPORT void
vis_startSimVis( const char* path, int port );

/*! Initialize the visualization system
 * Needs to be called by the simulation before any other function is used
 * @param directToReplay if true the data is written directly into replay
 *   without visualizing it
 * @param host host name where the visualization is currently running
 *   (ignored for directToReplay!=0)
 * @param port TCP port of the visualization (ignored for directToReplay!=0)
 * @param compress if true the data is compressed before sending it to the
 *   visualization, useful if the bandwidth is limiting
 * @param replayFileName the target filename if directToReplay is used
 * @param keyFrameInterval how often keyframes are inserted into the replay
 * @return the connection to the visualization or null on error
 */
EXPORT vis_connection*
vis_init( int directToReplay, const char* host, int port, int compress,
    const char* replayFileName, int keyFrameInterval );

/*! Advance simulation
* This function needs to be called after all objects for the current timestamp
* were updated.
* @param time time to advance to
*/
EXPORT void
vis_advance( vis_connection* con, double time );

/*! Terminate connection
* @param connection connection to terminate
* @post connection may no longer be used by any other function and all
*     objects created for this connection become invalid.
*/
EXPORT void
vis_terminate( vis_connection* con );

/*! Check if connection is active
 * @return returns 1 if connection is active, returns 0 if the connection is
 *     is down or experiencing problems
 */
EXPORT int
vis_isActive( vis_connection* con );

/*!
 * get the path to to the default binary relative to the library root
 * @return the relative path the default binary
 */
EXPORT const char*
vis_getDefaultBinaryPath();

/* Objects */
#ifdef USE_VOID_INTERFACE
#define Primitive_t int
#else
typedef enum
{
    Primitive_Cube = 1,
    Primitive_Sphere,
    Primitive_Cylinder,
    Primitive_Capsule,
    Primitive_Plane,
    Primitive_Prism,
    Primitive_Dodecahedron,
    Primitive_Icosahedron,
    Primitive_Tube,
    Primitive_Cone,
    Primitive_HollowCone
} Primitive_t;
#endif
/*! Metafunction to create a primitive
 * Used by Modelica, takes parameters of all primitve create functions and 
 * creates the one specified by type.
 */
EXPORT vis_primitive*
vis_primitive_create( vis_connection* con, const char* name, 
    Primitive_t type, int subdivisions, int numSides );

/*! Create a Cube object
 */
EXPORT vis_primitive*
vis_primitive_createCube( vis_connection* con, const char* name );

/*! Create a sphere object
 * @param subdivisions number vertical and horizontal subdivisions
 */
EXPORT vis_primitive*
vis_primitive_createSphere( vis_connection* con, const char* name,
    int subdivisions );

/*! Create a cylinder object
 * @param radius radius of the cylinder
 * @param height height of the cylinder
 * @param slices number of rotary subdivisions
 */
EXPORT vis_primitive*
vis_primitive_createCylinder( vis_connection* con, const char* name,
    int slices );

/*! Create a capsule object
 * @param radius radius of the capsule
 * @param height height of the capsule
 * @param subdivisions number vertical and horizontal subdivisions
 */
EXPORT vis_primitive*
vis_primitive_createCapsule( vis_connection* con, const char* name,
    int subdivisions );

/*! Create a plane object
 */
EXPORT vis_primitive*
vis_primitive_createPlane( vis_connection* con, const char* name );

/*! Create a prism object
 */
EXPORT vis_primitive*
vis_primitive_createPrism( vis_connection* con, const char* name,
    int numSides );

/*! Create a dodecahedron object
 */
EXPORT vis_primitive*
vis_primitive_createDodecahedron( vis_connection* con, const char* name );

/*! Create a icosahedron object
 */
EXPORT vis_primitive*
vis_primitive_createIcosahedron( vis_connection* con, const char* name );

/*! Create a tube object
 * @param slices number of rotary subdivisions
 */
EXPORT vis_primitive*
vis_primitive_createTube( vis_connection* con, const char* name, int slices );

EXPORT vis_primitive*
vis_primitive_createCone( vis_connection* con, const char* name, int slices );

EXPORT vis_primitive*
vis_primitive_createHollowCone( vis_connection* con, const char* name, int slices );

/*! Destroys th primitive object
 */
EXPORT void
vis_primitive_destroy( vis_primitive* primitiveObject );

/*! Set position and orientation of primitive
 * @param position new position of the object ([x,y,z])
 * @param orientation new orientation of object as quaternion ([i,j,k,w])
 */
EXPORT void
vis_primitive_setPose( vis_primitive* obj, const double position[ 3 ],
    const double orientation[ 4 ], const double scale[ 3 ] );

/*! Set material of primitive
 * @param color color of the object ([r,g,b,a])
 * @param metal if true, the object will look metalic (bool)
 * @param roughness roughness of the objects surface ([0,1])
 * @param emission if true, the object seems to emit light
 *               and unaffected by any light source
 * @param emissionBrightness the brightness of the emission
 * @param wireframe if true, the object will be shown in wireframe mode (bool)
 * @param wireFrameColor color of the wireframe if in wireframe mode ([r,g,b,a])
 */
EXPORT void
vis_primitive_setMaterial( vis_primitive* obj, const double color[ 4 ],
    int metal, double roughness, int emission, double emissionBrightness,
    int wireframe, const double wireframeColor[ 4 ] );

/*! Set the size of the cube (only works if primitive is a cube)
 * @param length new length of the cube
 * @param width new width of the cube
 * @param height new height of the cube
 */
EXPORT void
vis_primitive_setCubeSize( vis_primitive*obj, double length, double width,
    double height );

/*! Set the radius of the sphere (only works if primitive is a sphere)
 * @param radius new radius of the sphere
 */
EXPORT void
vis_primitive_setSphereRadius( vis_primitive* obj, double radius );

/*! Set the size of the cylinder (only works if primitive is a cylinder)
 * @param radius new radius of the cylinder
 * @param height new height of the cylinder
 */
EXPORT void
vis_primitive_setCylinderSize( vis_primitive* obj, double radius,
    double height );

/*! Set the size of the capsule (only works if primitive is a capsule)
 * @param radius new radius of the capsule
 * @param height new height of the capsule
 */
EXPORT void
vis_primitive_setCapsuleSize( vis_primitive* obj, double radius,
    double height );

/*! Set the size of the plane (only works if primitive is a plane)
 * @param length new length of the plane
 * @param width new width of the plane
 */
EXPORT void
vis_primitive_setPlaneSize( vis_primitive* obj, double length, double width );

/*! Set the size of the prism (only works if primitive is a prism)
 * @param frontRadius new radius of the circumcircle on the front
 * @param backRadius new radius of the circumcircle on the back
 * @param height new height of the prism
 */
EXPORT void
vis_primitive_setPrismSize( vis_primitive* obj, double frontRadius,
    double backRadius, double height );

/*! Set the radius of the Dodecahedron 
 * (only works if primitive is a dodecahedron)
 * @param radius new radius of the dodecahedron
 */
EXPORT void
vis_primitive_setDodecahedronRadius( vis_primitive* obj, double radius );

/*! Set the radius of the Dodecahedron
 * (only works if primitive is a icosahedron)
 * @param radius new radius of the icosahedron
 */
EXPORT void
vis_primitive_setIcosahedronRadius( vis_primitive* obj, double radius );

/*! Set the length and radii of the tube (only works if primitive is a tube)
 * @param length the length of the tube
 * @param outerRadius1 new outer radius of the tube on one side
 * @param innerRadius1 new inner radius of the tube on one side
 * @param outerRadius2 new outer radius of the tube on other side
 * @param innerRadius2 new inner radius of the tube on other side
 */
EXPORT void
vis_primitive_setTubeSize( vis_primitive* obj, double length,
    double outerRadius1, double innerRadius1, double outerRadius2,
    double innerRadius2 );

EXPORT void
vis_primitive_setConeSize( vis_primitive* obj, double length,
    double radius );

EXPORT void
vis_primitive_setHollowConeSize( vis_primitive* obj, double length,
    double radius, double innerToOuterLength, double innerToOuterRadius );

/*! Set the masks of the primitive
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_primitive_setMasks( vis_primitive* obj, int intersection,
    int visibility, int shadow );

/*! creates a gearwheel
* @param radius radius of the gearwheel
* @param axisRadius radius of the axis the gearwheel is mounted on
* @param width the width of the gearwheel
* @param numTeeth number of teeth of the gearwheel. use negative number
*     for inner gear.
* @param teethDepthRatio the ratio of the teeth depth to radius of the gear wheel
* @param bevelAngle bevel angle in radians. 0 means the gearwheel
*     has no bevel.
*/
EXPORT vis_gearwheel*
vis_gearwheel_create( vis_connection* con, const char* name,
    double radius, double axisRadius, double width, int numTeeth,
    double teethDepthRatio, double bevelAngle);

/*! Destroys the gearwheel object
 */
EXPORT void
vis_gearwheel_destroy( vis_gearwheel* gearwheelObject );

/*! Set position and orientation of gearwheel 
 * @param position new position of the gearwheel ([x,y,z])
 * @param orientation new orientation of gearwheel as quaternion ([i,j,k,w])
 */
EXPORT void
vis_gearwheel_setPose( vis_gearwheel* obj, const double position[ 3 ],
    const double orientation[ 4 ], const double scale[ 3 ] );

/*! Set material of gearwheel 
 * @param color color of the gearwheel ([r,g,b,a])
 * @param metal if true, the gearwheel will look metalic (bool)
 * @param roughness roughness of the gearwheel surface ([0,1])
 * @param emission if true, the object seems to emit light
 *               and unaffected by any light source
 * @param emissionBrightness the brightness of the emission
 * @param wireframe if true, the gearwheel will be shown in wireframe mode (bool)
 * @param wireFrameColor color of the wireframe if in wireframe mode ([r,g,b,a])
 */
EXPORT void
vis_gearwheel_setMaterial( vis_gearwheel* obj, const double color[ 4 ],
    int metal, double roughness, int emission, double emissionBrightness,
    int wireframe, const double wireframeColor[ 4 ] );

/*! Set the masks of the gearwheel
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_gearwheel_setMasks( vis_gearwheel* obj, int intersection,
    int visibility, int shadow );

/*! create extrudedShape 
 * defined by a cross section and points of a path, along which
 * the cross section is extruded
 * @param sectionPoints vertices that make up the 2D shape to be extruded
 *     (each two doubles for x,y)
 *     total size expected: 2 * numSectionPoints * sizeof(double)
 * @param numSectionPoints number of points, that define the cross section
 * @param pathPoints vertices that make up the path that the given 2D shape
 *     should be extruded along (each three doubles for x,y,z)
 *     total size expected: 3 * numPathPoints * sizeof(double)
 * @param upVectors vertices that can make the extruded object turn while
 *     following the path. default upVector is (0,0,1) and lets the object stay
       "head up" (each three doubles for x,y,z)
 *     total size expected: 3 * numPathPoints * sizeof(double)
 * @param numPathPoints number of points that define the path
 * @param interpolation number of interpolation points between two given
 *     pathPoints. set to 0 if no interpolation between pathPoints needed.
 * @param lodDistances the maximum visible distances of the detail levels
 * @param numLods the number of detail levels
 * @param smoothnessAngle the angle in degrees below which everything is
       considered smooth
 */
EXPORT vis_extrudedshape*
vis_extrudedshape_create(vis_connection* con, const char* name,
    const double* sectionPoints, size_t numSectionPoints,
    const double* pathPoints, const double* upVectors, size_t numPathPoints,
    int interpolation, const double* lodDistances, size_t numLods,
    double smoothnessAngle );

/*! Destroys the extruded shape object
 */
EXPORT void
vis_extrudedshape_destroy( vis_extrudedshape* extrudedShapeObject );

/*! Set position and orientation of extrudedShape
 * @param position new position of the extrudedShape ([x,y,z])
 * @param orientation new orientation of extrudedShape as quaternion ([i,j,k,w])
 */
EXPORT void
vis_extrudedshape_setPose(vis_extrudedshape* obj, const double* position,
    const double* orientation, const double* scale);

/*! Set material of extrudedShape
 * @param color color of the extrudedShape ([r,g,b,a])
 * @param metal if true, the extrudedShape will look metalic (bool)
 * @param roughness roughness of the extrudedShape surface ([0,1])
 * @param emission if true, the object seems to emit light
 *               and unaffected by any light source
 * @param emissionBrightness the brightness of the emission
 * @param wireframe if true, the extrudedShape will be shown in wireframe mode (bool)
 * @param wireFrameColor color of the wireframe if in wireframe mode ([r,g,b,a])
 */
EXPORT void
vis_extrudedshape_setMaterial( vis_extrudedshape* obj, const double* color,
    int metal, double roughness, int emission, double emissionBrightness,
    int wireframe, const double wireframeColor[ 4 ] );

/*! Set the masks of the extruded shape
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_extrudedshape_setMasks( vis_extrudedshape* obj, int intersection,
    int visibility, int shadow );

/*! creates a pipe 
 * defined by its radii and path points
 * @param innerRadius inner radius of the pipe
 * @param outerRadius outer radius of the pipe
 * @param pathPoints vertices that make up the path that the pipe should run
 *     along (each three doubles for x,y,z)
 *     total size expected: 3 * numPathPoints * sizeof(double)
 * @param numPathPoints number of path points
 * @param circleInterpolation number of points to use along the circle
 *     (20 => looks okay; 100 => looks great)
 * @param pathInterpolation number of InterpolationSteps between two pathPoints.
 *     set to 0 if no interpolation between pathPoints needed.
 * @param lodDistances the maximum visible distances of the detail levels
 * @param numLods the number of detail levels
*/
EXPORT vis_pipe*
vis_pipe_create(vis_connection* con, const char* name,
    double innerRadius, double outerRadius, const double* pathPoints,
    size_t numPathPoints, int circleInterpolation, int pathInterpolation,
    const double* lodDistances, size_t numLods );

/*! Destroys the pipe object
 */
EXPORT void
vis_pipe_destroy( vis_pipe* pipeObject );

/*! Set position and orientation of Pipe
 * @param position new position of the extrudedShape ([x,y,z])
 * @param orientation new orientation of extrudedShape as quaternion ([i,j,k,w])
 */
EXPORT void
vis_pipe_setPose(vis_pipe* obj, const double* position,
    const double* orientation, const double* scale);

/*! Set material of pipe
 * @param color color of the pipe ([r,g,b,a])
 * @param metal if true, the pipe will look metalic (bool)
 * @param roughness roughness of the pipe surface ([0,1])
 * @param emission if true, the object seems to emit light
 *               and unaffected by any light source
 * @param emissionBrightness the brightness of the emission
 * @param wireframe if true, the pipe will be shown in wireframe mode (bool)
 * @param wireFrameColor color of the wireframe if in wireframe mode ([r,g,b,a])
 */
EXPORT void
vis_pipe_setMaterial( vis_pipe* obj, const double color[ 4 ],
    int metal, double roughness, int emission, double emissionBrightness,
    int wireframe, const double wireframeColor[ 4 ] );

/*! Set the masks of the pipe
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT
void vis_pipe_setMasks( vis_pipe* obj, int intersection,
    int visibility, int shadow );

/*! creates a spring
* @param numWindings number of spring windings
* @param springRadius radius of the spring
* @param springWireRadius radius of the spring wire
* @param detailLevel determines how many polygons are used
*     0 => very crude,
*     5 => looks okay,
*     10 => looks good,
*     > 10 => looks excellent
*/
EXPORT vis_spring*
vis_spring_create( vis_connection* con, const char* name,
    double numWindings, double springRadius, double springWireRadius,
    int detailLevel );

/*! Destroys the spring object
 */
EXPORT void
vis_spring_destroy( vis_spring* springObject );

/*! Set position and orientation of spring
 * @param position new position of the extrudedShape ([x,y,z])
 * @param orientation new orientation of extrudedShape as quaternion ([i,j,k,w])
 */
EXPORT void
vis_spring_setPose(vis_spring* obj, const double* position,
    const double* orientation, const double* scale);

/*! Set material of spring
 * @param color color of the extrudedShape ([r,g,b,a])
 * @param metal if true, the extrudedShape will look metalic (bool)
 * @param roughness roughness of the extrudedShape surface ([0,1])
 * @param emission if true, the object seems to emit light
 *               and unaffected by any light source
 * @param emissionBrightness the brightness of the emission
 * @param wireframe if true, the extrudedShape will be shown in wireframe mode (bool)
 * @param wireFrameColor color of the wireframe if in wireframe mode ([r,g,b,a])
 */
EXPORT void
vis_spring_setMaterial( vis_spring* obj, const double color[ 4 ],
    int metal, double roughness, int emission, double emissionBrightness,
    int wireframe, const double wireframeColor[ 4 ] );

/*! Set the extension of the spring
 * @param new length of the spring
 */
EXPORT void
vis_spring_setLength(vis_spring* obj, double springLength);

/*! Set the masks of the spring
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_spring_setMasks( vis_spring* obj, int intersection,
    int visibility, int shadow );

/*! Create a CAD File object
 * CAD file objects render CAD or other 3D objects
 * @param filenames full path to the object lod files
 * @param maxDistances maximum visible distance of each detail level
 * @param numLODs number of detail levels
 * @param twosided overwrite the object properties and make all surface double
 *     sided, even if they aren't in the file description.
 * @param vertexColor use the vertex colors defined in the CAD
 */
EXPORT vis_CADFile*
vis_CADFile_create( vis_connection* con, const char* name,
    const char** filenames, const double * maxDistances, size_t numLODs,
    int twoSided, int vertexColor );

/*! Destroys the cad file object
 */
EXPORT void
vis_CADFile_destroy( vis_CADFile* cadFileObject );

/*! Set position and orientation of CADFile
 * @param position new position of the CADFile ([x,y,z])
 * @param orientation new orientation of the CADFile as quaternion ([i,j,k,w])
 * @param scale new scale of the CADFile ([x,y,z])
 */
EXPORT void
vis_CADFile_setPose( vis_CADFile* obj, const double position[ 3 ],
    const double orientation[ 4 ], const double scale[ 3 ] );

/*! overwrites the color of the object with the specified one
 * don't call if you want to keep the original CAD file colors
 * @param color color to overwrite with ([r,g,b])
 */
EXPORT void
vis_CADFile_overwriteColor( vis_CADFile* obj, const double color[ 3 ] );

/*! overwrites material properties with the given ones
 * don't call if you want to keep the original CAD file properties
 * @param metal if true, elements have a mettallic appearance
 * @param roughness surface roughness ([0,1], 0:smooth, 1:rough)
 * @param emission if true, the object seems to emit light
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_CADFile_setSurfaceProperties( vis_CADFile* obj, int metal,
    double roughness, int emission, double emissionBrightness );

/*! set the brightness of the emission
 * if @ref vis_CADFile_setSurfaceProperties hasn't been called it controls the
 * emission that is set within the cad file
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_CADFile_setEmissionBrightness( vis_CADFile* obj, double emissionBrightness );

/*! Set the transparency of the object
 * keeps the original colors if it wasn't overwritten
 * @param transparency the transparency of the object
 *     ([0,1], 0:invisible, 1:opaque)
 */
EXPORT void
vis_CADFile_setTransparency( vis_CADFile* obj, double transparency );

/*! Set the wireframe mode for the object
 * @param enabled if true, the object is shown in wireframe mode
 * @param color the color of the wireframe lines ([r,g,b,a])
 */
EXPORT void
vis_CADFile_setWireframe( vis_CADFile* obj, int enabled,
    const double color[ 4 ] );

/*! Set the masks of the CADFile
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_CADFile_setMasks( vis_CADFile* obj, int intersection,
    int visibility, int shadow );

/*! creates an array of cad files
 * @param useFileInput if true, the transforms of the objects will be loaded
 *      from a file instead of set by vis_CADArray_setPoses
 * @param transformsFile path to csv file, that holds the transforms if
 *      useFileInput is used
 * @param filenames filenames of the objects at different detail levels
 * @param distances maximum visible distance of each detail level
 * @param numLODs the number of LODs passed to this function
 * @param twosided overwrite the object properties and make all surface double
 *      sided, even if they aren't in the file description.
 * @param vertexColor if true, the vertex colors specified
 *      in the cad files will be used (bool)
*/
EXPORT vis_CADArray*
vis_CADArray_create( vis_connection* con, const char* name,
    int useFileInput, const char* transformsFile,
    const char*const* filenames, const double* distances, size_t numLODs,
    int twoSided, int vertexColor );

/*! Destroys the cad array object
 */
EXPORT void
vis_CADArray_destroy( vis_CADArray* cadArrayObject );

/*! Set the poses of the individual objects
 * only usabale when useFileInput was false
 * @param cadArray the cadArray to set the positions of
 * @param numObjects the number of objects to display
 * @param positions array of the new positions of the objects
                    ([x0,y0,z0,x1,y1,z1,..])
 * @param rotations array of the new rotations of the objects as quaternions
                    ([i0,j0,k0,w0,i1,j1,k1,w1..])
 * @param scales array of the new scales of the objects
                 ([x0,y0,z0,x1,y1,z1,..])
 */
EXPORT void
vis_CADArray_setPoses( vis_CADArray* cadArray, size_t numObjects,
    double* positions, double* rotations, double* scales );

/*! Sets an offset pose that is applied to the individual positions
 * @param position the position offset ([x,y,z])
 * @param orientation the orientation offset ([i,j,k,w])
 */
EXPORT void
vis_CADArray_setPoseOffset( vis_CADArray* cadArray, const double position[ 3 ],
    const double orientation[ 4 ] );

/*! Set the masks of the CADFile
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_CADArray_setMasks( vis_CADArray* obj, int intersection,
        int visibility, int shadow );

/*! overwrites the color of the object with the specified one
 * don't call if you want to keep the original CAD file colors
 * @param color color to overwrite with ([r,g,b])
 */
EXPORT void
vis_CADArray_overwriteColor( vis_CADArray* obj, const double color[ 3 ] );

/*! overwrites the color of the object with the specified one
 * don't call if you want to keep the original CAD file properties
 * @param metal if true, the object will look metallic (bool)
 * @param roughness surface roughness ([0,1], 0:smooth, 1:rough)
 * @param emission if true, the object seems to emit light
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_CADArray_setSurfaceProperties( vis_CADArray* obj, int metal,
    double roughness, int emission, double emissionBrightness );

/*! set the brightness of the emission
 * if @ref vis_CADFile_setSurfaceProperties hasn't been called it controls the
 * emission that is set within the cad file
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_CADArray_setEmissionBrightness( vis_CADArray* obj, double emissionBrightness );

/*! overwrites the color of the object with the specified one
 * keeps the original colors if it wasn't overwritten
 * @param transparency the transparency of the object
 *     ([0,1], 0:invisible, 1:opaque)
 */
EXPORT void
vis_CADArray_setTransparency( vis_CADArray* obj, double transparency );

/*! Set the wireframe mode for the object
 * @param enabled if true, the object is shown in wireframe mode
 * @param color the color of the wireframe lines ([r,g,b,a])
 */
EXPORT void
vis_CADArray_setWireframe( vis_CADArray* obj, int enabled,
    const double color[ 4 ] );

/*! creates a Skeleton CAD object, which can be deformed with bones
 * @param filename full path to the object to render
 * @param twosided overwrite the object properties and make all surface double
 *     sided, even if they aren't in the file description.
 * @param vertexColor use vertex colors ([r,g,b,a])
 */
EXPORT vis_riggedCADFile*
vis_riggedCADFile_create( vis_connection* con, const char* name,
    const char* filename, int twoSided, int vertexColor );

/*! Destroys the rigged cad file object
 */
EXPORT void
vis_riggedCADFile_destroy( vis_riggedCADFile* riggedCadFileObject );

/* Set position and orientation of riggedCADFile
 * @param position set the objects position ([x,y,z])
 * @param orientation orientation of object ([i,j,k,w])
 * @param scale the scale of the object ([x,y,z])
 */
EXPORT void
vis_riggedCADFile_setPose( vis_riggedCADFile* obj, const double position[ 3 ],
    const double orientation[ 4 ], const double scale[ 3 ] );

/*! overwrites the color of the object with the specified one
 * @param color color to overwrite with ([r,g,b])
 */
EXPORT void
vis_riggedCADFile_overwriteColor( vis_riggedCADFile* obj, const double color[ 3 ] );

/*! overwrites the color of the object with the specified one
 * @param metal whether the object should be metallic or not (bool)
 * @param roughness surface roughness ([0,1], 0:smooth, 1:rough)
 * @param emission if true, the object seems to emit light
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_riggedCADFile_setSurfaceProperties( vis_riggedCADFile* obj, int metal,
    double roughness, int emission, double emissionBrightness );

/*! set the brightness of the emission
 * if @ref vis_CADFile_setSurfaceProperties hasn't been called it controls the
 * emission that is set within the cad file
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_riggedCADFile_setEmissionBrightness( vis_riggedCADFile* obj, double emissionBrightness );

/*! overwrites the color of the object with the specified one
 * @param transparency the transparency of the object
 *     ([0,1], 0:invisible, 1:opaque)
 */
EXPORT void
vis_riggedCADFile_setTransparency( vis_riggedCADFile* obj, double transparency );

/*! Set the wireframe mode for the object
 * @param enabled if true, the object is shown in wireframe mode
 * @param color the color of the wireframe lines ([r,g,b,a])
 */
EXPORT void
vis_riggedCADFile_setWireframe( vis_riggedCADFile* obj, int enabled, const double color[ 4 ] );

/*! sets the masks of the riggedCADFile
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_riggedCADFile_setMasks( vis_riggedCADFile* obj, int intersection,
    int visibility, int shadow );

/*! sets the poses of the bones, ignored if animation is used
 * @param poses vector of matrizes for the bones[numPoses*16]
 * @param numPoses number of poses given
 */
EXPORT void
vis_riggedCADFile_setBonePoses( vis_riggedCADFile* obj, const double* const poses, size_t numPoses );

/*! sets the animation of the rigged CAD file
 * @param frame the animation frame to be used, intermediate values are interpolated
 * @param aniamtionId the id of the animation to be displayed
*/
EXPORT void
vis_riggedCADFile_setAnimation( vis_riggedCADFile* obj, double frame, int animationId );

/*! creates a flexible surface
 * rectangle mesh with user defined vertex positions
 * @parma intersectionMask the mask used for intersection test selection
 * @param resolution number of vertices ([x,y])
 * @param tileSize the size of the tiles the surface consists out of
 */
EXPORT vis_flexibleSurface*
vis_flexiblesurface_create( vis_connection* con,
 const char* name, const int resolution[ 2 ], int tileSize );

/*! Destroys the rigged flexible surface object
 */
EXPORT void
vis_flexiblesurface_destroy( vis_flexibleSurface* flexibleSurfaceObject );

/*! Set vertices of a flexible surface
 *! Set position of vertices in a given sub-area
 * @param vertices vertex position data ([x0,y0,z0,x1,y1,z1,..])
 */
EXPORT void
vis_flexiblesurface_setVertices( vis_flexibleSurface* obj,
    const double* vertices );

/*! Set pose of a flexible surface
 * @param position objects position ([x,y,z])
 * @param orientation objects orientation ([i,j,k,w])
 * @param scale object scale along axis ([x,y,z])
 */
EXPORT void
vis_flexiblesurface_setPose( vis_flexibleSurface* obj,
    const double position[ 3 ], const double orientation[ 4 ],
    const double scale[ 3 ] );

/*! Set material options for flexible surface
 * @param newColor color of the object ([r,g,b,a])
 * @param metal if true, the surface will look metalic (bool)
 * @param roughness object roughness ([0,1], 0:smooth, 1:rough)
 * @param emission if true, the object seems to emit light
 * @param emissionBrightness the brightness of the emission
 * @param twoSided if true the object will be visible from both sides (bool)
 * @param wireframe if true, the objects will be shown in wireframe mode (bool)
 * @param wireframeColor color used in wireframe mode
 */
EXPORT void
vis_flexiblesurface_setMaterial( vis_flexibleSurface* obj,
    const double newColor[ 4 ], int metal, double roughness, int emission,
    double emissionBrightness, int twoSided, int wireframe,
    const double wireframeColor[ 4 ] );

/*! Set colors of a flexible surface
* @see vis_flexiblesurface_setVertices
* @param colors color data ([r0,g0,b0,a0,r1,g1,b1,a1,..])
*/
EXPORT void
vis_flexiblesurface_setColors( vis_flexibleSurface* obj,
    const double* colors );

/*! Set the masks of the flexible surface
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_flexiblesurface_setMasks( vis_flexibleSurface* obj, int intersection,
    int visibility, int shadow );

#ifdef USE_VOID_INTERFACE
typedef int SolverMethod_t;
#else
typedef enum SolverMethod_t
{
    SolverMethod_DGELS = 1,
    SolverMethod_DGELSY,
    SolverMethod_DGELSD,
    SolverMethod_DGELSS
} SolverMethod_t;
#endif

/*! creates flexible body
 * a cad object that can be deformed via points
 * @param filename path to the cad file
 * @param controlPoints initial positions of the control points
 *      relative to the origin of the cad file (x0,y0,z0,x1,y1,z1,..)
 * @param numControlPoints number of control points
 * @param showControlPoint visualize the control points as speheres
 * @param controlPointsSize size of the control points if visualized
 * @param twosided overwrite the object properties and make all surface double
 *      sided, even if they aren't in the file description.
 * @param vertexColor if true, the vertex colors specified
 *      in the cad files will be used (bool)
 * @param wireframe if true, the objects will be shown in wireframe mode (bool)
 * @param wireframeColor color to use in wireframe mode ([r,g,b,a])
 * @param solverMethod which solver should be used for deforming
 */
EXPORT vis_flexibleBody*
vis_flexibleBody_create( vis_connection* con, const char* name,
    const char* filename, const double* controlPoints, size_t numControlPoints,
    int showControlPoints, double controlPointsSize, int twoSided,
    int vertexColor, SolverMethod_t solverMethod );

/*! Destroys the flexible body object
 */
EXPORT void
vis_flexibleBody_destroy( vis_flexibleBody* flexibleBodyObject );

/*! Set position and orientation of flexibleBody mesh and control points
 * @param position set the objects position ([x,y,z])
 * @param orientation orientation of object ([i,j,k,w])
 */
EXPORT void
vis_flexibleBody_setPose( vis_flexibleBody* obj, const double position[ 3 ],
    const double orientation[ 4 ] );

/*! Set offset position and orientation of the mesh
 * @param position set the objects position ([x,y,z])
 * @param orientation orientation of object ([i,j,k,w])
 * @param scale the scale of the object ([x,y,z])
 */
EXPORT void
vis_flexibleBody_setCadOffsetPose( vis_flexibleBody* obj, const double position[ 3 ],
    const double orientation[ 4 ], const double scale[ 3 ] );

/*! overwrites the color of the object with the specified one
 * don't call if you want to keep the original CAD file colors
 * @param color color to overwrite with ([r,g,b])
 */
EXPORT void
vis_flexibleBody_overwriteColor( vis_flexibleBody* obj, const double color[ 3 ] );

/*! overwrites the color of the object with the specified one
 * don't call if you want to keep the original CAD file properties
 * @param metal if true, the surface will look metalic (bool)
 * @param roughness surface roughness ([0,1], 0:smooth, 1:rough)
 * @param emission if true, the object seems to emit light
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_flexibleBody_setSurfaceProperties( vis_flexibleBody* obj, int metal,
    double roughness, int emission, double emissionBrightness );

/*! set the brightness of the emission
 * if @ref vis_CADFile_setSurfaceProperties hasn't been called it controls the
 * emission that is set within the cad file
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_flexibleBody_setEmissionBrightness( vis_flexibleBody* obj, double emissionBrightness );

/*! overwrites the color of the object with the specified one
 * keeps the original colors if it wasn't overwritten
 * @param transparency the transparency of the object
 *     ([0,1], 0:invisible, 1:opaque)
 */
EXPORT void
vis_flexibleBody_setTransparency( vis_flexibleBody* obj, double transparency );

/*! Set the wireframe mode for the object
 * @param enabled if true, the object is shown in wireframe mode
 * @param color the color of the wireframe lines ([r,g,b,a])
 */
EXPORT void
vis_flexibleBody_setWireframe( vis_flexibleBody* obj, int enabled,
    const double color[ 4 ] );

/*! Set the masks of the flexibleBody
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_flexibleBody_setMasks( vis_flexibleBody* obj, int intersection,
    int visibility, int shadow );
/*! Set the displacements relative to the initial control points position
 * @param displacements new point displacements ([x0,y0,z0,x1,y1,z1,..])
 */
EXPORT void
vis_flexibleBody_setDisplacements( vis_flexibleBody* obj,
    const double* displacements );

#ifdef USE_VOID_INTERFACE
#define Camera_t int
#else
typedef enum
{
    Camera_Standard = 1,
    Camera_Texture,
    Camera_Stream,
    Camera_VR,
    Camera_Depth
} Camera_t;
#endif

#ifdef USE_VOID_INTERFACE
#define CameraOverlayResizePolicy_t int
#else
typedef enum
{
    CameraOverlayResizePolicy_Stretch = 1,
    CameraOverlayResizePolicy_Horizontal,
    CameraOverlayResizePolicy_Vertical,
    CameraOverlayResizePolicy_Shortest,
    CameraOverlayResizePolicy_Longest,
    CameraOverlayResizePolicy_Absolute
} CameraOverlayResizePolicy_t;
#endif

#ifdef USE_VOID_INTERFACE
#define VrSDK_t int
#else
typedef enum
{
    OpenVr = 1,
    Varjo
} VrSDK_t;
#endif

/*! Create a camera object
 * combined constructor for Modelica see the individual constructors
 */
EXPORT vis_camera*
vis_camera_create( vis_connection* con, const char* name, Camera_t type,
    const double* viewPos, const double* viewSize, int order, int windowId,
    int overlaysMask, CameraOverlayResizePolicy_t overlayResizePolicy,
    const int* textureResolution, int textureId, int useHDR, int enableScreenView,
    int renderWorld, int vrShowOverlay, const double vrOverlayPlaneSize[2],
    double planeDamping, const char* streamOutput, double streamRate,
    int visibilityMask, VrSDK_t vrSDK );

/*! creates a camera where the image is displayed on the scree
 * @param viewPos relative position of the viewport in the window ([0,1])
 * @param viewSize relative size of the viewport ([0,1])
 * @param order render order of the viewport (for overlaps)
 * @param windowId ID of the window to render viewport in
 * @param overlayMask mask that specifies which overlays will be shown
 * @param overlayResizePolicy select the way overlays are resized, when the
 *     window is resized
 * @param renderWorld if true, the viewport will render the world
 *     can be set to false to only render overlays (bool)
 * @param visibilityMask mask that specifies which 3d objects are visible
 */
EXPORT vis_camera*
vis_camera_createScreen( vis_connection* con, const char* name,
    const double viewPos[ 2 ], const double viewSize[ 2 ],int order,
    int windowId, int overlaysMask,
    CameraOverlayResizePolicy_t overlayResizePolicy, int renderWorld,
    int visibilityMask );

/*! creates a vr camera
 * @param useMirrorScreen show window displaying what is currently presented
 *     in the VR headset
 * @param viewPos relative position of the mirrorScreen in the window ([0,1])
 * @param viewSize relative size of the mirrorScreen in the window  ([0,1])
 * @param order render order of the mirrorScreen (for overlaps)
 * @param windowId ID of the window to render the mirrorScreen in
 * @param useOverlayPlane if true, a plane should be created to render an overlay onto
 * @param overlayMask mask that specifies which overlays will be shown
 * @param overlayResizePolicy the default resize policy that should be applied
 * @param overlayPlaneSize the physical size of the overlay plane ([x,y])
 * @param planeDamping how much the movement of the plane should be damped (0..1)
 * @param visibilityMask mask that specifies which objects are visible
 * @param vrSDK defines which vr sdk to use
 */
EXPORT vis_camera*
vis_camera_createVR( vis_connection* con, const char* name,
    int useMirrorScreen, const double viewPos[ 2 ], const double viewSize[ 2 ],
    int order, int windowId, int useOverlayPlane, int overlaysMask,
    CameraOverlayResizePolicy_t overlayResizePolicy,
    const double overlayPlaneSize[ 2 ], double planeDamping, int visibilityMask,
    VrSDK_t vrSDK );

/*! creates a texture camera where the image is put into a texture
 * @param texBufferId the id of the created image texture
 * @param resolution resolution of the camera image
 * @param overlayMask mask that specifies which overlays will be shown
 * @param overlayResizePolicy the default resize policy that should be applied
 * @param renderWorld if true, the viewport will render the world
 *     can be set to false to only render overlays (bool)
 * @param visibilityMask mask that specifies which 3d objects are visible
 */
EXPORT vis_camera*
vis_camera_createTexture( vis_connection* con, const char* name,
    int texBufferId, const int resolution[ 2 ], int useHDR, int overlaysMask,
    CameraOverlayResizePolicy_t overlayResizePolicy, int renderWorld,
    int visibilityMask );

/*! creates a stream camera where the image is sent via network to a target
 * @param resolution resolution of the camera image
 * @param output the file/network target where the video should be streamed to
 *      eg. udp://localhost:12345
 * @param streamRate the target frame rate of the stream
 * @param overlayMask mask that specifies which overlays will be shown
 * @param overlayResizePolicy the default resize policy that should be applied
  * @param renderWorld if true, the viewport will render the world
 *     can be set to false to only render overlays (bool)
 * @param visibilityMask mask that specifies which 3d objects are visible
 */
EXPORT vis_camera*
vis_camera_createStream( vis_connection* con, const char* name,
    const int resolution[ 2 ], const char* output, double streamRate,
    int overlaysMask, CameraOverlayResizePolicy_t overlayResizePolicy,
    int renderWorld, int visibilityMask );

/*! creates a depth camera
 * that creates a texture with the distances to the seen objects
 * @param resolution resolution of the depth image
 * @param depthTexureId id of the created depth image texture
 * @param enableDepthView if true, a screenn with the depth will be shown
 * @param depthViewSize relative size of the depth view ([0,1])
 * @param depthViewpPosition relative position of the depth view ([0,1])
 * @param windowId id of the window the depth view is shown in
 * @param order the order of the depth view
 * @param visibilityMask mask that specifies which objects are visible
 */
EXPORT vis_camera*
vis_camera_createDepth( vis_connection*con, const char* name,
    const int resolution[ 2 ], int textureId, int enableDepthView,
    const double depthViewSize[ 2 ], const double depthViewPosition[ 2 ],
    int windowId, int order, int visibilityMask );

/*! Destroys the camera object
 */
EXPORT void
vis_camera_destroy( vis_camera* cameraObject );

/*! Set whether the camera should be rendered or not
 * @param active if true, the camera is active (bool)
 */
EXPORT void
vis_camera_setActive( vis_camera* con, int active );

/*! Set camera control type
 * needs to be called in the same timestep as the camera creation
 * @param orbit if true the camera will look at and rotate around a target,
 *      if false it will rotate around itself (bool)
 * @param canControlGaze if true, the user can control the gaze (bool)
 * @param canControlMove if true, the user can control the position
 *      the camera (bool)
 */
EXPORT void
vis_camera_setControlType( vis_camera* camera, int orbit,
    int canControlGaze, int canControlMove, int lockZAxis );

/*! Adds a video stream to the camera
 * needs to be called in the same timestep as the camera creation
 * ignored for pure stream cameras
 * @param resolution resolution of the exported video stream
 * @param output the file/network target where the video should be streamed to
 *      eg. udp://localhost:12345
 * @param streamRate the target frame rate of the stream
 */
EXPORT void
vis_camera_addVideoStream( vis_camera* camera, const int resolution[ 2 ],
    const char* output, double streamRate );

/*! Adds rendering of the camera image additionally to a texture
 * needs to be called in the same timestep as the camera creation
 * ignored for texture cameras
 * @param resolution resolution of the exported video
 * @param textureId id of the created texture
 * @param useHDR If true, 16 bit is used per channel for the texture
 */
EXPORT void
vis_camera_addRenderToTexture( vis_camera* camera, const int resolution[ 2 ],
    int textureId, int useHDR );

/*! Returns the current postion and orientation of the camera
 * @param camera the camera to get the position from
 * @param position the current position is written into this vector ([x,y,z])
 * @param viewingDirection the current viewing direction is written
 *      into this vector([x,y,z])
 * @param upVector the current upVector is written into this vector ([x,y,z])
 * @return error 0 on success, 1 on error
 */
EXPORT int
vis_camera_getPosition( vis_camera* camera, double position[ 3 ],
    double viewingDirection[ 3 ], double upVector[ 3 ] );

/*! Triggers a screenshot of the current view
 * and saves it to the specified path
 * the camera needs to be render its image to texture to work
 * the screenshot is taken only during simulation
 * @param outputPath the path where the image should be stored
 * @param replaceExisting if true, existing files will be replaced, otherwise
 *      the filename will be changed by appending increasing numbers
 */
EXPORT void
vis_camera_triggerScreenshot( vis_camera* camera, const char* outputPath,
    int replaceExisting );

#ifdef USE_VOID_INTERFACE
#define VarjoPrimitiveMaskType_t int
#else
typedef enum
{
    VarjoPrimitiveMask_Plane = 1,
    VarjoPrimitiveMask_Cube,
    VarjoPrimitiveMask_Sphere
} VarjoPrimitiveMaskType_t;
#endif

#ifdef USE_VOID_INTERFACE
#define VarjoMaskReferencType_t int
#else
typedef enum
{
    VarjoMaskReference_WorldOrigin = 1,
    VarjoMaskReference_Marker,
    VarjoMaskReference_OpenVrTracker
} VarjoMaskReferencType_t;
#endif

#ifdef USE_VOID_INTERFACE
#define VarjoMarkerOrientation_t int
#else
typedef enum
{
    VarjoMarkerOrientation_Complete = 1,
    VarjoMarkerOrientation_YawOnly,
    VarjoMarkerOrientation_Ignore
} VarjoMarkerOrientation_t;
#endif


#ifdef USE_VOID_INTERFACE
#define VarjoMaskingMode_t int
#else
typedef enum
{
    VarjoMaskingMode_Off = 1,
    VarjoMaskingMode_ShowCameraOnMask,
    VarjoMaskingMode_ShowVrOnMask
} VarjoMaskingMode_t;
#endif

#ifdef USE_VOID_INTERFACE
#define VarjoHmdReferenceType_t int
#else
typedef enum
{
    VarjoHmdReferenceType_Origin = 1,
    VarjoHmdReferenceType_Marker,
    VarjoHmdReferenceType_ARTOrigin,
    VarjoHmdReferenceType_OpenVrDevice
} VarjoHmdReferenceType_t;
#endif

#ifdef USE_VOID_INTERFACE
#define VarjoOpenVrDeviceType_t int
#else
typedef enum
{
    VarjoOpenVrDeviceType_t_BaseStation = 1,
    VarjoOpenVrDeviceType_t_Controller,
    VarjoOpenVrDeviceType_t_Generic
} VarjoOpenVrDeviceType_t;
#endif

/* Sets the parameters for the Varjo SDK
 * Needs to be called in the same frame of the creation
 * @param chromaKeying if true, MR with chromakeying is used
 * @param alphaBlend if true, the vr content will be blended with the camera
 *               image based the on the alpha value
 * @param depthTest if true, the real world distance is compared against 
 *               the distance to virtual objects is tested
 * @param depthRange specifies the range in which the distance is tested
 * @param viewOffset offset to the eye position. Set to 0 for vr dominant apps and to 1 for mixed reality apps
 * @param maskingMode the kind of masking is used
 * @param hmdReferencType defines the type of hmd reference point used
 * @param referenceId either the id of the marker that is used as reference, or the ART id of the headset
 * @param dtrackPort the port of the ART data if ART origin is used
 * @param referenceMarkerId the id of the marker that will be used as origin reference
 * @param markerReferenceOrientation defines what orientations will be taken into account of the origin reference marker
 */

EXPORT void
vis_camera_setVarjoParameters( vis_camera* camera, int chromaKeying,
    int alphaBlend, int depthTest, const double depthRange[2],
    double viewOffset, VarjoMaskingMode_t maskingMode,
    VarjoHmdReferenceType_t hmdReferencType, int referenceId, int dtrackPort,
    VarjoOpenVrDeviceType_t openVrDeviceType, VarjoMarkerOrientation_t referenceOrientation );

/* Adds a new primitve mask to the Varjo environment
 * @param maskType the type of the primitve mask
 * @param referenceType the type of the mask reference
 * @param referenceId either the id of varjo marker or open vr tracker
 * @param orientation defines what orientations will be taken into account of the origin reference marker
 */
EXPORT void
vis_camera_addPrimitiveVarjoMask( vis_camera* camera,
    VarjoPrimitiveMaskType_t maskType, VarjoMaskReferencType_t referenceType,
    int referenceId, VarjoOpenVrDeviceType_t openVrDeviceType, VarjoMarkerOrientation_t orientation );

/* Adds a new primitve mask to the Varjo environment
 * @param filename the path to the filename with the mask
 * @param referenceType the type of the mask reference
 * @param referenceId either the id of varjo marker or open vr tracker
 * @param orientation defines what orientations will be taken into account of the origin reference marker
 */
EXPORT void
vis_camera_addCadVarjoMask( vis_camera* camera, const char* filename,
    VarjoMaskReferencType_t referenceType, int referenceId, VarjoOpenVrDeviceType_t openVrDeviceType,
    VarjoMarkerOrientation_t orientation );

/* Adds a new primitve mask to the Varjo environment
 * @param resolution the resolution of the mask mesh (x,y)
 * @param vertices the positions of the vertices of the mesh {{x1,y1,z1},..}
 * @param referenceType the type of the mask reference
 * @param referenceId either the id of varjo marker or open vr tracker
 * @param orientation defines what orientations will be taken into account of the origin reference marker
 */
EXPORT void
vis_camera_addCustomMeshVarjoMask( vis_camera* camera, const int resolution[ 2 ],
    const double* vertices, VarjoMaskReferencType_t referenceType,
    int referenceId, VarjoOpenVrDeviceType_t openVrDeviceType, VarjoMarkerOrientation_t orientation );

/* Updates the pose of a mask
 * @param index the index of the mask
 * @param position set the mask position ([x,y,z])
 * @param orientation orientation of the mask ([i,j,k,w])
 * @param scale the scale of the mask ([x,y,z])
 */
EXPORT void
vis_camera_setVarjoMaskPose( vis_camera* camera, int index, const double position[ 3 ],
    const double orientation[ 4 ], const double scale[ 3 ] );

/* Sets the real world camera parameters
 * The parameters need to be supported by the headset
 * @param exposure the exposure in 1/x of the camera
 * @param iso the iso of the camera
 * @param whitebalance the whitebalance of the camera
 */
EXPORT void
vis_camera_setVarjoCameraParameters( vis_camera* camera, double exposure,
    int iso, int whitebalance );

#ifdef USE_VOID_INTERFACE
#define DepthStream_t int
#else
typedef enum
{
    // streams the depth information via an encoded video stream via UDP
    DepthStream_Video = 1,
    // streams the depth information in a flatbuffers struct via TCP
    DepthStream_Raw,
    // streams the resulting points  in a flatbuffers struct via TCP
    DepthStream_Points
} DepthStream_t;
#endif

/*! creates a depth data stream which streams the depth information
 * needs to be called in the same timestep as the camera creation
 * only valid for the depth camera
 * @param streamType how the data should be streamed
 * @param adress the network adress of the stream target
 * @param port the port of the stream target
 * @param dataId id the data should have to identify the data,
 *      only used for DepthStream_Points
 * @param triggeredStream If true, the data is only streamed if triggered
 * @param streamRate the target frame rate of the point data stream
 */
EXPORT void
vis_camera_addDepthStream( vis_camera* camera, DepthStream_t streamType,
    const char* adress, int port, int dataId, int triggeredStream,
    double streamRate );

/* Triggers that depth data is sent in the next frame
 */
EXPORT void
vis_camera_triggerDepthStream( vis_camera* camera );

/*! Set position for camera
 * only has an effect based on the settings set in vis_camera_setControlType
 * canControlMove needs to be false
 * if orbit is set to true gaze needs to be false, this will move the camera
 * while still looking at the target
 * @param position new position of the camera ([x,y,z])
 */
EXPORT void
vis_camera_setPosition( vis_camera* obj, const double position[ 3 ] );

/*! Set the pose of the target the camera looks at and moves around
 * only has an effect based on the settings set in vis_camera_setControlType
 * orbit needs to be true and canControlMove to be false
 * the camera will move and rotate together with the target
 * @param targetPos new position of the target ([x,y,z])
 * @param targetOri new orientation of the target as quaternion ([i,j,k,w])
 */
EXPORT void
vis_camera_setTargetPose( vis_camera* obj, const double targetPos[ 3 ],
    const double targetOri[ 4 ] );

/*! Set camera the camera viewing direction and up vector
 * only has an effect based on the settings set in vis_camera_setControlType
 * canControlGaze needs to be false
 * if orbit is set to true it will also move the camera and the length
 * of the direction will define the distance to the target
 * @param direction ([x,y,z])
 * @param upVector the camera up-vector ([x,y,z])
 */
EXPORT void
vis_camera_setDirection( vis_camera* obj, const double direction[ 3 ],
    const double upvector[ 3 ] );

#ifdef USE_VOID_INTERFACE
#define ProjectionType_t int
#else
typedef enum
{
    ProjectionType_Perspective=1,
    ProjectionType_Orthographic,
    ProjectionType_Fisheye,
} ProjectionType_t;
#endif
/*! Set the cameras frustum parameters
 * needs to be called in the same timestep as the camera creation
 * @param projectionType projection type of the camera
 * @param fov the vertical field of view in degrees for perspective camera
 * @param height the height of the viewport for the orthographic camera
 * @param zNear near plane distance (minimum render dinstance)
 * @param zFar far plane distance (maximum render distance)
 */
EXPORT void
vis_camera_setFrustrum( vis_camera* obj, ProjectionType_t projectionType,
    double fov, double height, double zNear, double zFar );

/*! Set camera background
 * needs to be called in the same timestep as the camera creation
 * @param setBackground if true overwrites the cameras background
 *      with specified color (environments included) (bool)
 * @param backgroundColor color of the cameras background ([r,g,b,a])
 */
EXPORT void
vis_camera_setBackground( vis_camera* obj, int setBackground,
    const double backgroundColor[ 4 ] );

/*! Set the initial move speed of the free ego camera
 * needs to be called in the same timestep as the camera creation
 * @param moveSpeed new move speed of the camera
 */
EXPORT void
vis_camera_setEgoMoveSpeed( vis_camera* obj, double moveSpeed );

/*! Set the eye dome lighting shading Strength
 * only valid for the depth cameras and if the depth image displayed
 * or a color texture is created
 * @param edlStrength the edl Strength, 0 means no shading
 */
EXPORT void
vis_camera_setEDLStrength( vis_camera* obj, double edlStrenght );

/*! Adds the colored depth image to the texture buffer,
 * so that it can be used by other objects
 * @param textureId id of the created color depth texture
*/
EXPORT void
vis_camera_addDepthColorTexture( vis_camera* camera, int textureId );

/*! Set the exposure data of the camera
 * @param adaptiveExposure if true, the camera's exposure will adapt
 * @param exposure the exposure(static), or exposure multiplier(adaptive)
 * @param adaptionTime the time the camera takes to adapt to new lighting 
 */
EXPORT void
vis_camera_setExposure( vis_camera* obj, int adaptiveExposure, double exposure,
    double adaptionTime );

/*! Creates a sync object
 * synchronizes the simulation with the visualization
 */
EXPORT vis_sync*
vis_sync_create( vis_connection* con, const char* name );

/*! Destroys the sync object
 */
EXPORT void
vis_sync_destroy( vis_sync* syncObject );

/*! Set synchronization time */
EXPORT void
vis_sync_setTime( vis_sync* obj, double time );

/*! Gets the synchronization time
 * To synchronize, the simulation needs to wait until the time of the last
 * times step is returned
 * @param time returns last reported time
 * @return 0 on success 1 on error (bool)
 */
EXPORT int
vis_sync_getTime( vis_sync* obj, double* time );

#ifdef USE_VOID_INTERFACE
#define WindowStyle_t int
#else
typedef enum
{
    WindowStyle_Window = 1,
    WindowStyle_Borderless,
    WindowStyle_Fullscreen
} WindowStyle_t;
#endif

/*! creates an empty windowSettings object
*/
EXPORT vis_windowSettings*
vis_windowSettings_create( vis_connection* con );

/*! Destroys the window settings object
 */
EXPORT void
vis_windowSettings_destroy( vis_windowSettings* windowSettingsObject );

/*! Create a render window
 * adds the specified WindowStyle_Window to the WindowStyle_Window
 * settings object
 * @param id window id of the new window, 0 is reserved for main window
 * @param title window title
 * @param style style of the window(normal, borderless,fullscree)
 * @param setPosition if true the position will be set (bool)
 * @param position relative position of the window to the screen
 * @param screenId id of the screen the window should be on
 * @param setSize if true the size will be set (bool)
 * @param size relative size of the window to the screen
 * @param alwaysOnTop If true, the window will move to be on top, 
        if there is no user activity for more than 5s
 */
EXPORT void
vis_windowSettings_addWindow( vis_windowSettings* settings, int id,
    const char* title, WindowStyle_t style, int setPosition,
    const double position[ 2 ], int screenId, int setSize,
    const double size[ 2 ], int alwaysOnTop );

#ifdef USE_VOID_INTERFACE
#define Environment_t int
#else
typedef enum
{
    Environment_simple = 1,
    Environment_atmospheric,
    Environment_skybox
} Environment_t;
#endif
/*! Metafunction to create an environment
 * needed for Modelica, takes parameters of
 * all environment create functions and creates the
 * one specified by type
 */
EXPORT vis_environment*
vis_environment_create( vis_connection* con, const char* name,
    Environment_t type, const double* ambientColor, double ambientMultiplier,
    const char* skyboxPath );

/*! Create a simple environment
 * @param ambientColor ambient light color in the shadow of objects ([r,g,b])
 */
EXPORT vis_environment*
vis_environment_createSimple( vis_connection* con, const char* name,
    const double ambientColor[ 3 ] );

/*! Create an atmospheric environment
 * @param ambientMultiplier the ambient light is scaled by this value
 */
EXPORT vis_environment*
vis_environment_createAtmospheric( vis_connection* con, const char* name,
    double ambientMultiplier );

/*! Create a skybox environment
 * @param skyBoxPath full path to the skybox file, needs to be a dds cube texture
 * @param ambientMultiplier the ambient light is scaled by this value
 */
EXPORT vis_environment*
vis_environment_createSkybox( vis_connection* con, const char* name,
    const char* skyBoxPath, double ambientMultiplier );

/*! Destroys the environment object
 */
EXPORT void
vis_environment_destroy( vis_environment* environmentObject );

/*! Set sun properties
 * @param enable if true, the sun will be active (bool)
 * @param lightDirection the direction of the sun light ([x,y,z])
 * @param color color of the sun ([r,g,b])
 * @param brightness the brightness of the sun
 * @param lightMask mask that specifies which objects will cast shadows
 */
EXPORT void
vis_environment_setSun( vis_environment* obj, int enabled,
    const double lightDirection[ 3 ], const double color[ 3 ], double brightness,
    int lightMask );

/*! Set sky box rotation
 * only valid for sky box environment
 * @param environment the sky box envrionment to set the rotation of
 * @param angle the euler angles of the rotated sky ([x,y,z])
 */
EXPORT void
vis_environment_setSkyRotation( vis_environment* environment, double rotation[ 3 ] );

/*! Set the haze options
 * @param enableHaze if true haze is enabled
 * @param maxHazeDistance the distance at which objects dissapear into haze
 * @param hazeColor the color the haze should have ([r,g,b])
 */
EXPORT void
vis_environment_setHaze( vis_environment* obj, int enableHaze, double maxHazeDistance, double color[3] );

/*! Enable / Disable shadows
 * @param enabled set shadows on/off(bool)
 */
EXPORT void
vis_environment_setShadowsEnabled( vis_environment* obj, int enabled );

/*! Adds clouds to an atmospheric environment
 * The clouds are only 2D and consist out of two layers
 * No clouds are present if not called
 * @param intensity the intensity/brightness of the two layers
 * @param coverage how much of the sky should be covered by the clouds
 * @param scattering how strong the light of the clouds should be scattered
 *          influences the visibility in the distance
 */
EXPORT void
vis_environment_setAtmosphericClouds( vis_environment* obj,
    double intensity[ 2 ], double coverage[ 2 ], double scattering );

#ifdef USE_VOID_INTERFACE
#define ShadowResolution_t int
#else
typedef enum
{
    ShadowResolution_64x64 = 1,
    ShadowResolution_128x128,
    ShadowResolution_256x256,
    ShadowResolution_512x512,
    ShadowResolution_1024x1024,
    ShadowResolution_2048x2048,
    ShadowResolution_4096x4096,
    ShadowResolution_8192x8192
} ShadowResolution_t;
#endif
/*! Set shadow parameters
 * @param resolution the resolution of the shadow texture
 * @param bias bias used for shadow mapping (default:1)
 * @param normalBias normal bias used for shadow mapping (default:1)
 * @param softness softness of the shadow 
 * @param numCascades number if shadow cascades
 * @param cascadeBorders the relative border distances between
 *      the shadow cascades, size is numCascades-1,
 *      the default is {0.015,0.05,0.2}
 */
EXPORT void
vis_environment_setShadows( vis_environment* obj,
    ShadowResolution_t resolution, double maxShadowDistance, double bias,
    double normalBias,double softness, int numCascades,
    const double* cascadeBorders );
#ifdef USE_VOID_INTERFACE
#define OverlayType_t int
#else
typedef enum
{
    OverlayType_Label = 1,
    OverlayType_Polygon,
    OverlayType_Line,
    OverlayType_Image,
    OverlayType_Button,
    OverlayType_TextureTarget

} OverlayType_t;
#endif

#ifdef USE_VOID_INTERFACE
#define OverlayResizePolicy_t int
#else
typedef enum
{
    OverlayResizePolicy_Stretch = 1,
    OverlayResizePolicy_Horizontal,
    OverlayResizePolicy_Vertical,
    OverlayResizePolicy_Shortest,
    OverlayResizePolicy_Longest,
    OverlayResizePolicy_Absolute,
    OverlayResizePolicy_Camera
} OverlayResizePolicy_t;
#endif

/*! Create an overlay
 * @param type the type of the overlay created
 * @param resizePolicy defines how the overlay item reacts to screen size changes
 * @param anchor relative position to the overlay it is anchored 
 *      e.g. [0,0] bottom left, [1,1] top right, [0.5,0] bottom center
 * @param order render order (for overlaps)
 */
EXPORT vis_overlay*
vis_overlay_create( vis_connection* con, const char* name, OverlayType_t type,
    OverlayResizePolicy_t resizePolicy, const double anchor[ 2 ], int order );

/*! Destroys the overlay object
 */
EXPORT void
vis_overlay_destroy( vis_overlay* overlayObject );

/*! Set the mask to set the visibility in cameras
 * @param mask the visibility mask
 */

EXPORT void
vis_overlay_setVisibility( vis_overlay* obj, int mask );

/*! Set the position of an overlay item
 * @param origin the relativ screen origin used
 *       ([0,0] bottom left corner, [0.5,0.5] center)
 * @param position position of the overlay relative to the origin in pixel
 *      equivalent units, the position is influenced by resizePolicy
 */
EXPORT void
vis_overlay_setPosition( vis_overlay* obj, const double origin[ 2 ],
                         const double position[ 2 ] );

/*! Set scaling factor
 * @param scaleFactor factor the overlay item is scaled with
 */
EXPORT void
vis_overlay_setScaleFactor( vis_overlay* obj, const double scaleFactor[ 2 ] );

/*! Set the color of an overlay item
 * @param color new color of the overlay item ([r,g,b,a])
 */
EXPORT void
vis_overlay_setColor( vis_overlay* obj, const double color[ 4 ] );

/*! Set the angle of an overlay item
 * @param angle new rotation angle of the overlay item (in degrees)
 */
EXPORT void
vis_overlay_setAngle( vis_overlay* obj, double angle );

/*! adds a tooltip to the overlay item that apears if the mouse hovers over it
 * @param text the text to be displayed
 * @param showDelay the time time in seconds the mouse needs to be in the
 *                  overlay to show the tooltip
 * @param textSize the size of the text in pixels
 * @param backgroundColor the color of the tooltip background
 * @param textColor the color the tooltip text
 */
EXPORT void
vis_overlay_addTooltip( vis_overlay* obj, const char* text, double showDelay,
    int textSize, const double backgroundColor[ 4 ],
    const double textColor[ 4 ] );

/*! Set the text of a label
 * only has effect if item is a label overlay
 * @param text the lable text
 */
EXPORT void
vis_overlay_label_setText( vis_overlay* obj, const char* text );

/*! Set the font size of a label
 * only has effect if item is a label overlay
 * @param size font size
 */
EXPORT void
vis_overlay_label_setFontSize( vis_overlay* obj, int size );

/*! Set the font of the text
 * only has effect if item is a label overlay
 * @param fontPath the name or path of the font
 * @param richText if true, the text is rich text formatted
 *      and bold and italic is ignored (bool)
 * @param bold if true, the text is displayed bold (bool)
 * @param italic if true, the text is displayed italic/cursive (bool)
 * @param outline if true, the text gets a black outline (bool)
 */
EXPORT void
vis_overlay_label_setFont( vis_overlay* obj, const char* fontPath,
    int richText, int bold, int italic, int outline );

/*! Set the filename of an image
 * only has effect if item is an image overlay
 * @param fileName file path to the the image
 */
EXPORT void
vis_overlay_image_setFileName( vis_overlay* obj, const char* fileName );

/*! Overwrite the size of the image
 * only has effect if item is an image overlay
 * @param setWidth if true, the image width is overwritten (bool)
 * @param width new width of the image, ignored if setWidth is false
 * @param setHeight if true, the image width is overwritten (bool)
 * @param height new height of the image, ignored if setHeight is false
 */
EXPORT void
vis_overlay_image_overwriteSize( vis_overlay* obj, int setWidth, double width,
    int setHeight, double height );

/* Initializes a polygon overlay item
 * @param overrideBackground If true, no alpha blending takes place and overlay items behind will be hidden
 */
EXPORT void
vis_overlay_polygon_init( vis_overlay* obj, int overrideBackground );

/*! Set the points of a polygon
 * only has effect if item is an polygon overlay
 * @param points polygon points (each [x,y] for a total of:
 *     numPoints*2*sizeof(double))
 * @param numPoints number of polygon points
 */
EXPORT void
vis_overlay_polygon_setPoints( vis_overlay* obj, const double* points,
    size_t numPoints );

/*! Set the texture coordinates and texture path of a polygon
 * only has effect if item is an polygon overlay
 * @param texCoords texture coordinates (each [x,y] for a total of
 *     numCoords*2*sizeof(double), must be same as polygon points)
 * @param numCoords number of texture coordinate points
 * @param texturePath path to the image file
 */
EXPORT void
vis_overlay_polygon_setTexture( vis_overlay* obj, const double* texCoords,
    size_t numCoords, const char* texturePath );

/*! Set the points of a line
 * only has effect if item is an line overlay
 * @param points line points (each [x,y] for a total of
 *     numPoints*2*siszeof(double))
 * @param numPoints number of line points
 * @param lineWidth width of the line
 * @param segmented if true the line is segmented into lines every two
 *      points form a segment (bool)
 */
EXPORT void
vis_overlay_line_setPoints( vis_overlay* obj, const double* points,
    size_t numPoints, double lineWidth, int segmented );

/*! Set points that define the button polygon
 * only has effect if item is an button overlay
 * @param points (each [x,y] for a total of numPoints*2*siszeof(double))
 * @param numPoints number of points
 */
EXPORT void
vis_overlay_button_setPoints( vis_overlay* obj, const double* points,
    size_t numPoints );

/*! Get events from an obverlay button
 * only has effect if item is an button overlay
 * When the user interacts with a button, the events created by this interaction
 * can be accessed with this function
 * @param events the result will be written into this data structure
 * @return 0 on success, 1 on error
 */
EXPORT int
vis_overlay_button_getEvents( vis_overlay* obj,
    vis_overlay_buttonEvents* events );

/*! Set texture target textureId
 * only has effect if item is an textureTarget overlay
 * has to be called in the same timestep as vis_overlay_create
 * @param textureId the id of the texture to be displayed
 */
EXPORT void
vis_overlay_textureTarget_setTextureId( vis_overlay* obj, int textureId );

/*! Set the size parameters for the texture target
 * only has effect if item is an textureTarget overlay
 * @param setWidth if true, the texture width is overwritten (bool)
 * @param width new width of the texture, ignored if setWidth is false
 * @param setHeight if true, the texture width is overwritten (bool)
 * @param height new height of the texture, ignored if setHeight is false
 */
EXPORT void
vis_overlay_textureTarget_overwriteSize( vis_overlay* obj, int setWidth,
    double width, int setHeight, double height );

#ifdef USE_VOID_INTERFACE
#define BillboardAnimation_t int
#else
typedef enum
{
    BillboardAnimation_None = 1, // no automatic animation
    BillboardAnimation_Simulation, // animation is dependant on simulation 
    BillboardAnimation_Independant // animation is independant on simulation 
} BillboardAnimation_t;
#endif

#ifdef USE_VOID_INTERFACE
#define BillboardAlignment_t int
#else
typedef enum
{
    BillboardAlignment_Screen = 1,
    BillboardAlignment_X_Axis = 2,
    BillboardAlignment_Z_Axis = 3,
    BillboardAlignment_XZ_Axes = 4,
} BillboardAlignment_t;
#endif

/*! creates a that automatically rotates towards the camera
 * @param imageFile the path to the image that is shown on the billboard
 * @param alignment defines how the billboard will be oriented
 * @param animation how the billboard should be animated
 * @param numImages number of subimages within the main image([width,height])
 */
EXPORT vis_billboard*
vis_billboard_create( vis_connection* con, const char* name,
    const char* imageFile, BillboardAlignment_t alignment,
    BillboardAnimation_t animation, const int numImages[2] );

/*! Destroys the billboard object
 */
EXPORT void
vis_billboard_destroy( vis_billboard* billboardObject );

/*!sets the position and orientation of the billboard
 * @param position new position of the billboard ([x,y,z])
 * @param orientation new orientation as quaternion ([i,j,k,w])
 */
EXPORT void
vis_billboard_setPose( vis_billboard* billboard, const double position[ 3 ],
    const double orientation[ 4 ] );

/*! Set the size of the billboard
 * @param width new width of the billboard
 * @parma height new height of the billboard
 */
EXPORT void
vis_billboard_setSize( vis_billboard* billboard, double width, double height );

/*! Set the animation speed of the billboard 
 * only has effect if animation type is not set to none
 * @param speed the new animation speed to use
 */
EXPORT void
vis_billboard_setAnimationSpeed( vis_billboard* billboard, double speed );

/*! Set the frame to be shown if BillboardAnimation_None is being used
 * only has effect if animation type is set to none
 * @param frameNumber the frame with this number will be shown
 */
EXPORT void
vis_billboard_setFrameNumber( vis_billboard* billboard, int frameNumber );

/*! Set the color of the billboard
 * the image color is multiplied by this color
 * @param color the new color of the billboard ([r,g,b,a])
 */
EXPORT void
vis_billboard_setColor( vis_billboard* billboard, const double color[ 4 ] );

/*! Set the visibility mask of the billboard
 * @param visibility mask that specifies which camera sees this object
 */
EXPORT void
vis_billboard_setVisibilityMask( vis_billboard* obj, int visibility );

/*! Sets an offset in world units in text space
 * @param offset the new offset
 */
EXPORT void
vis_billboard_setOffset( vis_billboard* obj, const double offset[ 3 ] );

/*! creates a 3d text, that is floating in space
 * can be oriented automatically towards the camera
 * @param alignment defines how the text will be oriented
 */
EXPORT vis_text3d*
vis_text3d_create( vis_connection* con, const char* name,
    BillboardAlignment_t alignment );

/*! Destroys the text3d object
 */
EXPORT void
vis_text3d_destroy( vis_text3d* text3dObject );

/*! Set the text that will be displayed
 * @param text new text to display
 */
EXPORT void
vis_text3d_setText( vis_text3d* text3d, const char* text );

/*! Set the position and orientation of the text object
 * @param position new position of the object ([x,y,z])
 * @param orientation new orientation as quaternion ([i,j,k,w])
 */
EXPORT void
vis_text3d_setPose( vis_text3d* text3d, const double position[ 3 ],
    const double orientation[ 4 ] );

/*! Set the text color of the text3d object
 * @param color new text color ([r,g,b,a])
 */
EXPORT void
vis_text3d_setColor( vis_text3d* text3d, const double color[ 4 ] );

/*! Set the size of the text
 * @param size new size of the text in world units
 */
EXPORT void
vis_text3d_setTextSize( vis_text3d* text3d, double size );

/*! Set which point of the text relative to the text is used for positioning
 * (0,0) is the bottom left corner, (0.5,0.5) the middle, (1,1) top right
 * @param alignment the alignment of the text
 */
EXPORT void
vis_text3d_setAlignment( vis_text3d* text3d, const double alignment[ 2 ] );

/*! Set the font of the text, either the complete path of just the font name
 * @param fontPath the name or path of the font
 * @param richText if true, the text is rich text formatted
 *      and bold and italic is ignored (bool)
 * @param bold if true, the text is displayed bold (bool)
 * @param italic if true, the text is displayed italic/cursive (bool)
 * @param outline if true, the text gets a black outline (bool)
 */
EXPORT void
vis_text3d_setFont( vis_text3d* text3d, const char* fontPath, int richText,
    int bold, int italic, int outline );

/*! Set the visibility mask of the text3d
 * @param visibility mask that specifies which camera sees this object
 */
EXPORT void
vis_text3d_setVisibilityMask( vis_text3d* obj, int visibility );

/*! Sets an offset in world units in text space
 * @param offset the new offset
 */
EXPORT void
vis_text3d_setOffset( vis_text3d* obj, const double offset[ 3 ] );

#ifdef USE_VOID_INTERFACE
#define TexturObject3dType_t int
#else
typedef enum
{
    TexturObject3dType_Plane = 1,
    TexturObject3dType_StaticMesh = 2,
    TexturObject3dType_DynamicMesh = 3
} TexturObject3dType_t;
#endif

#ifdef USE_VOID_INTERFACE
#define TexturWrapMode_t int
#else
typedef enum
{
    TexturWrapMode_Repeat = 1,
    TexturWrapMode_Transparent = 2,
    TexturWrapMode_Clamp = 3
} TexturWrapMode_t;
#endif

/*! Create a textured Surface, that can be used to project a texture onto
 * a parameterized surface
 * @param useTextureSource if true the displayed texture is given from the
 *      texture pool by the textureId, if false an image from a file is displayed
 * @param textureId the id of the texture to be displayed,
 *      if textureSource is true
 * @param filename the path to the image file to be displayed,
 *      if textureSource is false
 * @param type type of texture target that is created
 * @param wrapMode defines how the texture sampling is handled outside of (0,1)
 * @param hudMode if true, 
 */
EXPORT vis_texturedSurface*
vis_texturedSurface_create( vis_connection* con, const char* name,
    int useTextureSource, int textureId, const char* filename,
    TexturObject3dType_t type, TexturWrapMode_t wrapMode );

/*! Destroys the textured surface object
 */
EXPORT void
vis_texturedSurface_destroy( vis_texturedSurface* texturedSurfaceObject );

/*! Set the position and orientation of the textured surface
 * @param position position ([x,y,z])
 * @param orientation orientation as quaternion ([i,j,k,w])
 */
EXPORT void
vis_texturedSurface_setPose( vis_texturedSurface* obj,
    const double position[ 3 ], const double orientation[ 4 ] );

/*! Set the size of the plane if a simple plane mesh is used
 * @param size new size of the plane
 */
EXPORT void
vis_texturedSurface_setPlaneSize( vis_texturedSurface* obj,
    const double size[ 2 ] );

/*! Set the size of the plane in meters per pixel
 * i.e. how large one pixel should be for the plane mesh
 * @param meterPerPixel the size of each pixel in meters
 */
EXPORT void
vis_texturedSurface_setPlaneMeterPerPixel( vis_texturedSurface* obj,
    double meterPerPixel );

/*! Set points of the user defined mesh
 * needs to be called in the same timestep as create
 * if the surface is dynamic it can be also called afterwards
 * @param resolutionX the resolution of the mesh in x direction
 * @param resolutionY the resolution of the mesh in y direction
 * @param points the new points of the mesh (length=3*x*y)
 * @param setTexCoords if true, texture coordinates can be set
 * @param texCoords the new texture coordinates if setTexCoords (length=x*y)
 * @param setAlphaValues if true, alpha values can be set
 * @param alphaValues the new alpha values if setAlphaValues (length=x*y)
 */
EXPORT void
vis_texturedSurface_updateVertices( vis_texturedSurface* obj,
    const int resolution[ 2 ], const double* points, int setTexCoords,
    const double* texCoords, int setAlphaValues, const double* alphaValues );

/*! Set the scale of the user defined mesh
 * has no effect on plane mesh
 * @param scale scaling along axis ([x,y,z])
 */
EXPORT void
vis_texturedSurface_setMeshScale( vis_texturedSurface* obj,
    const double scale[ 3 ] );

/*! Set the material parameters of the textured surface
 * @param color the base color the image is multiplied with ([r,g,b,a])
 * @param isMetallic if true, the object is treated as metallic object (bool)
 * @param roughness the surface roughness of the object ([0,1], 0:smooth, 1:rough)
 * @param emission if true, the object seems to emit light
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_texturedSurface_setMaterial( vis_texturedSurface* obj, double color[ 4 ],
    int isMetallic, double roughness, int emission, double emissionBrightness );

/*! Set the masks of the textured surface
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_texturedSurface_setMasks( vis_texturedSurface* obj, int intersection,
    int visibility, int shadow );

/*! Set the textured surface as billboard
 * @param alignment definees how the surface should be rotated to the screen
 * @param offset the offset relative to the rotated surface
 */
EXPORT void
vis_texturedSurface_setBillboard( vis_texturedSurface* obj,
    BillboardAlignment_t alignment, const double offset[3] );

/* Set the data for the hud mode
 * the texture surface acts like a real life head up display
 * the texture coordinates will be ignored
 * @param hudNormal the direction in which the hud will be projected
 * @param hudUp the up direction vector of the hud
 * @param hudOffset the offset of the hud relative to its size
 * @param hudSize apparent size of the at a distance of 1 meter
 */
EXPORT void
vis_texturedSurface_setHudModeData( vis_texturedSurface* obj,
    const double hudNormal[ 3 ], const double hudUp[ 3 ],
    const double hudOffset[ 2 ], const double hudSize[ 2 ] );

/*! Set replay settings
 * overwrites the settings in the visualization application
 * @param useReplay if true, a replay is created (bool)
 * @param replayName full filename where the replay should be saved
 */
EXPORT void
vis_settings_setReplay( vis_connection* con, int useReplay,
    const char* replayName );

#ifdef USE_VOID_INTERFACE
#define CodecType_t int
#else
typedef enum
{
    CodecType_H264 = 1,
    CodecType_H265,
    CodecType_WMV,
    CodecType_MPEG4,
    CodecType_MS_MPEG4
}CodecType_t;
#endif

/*! Export video of the main window view
 * @param videoName full filename of the exported video file
 * @param codecType codec used for creating the video
 * @param encodingQuality quality of the encoding ([0,1], 0: worst, 1:best)
 * @param framerate frames per second in the resulting video
 * @param videoResolution the resulting resolution of the video ([x,y])
 * @param replaySpeed speed multiplier for the recorded simulation
 * @param startTime simulation time at which the video should start
 * @param endTime simulation time at which the video should end
 * @param highRenderQuality if true, render artifacts are reduced, 
 *      but it takes much longer
 */
EXPORT void
vis_settings_addVideoExport( vis_connection* con,
    const char* videoName, CodecType_t codecType, double encodingQuality,
    double framerate, const int videoResolution[ 2 ], double replaySpeed,
    double startTime, double endTime, int highRenderQuality );

/*! Set times when automatic screenshots of the main window will be created
 * simulation will halt at the specified times and has no effect during replay
 * the name of the screenshot will be composed from the image path and the
 * simulation time.
 * E.g. imagePath Screenshot_.png, time: 1.0 -> Screenshot_001.000.png
 * @param imagePath full path where the screenshots will be created
 * @param resolution the screenshots resolution
 * @param times the time points (in simulation time) at which screen screen
 *     shots will created
 * @param numTimes number of times in times
 */
EXPORT void
vis_settings_addScreenshot( vis_connection* con, const char* imagePath,
    const int resolution[ 2 ], const double* times, size_t numTimes );

/*!
 * Set additional searchpaths
 * At these locations CAD files, images and other resources are searched for
 * @param searchPaths additional paths to search for CADfiles
 * @param numPaths number of searchPaths to add
 */
EXPORT void
vis_settings_setSearchPath( vis_connection* con, const char*const* searchPath,
    size_t numPaths );

#ifdef USE_VOID_INTERFACE
#define StreamType_t int
#else
typedef enum
{
    StreamType_Network = 1,
    StreamType_Webcam,
    StreamType_File
}StreamType_t;
#endif
/*! Create a stream source, that receives a stream either from file, network or
 * webcam and sets the received data to a texture that can be accessed via index
 * @param streamType from which source the stream should be received
 * @param sourceUrl either an path to a file(e.g. Video.mp4), * a network
 * source(e.g. udp://@:12345) or a webcam name(e.g. Brio 4K Stream Edition)
 * @param textureId the id with which the resulting texture can be accessed with
 * @param setRes if true, a desired resolution for the webcam is set (bool)
 * @param resolution desired resoltion fore the webcam if setRes is true ([x,y])
 * @param enableChromakeying if true, chroma keying for the webcam is enabled
 */
EXPORT vis_streamSource*
vis_streamSource_create( vis_connection* con, const char* name,
    StreamType_t streamType, const char* sourceUrl, int textureId,
    int setRes, const int resolution[ 2 ], int enableChromaKeying );

/*! Destroys the stream source object
 */
EXPORT void
vis_streamSource_destroy( vis_streamSource* streamSourceObject );

/*! Create a depth source, that receives a depth stream
 * for now, it only works for network sources
 * @param streamType ignored, may be applicable in the future
 * @param sourceUrl url to a network source(e.g. udp://@:12345)
 * @param textureId the id with which the resulting texture can be accessed with
 * @param isRawDepthStream if true, the received stream is a raw depth stream
 * @param maxDepth the maximum depth the streamed image has, if not raw depth
 */
EXPORT vis_depthStreamSource*
vis_depthStreamSource_create( vis_connection* con, const char* name,
    StreamType_t streamType, const char* sourceUrl, int textureId,
    int isRawDepthStream, double maxDepth );

/*! Destroys the depth stream source object
 */
EXPORT void
vis_depthStreamSource_destroy( vis_depthStreamSource* depthStreamSourceObject );

#ifdef USE_VOID_INTERFACE
#define LightType_t int
#else
typedef enum
{
    LightType_Point = 1,
    LightType_Spot,
    LightType_Directional
} LightType_t;
#endif
/*! Create a light
 * @param lightType type of the light to create
 * @param lightMask mask used to define which object casts shadow
 */
EXPORT vis_light*
vis_light_create( vis_connection* con, const char* name,
    LightType_t lightType, int lightMask );

/*! Destroys the light object
 */
EXPORT void
vis_light_destroy( vis_light* lightObject );

/*! Activate / Deactivate light
 * @param enabled if true, light is enabled (bool)
 */
EXPORT void
vis_light_setEnabled( vis_light* con, int enabled );

/*! Set light color
 * @param color the color of the light ([r,g,b])
 * @param brightness the brightness of the light in lux (default: 1e5)
 */
EXPORT void
vis_light_setColor( vis_light* obj, const double color[ 3 ], double brightness );

/*! Activate / Deactivate whether a light casts shadows
 * @param shadowEnabled if true, the light creates shadows default: true(bool)
 */
EXPORT void
vis_light_setShadowEnabled( vis_light* obj, int shadowEnabled );

/*! Set a lights shadow options
 * @param shadowResolution resolution of the shadow textures
 * @param bias bias of the shadow map default:1
 * @param normalBias normal bias of the shadow map default:1
 * @param softness smoothening of the shadow edges
 */
EXPORT void
vis_light_setShadowOptions( vis_light* obj,
    ShadowResolution_t shadowResolution, double bias, double normalBias,
    double softness );

/*! Set a lights position
 * @param postion new postion of the light ([x,y,z])
 */
EXPORT void
vis_light_point_setPosition( vis_light* obj, const double position[ 3 ] );

/*! Set light attenuation
 * only applicable for point light
 * @param power defines how fast the light looses power, 0 means no loss
 * @param distance the maximum light distance
 */
EXPORT void
vis_light_point_setAttenuation( vis_light* obj, double power,
    double distance );

/*! Set a lights postion and orientation
 * only applicable for spot light
 * @param position position of the light ([x,y,z])
 * @param direction new direction of the light ([x,y,z])
 */
EXPORT void
vis_light_spot_setPose( vis_light* obj, const double position[ 3 ],
    const double direction[ 3 ] );

/*! Set light attenuation
 * only applicable for spot light
 * @param power defines how fast the light looses power, 0 means no loss
 * @param distance the maximum light distance
 */
EXPORT void
vis_light_spot_setAttenuation( vis_light* obj, double power, double distance );

/*! Set spot light parameters
 * only applicable for spot light
 * @param beamAngle beam opening angle (in degrees)
 * @param penumbra sharpness of shadow borders 0:sharp edge
 *      increasing -> softer edge
 */
EXPORT void
vis_light_spot_setParams( vis_light* obj, double beamAngle, double penumbra );

/*! Set direction of a directional light
 * only applicable for directional light
 * @param direction direction vector in which the light shines
 */
EXPORT void
vis_light_directional_setDirection( vis_light* obj,
    const double direction[ 3 ] );

/*! Set a directional light shadow cascades
 * only applicable for directional light
 * @param numCascades number if shadow cascades
 * @param cascadeBorders the relative border distances between
 *      the shadow cascades, size is numCascades-1,
 *      the default is {0.015,0.05,0.2}
 */
EXPORT void
vis_light_directional_setShadowCascades( vis_light* obj, int numCascades,
    const double* cascadeBorders );

/*! Create a terrain object
 * @param fileName full filename of terrain object to load
 */
EXPORT vis_terrain*
vis_terrain_create( vis_connection* con, const char* name,
    const char* fileName );

/*! Destroys the terrain object
 */
EXPORT void
vis_terrain_destroy( vis_terrain* terrainObject );

/*! Set position of a terrain
 * @param position position of the terrain ([x,y,z])
 * @param orientation the orientation of the terrain as quaternion ([i,j,k,w])
 */
EXPORT void
vis_terrain_setPose( vis_terrain* terrain, const double position[ 3 ], 
    const double orientation[4] );

/*! Set the masks of the terrain
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_terrain_setMasks( vis_terrain* obj, int intersection,
    int visibility, int shadow );

/*! create a line intersector object,
 * that returns intersections for the specified line
 * @param intersectionMask a mask that indicates what collision groups
 *      should be tested against
 */
EXPORT vis_lineIntersector*
vis_lineIntersector_create( vis_connection* connection, const char* name,
    int intersectionMask );

/*! Destroys the line intersector object
 */
EXPORT void
vis_lineIntersector_destroy( vis_lineIntersector* lineIntersectorObject );

/*! Set the lines that should be tested
 * @param numLines how many lines are to be tested
 * @param startPoints the startPoints of the lines ([x0,y0,z0,x1,y1,z1,..])
 * @param endPoints the endPoints of the lines ([x0,y0,z0,x1,y1,z1,..])
 */
EXPORT void
vis_lineIntersector_setLines( vis_lineIntersector* intersector, size_t numLines,
    const double* startPoints, const double* endPoints );

/*! specifies whether the line should be rendered and in what color
 @param visualize if true, the line is visualized (bool)
 @param color color of the visualized line ([r,g,b,a])
 */
EXPORT void
vis_lineIntersector_setLineVisualize( vis_lineIntersector* intersector,
    int visualize, const double color[ 4 ] );

/*! retrieves the current intersection data from the object
 * @param intersector the line intersector
 * @param dataSize the size of the output vectors
 * @param[out] hasIntersectionVec output vector of whether an intersection
 *     has occured
 * @param[out] intersectionPoints output vector of the collision points
 * @param[out] intersectionNormals output vector of the normals at the
 *     collision points
 * @return 0 on success, 1 on error
 */
EXPORT int
vis_lineIntersector_getIntersections( vis_lineIntersector* intersector,
    int dataSize, int* hasIntersectionVec, double* intersectionPoints,
    double* intersectionNormals );

/*! creates a depth mesh which deforms based on a depth texture
 * by providing the parameters and pose of the camera that created the depth
 * the objects seen can be reproduced
 * @param resolution the resolution of the mesh
 * @param depthTextureId the id of the depth texture to display
 * @param colorTextureId the id of the color texture,
 *      negative value means no color texture
 */
EXPORT vis_depthMesh*
vis_depthMesh_create( vis_connection* con, const char* name,
    const int resolution[ 2 ], int depthTextureId, int colorTextureId );

/*! Destroys the depth mesh object
 */
EXPORT void
vis_depthMesh_destroy( vis_depthMesh* depthMeshObject );

/*! Set the affiliated camera postion and rotation
 * @param position the position of the camera ([x,y,z])
 * @param orientation the orientation of the camera ([i,j,k,w])
 */
EXPORT void
vis_depthMesh_setCameraPose( vis_depthMesh* obj, double position[3],
    double orientation[4] );

/*! Set the parameters of the affiliated camera
 * @param fov the vertical field of view of the camera
 * @param aspectRatio the aspect ratio of the camera
 * @param maxDepth the maximal depth to be displayed
 */
EXPORT void
vis_depthMesh_setCameraParams( vis_depthMesh* obj, double fov,
    double aspectRatio, double maxDepth );

/*! Set the material of the mesh
 * @param color the color of the mesh ([r,g,b])(0,1)
 * @param metal if true, the surface will look metalic (bool)
 * @param roughness the roughness of the mesh (0,1)
 * @param emission if true, the object seems to emit light
 * @param emissionBrightness the brightness of the emission
 * @param wireframe if true, the objects will be shown in wireframe mode (bool)
 * @param wireframeColor the color of the wireframe if wireframe is used
 */
EXPORT void
vis_depthMesh_setMaterial( vis_depthMesh* obj, double color[ 4 ], int metal,
    double roughness, int emission, double emissionBrightness, int wireframe,
    double wireframeColor[4] );

/*! Set the discard flags to specify which mesh part should not be shown
 * @param discardMaxDepth if true, parts of the mesh with max depth isn't shown
 * @param discardOrtho if true, nearly orthogonal parts of the mesh isn't shown
*/
EXPORT void
vis_depthMesh_setDiscardFlags( vis_depthMesh* obj, int discardMaxDepth,
    int discardOrtho );

/*! Set the masks of the depth mesh
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_depthMesh_setMasks( vis_depthMesh* obj, int intersection,
    int visibility, int shadow );

#ifdef USE_VOID_INTERFACE
typedef int PointCloudInputType_t;
#else
typedef enum 
{
    PointCloudInputType_points = 1,
    PointCloudInputType_file,
    PointCloudInputType_camera,
    PointCloudInputType_pointStream
} PointCloudInputType_t;
#endif

/*! creates a point cloud with data from various input types
 * @param type the type of the input, either points, file or camera
 * @param fileName the path to the csv file, if file input is used
 * @param scaleFactor the factor the loaded points positions are scaled with
 * @param textureId the id of the camera texture if camera input is used
 * @param port the TCP port the data is received on for the point stream
 * @param dataId the id of the data if point stream is used
 * @param useGrid if true, points are snapped to a grid and combined
 * @param gridResolution resolution of the grid
*/
EXPORT vis_pointCloud*
vis_pointCloud_create( vis_connection* con, const char* name,
    PointCloudInputType_t type, const char* fileName, double scaleFactor,
    int textureId, int port, int dataId, int useGrid,
    double gridResolution );

/*! Destroys the point cloud object
 */
EXPORT void
vis_pointCloud_destroy( vis_pointCloud* pointCloudObject );

/*! Set the masks of the depth mesh
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_pointCloud_setMasks( vis_pointCloud* pointCloud, int intersection,
    int visibility, int shadow );

/*! Set the points of the point cloud if the point input is used
 * @param points the points to display if point input is used,
 *      a vector of 3*numPoints
 * @param numPoints the number of input points
*/
EXPORT void
vis_pointCloud_setPoints( vis_pointCloud* pointCloud, double* points,
    int* intensities, int numPoints );

/*! Set the camera pose if camera input is used for the point cloud
 * it is assumed the camera points towards -z and y is up
 * @param pointCloud the point cloud to set the camera pose for
 * @param position the new position of the camera ([x,y,z])
 * @param orientation the new orientation of the camera
 *      as quaternion ([i,j,k,w]),
 */
EXPORT void
vis_pointCloud_setCameraPose( vis_pointCloud* pointCloud, double* position,
    double* orientation );

/*! Set camera parameters of the camera if the camera input is used
 * @param fov the field of view in degrees of the camera
 * @param aspectRatio the aspectRatio of the camera
 * @param maxDepth the maximum depth of the camera
 */
EXPORT void
vis_pointCloud_setCameraParams( vis_pointCloud* pointCloud, double fov,
    double aspectRatio, float maxDepth );

/*! Set the position and orientation of the point cloud
 * @param pointCloud the point cloud to set the pose of
 * @param position new position of the point cloud ([x,y,z])
 * @param orientation new orientation of the point cloud as quaternion ([i,j,k,w])
 */
EXPORT void
vis_pointCloud_setPose( vis_pointCloud* pointCloud, double* position,
    double* orientation );

/*! Set the point size of the pointCloud
 * @param pointCloud the point cloud to set the point size of
 * @param pointSize the new size of the points
 */
EXPORT void
vis_pointCloud_setPointSize( vis_pointCloud* pointCloud, double pointSize );

/*! Set the treshold of the points that should be displayed
 * only has an effect if useGrid is set to true
 * streamed points have an intensity value, which can represent the confidence
 * in the correctness of the point. For a depth texture the intensity is set
 * to be inversely to the depth value. Points with 0 depth will have a value
 * of 65535, at max depth 1. Points set via vis_pointCloud_setPoints() will
 * have a vaule of 1. This function can be used to either filter out grid
 * points where the sum of intensities of all points that fall into this grid
 * point is too low or the the maximum intensity received at a grid point is
 * too low
 * @param sumThreshold grid points with a accumulated intensities below this
 *      value are hidden
 * @param intensityThreshold grid points with a maximum intensity below this
 *      value are hidden
 * @param lodMultiplier defines how points far away from the camera are reduced
 *      to lower the rendering low, 0 means no reduction, balanced: 0.01
 */
EXPORT void
vis_pointCloud_setGridParameters( vis_pointCloud* pointCloud,
    int sumThreshold, int intensityThreshold, double lodMultiplier );

/*! Set the strenght of the dummy shading using eye dome lighting
 * @param edlShadingStrength shading strength, 0: no shading, 1 max shading
 */
EXPORT void
vis_pointCloud_setShadingStrength( vis_pointCloud* pointCloud,
    float edlShadingStrength );

#ifdef USE_VOID_INTERFACE
typedef int PointCloudIntensityVisType_t;
#else
typedef enum
{
    None = 1,
    Accumulated,
    Maximum,
} PointCloudIntensityVisType_t;
#endif

/*! Set visualiualization of intensities
 * the intensities can be visualized using a look up texture
 * @param visualize defines how the intensities should be visualized
 * @param minIntensity values at/below this value are visualized as the
 *      minimum intensity
 * @param maxIntensity values at/above this value are visualized as the
 *      maximum intensity
 */
EXPORT void
vis_pointCloud_setIntensityVisualize( vis_pointCloud* pointCloud, PointCloudIntensityVisType_t visualize, int minIntensity, int maxIntensity );

#ifdef USE_VOID_INTERFACE
typedef int Line3DVisType_t;
#else
typedef enum
{
    Line3DVisType_Solid = 1,
    Line3DVisType_Soft,
    Line3DVisType_ThinLine
} Line3DVisType_t;
#endif

/*! Creates an empty 3d line
 * @param visType visualization type of the line
 * @param segmentedif true the line is segmented into lines every two
 *      points form a segment (bool)
 */
EXPORT vis_line3d*
vis_line3d_create( vis_connection* con, const char* name, Line3DVisType_t visType, int segmented );

/*! Destroys the line3d object
 */
EXPORT void
vis_line3d_destroy( vis_line3d* line3dObject );

/*! Set the visibility mask of the line
 *!param visibilityMask the new visibilityMask of the line
 */
EXPORT void
vis_line3d_setVisibility( vis_line3d* line, int visibilityMask );

/*! Set the pose of of the line
 * @param line the line to set the pose of
 * @param position the new position of the line ([x,y,z])
 * @param orientation the new orientation of the line ([i,j,k,w])
 * @param scale the new scale of the line ([x,y,z])
 */
EXPORT void
vis_line3d_setPose( vis_line3d* line, double position[ 3 ],
    double orientation[ 4 ], double scale[ 3 ] );

/*! Set the line of the width either in screen space or world space
 * @param line the line to set the width of
 * @param lineWidth the new width of the line
 * @param screenSpace if true, the width is in screen space otherwise in
 *      world space
 */
EXPORT void
vis_line3d_setLineWidth( vis_line3d* line, double lineWidth, int screenSpace );

/*! Set the positions of the points relative to the origin of the line object
 * @param line the line to set the new points of
 * @param points the new points to set each point has [x,y,z]
 * @param numPoints the new number of points
 * @param setColors if true, the color of the individual points is set by colors
 * @param colors an array containing the colors of each point [r,g,b,a]
 */
EXPORT void
vis_line3d_setPoints( vis_line3d* line, double* points, size_t numPoints,
    int setColors, double* colors );

/*! Adds a single point to the existing line
 * @param line the line to add the point to
 * @param point the position of the point to add
 * @param maxPoints the maximum length of the line,
        first points of the line will be removed if line is longer
 */
EXPORT void
vis_line3d_addPoint( vis_line3d* line, const double point[ 3 ], int maxPoints );

/*! Adds a single point to the existing line
 * fails if line wasn't colored before
 * @param line the line to add the point to
 * @param point the position of the point to add
 * @param color the color of the point to add
 * @param maxPoints the maximum length of the line,
        first points of the line will be removed if line is longer
 */
EXPORT void
vis_line3d_addColoredPoint( vis_line3d* line, const double point[ 3 ],
    const double color[ 4 ], int maxPoints );

/*! Set the color of the whole line
 * @param color the new color of the line ([r,g,b,a])
 */
EXPORT void
vis_line3d_setColor( vis_line3d* line, double color[ 4 ] );

/*! Set the line as billboard
 * @param alignment definees how the line should be rotated to the screen
 * @param offset the offset relative to the rotated line
 */
EXPORT void
vis_line3d_setBillboard( vis_line3d* line, BillboardAlignment_t alignment,
    const double offset[ 3 ] );

#ifdef USE_VOID_INTERFACE
#define FlowVisStyle int
#else
typedef enum
{
    FlowVisStyle_Ring = 1,
    FlowVisStyle_Cone,
    FlowVisStyle_Arrow,
    FlowVisStyle_CADFile
} FlowVisStyle;
#endif

/*! creates a pathflow
 * @param visualizationStyle defines how the flow should be visualization
 * @param elementDensity number of elements per path meter
 * @param phaseShift shifts the starting point of an element by the portion
 *      of the distance to the next element
 * @param lodDistances maximum visible distance of each detail level
 * @param numLods number of detail levels
 * @param cadFilenames paths to the cad files if visualization style is
 *      cad files, needs to be same as numLods
 * @param cadOrientation the orientation of the cad file 
 * @param cadOverwriteColor if true, the color of the cad file will
 *      be overwritten
 * @param cadOverwriteProperties if true, the material properties of the cad
 *      file will be overwritten
 * @param twosided overwrite the object properties and make all surface double
 *     sided, even if they aren't in the file description
 * @param showPathPoints if true, the path points are visualized
 * @param pathPointSize size of the visualized path points
 * @param showPath if true, the path will be rendered as a line
 * @param useCubicInterpolation if true, the path will be cubically interpoleted
 *      linear interpolation otherwise
 */
EXPORT vis_pathFlow*
vis_pathFlow_create( vis_connection* con, const char* name,
    FlowVisStyle visualizationStyle, double elementsDensity, double phaseShift,
    const double* lodDistances, size_t numLods, const char** cadFilenames,
    const double* cadOrientation, int cadOverwriteColor,
    int cadOverwriteProperties, int cadIsTwoSided,int showPathPoints,
    double pathPointSize, int showPath, int useCubicInterpolation );

/*! Destroys the pathFlow object
 */
EXPORT void
vis_pathFlow_destroy( vis_pathFlow* pathFlowObject );

/*! Update the points of the path
 * @param numPathPoints number of points in the path
 * @param pathPoints positions of the path points
 * @param scales desired scaling of flow elements at the path points
 * @param colors desired color of flow elements at the path points
 * @param emissionBrightness desired brightness of the emission at the path points
 * @param speeds desired speeds of flow elements at the path points
 * @param setUpVectors if true, up vectors are provided e.g. to keep objects
 *      pointing upwards or manuall rotating them along the axis, only useful
 *      for cad files
 * @param upVectors desired up vectors of the flow elements at the path points
 */
EXPORT void
vis_pathFlow_setPath( vis_pathFlow* obj, size_t numPathPoints,
    const double* pathPoints, const double* scales, const double* colors,
    const double* emissionBrightness, const double* speeds, int setUpVectors,
    const double* upVectors  );

/*! Set position and orientation of PathFlow
 * @param position new position of the PathFlow ([x,y,z])
 * @param orientation new orientation of the PathFlow as quaternion ([i,j,k,w])
 * @param scale new scale of the PathFlow ([x,y,z])
 */
EXPORT void
vis_pathFlow_setPose( vis_pathFlow* obj, const double* position,
    const double* orientation, const double* scale );

/*! Set the surface properties of the flow elements
 * only has effect on cad files if overwrite surface properties is true
 * @param metal if true, elements have a mettallic appearance
 * @param roughness surface roughness ([0,1], 0:smooth, 1:rough)
 * @param emission if true, the object seems to emit light
 *               and unaffected by any light source
 */
EXPORT void
vis_pathFlow_setSurfaceProperties( vis_pathFlow* obj, int metal,
    double roughness, int emission );

/*! Set the masks of the PathFlow
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_pathFlow_setMasks( vis_pathFlow* obj, int intersectionMask,
    int visibilityMask, int shadowMask );

/*! creates a particle flow object, that visualizes the flow inside a medium
 * The user provides a 3 dimensional grid with flow speeds and seed points
 * at which particles are inserted. These particles are moved inside the flow
 * according to the flow speeds.
 * @param gridResolution number of grid points in each dimension ([x,y,z])
 * @param gridSpacing distance between the individual grid points
 * @param showParticle if true, particles are rendered that follow the flow
 * @param particleSize size of the individual particles
 * @param particleSeedInterval time interval between particle creation
 * @param showLines if true, lines are rendered that follow the flow
 * @param lineResolution time interval between line points
 * @param drawLinsIntant if true, simulates the complete path of
 *      the flow instantly
 * @param instantLinesMaxTime the maximum simulated time used to draw
 *      the lines instantly
 * @param animationSpeed speed multiplier of the flow animation
 */
EXPORT vis_gridParticleFlow*
vis_gridParticleFlow_create( vis_connection* con, const char* name,
    const int* gridResolution, double gridSpacing, int showParticles,
    double particleSize, double particleSeedInterval, int showLines,
    double lineResolution, int drawLinesInstant, double instantLinesMaxTime,
    double animationSpeed );

/*! Destroys the grid particle flow object
 */
EXPORT void
vis_gridParticleFlow_destroy( vis_gridParticleFlow* gridParticleFlowObject );

/*! Update the flow speed vectors at the grid pionts
 * @param gridPointVectors new grid flow speed vectors
 *      has to have length of gridResolution.x*gridResolution.y*gridResolution.z*4
 */
EXPORT void
vis_gridParticleFlow_setVectors( vis_gridParticleFlow* obj, 
    const double* gridPointVectors );

/*! Replaces the seeding points
* @param seedingPoints new seeding points at which the flow particles
*       should spawn
* @param numSeedingPoints number of new seeding points
 */
EXPORT void
vis_gridParticleFlow_setSeedingPoints( vis_gridParticleFlow* obj,
    const double* seedingPoints, size_t numSeedingPoints );

/*! Set position and orientation of particleFlow
 * @param position new position of the particleFlow ([x,y,z])
 * @param orientation new orientation of the particleFlow
 *      as quaternion ([i,j,k,w])
 * @param scale new scale of the particleFlow ([x,y,z])
 */
EXPORT void
vis_gridParticleFlow_setPose( vis_gridParticleFlow* obj, const double* position,
    const double* orientation, const double* scale );

/*! Set the color of the particles
 * @param color new color of the particles ([r,g,b,a])
 */
EXPORT void
vis_gridParticleFlow_setParticleColor( vis_gridParticleFlow* obj, const double* color );

/*! Set line color of particleFlow if the grid points have no color defined
 * @param color new color of the lines ([r,g,b])
 */
EXPORT void
vis_gridParticleFlow_setLineColor( vis_gridParticleFlow* obj, const double* color );

/*! sets the color at gridpoints which affect the visualized lines
 * @param gridPointColors new colors at the grid points ([r0,g0,b0,a0,r1,..])
 *      has to have length of gridResolution.x*gridResolution.y*gridResolution.z*4
 */
EXPORT void
vis_gridParticleFlow_setGridColors( vis_gridParticleFlow* obj,
    const double* gridPointColors );

/*! Set the visibilty mask of the particle flow
 * @param visibility mask that specifies which camera sees the particle flow
 */
EXPORT void
vis_gridParticleFlow_setVisibilityMask( vis_gridParticleFlow* obj, int mask );

/*! Creates a grid vector field
 * visualizes a 3 dimensional field of vectors as individual arrows
 * @param gridResolution number of grid points in each dimension ([x,y,z])
 * @param gridSpacing distance between the individual grid points
 */
EXPORT vis_gridVectorField*
vis_gridVectorField_create( vis_connection* con, const char* name,
    const int* gridResolution, double gridSpacing );

/*! Destroys the grid vector flow object
 */
EXPORT void
vis_gridVectorField_destroy( vis_gridVectorField* gridVectorFieldObject );

/*! Update the vectors at the grid pionts
 * @param gridPointVectors new direction vectors at the grid points
 *      has to have length of gridResolution.x*gridResolution.y*gridResolution.z*3
 * * @param gridPointColors new colors at the grid points ([r0,g0,b0,a0,r1,..])
 *      has to have length of gridResolution.x*gridResolution.y*gridResolution.y*4
 */
EXPORT void
vis_gridVectorField_setVectors( vis_gridVectorField* obj,
    const double* gridPointVectors, const double* gridPointColors );

/*! Set position and orientation of particleFlow
 * @param position new position of the particleFlow ([x,y,z])
 * @param orientation new orientation of the particleFlow
 *      as quaternion ([i,j,k,w])
 * @param scale new scale of the particleFlow ([x,y,z])
 */
EXPORT void
vis_gridVectorField_setPose( vis_gridVectorField* obj, const double* position,
    const double* orientation, const double* scale );

/*! Set the scaling that is applied to the individual vectors
 * @param vectorScale new scaling value for the vectors
 */
EXPORT void
vis_gridVectorField_setVectorScale( vis_gridVectorField* obj,
    double vectorScale );

/*! Set the visibilty mask of the particle flow
 * @param visibility mask that specifies which camera sees the grid vector field
 */
EXPORT void
vis_gridVectorField_setVisibilityMask( vis_gridVectorField* obj, int mask );

/*! Set the radius of the rendered arrows in world units
 * @param arrowSize new size of the arrows
 */
EXPORT void
vis_gridVectorField_setArrowSize( vis_gridVectorField* obj, double arrowSize );

#ifdef USE_VOID_INTERFACE
#define CesiumSourceType_t int
#else
typedef enum
{
    CesiumSourceType_Tiles = 1,
    CesiumSourceType_IonTiles = 2,
    CesiumSourceType_IonTerrain = 3
} CesiumSourceType_t;
#endif

/*! creates a cesium tile based on cesium data
 * @param cesiumSourceType the type of the cesium source
 * @param url either the path to a local tileset.json or an url
 * @param terrainId the id of the terrain data on the ion server
 * @param imageryId the id of the imagery data on the ion server
 * @param accessToken the access token for the ion server
 * @param useProxy if true, a proxy server is used to access the data
 * @param proxyUrl the url of the proxy server
 * @param proxyPort the port of the proxy server
 */
EXPORT vis_cesiumTile*
vis_cesiumTile_create( vis_connection* con, const char* name,
    CesiumSourceType_t cesiumSourceType, const char* url, int terrainId,
    int imageryId, const char* accessToken, int useProxy, const char* proxyUrl,
    int proxyPort );

/*! Destroys the cesium tile object
 */
EXPORT void
vis_cesiumTile_destroy( vis_cesiumTile* cesiumTileObject );

/*! sets options of the cesium tile
 * @param maxTerrainError the maximum screen pixel error for the terrain data,
            default is 16, lower value means higher quality but lower performance
 * @param maxImageryError the maximum screen pixel error for the imagery data,
            default is 2, lower value means higher quality but lower performance
 * @param maxCacheSize the maximum cache size in MB for the terrain/imagery data
 * @param enableFrustumCulling If true, only tiles that are inside the camera 
 *          view frustum are loaded
 * @param forceCreateNormals If true, new normals will be created, even if they 
 *           are already defined in the model
 * @param normalsAngle if forceCreateNormals is true, this value defines 
 *          the max angle in deg of an edge where it seems to be smooth 
 * @param clampTextures if true, the textures will be clamped instead of repeated
 */
EXPORT void
vis_cesiumTile_setOptions( vis_cesiumTile* obj, double maxTerrainError,
    double maxImageryError, int maxCacheSize, int loadSurroundingTiles,
    int enableFrustumCulling, double normalsAngle, int clampTextures );

/*! overwrites the materials of the loaded models
 * @param metal if true, elements have a mettallic appearance
 * @param roughness surface roughness ([0,1], 0:smooth, 1:rough)
 * @param emission if true, the object seems to emit light
 * @param emissionBrightness the brightness of the emission
 */
EXPORT void
vis_cesiumTile_overwriteMaterial( vis_cesiumTile* obj, int metal, double roughness,
    int emission, double emissionBrightness );

/*! Set position and orientation of the cesium tile
* @param position new position of the object ([x,y,z])
* @param orientation new orientation of object as quaternion ([i,j,k,w])
*/
EXPORT void
vis_cesiumTile_setPose( vis_cesiumTile* obj, const double position[ 3 ],
    const double orientation[ 4 ] );

/*! Set the masks of the cesium tile
* @param intersection mask that specifies which intersection is recognized
* @param visibility mask that specifies which camera sees this object
* @param shadow mask that specifies which light source creates the objects shadow
*/
EXPORT void
vis_cesiumTile_setMasks( vis_cesiumTile* obj, int intersectionMask,
    int visibilityMask, int shadowMask );

#ifdef USE_VOID_INTERFACE
#define CloudType_t int
#else
typedef enum
{
    CloudType_Altocumulus = 1,
    CloudType_Cumulus = 2,
    CloudType_Nimbostratus = 3,
    CloudType_Stratocumulus = 4,
    CloudType_Stratus = 5
} CloudType_t;
#endif

/* Creates a 3D cloud layer
 * @param cloudType the basic type of the cloud
 * @param castShadows if true, the cloud will cast a shadow on the ground
 */
EXPORT vis_cloudLayer*
vis_cloudLayer_create( vis_connection* con, const char* name,
    CloudType_t cloudType, int castShadows );

/*! Destroys the cloud layer object
 */
EXPORT void
vis_cloudLayer_destroy( vis_cloudLayer* cloudLayerObject );

/* Sets the parameters of the cloud
 * @param altitude the median height at which the cloud layer will be
 * @param heightExtent the height/thickness of the cloud layer
 * @param coverage how much of the sky should be covered by the clouds
 */
EXPORT void
vis_cloudLayer_setParameters( vis_cloudLayer* obj, double altitude,
    double heightExtent, double coverage );

/* Sets the wind speed that moves the clouds, needs to be set in the first frame
 * @param windSpeed the speed of the wind in m/s,
         z value is only internal cloud movement and does not move the layer
 */
EXPORT void
vis_cloudLayer_setWindSpeed( vis_cloudLayer* obj, double windSpeed[ 3 ] );

/* Sets a position offset of the clouds, instead of using wind
 * @param positionOffset the offset in m
         z value is only internal cloud movement and does not move the layer
 */
EXPORT void
vis_cloudLayer_setPositionOffset( vis_cloudLayer* obj, double positionOffset[ 3 ] );

/* Sets the visibility mask of the cloud layer to define in which cameras it
 * will be seen
 * @param mask the visibility mask
 */
EXPORT void
vis_cloudLayer_setVisibility( vis_cloudLayer* obj, int mask );

/* Creates a coordinate system
 * @param numAxes the number of axes of the coordinate system
 * @param directions the directions of the individual axis in x,y,z
 */
EXPORT vis_coordSystem*
vis_coordSystem_create( vis_connection* con, const char* name,
    size_t numAxes, const double* directions );

/* Adds labels to the coordinate system
 * needs to be called in the first frame
 * @param labelsText the text for each axis label
 * @param color the text color of the label
 */
EXPORT void
vis_coordSystem_addLabels( vis_coordSystem* obj,
    const char* const* labelsText, const double color[ 3 ] );

/* Sets the material for the axes of the coordinate system
 * @param colors an array of the rgb color for each axis
 * @param transparency the transparency of the coordinate system
 * @param emission if true, the coordinate system ignores external lighting
 */
EXPORT void
vis_coordSystem_setMaterial( vis_coordSystem* obj, const double* colors,
    double transparency, double emission );

/*! Set position and orientation of the coordinate system
* @param position new position of the object ([x,y,z])
* @param orientation new orientation of object as quaternion ([i,j,k,w])
*/
EXPORT void
vis_coordSystem_setPose( vis_coordSystem* obj, const double position[ 3 ],
    const double orientation[ 4 ] );

/*! Set the masks of the coordinate system
 * @param intersection mask that specifies which intersection is recognized
 * @param visibility mask that specifies which camera sees this object
 * @param shadow mask that specifies which light source creates the objects shadow
 */
EXPORT void
vis_coordSystem_setMasks( vis_coordSystem* obj, int intersection,
    int visibility, int shadow );

/* parses the xml-node file containing the information of a geodetic pivot
 * @param path the path to the node file
 * @param[out] origin the origin position of the geodetic pivot ([lat,long,alt])
 * @param[out] semimajorAxis the semiMajor axis of the geodetic pivot
 * @param[out] flattening the flattening of the geodetic pivot
 * @return true if reading was successful, false otherwise
 */
EXPORT int
vis_geodeticConverter_getInfoFromNodeFile( const char* path, double origin[ 3 ], double * semimajorAxis, double * flattening );

/*! initializes a converter to convert from geodetic to cartesian coordinates
 * @param geodeticOrigin the geodetic position where the converted position will be 
          the cartesian postion of the converter 
 * @param cartesianOriigin the cartesian position of the geodetic converter
 * @param semimajorAxis the equatorial radius of the planet
 * @param flattening the flattening of the planet
 */
EXPORT vis_geodeticConverter*
vis_geodeticConverter_create( const double geodicOrigin[ 3 ],
    double cartesianOrigin[ 3 ], double semimajorAxis, double flattening );

/*! destroys the given geodedic converter
 * @param converter the converter to destroy
 */
EXPORT void
vis_geodeticConverter_destroy( vis_geodeticConverter* converter );

/*! converts the given geodetic coordinates to cartesian
 * @param converter the converter that holds the infos for the conversion
 * @param geodeticPosition the input geodetic position ([lat,long,height])
 * @param cartesianPosition the converted cartesian position as output,
          has to point valid memory before calling
 */
EXPORT void
vis_geodeticConverter_getCartesianPosition( vis_geodeticConverter* converter, 
    const double geodeticPosition[ 3 ], double cartesianPosition[ 3 ] );

/*! converts the given geodetic coordinates to cartesian including the rotation
 * @param converter the converter that holds the infos for the conversion
 * @param geodeticPosition the input geodetic position ([lat,long,height])
 * @param cartesianPosition the converted cartesian position as output,
          has to point valid memory before calling ([x,y,z])
 * @param cartesianRotation the converted cartesian rotation as matrix as output,
          has to point valid memory before calling (3x3 Matrix)
 */
EXPORT void
vis_geodeticConverter_getCartesianTransform( vis_geodeticConverter* converter,
    const double geodeticPosition[ 3 ], double cartesianPosition[ 3 ],
    double cartesianRotation[ 9 ] );

EXPORT int
vis_fileInfo_getBonenames_Internal( const char* path, char** boneNames, size_t maxNumBones, int* actualNumBones, const char* fileInfoDir );

inline EXPORT int
vis_fileInfo_getBonenames( const char* path, char** boneNames, size_t maxNumBones, int* actualNumBones, const char* fileInfoDir )
{
#ifdef MODELICA
    vis_setModelicaFunctions();
#endif
    return vis_fileInfo_getBonenames_Internal( path, boneNames, maxNumBones, actualNumBones, fileInfoDir );
}

#ifndef LOGGER
#define LOGGER

/* Logging related function */

/*! Set logging function
* @param p function to be used for logging Visualization related events.
* Function has to use the same signature as printf.
*/
EXPORT void
vis_log_setLogger( int( *p )( const char* fmt, ... ) );

/*! Set logging level
* @param l level - 0: no logging, 1: only error, 2: warnings, 3: debugging
*/
EXPORT void
vis_log_setLevel( int l );

/*! Set logging prefix
*
* @param pre a prefix put in front of all logging output
*/
EXPORT void
vis_log_setPrefix( const char* pre );

/*! Log an error message (syntax similiar to printf) */
EXPORT void
vis_log_E( const char *fmt, ... );

/*! Log a warning message */
EXPORT void
vis_log_W( const char *fmt, ... );

/*! Log a debug message */
EXPORT void
vis_log_D( const char *fmt, ... );

/*! Log last OS networking related error message */
EXPORT void
vis_log_networkError();

/*! Losg last OS error message*/
EXPORT void
vis_log_systemError();

#endif // LOGGER

EXPORT void
vis_setAllocateString( char* ( *p )( size_t ) );

EXPORT int
CopyToClipboard( const char* strText );

EXPORT void
vis_setGlobalFlag( int id, int value );

EXPORT int
vis_getGlobalFlag( int id );

#ifdef MODELICA
inline void
vis_setModelicaFunctions()
{
    vis_log_setLogger( ( ( int( * )( const char*, ... ) ) &ModelicaFormatMessage ) );
    vis_setAllocateString( &ModelicaAllocateString );
}
#endif

#ifdef __cplusplus
}
#endif

// avoid poluting global namespaces
#undef EXPORT
#undef USE_VOID_INTERFACE
#undef EXTERNAL_FUNCTION_EXPORT

#endif // VISUALIZATION
