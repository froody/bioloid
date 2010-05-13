
#ifndef __SCENE_H__
#define __SCENE_H__

#ifdef __cplusplus
extern "C" {
#endif

void draw_scene(World *w, int textures, int x_size, int y_size);

// There's a dependency in the ODE libs on dDOUBLE/dSINGLE, which means that different
// graphics routines need to be called based on the way in which the ODE library was compiled.
// This interfaces the two types invisibly in scene.cpp
#ifdef dDOUBLE
// #define dREAL_TO_DRAWING const double *)(void *
 #define dsDrawBoxReal dsDrawBoxD
 #define dsDrawSphereReal dsDrawSphereD
 #define dsDrawCylinderReal dsDrawCylinderD
 #define dsDrawCappedCylinderReal dsDrawCappedCylinderD
#else
// #define dREAL_TO_DRAWING const float *)(void *
 #define dsDrawBoxReal dsDrawBox
 #define dsDrawSphereReal dsDrawSphere
 #define dsDrawCylinderReal dsDrawCylinder
 #define dsDrawCappedCylinderReal dsDrawCappedCylinder
#endif

/* closing bracket for extern "C" */
#ifdef __cplusplus
}
#endif

#endif

