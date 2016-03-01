Unity Raytracer
===========================

## Description
This project is used to demonstrate additional components of raytracing within Unity for our graphics course (CS 4731).

#### Features
* The mirrored surface on the sphere was done using the ray tracer implementation and recursively calculation the colors bouncing off other objects. From the first ray collision, the reflection was calculated and the ray tracing was run again from the new reflection ray.
* The texture mapping on the cube was done using bilinear pixel interpolation. For every texture coordinate that was hit by a ray, the coordinates were interpolated across the texture map. The coordinates were scaled and averaged between pixel coordinates on the texture.
* The light attenutation in the entire scene was done using the quadratic term in the attenuation function. The function was based on the distance from the light to the hit point in question during the calculation.

#### Notes
* This project demonstrates ray tracing in Unity, which is a real time rendering engine. The project is setup such that the image is rendered from left to right, top to bottom.
* Each pixel on the image plane is given the appropriate color after light, reflection and texture calculation.

## Resources
* For Barycentric coordinates, I used these references:
  * [Moller-Trumbore Paper](https://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf)
* For bilinear filtering, I used these references:
  * [Bilinear Filtering](https://en.wikipedia.org/wiki/Bilinear_filtering)
  * [Texturing](http://www.cs.uu.nl/docs/vakken/gr/2011/Slides/06-texturing.pdf)
* For light attenuation, I used these references:
  * [Phong Shading](http://www.gameprogrammer.net/delphi3dArchive/phongfordummies.htm)
  * [Lighting Falloff](https://developer.valvesoftware.com/wiki/Constant-Linear-Quadratic_Falloff)
* For mirror reflectance, I used these references:
  * [Mirroring In Ray Tracing](http://courses.durhamtech.edu/opticianry/oph142/NFOS-12/Geometric_Optics_files/mirror_ray_tracing.html)
  * [WPI CS4731 Slides](http://users.wpi.edu/~clindsay/teaching/CS4731A2015/)

## Building
This project can be built within Unity.

## Usage
Open this project in Unity and run in order to allow the ray tracing render the scene.

## Source Files
* RenderLoop.cs
  * This is where the raytracing functions are implemented.
* RaytracedCamera.cs

## Screenshots
* [mirror](https://github.com/SIZMW/unity-raytracer/blob/master/Screenshots/mirror.png)
  * This is a screenshot of just the mirror effect on the sphere.
* [attenuation](https://github.com/SIZMW/unity-raytracer/blob/master/Screenshots/attenuation.png)
  * This is a screenshot of the mirror effect and light attenuation on the entire scene.
* [texture](https://github.com/SIZMW/unity-raytracer/blob/master/Screenshots/texture.png)
  * This is a screenshot of the mirror effect, light attenuation and the texture mapping on the cube.
* [final-scene](https://github.com/SIZMW/unity-raytracer/blob/master/Screenshots/final-scene.png)
  * This is a final screenshot of all the effects listed above in one scene.
  * This is where the camera and rendering plane are setup for this project.
