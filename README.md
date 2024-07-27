# ImagePoster

A small test application to distort images in a poster-like way.

![grafik](https://github.com/user-attachments/assets/4cac117f-703f-4154-9ec0-0bc9894ea8a9)

## Background

A number of seed points is distributed randomly over the source image. 
From these points a [*Delaunay* triangulation](https://en.wikipedia.org/wiki/Delaunay_triangulation) as well as a network of [*Voronoi* polygons](https://en.wikipedia.org/wiki/Voronoi_diagram) is created. 
[*Lloyd's algorithm*](https://en.wikipedia.org/wiki/Lloyd%27s_algorithm) can be applied to relax the point positions so that they tend to accumulate at darker image parts.

There are several ways to display the modified image:
1. Just display the seed points. They can take the color of the original image points at their location,
   and their size can be selected according to the darkness of the image at the seed points - the darker to image the darker to point
2. Draw the network of Delaunay triangles. The triangles can be filled with a triangular gradient where the start colors are those at the triangle corners in the original image.
3. Draw the Voronoi polygons. They can be filled uniformly with the original image's color at the center point of the polygon (Delaunay triangle corner).
   The border of the polygon can be drawn in an arbitrary color.
4. Or any combination.

## Compilation
The application must be compiled from its Pascal sources by means of Lazarus (v2.0 or newer) / Free Pascal (v3.0 or newer) for Windows, Linux (gtk2, qt5, qt6) or macOS.
Additional third-party packages are not required.

## Acknowledgments
The "lenna" test image is taken from https://en.wikipedia.org/wiki/Lenna
