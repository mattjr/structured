Software Renderer
Version 1.6.3

Does vertex transformation using a user supplied VertexShader.
The output coordinates x, y, z and w are all to be specified in 
fixed point 16.16 format.

The clipping and rasterization pipeline only uses integer arithmetic.
The GeometryProcessor can be configured with a vertex shader and a rasterizer 
can be set. There can be various implementations of Rasterizers. Currently only
one is provided.

For usage examples please look at my homepage.