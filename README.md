# geocomb-cpp

# This is the base c++ library

so it can be easily used in other platforms with a single core code base. Here's the [preview[(https://codesandbox.io/p/sandbox/hex-map-dev-z0qc0?file=%2Fsrc%2Fsketch.ts%3A49%2C24)

the original plan was to use vulkan instead of p5js as I was doing with the typescript version for seeing if everything works, but I'll work on this later since I want to learn vulkan properly, and I'll need to dedicate some time to do that. Instead I'll be working on napi version that relies on this library, geocomb-cpp, and draw everything with p5js.

This library used to be called hexmap, but since there are a couple of 2d hexagon generating libraries with similar names (in npm), and considering hexmap should be in beta but is in version 2 already (first package, wasn't really sure what I was doing), I'm going to mark hexmap as deprecated, and start fresh with geocomb (proper versioning). Geocomb comes from the combination of the words Geography and Honeycomb. Honeycomb because they're made up of hexagons, and they also look pretty cool.

## project structure:

- all geocomb code goes in ./src/
- ./src/ contains all code hexmap needs to work (no graphics)
- ./tests/ contains multiple programs (each with with their own main()) that each do different things.
- that's pretty much it for now.

## important notes

- to build just type in terminal `make [test name]`. Currently the only working test is hash_test
- hash is returned through hash_properties struct, encoding and decoding needs to be implemented by you or your team. This is because there are multiple different ways to encode/decode, so you should use one that works best for your application. In the future I might add some default ones, but I was messing around with converting hash string to int and there were some collisions so, this will probably be added eventually but it's not a priority.

## compatibility notes (with dart & js)

### general

- node:
  - will have ts wrapper that interacts with node-addon-api, and node-addon-api will call c++ geocomb
  - node-addon-api interface will store static icosahedron object, and ts wrapper will have icosahedron class. ts icosahedron has getters and setters, and will interact with c++ through node-addon-api static icosahedron obj and its instance methods. Triangles, Points, & their arrays can be converted from c++ with NAPI, but use with getters & setters since need to be translated
- dart:
  - will have dart wrapper that wraps c wrapper of c++ library
  - dart wrapper will have static icosahedron obj, and call instance methods on it
  - c wrapper will also store icosahedron struct, and call instance methods as static functions that require an icosahedron struct to access triangles, points, etc.
  - only thing need to see now is how to store triangle & point arrays? (might be able to pass vector or array using dart pointer, but still in beta so who knows? NOTE: gonna do nodejs implimentation first, then try out creating a dart package, hopefully will support some more features, maybe even web)

### js / node

- this should be pretty easy, since can pass objects back & forth between js & c++

### dart

- this will be slightly more involved:
  - need to write wrapper class for c++ in c since can only call c from dart

<del>

### general (all things considered)

- should icosahedron have instance methods or only have obj with static functions that take params?
  - instance:
    - (node) can you store obj in node env or something like that? (either way need way to store obj somewhere, +1 to instance)
    - (dart) need to store icosahedron obj in c++ c wrapper, same as above (+1 to instance, just need to call static functions from c wrapper with icosahedron obj, but how to get orientation & rotation_method?)
  - static:
    - (node) no benefits really
    - (dart) easier to implement c wrapper, but not by much

</del>
