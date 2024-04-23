# tile2d

tile2d, an improved version of its predecessor [kin2d](https://github.com/Kubic-C/kin2d), is a physics engine built specifically around tiles.
By employing multiple threads, it speeds up the processing of thousands of Rigid-Tile bodies.

The goal is to super-optimize tile-to-tile collisions, allowing for thousands or even tens of thousands of collisions
to happen at a time.

The data which is contained inside of tiles is entirely user-decided because of the use of templates. 

## Documentation

See [this](https://kubic-c.github.io/tile2d/index.html) for documentation.

For now, public documentation is mostly done, but it still needs work in some places. 
