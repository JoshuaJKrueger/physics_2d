# physics_2d #

This project is a simple physics sandbox implemented in Rust using the [Piston graphics crate](https://github.com/PistonDevelopers/graphics). The sandbox simulates basic physics interactions between objects. The main goal is to provide a starting point for experimenting with physics simulations in a 2D environment.

## Building and Running ##

1. Clone the repository
2. Build and run the project using Cargo: `cargo run`

## Usage ##

Upon running the project, a window titled "Sandbox" will open.
The sandbox contains a few test objects with varying properties.
You can see how they interact, and modify them to create new situations.

## Demonstration ##

<figure class="video_container">
    <video controls="true" allowfullscreen="true">
        <source src="demonstration.mp4", type="video/mp4">
    </video>
</figure>

## Credits ##

- Most of the documentation comments were generated with chatGPT
- I tried to mark the functions I adapted from this [C++ 2d physics tutorial](https://code.tutsplus.com/series/how-to-create-a-custom-physics-engine--gamedev-12715), but I am also stating its influence here incase I missed any. I also got some concepts from the tutorial, such as using a manifold to store the object interactions.
