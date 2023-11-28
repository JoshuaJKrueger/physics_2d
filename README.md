# physics_2d #

This project is a simple physics sandbox implemented in Rust using the [Piston graphics crate](https://github.com/PistonDevelopers/graphics). The sandbox simulates basic physics interactions between objects with different shapes, such as circles. The main goal is to provide a starting point for experimenting with physics simulations in a 2D environment.

## Table of Contents ##

- [Building and Running](#building-and-running)
- [Usage](#usage)
- [Project Structure](#project-structure)

## Building and Running ##

1. Clone the repository
2. Build and run the project using Cargo: `cargo run`

## Usage ##

Upon running the project, a window titled "Sandbox" will open.
The sandbox contains two test objects: two circles moving toward one another.
They should move past each other as collision restitution has not been implemented yet.

## Project Structure ##

main.rs: Contains the main application code, including the setup of the Piston window, the main game loop, and the simulation logic.
collision.rs: Defines collision-related structures and functions, including Axis-Aligned Bounding Box (AABB) and circle-to-circle collision detection.
