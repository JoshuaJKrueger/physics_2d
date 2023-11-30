extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate piston;

use glutin_window::GlutinWindow as Window;
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop::{EventSettings, Events};
use piston::input::{RenderArgs, RenderEvent, UpdateArgs, UpdateEvent};
use piston::window::WindowSettings;

use nalgebra::{Point2, Vector2};
use ordered_float::OrderedFloat;

mod collision;
mod manifold;
mod object;
mod scene;
mod types;

use scene::Scene;
use object::{Object, Shape, Transform, Material, MassData, Kinematics};

pub struct App<'a> {
    gl: GlGraphics, // OpenGL drawing backend.
    scene: Scene<'a>,
}

impl<'a> App<'a> {
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

        self.gl.draw(args.viewport(), |c: Context, gl: &mut GlGraphics| {
            clear(BLACK, gl);

            self.scene.render(c, gl);
        });
    }

    fn update(&mut self, args: &UpdateArgs) {
        self.scene.step(args.dt);
    }
}

fn create_test_objects() -> Vec<Object> {
    // Create a circle
    let circle = Object::new(
        Shape::Circle { radius: OrderedFloat(50.0) },
        Transform::new(Point2::new(100.0, 100.0)),
        Some(Material::new(1.0, OrderedFloat(0.5), OrderedFloat(0.3), OrderedFloat(0.2))),
        Some(MassData::new(5.0, 1000.0)),
        Some(Kinematics::new(Vector2::new(0.0, 0.0), 0.0, 0.0)),
    );

    // Create a convex polygon (triangle)
    let convex_polygon = Object::new(
        Shape::Polygon {
            vertices: vec![
                Point2::new(200.0, 200.0),
                Point2::new(250.0, 300.0),
                Point2::new(300.0, 200.0),
            ],
        },
        Transform::new(Point2::new(0.0, 0.0)), // Set the position accordingly
        Some(Material::new(1.0, OrderedFloat(0.5), OrderedFloat(0.3), OrderedFloat(0.2))),
        Some(MassData::new(5.0, 1000.0)),
        Some(Kinematics::new(Vector2::new(0.0, 0.0), 0.0, 0.0)),
    );

    // Create a concave polygon (hourglass shape)
    let concave_polygon = Object::new(
        Shape::Polygon {
            vertices: vec![
                Point2::new(350.0, 200.0),
                Point2::new(350.0, 300.0),
                Point2::new(500.0, 400.0),
                Point2::new(400.0, 300.0),
                Point2::new(500.0, 200.0),
            ],
        },
        Transform::new(Point2::new(0.0, 0.0)), // Set the position accordingly
        Some(Material::new(1.0, OrderedFloat(0.5), OrderedFloat(0.3), OrderedFloat(0.2))),
        Some(MassData::new(5.0, 1000.0)),
        Some(Kinematics::new(Vector2::new(0.0, 0.0), 0.0, 0.0)),
    );

    vec![circle, convex_polygon, concave_polygon]
}

fn main() {
    // Change this to OpenGL::V2_1 if not working.
    let opengl = OpenGL::V3_2;

    // Create a Glutin window.
    let mut window: Window = WindowSettings::new("Sandbox", [800, 600])
        .graphics_api(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    // Create a new game and run it.
    let mut app = App {
        gl: GlGraphics::new(opengl),
        scene: Scene { objects: create_test_objects(), contacts: Vec::new() }
    };

    let mut events = Events::new(EventSettings::new());
    while let Some(e) = events.next(&mut window) {
        if let Some(args) = e.update_args() {
            app.update(&args);
        }

        if let Some(args) = e.render_args() {
            app.render(&args);
        }
    }
}