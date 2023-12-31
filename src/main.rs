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
use std::cell::RefCell;
use std::f64::INFINITY;
use std::rc::Rc;

mod circle;
mod collision;
mod constants;
mod custom_math;
mod kinematics;
mod manifold;
mod mass_data;
mod material;
mod object;
mod polygon;
mod scene;
mod shapes;
mod transform;
mod types;

use circle::Circle;
use constants::BLACK;
use kinematics::Kinematics;
use mass_data::MassData;
use material::Material;
use object::Object;
use polygon::Polygon;
use scene::Scene;
use shapes::Shapes;
use transform::Transform;

/// Represents the application's main structure, including the OpenGL backend and scene.
pub struct App {
    /// The OpenGL drawing backend.
    gl: GlGraphics,
    /// The scene containing objects and contacts.
    scene: Scene,
}

impl App {
    /// Renders the scene using the specified rendering arguments.
    ///
    /// # Arguments
    ///
    /// * `args` - Rendering arguments containing viewport information.
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        self.gl
            .draw(args.viewport(), |c: Context, gl: &mut GlGraphics| {
                clear(BLACK, gl);

                self.scene.render(c, gl);
            });
    }

    /// Updates the scene based on the specified update arguments.
    ///
    /// # Arguments
    ///
    /// * `args` - Update arguments containing the time delta (`dt`).
    fn update(&mut self, args: &UpdateArgs) {
        self.scene.step(args.dt);
    }
}

/// Creates test objects for the scene and returns them as a vector of `Object` references.
fn create_test_objects() -> Vec<Rc<RefCell<Object>>> {
    // Create a circle
    let circle = Object::new(
        Shapes::Circle(Circle {
            radius: OrderedFloat(50.0),
        }),
        Transform::new(Point2::new(100.0, 100.0)),
        None,
        None,
        Some(Kinematics::new(Vector2::new(30.0, 0.0), 0.0, 0.0)),
    );

    // Create a circle
    let circle2 = Object::new(
        Shapes::Circle(Circle {
            radius: OrderedFloat(25.0),
        }),
        Transform::new(Point2::new(100.0, 300.0)),
        None,
        None,
        Some(Kinematics::new(Vector2::new(30.0, -50.0), 0.0, 0.0)),
    );

    // Create a convex polygon (triangle)
    let convex_polygon = Object::new(
        Shapes::Polygon(Polygon::new(
            vec![
                Point2::new(200.0, 200.0),
                Point2::new(250.0, 300.0),
                Point2::new(300.0, 200.0),
            ],
            None,
        )),
        Transform::new(Point2::new(400.0, 200.0)),
        None,
        None,
        Some(Kinematics::new(Vector2::new(-20.0, -20.0), 0.0, 0.0)),
    );

    // Create a concave polygon
    let concave_polygon = Object::new(
        Shapes::Polygon(Polygon::new(
            vec![
                Point2::new(350.0, 200.0),
                Point2::new(350.0, 300.0),
                Point2::new(500.0, 400.0),
                Point2::new(400.0, 300.0),
                Point2::new(500.0, 200.0),
            ],
            None,
        )),
        Transform::new(Point2::new(600.0, 300.0)),
        None,
        None,
        Some(Kinematics::new(Vector2::new(-30.0, -30.0), 0.0, 0.0)),
    );

    // Create a floor
    let floor = Object::new(
        Shapes::Polygon(Polygon::new(
            vec![
                Point2::new(10.0, 580.0),
                Point2::new(790.0, 580.0),
                Point2::new(790.0, 590.0),
                Point2::new(10.0, 590.0),
            ],
            None,
        )),
        Transform::new(Point2::new(0.0, 0.0)),
        Some(Material::new(
            INFINITY,
            OrderedFloat(1.0),
            OrderedFloat(1.0),
            OrderedFloat(1.0),
        )),
        Some(MassData::new(INFINITY, INFINITY)),
        Some(Kinematics::new(Vector2::zeros(), 0.0, 0.0)),
    );

    [circle, circle2, convex_polygon, concave_polygon, floor]
        .into_iter()
        .map(|s| Rc::new(RefCell::new(s)))
        .collect()
}

/// The main function responsible for creating the window, initializing the application,
/// and handling the main game loop.
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
        scene: Scene {
            objects: create_test_objects(),
            contacts: Vec::new(),
        },
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
