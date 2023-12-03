extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate piston;

use glutin_window::GlutinWindow as Window;
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop::{EventSettings, Events};
use piston::input::{RenderArgs, RenderEvent, UpdateArgs, UpdateEvent};
use piston::window::WindowSettings;

use nalgebra::{Point2, Vector2, Matrix2};
use ordered_float::OrderedFloat;
use std::rc::Rc;
use std::cell::RefCell;
use std::f64::INFINITY;

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

use scene::Scene;
use object::Object;
use transform::Transform;
use material::Material;
use mass_data::MassData;
use kinematics::Kinematics;
use constants::BLACK;
use shapes::Shapes;
use circle::Circle;
use polygon::Polygon;

pub struct App {
    gl: GlGraphics, // OpenGL drawing backend.
    scene: Scene,
}

impl App {
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        self.gl.draw(args.viewport(), |c: Context, gl: &mut GlGraphics| {
            clear(BLACK, gl);

            self.scene.render(c, gl);
        });
    }

    fn update(&mut self, args: &UpdateArgs) {
        self.scene.step(args.dt);
    }
}

fn create_test_objects() -> Vec<Rc<RefCell<Object>>> {
    // Create a circle
    let circle = Object::new(
    Shapes::Circle(Circle { radius: OrderedFloat(50.0) } ),
        Transform::new(Point2::new(100.0, 100.0)),
        None,
        None,
        Some(Kinematics::new(Vector2::new(30.0, 0.0), 0.0, 0.0)),
    );

    // Create a circle
    let circle2 = Object::new(
        Shapes::Circle(Circle { radius: OrderedFloat(25.0) }),
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
            None
        )),
        Transform::new(Point2::new(600.0, 300.0)),
        None,
        None,
        Some(Kinematics::new(Vector2::zeros(), 0.0, 0.0)),
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
        Some(Material::new(INFINITY, OrderedFloat(1.0), OrderedFloat(1.0), OrderedFloat(1.0))),
        Some(MassData::new(INFINITY, INFINITY)),
        Some(Kinematics::new(Vector2::zeros(), 0.0, 0.0)),
    );

    [circle, circle2, convex_polygon, concave_polygon, floor].into_iter().map(| s | { Rc::new(RefCell::new(s)) }).collect()
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