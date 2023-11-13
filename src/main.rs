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

mod collision;
use collision::{Object, Shape};

pub struct App {
    gl: GlGraphics, // OpenGL drawing backend.
    objects: Vec<Object>,
}

impl App {
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
        const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

        self.gl.draw(args.viewport(), |c: Context, gl: &mut GlGraphics| {
            // Clear the screen.
            clear(BLACK, gl);

            for object in &self.objects {
                let transform = c.transform
                    .trans(object.position.x, object.position.y)
                    .rot_rad(object.angle);

                match &object.shape {
                    Shape::Circle { radius } => {
                        let circle = ellipse::circle(0.0, 0.0, *radius);
                        ellipse(WHITE, circle, transform, gl);
                    }
                    Shape::Rectangle { .. } => unimplemented!(),
                }
            }
        });
    }

    fn update(&mut self, args: &UpdateArgs) {
        for object in &mut self.objects {
            // TODO: Make core update
            object.position.x += object.velocity.x * args.dt;
            object.position.y += object.velocity.y * args.dt
        }
    }
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
        objects: Vec::new(),
    };

    // Add test objects
    let circle1 = Object {
        position: Point2::new(100.0, 100.0),
        velocity: Vector2::new(50.0, 0.0),
        angle: 0.0,
        shape: Shape::Circle { radius: 30.0 },
        mass: 1.0,
        restitution: 0.8,
        inv_mass: 1.0,
    };

    let circle2 = Object {
        position: Point2::new(200.0, 100.0),
        velocity: Vector2::new(-30.0, 0.0),
        angle: 0.0,
        shape: Shape::Circle { radius: 25.0 },
        mass: 1.0,
        restitution: 0.8,
        inv_mass: 1.0,
    };

    app.objects.push(circle1);
    app.objects.push(circle2);

    let mut events = Events::new(EventSettings::new());
    while let Some(e) = events.next(&mut window) {
        if let Some(args) = e.render_args() {
            app.render(&args);
        }

        if let Some(args) = e.update_args() {
            app.update(&args);
        }
    }
}