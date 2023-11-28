use nalgebra::{Point2, Vector2, Rotation2};

struct AABB
{
    min: Point2<f64>,
    max: Point2<f64>,
}

struct Circle
{
    center: Point2<f64>,
    radius: Meter,
}

struct Pair<'a> {
    a: &'a Object,
    b: &'a Object,
}

struct BroadPhase<'a> {
    objects: Vec<& 'a Object>,
    pairs: Vec<Pair<'a>>
}

impl<'a> BroadPhase<'a> {
    fn generate_pairs(&mut self) {
        self.pairs.clear();

        for (i, a) in self.objects.iter().enumerate() {
            for b in self.objects.iter().skip(i + 1) {
                // Skip self-check
                if a as *const _ == b as *const _ {
                    continue;
                }
    
                let a_aabb = a.compute_aabb();
                let b_aabb = b.compute_aabb();
    
                if aabb_aabb(&a_aabb, &b_aabb) {
                    self.pairs.push(Pair { a: *a, b: *b });
                }
            }
        }
    }

    fn sort_pairs(&mut self) {
        self.pairs.sort_by(|lhs, rhs| {
            lhs.a as *const _.
                cmp(&rhs.a as *const _)
                .then_with(|| lhs.b as *const _.
                    cmp(&rhs.b as *const _))
        });
    }

    fn cull_duplicates(&mut self) {
        let mut unique_pairs = Vec::new();
        let mut i = 0;
    
        while i < self.pairs.len() {
            let pair = &self.pairs[i];
            unique_pairs.push(pair.clone());
            i += self.pairs[i + 1..]
                .iter()
                .position(|potential_dup| {
                    pair.a as *const _ == potential_dup.b as *const _
                        && pair.b as *const _ == potential_dup.a as *const _
                })
                .map_or(1, |pos| pos + 1);
        }
    
        self.pairs = unique_pairs;
    }    
}

fn aabb_aabb(a: &AABB, b: &AABB) -> bool {
    // Check if the two Axis-Aligned Bounding Boxes (AABB) intersect along any axis.
    // If there is separation along any axis, return false; otherwise, return true.
    !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y || a.min.y > b.max.y)
}

// TODO: Refactor this function
fn ResolveCollision(a: &Object, b: &Object) {
    let rel_vel = b.velocity - a.velocity;
    // TODO: Get normal
    let vel_norm = rel_vel.dot(&normal);

    // Make sure objects are moving towards one another
    if (vel_norm < 0) {
        let e = min(a.restitution, b.restitution);
        let j = -(1 + e) * vel_norm;

        j /= a.inv_mass + b.inv_mass;

        let impulse = j * normal;

        a.velocity -= a.inv_mass * impulse;
        b.velocity += b.inv_mass * impulse;

        // TODO: Might be simpler, check if it functions the same
        // let mass_sum = a.mass + b.mass;
        // let ratio = a.mass / mass_sum;
        // a.velocity -= ratio * impulse;
        // ratio = b.mass / mass_sum;
        // b.velocity += ratio * impulse;
    }
}