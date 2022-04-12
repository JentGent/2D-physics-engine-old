let DEBUG = !true;
const E = (() => {

const SLOP = 0,
      SLOP_PERCENT = 1,
      MAX_POSITION_CHANGE = 100,

      MAX_VELOCITY_CHANGE = 1,
      VELOCITY_CHANGE_SCALE = 0.05;

const bodies = [];

const sq = e => e * e;
const constrain = (a, b, c) => Math.min(c, Math.max(b, a));
const mod = (n, m) => n - m * Math.floor(n / m);

// Shapes
const Circle = (() => {
    const proto = {
        Type() { return "Circle"; },
        ToLocal(v) { return V2D.Sub(v, this.position); },
        ToWorld(v) { return V2D.Add(v, this.position); },
    };
    return ({ position = [0, 0], radius = 10 } = {}) => Object.setPrototypeOf({ position, radius, bound: radius }, proto);
})();
const Box = (() => {
    const proto = {
        Type() { return "Box"; },
        ToLocal(v) { return V2D.Sub(v, this.position); },
        ToWorld(v) { return V2D.Add(v, this.position); },
    };
    return ({ position = [0, 0], size = [1, 1] } = {}) => Object.setPrototypeOf({ position, size, bound: Math.sqrt(sq(size[0]) + sq(size[1])) }, proto);
})();
const ConvexPolygon = (() => {
    const proto = {
        Type() { return "ConvexPolygon"; },
        ToLocal(v) { return V2D.Sub(v, this.position); },
        ToWorld(v) { return V2D.Add(v, this.position); },
    };
    return ({ position = [0, 0], vertices = [] } = {}) => Object.setPrototypeOf({ position, vertices, bound: (() => {
        let m = 0;
        for(let i = 0; i < vertices.length; i += 1) {
            const v = vertices[i], d = sq(v[0]) + sq(v[1]);
            if(d > m) m = d;
        }
        return Math.sqrt(m);
    })() }, proto);
})();

// Intersection detection & contact point calculation (body x body)
{
    // Box x circle
    function boxCircle(box, circle, f = 1) {
        const boxShape = box.shape, circleShape = circle.shape;
        const sx1 = boxShape.position[0] + box.position[0], sy1 = boxShape.position[1] + box.position[1];
        const sx2 = circleShape.position[0] + circle.position[0], sy2 = circleShape.position[1] + circle.position[1];
        const c = V2D.iRotate([sx2, sy2], box.position, -box.rotation);
        const b = [constrain(c[0], sx1 - boxShape.size[0], sx1 + boxShape.size[0]), constrain(c[1], sy1 - boxShape.size[1], sy1 + boxShape.size[1])];
        let d = sq(b[0] - c[0]) + sq(b[1] - c[1]);
        if(d > sq(circleShape.radius)) return false;
        if(!d) {
            const left = c[0] - sx1 + boxShape.size[0], right = sx1 + boxShape.size[0] - c[0], up = c[1] - sy1 + boxShape.size[1], down = sy1 + boxShape.size[1] - c[1];
            d = Math.min(left, right, up, down);
            let normal = [], j = circleShape.radius - (circleShape.radius + d) / 2;
            switch(d) {
                case left: normal = V2D.iRotate([1, 0], [0, 0], box.rotation);
                break;
                case right: normal = V2D.iRotate([-1, 0], [0, 0], box.rotation);
                break;
                case up: normal = V2D.iRotate([0, 1], [0, 0], box.rotation);
                break;
                case down: normal = V2D.iRotate([0, -1], [0, 0], box.rotation);
                break;
            }
            return [{
                contactPoint: [
                    sx2 + normal[0] * j,
                    sy2 + normal[1] * j
                ],
                normal: V2D.iScale(normal, f),
                penetrationDepth: (j + d) * 2,
            }]
        }
        d = Math.sqrt(d);
        const id = 1 / d;
        const normal = V2D.iRotate([(b[0] - c[0]) * id, (b[1] - c[1]) * id], [0, 0], box.rotation);
        const j = (circleShape.radius + d) / 2;
        return [{
            contactPoint: [
                sx2 + normal[0] * j,
                sy2 + normal[1] * j
            ],
            normal: V2D.iScale(normal, f),
            penetrationDepth: circleShape.radius - d,
        }];
    }

    // Circle x circle
    function circleCircle(body1, body2, f = 1) {
        const shape1 = body1.shape, shape2 = body2.shape;
        const sx1 = shape1.position[0] + body1.position[0], sy1 = shape1.position[1] + body1.position[1];
        const sx2 = shape2.position[0] + body2.position[0], sy2 = shape2.position[1] + body2.position[1];
        let d = sq(sx1 - sx2) + sq(sy1 - sy2);
        if(d > sq(shape1.radius + shape2.radius)) return false;
        d = Math.sqrt(d);
        const id = 1 / d;
        const normal = [(sx1 - sx2) * id, (sy1 - sy2) * id];
        const j = (shape2.radius + d - shape1.radius) / 2;
        return [{
            contactPoint: [
                sx2 + normal[0] * j,
                sy2 + normal[1] * j
            ],
            normal: V2D.Scale(normal, f),
            penetrationDepth: shape1.radius + shape2.radius - d,
        }, {
            contactPoint: [
                sx2 + normal[0] * j,
                sy2 + normal[1] * j
            ],
            normal: V2D.Scale(normal, f),
            penetrationDepth: shape1.radius + shape2.radius - d,
        }];
    }

    // Convex x circle
    function convexCircle(convex, circle, f = 1) {
        const shape1 = convex.shape, shape2 = circle.shape;
        const cp = shape1.ToLocal(convex.ToLocal(V2D.Add(circle.position, shape2.position)));
        const p = shape1.vertices;
        const flip = V2D.Cross(V2D.Sub(p[1], p[0]), V2D.Sub(p[2], p[1])) > 0;
        let penetration = Infinity, point = [0, 0], normal = [1, 0];
        for(let i = 0; i < p.length; i += 1) {
            const v1 = p[i], v2 = p[i === p.length - 1 ? 0 : i + 1];
            const edgeNormal = (v => (flip ? V2D.iScale(v, -1) : v))([v1[1] - v2[1], v2[0] - v1[0]]);
            const tangent = V2D.Sub(v2, v1), sqMag = V2D.SqMag(tangent), invSqMag = 1 / sqMag;
            const dot = V2D.Dot(V2D.Sub(cp, v1), edgeNormal);
            if(dot > shape2.radius * sqMag) return false; // Circle outside poly
            const projectedInter = constrain(V2D.Dot(V2D.Sub(cp, v1), tangent) * invSqMag, 0, 1);
            const projectedPoint = V2D.Add(v1, V2D.Scale(tangent, projectedInter));
            const n = V2D.Sub(projectedPoint, cp);
            const sqDist = V2D.SqMag(n);
            if(sqDist > shape2.radius * shape2.radius) continue; // Circle not touching edge
            const sign = Math.sign(dot);
            const edgePenetration = shape2.radius - Math.sqrt(sqDist) * sign;
            if(edgePenetration < penetration) {
                penetration = edgePenetration;
                point = projectedPoint;
                normal = V2D.iScale(n, sign);
            }
        }
        if(penetration === Infinity) return false;
        V2D.iNormalize(V2D.iRotate(normal, [0, 0], convex.rotation));
        point = V2D.iSub(convex.ToWorld(shape1.ToWorld(point)), V2D.Scale(normal, -penetration / 2));
        return [{
            penetrationDepth: penetration, contactPoint: point, normal: V2D.iScale(normal, f)
        }];
    }

    // Box x box
    function boxBox(body1, body2, f = 1) {
        const shape1 = body1.shape, shape2 = body2.shape;
        const sx1 = shape1.position[0] + body1.position[0], sy1 = shape1.position[1] + body1.position[1];
        const sx2 = shape2.position[0] + body2.position[0], sy2 = shape2.position[1] + body2.position[1];
        const p1 = [[sx1 - shape1.size[0], sy1 - shape1.size[1]], [sx1 + shape1.size[0], sy1 - shape1.size[1]], [sx1 + shape1.size[0], sy1 + shape1.size[1]], [sx1 - shape1.size[0], sy1 + shape1.size[1]]];
        const p2 = [[sx2 - shape2.size[0], sy2 - shape2.size[1]], [sx2 + shape2.size[0], sy2 - shape2.size[1]], [sx2 + shape2.size[0], sy2 + shape2.size[1]], [sx2 - shape2.size[0], sy2 + shape2.size[1]]];
        for(let i = 0; i < 4; i += 1) {
            V2D.iRotate(p1[i], body1.position, body1.rotation);
            V2D.iRotate(p2[i], body2.position, body2.rotation);
        }
        return polyPoly(p1, p2, f);
    }

    // Convex x box
    function convexBox(convex, box, f = 1) {
        const shape1 = convex.shape, shape2 = box.shape;
        const sx2 = shape2.position[0] + box.position[0], sy2 = shape2.position[1] + box.position[1];
        const p1 = [];
        for(let i = 0; i < shape1.vertices.length; i += 1) {
            p1[i] = V2D.iRotate(V2D.iAdd(V2D.Add(shape1.vertices[i], convex.position), shape1.position), convex.position, convex.rotation);
        }
        const p2 = [[sx2 - shape2.size[0], sy2 - shape2.size[1]], [sx2 + shape2.size[0], sy2 - shape2.size[1]], [sx2 + shape2.size[0], sy2 + shape2.size[1]], [sx2 - shape2.size[0], sy2 + shape2.size[1]]];
        for(let i = 0; i < 4; i += 1) {
            V2D.iRotate(p2[i], box.position, box.rotation);
        }
        return polyPoly(p1, p2, f);
    }

    // Convex polygon x convex polygon
    function penetration(poly1, poly2, flip) {
        let penetration = Infinity, point = [0, 0], normal = [1, 0];
        for(let i = 0; i < poly1.length; i += 1) {
            const v1 = poly1[i], v2 = poly1[i === poly1.length - 1 ? 0 : i + 1];
            const edgeNormal = (v => (flip ? V2D.iScale(v, -1) : v))([v1[1] - v2[1], v2[0] - v1[0]]);
            const tangent = V2D.Sub(v2, v1), invSqMag = 1 / V2D.SqMag(tangent);
            let edgePenetration = -Infinity, edgePoint = [];
            for(let j = 0; j < poly2.length; j += 1) {
                const v = poly2[j];
                if(V2D.Dot(V2D.Sub(v, v1), edgeNormal) > 0) continue; // Point outside poly1
                const projectedInter = V2D.Dot(V2D.Sub(v, v1), tangent) * invSqMag;
                // if(projectedInter < 0 || projectedInter > 1) continue; // Point not within poly1 edge
                const projectedPoint = V2D.Add(v1, V2D.Scale(tangent, projectedInter));
                const sqPenetration = V2D.SqMag(V2D.Sub(v, projectedPoint));
                if(sqPenetration > edgePenetration) {
                    edgePenetration = sqPenetration;
                    edgePoint = projectedPoint;
                }
            }
            if(edgePenetration < penetration) {
                penetration = edgePenetration;
                point = edgePoint;
                normal = edgeNormal;
            }
        }
        return { penetration: Math.sqrt(penetration), point: point, normal: V2D.iNormalize(normal) };
    }
    function SAT(axis, pol1, pol2) {
        let min1 = min2 = Infinity, max1 = max2 = -Infinity;
        for(let i = 0; i < Math.max(pol2.length, pol1.length); i += 1) {
            if(i < pol1.length) {
                const d = V2D.Dot(axis, pol1[i]);
                min1 = Math.min(min1, d);
                max1 = Math.max(max1, d);
            }
            if(i < pol2.length) {
                const d = V2D.Dot(axis, pol2[i]);
                min2 = Math.min(min2, d);
                max2 = Math.max(max2, d);
            }
            if((min1 < max2 && max1 > min2) || (min2 < max1 && max2 > min1)) return true;
        }
        return false;
    };
    function polyPoly(p1, p2, f = 1) {
        for(let i = 0; i < p1.length; i += 1) {
            const v1 = p1[i], v2 = p1[(i + 1) % p1.length], v = [v2[1] - v1[1], v1[0] - v2[0]];
            if(!SAT(v, p1, p2)) return false;
        }
        for(let i = 0; i < p2.length; i += 1) {
            const v1 = p2[i], v2 = p2[(i + 1) % p2.length], v = [v2[1] - v1[1], v1[0] - v2[0]];
            if(!SAT(v, p1, p2)) return false;
        }

        const flip1 = V2D.Cross(V2D.Sub(p1[1], p1[0]), V2D.Sub(p1[2], p1[1])) > 0, flip2 = V2D.Cross(V2D.Sub(p2[1], p2[0]), V2D.Sub(p2[2], p2[1])) > 0;
        const { penetration: penetration1, point: point1, normal: normal1 } = penetration(p1, p2, flip1);
        const { penetration: penetration2, point: point2, normal: normal2 } = penetration(p2, p1, flip2);
        if((!point2[0] && point2[0] !== 0) || penetration1 < penetration2) {
            return [{
                contactPoint: [
                    point1[0] - normal1[0] * penetration1 / 2,
                    point1[1] - normal1[1] * penetration1 / 2,
                ],
                normal: V2D.Scale(normal1, -f),
                penetrationDepth: penetration1
            }];
        }
        return [{
            contactPoint: [
                point2[0] - normal2[0] * penetration2 / 2,
                point2[1] - normal2[1] * penetration2 / 2,
            ],
            normal: V2D.Scale(normal2, f),
            penetrationDepth: penetration2
        }];
    }
    function convexConvex(body1, body2, f = 1) {
        const p1 = [], p2 = [];
        for(let i = 0; i < body1.shape.vertices.length; i += 1) {
            p1[i] = V2D.iRotate(V2D.iAdd(V2D.Add(body1.shape.vertices[i], body1.position), body1.shape.position), body1.position, body1.rotation);
        }
        for(let i = 0; i < body2.shape.vertices.length; i += 1) {
            p2[i] = V2D.iRotate(V2D.iAdd(V2D.Add(body2.shape.vertices[i], body2.position), body2.shape.position), body2.position, body2.rotation);
        }
        for(let i = 0; i < p1.length; i += 1) {
            const v1 = p1[i], v2 = p1[(i + 1) % p1.length], v = [v2[1] - v1[1], v1[0] - v2[0]];
            if(!SAT(v, p1, p2)) return false;
        }
        for(let i = 0; i < p2.length; i += 1) {
            const v1 = p2[i], v2 = p2[(i + 1) % p2.length], v = [v2[1] - v1[1], v1[0] - v2[0]];
            if(!SAT(v, p1, p2)) return false;
        }

        const flip1 = V2D.Cross(V2D.Sub(p1[1], p1[0]), V2D.Sub(p1[2], p1[1])) > 0, flip2 = V2D.Cross(V2D.Sub(p2[1], p2[0]), V2D.Sub(p2[2], p2[1])) > 0;
        const { penetration: penetration1, point: point1, normal: normal1 } = penetration(p1, p2, flip1);
        const { penetration: penetration2, point: point2, normal: normal2 } = penetration(p2, p1, flip2);
        if((!point2[0] && point2[0] !== 0) || penetration1 < penetration2) {
            return [{
                contactPoint: [
                    point1[0] - normal1[0] * penetration1 / 2,
                    point1[1] - normal1[1] * penetration1 / 2,
                ],
                normal: V2D.Scale(normal1, -1),
                penetrationDepth: penetration1
            }];
        }
        return [{
            contactPoint: [
                point2[0] - normal2[0] * penetration2 / 2,
                point2[1] - normal2[1] * penetration2 / 2,
            ],
            normal: V2D.iScale(normal2, f),
            penetrationDepth: penetration2
        }];
    }
}
function collisionData(body1, body2) {
    const shape1 = body1.shape, shape2 = body2.shape;
    switch(shape1.Type()) {
        case "Box":
            switch(shape2.Type()) {
                case "Circle": return boxCircle(body1, body2);
                break;
                case "Box": return boxBox(body1, body2);
                break;
                case "ConvexPolygon": return convexBox(body2, body1, -1);
                break;
            }
        break;
        case "Circle":
            switch(shape2.Type()) {
                case "Circle": return circleCircle(body1, body2);
                break;
                case "Box": return boxCircle(body2, body1, -1);
                break;
                case "ConvexPolygon": return convexCircle(body2, body1, -1);
                break;
            }
        break;
        case "ConvexPolygon":
            switch(shape2.Type()) {
                case "Circle": return convexCircle(body1, body2);
                break;
                case "Box": return convexBox(body1, body2);
                break;
                case "ConvexPolygon": return convexConvex(body1, body2);
                break;
            }
        break;
    }
}

// Rigid body
const RigidBody = (() => {
    const proto = {
        Instance() { return "RigidBody"; },
        ApplyForce(a, b) { // ([linear]) or ([linear], angular) or ([origin], [vector])
            if(b && b.length) {
                V2D.iAdd(this.acceleration, V2D.Scale(b, this.invMass));
                this.angularAcceleration += V2D.Cross(b, V2D.Sub(this.position, a)) * this.invMomentOfInertia;
            }
            else {
                V2D.iAdd(this.acceleration, V2D.Scale(a, this.invMass));
                if(typeof b === "number") this.angularAcceleration += b * this.invMomentOfInertia;
            }
        },
        VelocityAtPoint(v) {
            const r = Math.sqrt(sq(v[0] - this.position[0]) + sq(v[1] - this.position[1]));
            return V2D.Add(this.velocity, [(this.position[1] - v[1]) * this.angularVelocity, (v[0] - this.position[0]) * this.angularVelocity]);
        },
        Momentum() { return V2D.Mag(this.velocity) * this.mass; },
        ToLocal(v) { return V2D.iRotate(V2D.Sub(v, this.position), [0, 0], -this.rotation); },
        ToWorld(v) { return V2D.iAdd(V2D.Rotate(v, [0, 0], this.rotation), this.position); },
        IntersectingPoint(point) {
            switch(this.shape.Type()) {
                case "Box":{
                    const localPoint = this.shape.ToLocal(this.ToLocal(point));
                    return localPoint[0] > -this.shape.size[0] && localPoint[1] > -this.shape.size[1] && localPoint[0] < this.shape.size[0] && localPoint[1] < this.shape.size[1];
                }break;
                case "Circle":{
                    return V2D.SqMag(V2D.iSub(V2D.Sub(point, this.position), this.shape.position)) < this.shape.radius * this.shape.radius;
                }break;
                case "ConvexPolygon":{
                    const localPoint = this.shape.ToLocal(this.ToLocal(point));
                    let dot = 0;
                    const vs = this.shape.vertices;
                    for(let i = 0; i < vs.length; i += 1) {
                        const v1 = vs[i], v2 = vs[(i || vs.length) - 1];
                        const c = V2D.Cross(V2D.Sub(v2, v1), V2D.Sub(localPoint, v1));
                        if((c > 0 && dot < 0) || (c < 0 && dot > 0)) return false;
                        dot = c;
                    }
                    return true;
                }break;
            }
        },
        set mass(a) {
            this.mass = a;
            this.invMass = 1 / a;
        },
        set momentOfInertia(a) {
            this.momentOfInertia = a;
            this.invMomentOfInertia = 1 / a;
        },
    };
    return ({ position = [0, 0], velocity = [0, 0], angularVelocity = 0, rotation = 0, mass = 1, momentOfInertia = sq(mass) * 2, shape = Circle(), staticFriction = 0.8, dynamicFriction = 0.8, elasticity = 0.1, static = false } = {}, indices = [0]) => {
        const body = Object.setPrototypeOf({ position, velocity, acceleration: [0, 0], angularVelocity, angularAcceleration: 0, rotation, mass, invMass: 1 / mass, momentOfInertia, invMomentOfInertia: 1 / momentOfInertia, shape, staticFriction, dynamicFriction, elasticity, static }, proto);
        for(let i = 0; i < indices.length; i += 1) {
            if(!bodies[indices[i]]) bodies[indices[i]] = [];
            bodies[indices[i]].push(body);
        }
        return body;
    };
})();

function run(dt = 16, iterations = 1) {
    const invDt = 1 / dt;
    dt /= iterations;
for(let iter = 0; iter < iterations; iter += 1) {
    for(let i = 0; i < bodies.length; i += 1) {
        const layer = bodies[i];
        for(let j = 0; j < layer.length; j += 1) {
            const body = layer[j];
            V2D.iAdd(body.position, V2D.Scale(body.velocity, dt));
            V2D.iAdd(body.velocity, V2D.Scale(body.acceleration, dt));
            body.rotation += body.angularVelocity * dt;
            body.angularVelocity += body.angularAcceleration * dt;
            body.acceleration[0] = body.acceleration[1] = body.angularAcceleration = 0;
        }
    }
    const changes = [];
    const forces = [];
    for(let i = 0; i < bodies.length; i += 1) {
        const layer = bodies[i];
        changes[i] = [];
        for(let j = 0; j < layer.length; j += 1) {
            let body1 = layer[j];
            for(let k = j + 1; k < layer.length; k += 1) {
                let body2 = layer[k];
                if(body1.static && body2.static) { continue; }
                if(V2D.SqMag(V2D.iSub(V2D.Add(body1.shape.position, body1.position), V2D.Add(body2.shape.position, body2.position))) > sq(body1.shape.bound + body2.shape.bound)) continue;
                const contactPoints = collisionData(body1, body2);
                if(!contactPoints || !contactPoints.length) continue;
                if(DEBUG) {
                    for(let l = 0; l < contactPoints.length; l += 1) {
                        const { contactPoint, normal, penetrationDepth } = contactPoints[l];
                        stroke(255, 255, 0);
                        strokeWeight(2);
                        line(contactPoint[0] - normal[0] * penetrationDepth / 2, contactPoint[1] - normal[1] * penetrationDepth / 2, contactPoint[0] + normal[0] * penetrationDepth / 2, contactPoint[1] + normal[1] * penetrationDepth / 2);
                        stroke(255, 0, 0);
                        strokeWeight(1);
                        line(contactPoint[0] + normal[0] * -penetrationDepth / 2, contactPoint[1] + normal[1] * -penetrationDepth / 2, contactPoint[0] + normal[0] * (-penetrationDepth / 2 + 8), contactPoint[1] + normal[1] * (-penetrationDepth / 2 + 8));
                    }
                }
                if(!body1.static && !body2.static) {
                    // nonstatic v nonstatic
                    const massRatio = body1.mass / (body1.mass + body2.mass);
                    for(let l = 0; l < contactPoints.length; l += 1) {
                        let { contactPoint, normal, penetrationDepth } = contactPoints[l];
                        penetrationDepth = Math.max(penetrationDepth - SLOP, 0) * SLOP_PERCENT;
                        const coefficientOfRestitution = Math.min(body1.elasticity, body2.elasticity) + 1;
                        const contactVector1 = V2D.Sub(contactPoint, body1.position), contactVector2 = V2D.Sub(contactPoint, body2.position);
                        const contactVelocity1 = body1.VelocityAtPoint(contactPoint), contactVelocity2 = body2.VelocityAtPoint(contactPoint);
                        const velocityDifference = V2D.Sub(contactVelocity2, contactVelocity1);

                        // Collision impulses
                        const impulse = Math.max(0, coefficientOfRestitution * V2D.Dot(normal, velocityDifference) / (body1.invMass + body2.invMass + sq(V2D.Cross(contactVector1, normal)) * body1.invMomentOfInertia + sq(V2D.Cross(contactVector2, normal)) * body2.invMomentOfInertia));
                        // const impulse = Math.max(0, coefficientOfRestitution * V2D.Dot(normal, velocityDifference) / (body1.invMass + body2.invMass));
                        changes[i].push([body1, V2D.Scale(normal, penetrationDepth * (1 - massRatio)), contactPoint, V2D.Scale(normal, impulse)]);
                        changes[i].push([body2, V2D.Scale(normal, -penetrationDepth * massRatio), contactPoint, V2D.Scale(normal, -impulse)]);

                        // Friction impulse
                        const frictionVector = V2D.iNormalize(V2D.Sub(velocityDifference, V2D.Scale(normal, V2D.Dot(velocityDifference, normal))));
                        if((!frictionVector[0] && frictionVector[0] !== 0) || (!frictionVector[1] && frictionVector[1] !== 0)) continue;
                        let frictionImpulse = Math.max(0, V2D.Dot(velocityDifference, frictionVector) / (body1.invMass + body2.invMass + sq(V2D.Cross(contactVector1, frictionVector)) * body1.invMomentOfInertia + sq(V2D.Cross(contactVector2, frictionVector)) * body2.invMomentOfInertia));
                        const friction = V2D.Mag([body1.staticFriction, body2.staticFriction]);
                        if(frictionImpulse > impulse * friction) frictionImpulse = impulse * V2D.Mag([body1.dynamicFriction, body2.dynamicFriction]);
                        changes[i].push([body1, [0, 0], contactPoint, V2D.Scale(frictionVector, frictionImpulse)]);
                        changes[i].push([body2, [0, 0], contactPoint, V2D.Scale(frictionVector, -frictionImpulse)]);
                    }
                }
                else if(body2.static) {
                    // nonstatic v static
                    for(let l = 0; l < contactPoints.length; l += 1) {
                        let { contactPoint, normal, penetrationDepth } = contactPoints[l];
                        penetrationDepth = Math.max(penetrationDepth - SLOP, 0) * SLOP_PERCENT;
                        const coefficientOfRestitution = Math.min(body1.elasticity, body2.elasticity) + 1;
                        const contactVector1 = V2D.Sub(contactPoint, body1.position), contactVector2 = V2D.Sub(contactPoint, body2.position);
                        const contactVelocity1 = body1.VelocityAtPoint(contactPoint), contactVelocity2 = body2.VelocityAtPoint(contactPoint);
                        const velocityDifference = V2D.Sub(contactVelocity2, contactVelocity1);

                        // Collision impulse
                        const impulse = Math.max(0, coefficientOfRestitution * V2D.Dot(normal, velocityDifference) / (body1.invMass + sq(V2D.Cross(V2D.Sub(contactPoint, body1.position), normal)) * body1.invMomentOfInertia));
                        // const impulse = Math.max(0, coefficientOfRestitution * V2D.Dot(normal, velocityDifference) / (body1.invMass));
                        changes[i].push([body1, V2D.Scale(normal, penetrationDepth), contactPoint, V2D.Scale(normal, impulse)]);

                        // Friction impulse
                        const frictionVector = V2D.iNormalize(V2D.Sub(velocityDifference, V2D.Scale(normal, V2D.Dot(velocityDifference, normal))));
                        if((!frictionVector[0] && frictionVector[0] !== 0) || (!frictionVector[1] && frictionVector[1] !== 0)) continue;
                        let frictionImpulse = Math.max(0, V2D.Dot(velocityDifference, frictionVector) / (body1.invMass + sq(V2D.Cross(contactVector1, frictionVector)) * body1.invMomentOfInertia));
                        const friction = V2D.Mag([body1.staticFriction, body2.staticFriction]);
                        if(frictionImpulse > impulse * friction) frictionImpulse = impulse * V2D.Mag([body1.dynamicFriction, body2.dynamicFriction]);
                        changes[i].push([body1, [0, 0], contactPoint, V2D.Scale(frictionVector, frictionImpulse)]);
                    }
                }
                else {
                    // static v nonstatic
                    for(let l = 0; l < contactPoints.length; l += 1) {
                        let { contactPoint, normal, penetrationDepth } = contactPoints[l];
                        penetrationDepth = Math.max(penetrationDepth - SLOP, 0) * SLOP_PERCENT;
                        const coefficientOfRestitution = Math.min(body1.elasticity, body2.elasticity) + 1;
                        const contactVector1 = V2D.Sub(contactPoint, body1.position), contactVector2 = V2D.Sub(contactPoint, body2.position);
                        const contactVelocity1 = body1.VelocityAtPoint(contactPoint), contactVelocity2 = body2.VelocityAtPoint(contactPoint);
                        const velocityDifference = V2D.Sub(contactVelocity2, contactVelocity1);

                        // Collision impulse
                        const impulse = Math.max(0, coefficientOfRestitution * V2D.Dot(normal, velocityDifference) / (body2.invMass + sq(V2D.Cross(V2D.Sub(contactPoint, body2.position), normal)) * body2.invMomentOfInertia));
                        // const impulse = Math.max(0, coefficientOfRestitution * V2D.Dot(normal, velocityDifference) / (body2.invMass));
                        changes[i].push([body2, V2D.Scale(normal, -penetrationDepth), contactPoint, V2D.Scale(normal, -impulse)]);

                        // Friction impulse
                        const frictionVector = V2D.iNormalize(V2D.Sub(velocityDifference, V2D.Scale(normal, V2D.Dot(velocityDifference, normal))));
                        if((!frictionVector[0] && frictionVector[0] !== 0) || (!frictionVector[1] && frictionVector[1] !== 0)) continue;
                        let frictionImpulse = Math.max(0, V2D.Dot(velocityDifference, frictionVector) / (body2.invMass + sq(V2D.Cross(contactVector2, frictionVector)) * body2.invMomentOfInertia));
                        const friction = V2D.Mag([body1.staticFriction, body2.staticFriction]);
                        if(frictionImpulse > impulse * friction) frictionImpulse = impulse * V2D.Mag([body1.dynamicFriction, body2.dynamicFriction]);
                        changes[i].push([body2, [0, 0], contactPoint, V2D.Scale(frictionVector, -frictionImpulse)]);
                    }
                }
            }
        }
    }
    for(let i = 0; i < changes.length; i += 1) {
        const layer = changes[i];
        for(let j = 0; j < layer.length; j += 1) {
            layer[j][0].ApplyForce(layer[j][2], V2D.iScale(layer[j][3], invDt));
            const vd = V2D.Mag(layer[j][1]);
            if(!vd) continue;
            V2D.iAdd(layer[j][0].velocity, V2D.Scale(layer[j][1], Math.min(MAX_VELOCITY_CHANGE, vd) * VELOCITY_CHANGE_SCALE / vd));
        }
        if(MAX_POSITION_CHANGE) {
            for(let j = 0; j < layer.length; j += 1) {
                const vd = V2D.Mag(layer[j][1]);
                if(!vd) continue;
                V2D.iAdd(layer[j][0].position, V2D.Scale(layer[j][1], Math.min(MAX_POSITION_CHANGE, vd) / vd));
            }
        }
    }
}
}

return { bodies, RigidBody, shape: { Circle, Box, ConvexPolygon }, run };

})();
