
// Setup
function setup() {
    createCanvas(600, 600);
    angleMode(RADIANS);
    frameRate(60);
}


// Keys
function keyReleased() {
    DEBUG = !DEBUG;
}

E.RigidBody({
    position: [300, -75],
    shape: E.shape.Box({ size: [600, 100] }),
    rotation: 0,
    static: 1
});
E.RigidBody({
    position: [300, 600],
    shape: E.shape.Box({ size: [600, 100] }),
    rotation: 0.1,
    static: 1
});
E.RigidBody({
    position: [-50, 300],
    shape: E.shape.Box({ size: [100, 600] }),
    static: 1
});
E.RigidBody({
    position: [650, 300],
    shape: E.shape.Box({ size: [100, 600] }),
    static: 1
});


function randomShape(m, M) {
    const poly = [];
    const d = Math.random(M - m) * 30 + m;
    let j = 0;
    while(j < Math.PI * 2) {
        poly.push([Math.cos(j) * d, Math.sin(j) * d]);
        j += 1 + Math.random();
    }
    return poly;
}
function polyArea(verts) {
    let area = 0;
    for(let i = 0; i < verts.length; i += 1) {
        const j = (i || verts.length) - 1;
        area += (verts[j][1] - verts[i][1]) * (verts[i][0] + verts[j][0]);
    }
    return Math.abs(area / 2);
}

for(let i = 0; i < 6; i += 1) {
    for(let j = 0; j < 6; j += 1) {
        const x = i * 80 + 300 - 2.5 * 80,
              y = j * 80 + 50;
        switch((Math.random() * 3) | 0) {
            case 0:
                const r = Math.random() * 30 + 10;
                E.RigidBody({
                    position: [x, y],
                    shape: E.shape.Circle({ radius: r }),
                    mass: r * r * Math.PI,
                    // mass: 160 * Math.PI,
                });
            break;
            case 1:
                const w = Math.random() * 30 + 10;
                const h = Math.random() * 30 + 10;
                E.RigidBody({
                    position: [x, y],
                    shape: E.shape.Box({ size: [w, h] }),
                    mass: w * h,
                });
            break;
            case 2:
                const v = randomShape(15, 45);
                E.RigidBody({
                    position: [x, y],
                    shape: E.shape.ConvexPolygon({ vertices: v }),
                    mass: polyArea(v),
                });
            break;
        }
    }
}

let dragging = 0, draggingPoint = [];
const FRICTION = 0.01;

// Draw
let before = Date.now();
function draw() {
    const now = Date.now();
    const dt = Math.min(now - before, 40);
    background(0);
    noFill();
    strokeWeight(1);
    for(let i = E.bodies[0].length - 1; i >= 0; i -= 1) {
        const body = E.bodies[0][i];
        const shape = body.shape;
        if(!dragging && mouseIsPressed && !body.static && body.IntersectingPoint([mouseX, mouseY])) {
            dragging = body;
            draggingPoint = body.ToLocal([mouseX, mouseY]);
        }
        if(dragging === body) {
            const p = body.ToWorld(draggingPoint), v = V2D.iSub([mouseX, mouseY], p);
            const m = V2D.SqMag(v);
            if(m) {
                body.ApplyForce(p, V2D.Scale(v, V2D.Mag(v) * body.mass * 0.000001 / dt));
                // V2D.iAdd(body.velocity, V2D.Scale(body.velocity, -0.000001 / m / dt));
                V2D.iAdd(body.position, V2D.Scale(v, 0.01));
            }
            stroke(255);
            strokeWeight(1);
            line(p[0], p[1], mouseX, mouseY);
            ellipse(p[0], p[1], 5, 5);
        }
        push();
        translate(body.position[0], body.position[1]);
        rotate(body.rotation);
        switch(shape.Type()) {
            case "Circle":
                stroke(255);
                ellipse(shape.position[0], shape.position[1], shape.radius * 2, shape.radius * 2);
                line(0, 0, shape.radius, 0);
            break;
            case "Box":
                stroke(255);
                rect(-shape.size[0] + shape.position[0], -shape.size[1] + shape.position[1], shape.size[0] * 2, shape.size[1] * 2);
            break;
            case "ConvexPolygon":
                stroke(255);
                beginShape();
                for(let i = 0; i < body.shape.vertices.length; i += 1) {
                    vertex(body.shape.vertices[i][0] + body.shape.position[0], body.shape.vertices[i][1] + body.shape.position[1]);
                }
                endShape(CLOSE);
        }
        if(DEBUG) {
            stroke(0, 255, 255);
            line(0, 0, 10, 0);
            stroke(255, 0, 255);
            line(0, 0, 0, -10);
        }
        pop();
    }
    if(!mouseIsPressed) dragging = 0;
    for(let i = 0; i < 2; i += 1) {
        for(const body of E.bodies[0]) {
            if(!body.static) {
                body.velocity[1] += 0.00008 * dt;
                body.ApplyForce(V2D.Scale(body.velocity, -FRICTION), body.angularVelocity * -FRICTION);
            }
        }
        E.run(dt);
    }
    before = now;
}

