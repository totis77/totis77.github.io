let flock;

function setup() {
  let canvas = createCanvas(270, 1400);
//   createP("Drag the mouse to generate new boids.");
  canvas.parent('flock_container');
  frameRate(25);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (let i = 0; i < 50; i++) {
    let b = new Boid(Math.random() * width, Math.random() * height);
    flock.addBoid(b);
  }
}

function draw() {
    background(10, 10, 25, 100);
    flock.run();
}

// Add a new boid into the System
function mouseDragged() {
  // if (flock.boids.length < 100)
  //   flock.addBoid(new Boid(mouseX, mouseY));
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flock object
// Does very little, simply manages the array of all the boids

function Flock() {
  // An array for all the boids
  this.boids = []; // Initialize the array
}

Flock.prototype.run = function() {
  for (let i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually
  }
}

Flock.prototype.addBoid = function(b) {
  this.boids.push(b);
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Boid class
// Methods for Separation, Cohesion, Alignment added

function Boid(x, y) {
  this.acceleration = createVector(0, 0);
  this.r = 3.50;
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.1; // Maximum steering force
  this.position = createVector(x, y);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.velocity.mult(this.maxspeed);
}

Boid.prototype.run = function(boids) {
  this.flock(boids);
  this.update();
  this.borders();
  this.render();
}

Boid.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function(boids) {
    // Avoid the mouse!
    let avm = this.followMouse();
  let sep = this.separate(boids);   // Separation
  let ali = this.align(boids);      // Alignment
  let coh = this.cohesion(boids);   // Cohesion
  // Arbitrarily weight these forces
  avm.mult(0.3);
  sep.mult(2.0);
  ali.mult(1.0);
  coh.mult(1.0);
  // Add the force vectors to acceleration
  // this.applyForce(avm);
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
}

// Method to update location
Boid.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function(target) {
  let desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  let steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}

Boid.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  let theta = this.velocity.heading() + radians(90);
  fill(150 + this.velocity.x * 50, 150 , 150 + this.velocity.y * 50, 100);
//   fill(150, 200, 150, 255);
  stroke(250, 250, 250, 80);
  push();
  translate(this.position.x, this.position.y);
  rotate(theta);
//   circle(0, 0, 2 * this.r)
  beginShape();
  vertex(0, -this.r * 2);
  vertex(-this.r, this.r * 2);
  vertex(this.r, this.r * 2);
  endShape(CLOSE);
  pop();
}

// Wraparound
Boid.prototype.borders = function() {
  if (this.position.x < -this.r)  this.position.x = width + this.r;
  if (this.position.y < -this.r)  this.position.y = height + this.r;
  if (this.position.x > width + this.r) this.position.x = -this.r;
  if (this.position.y > height + this.r) this.position.y = -this.r;
}

Boid.prototype.followMouse = function() {
    let mp = createVector(mouseX, mouseY);

    steer = p5.Vector.sub(mp, this.position); 
    let length = steer.mag();
    steer.normalize();
    // steer = p5.Vector.normalize(steer);
    steer = p5.Vector.div(steer, 1 + length/100); 
  
    return steer;
  }

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function(boids) {
  let desiredseparation = 20.0;
  let steer = createVector(0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = p5.Vector.sub(this.position, boids[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many

    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  let neighbordist = 40;
  let sum = createVector(0,0);
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].velocity);
      count++;
      // Draw the vector
      let c = (d / neighbordist) * 255.0; 
      let d1 = p5.Vector.sub(boids[i].position, this.position);
      d1.normalize();
      d1.mult(2 * this.r);
      let p1 = p5.Vector.add(this.position, d1);
      let p2 = p5.Vector.sub(boids[i].position, d1);
      push();
      translate(0.0, 0.0, -10.1);

      stroke(100 + c/3, 100 + c/3, 100 + c/3, 100);
      // line(this.position.x, this.position.y, boids[i].position.x, boids[i].position.y); 
      line(p1.x, p1.y, p2.x, p2.y); 
      pop();
     }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    let steer = p5.Vector.sub(sum, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0, 0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {
  let neighbordist = 40;
  let sum = createVector(0, 0);   // Start with empty vector to accumulate all locations
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0, 0);
  }
}


