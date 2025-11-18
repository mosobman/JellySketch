
PFont font;
ArrayList<Shape> shapes = new ArrayList<>();
PVector translation = new PVector(0,0);
float zoom = -0.4;
int mode = 0;
void keyPressed() { mode = (mode+1) % 6; }

void settings() {
  size(1080, 720, P3D);
  //fullScreen(P3D);
  smooth(8);
}

void setup() {
  frameRate(60);
  translation.set(-width/2.0, -height/2.0);
  font = createFont("Consolas", 24.0f, true); // "https://github.com/notofonts/noto-fonts/raw/main/hinted/ttf/NotoSansMono/NotoSansMono-Regular.ttf" is a good font
  
  shapes.add(shapeCircle(16, width/2.0, 0.0, 60.0, 5.0f));
  
  shapes.add(shapeRect(-width, height-60, 3.0*width, height, 9999999.0f, 10).makeStatic());
  shapes.add(shapeRect(-width, height/9.0, width, height, 9999999.0f, 10).makeStatic());
  shapes.add(shapeRect(width, height/9.0, width, height, 9999999.0f, 10).makeStatic());
}

void mouseScroll(float amount) { zoom += amount; }
void mouseWheel(processing.event.MouseEvent e) {
  if (e.getCount()!=0 && mode==0) mouseScroll(e.getCount()/10.0f);
}

float mouseXw, mouseYw;
Shape locked = null;

boolean mouseJustPressed, mouseJustReleased;
boolean[] check = new boolean[3];

ArrayList<Shape> arr = new ArrayList<>();
void mousePressed() {
  mouseJustPressed = true;
  if (mode==3) {
    arr.clear();
    for (var shape : shapes) {
      if (shape.pointInBBox(mouseXw,mouseYw)) {
        arr.add(shape);
      }
    }
    if (!arr.isEmpty()) locked = arr.get((int) random(0,arr.size()));
  } else if (mode == 4) {
    shapes.add(shapeCircle(16, mouseXw, mouseYw, random(40.0, 150.0), 5.0f));
  } else if (mode == 5) {
    float r = random(40.0, 150.0);
    shapes.add(shapeCube(6, mouseXw-r/2.0, mouseYw-r/2.0, r, 5.0f+r/10.0));
  }
}
void mouseReleased() {
  mouseJustReleased = true;
  if (mode==3) {
    locked = null;
  }
}

void draw() {
  background(220);
  textAlign(LEFT, TOP);
  textFont(font); noStroke();
  mouseXw = mouseX; mouseYw = mouseY;
  mouseXw -= width/2.0; mouseYw -= height/2.0;
  mouseXw /= exp(zoom); mouseYw /= exp(zoom);
  mouseXw -= translation.x; mouseYw -= translation.y;
  
  if (mousePressed && mode==0) translation.add((mouseX-pmouseX)/exp(zoom), (mouseY-pmouseY)/exp(zoom));
  if (locked != null) {
    PVector TM = new PVector(0,0);
    for (var point : locked.points) {
      TM.set(mouseXw, mouseYw);
      TM.sub(point.position);
      point.velocity.add(TM.mult(10.0));
    }
  }
  
  for (int sn = 0; sn < SN; sn++) {
  shapes.forEach(Shape::update);
  shapes.forEach(Shape::calcBoundingBox);
  for (var shape : shapes) {
    for (var othershape : shapes) {
    if (shape == othershape) continue;
    for (var point : othershape.points) {
    if (othershape.static_) continue;
    if (shape.pointInShape(point.position.x,point.position.y)) {
      ClosestEdgeResult r = shape.findClosestEdgePoint(point.position.x,point.position.y);
      PointMass A = shape.points.get(r.edgeStart);
      PointMass B = shape.points.get(r.edgeEnd);
      
      //circle(A.position.x, A.position.y, 10.0f);
      //circle(B.position.x, B.position.y, 10.0f);
      //circle(
      //  A.position.x + r.t * (B.position.x - A.position.x),
      //  A.position.y + r.t * (B.position.y - A.position.y),
      //  10.0f
      //);
      
      float t = r.t;
  
      PVector P = point.position;
  
      // 1. Closest point on edge (C)
      PVector AB = PVector.sub(B.position, A.position);
      PVector C = PVector.add(A.position, PVector.mult(AB, t));
  
      // 2. Vector from P to C
      PVector PC = PVector.sub(C, P);
  
      // 3. Total mass
      float totalMass = point.mass + A.mass + B.mass;
  
      // 4. Compute how much each moves
      float factorP = (A.mass + B.mass) / totalMass;   // fraction of PC that moves P
      float factorEdge = point.mass / totalMass;            // fraction that moves edge points
  
      // 5. Apply displacement
      P.add(PVector.mult(PC, factorP));               // move P along PC
  
      // Move edge points along same vector, weighted by t
      A.position.sub(PVector.mult(PC, factorEdge * (1 - t)));
      B.position.sub(PVector.mult(PC, factorEdge * t));
      
      // --- 2. Collision normal ---
      float dist = PC.mag();
      if (dist < 1e-6f) continue; // skip if too small
      PVector normal = PC.copy().normalize();
  
      // --- 3. Average velocity of the edge ---
      PVector edgeVel = PVector.add(A.velocity, B.velocity).mult(0.5f);
      float massEdge = A.mass + B.mass;
  
      // --- 4. Relative velocity along normal ---
      PVector relVel = PVector.sub(point.velocity, edgeVel);
      float velAlongNormal = relVel.dot(normal);
      if (velAlongNormal >= 0) continue; // already separating
  
      // --- 5. Compute impulse scalar (elastic collision) ---
      totalMass = point.mass + massEdge;
      float j = -(1.0f) * velAlongNormal; // coefficient of restitution = 1 for elastic
      j /= (1.0f / point.mass + 1.0f / massEdge);
  
      PVector impulse = PVector.mult(normal, j);
  
      // --- 6. Apply impulse to velocities ---
      point.velocity.add(PVector.div(impulse, point.mass));               // P moves away
      A.velocity.sub(PVector.mult(impulse, (1-t)/massEdge));      // edge weighted by t
      B.velocity.sub(PVector.mult(impulse, t/massEdge));
      
      // --- 7. Optional: positional correction to avoid sinking ---
      float percent = 1.0f;  // positional correction factor
      float slop = -0.1f;    // allowed penetration
      float correctionMag = Math.max(dist - slop, 0.0f) / totalMass * percent;
      point.position.add(PVector.mult(normal, correctionMag * massEdge));
      A.position.sub(PVector.mult(normal, correctionMag * (1-t) * point.mass));
      B.position.sub(PVector.mult(normal, correctionMag * t * point.mass));
    }
    }
    }
  }
  }
  
  
  translate(width/2.0, height/2.0);
  scale(exp(zoom));
  translate(translation.x, translation.y);
  if (check[0]) {
    shapes.forEach(Shape::drawMesh);
  } else {
    shapes.forEach(Shape::draw);
  }
  if (check[1]) {
    shapes.forEach(Shape::drawSprings);
  }
  if (check[2]) {
    shapes.forEach(Shape::drawBBox);
  }
  translate(-translation.x, -translation.y);
  scale(1.0/exp(zoom));
  translate(-width/2.0, -height/2.0);
  
  fill(0);
  text(String.format("FPS: %.3f", frameRate), 20, 20+24*0);
  String info = "???";
  switch (mode) {
    case (0): info = "Pan and zoom";   break;
    case (1): info = "Attract";        break;
    case (2): info = "Repulse";        break;
    case (3): info = "Click and drag"; break;
    case (4): info = "Place ball";     break;
    case (5): info = "Place square";     break;
  }
  if (mode == 0) info = "Pan and zoom";
  text(String.format("Input Mode (Press Space to cycle): %d (%s)", mode, info), 20, 20+24*1);
  text(String.format("Shape Count: %d", shapes.size()), 20, 20+24*2);
  text(String.format("Translation: (%.3f, %.3f)", translation.x, translation.y), 20, 20+24*3);
  text(String.format("Zoom: %.3f", exp(zoom)), 20, 20+24*4);
  
  stroke(0); strokeWeight(1);
  check[0] = button("Meshes", width-260, 60, check[0]);
  check[1] = button("Springs", width-260, 85, check[1]);
  check[2] = button("Bounding Boxes", width-260, 110, check[2]);
  
  mouseJustPressed = false; mouseJustReleased = false;
}

boolean button(String text, float x, float y, boolean value) {
  if (value) fill(0,255,0,200);
  else fill(5,5,5,200);
  rect(x, y, 20,20);
  text(text, x+25, y+1);
  
  if (mouseJustPressed) {
    if (x <= mouseX && mouseX <= x+20 &&
        y <= mouseY && mouseY <= y+20) {
        value = !value;
    }
  }
  return value;
}
