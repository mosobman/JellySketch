
import java.util.stream.IntStream;
import java.util.concurrent.atomic.AtomicInteger;

Shape shapeCircle(int I, float X, float Y, float S, float mass) {
  Shape neww = new Shape(false);
  float x, y;
  for (int i = 0; i < I; i++) {
    x = S * cos(((float) i) / ((float) I) * PI*2.0f);
    y = S * sin(((float) i) / ((float) I) * PI*2.0f);
    neww.points.add(new PointMass(X + x, Y + y, mass));
  }
  neww.completeSprings();
  neww.smooth = true;
  neww.stiffness *= 3.0;
  return neww;
}
Shape shapeCube(int I, float X, float Y, float S, float mass) {
  Shape neww = new Shape(false);
  
  for (float n = 0; n < I; n++) {
    neww.points.add(new PointMass(X+(S*(n/I)),   Y,   mass));
  }
  for (float n = 0; n < I; n++) {
    neww.points.add(new PointMass(X+S, Y+(S*(n/I)),   mass));
  }
  for (float n = I; n > 0; n--) {
    neww.points.add(new PointMass(X+(S*(n/I)),   Y+S,   mass));
  }
  for (float n = I; n > 0; n--) {
    neww.points.add(new PointMass(X, Y+(S*(n/I)),   mass));
  }
  
  neww.completeSprings();
  neww.smooth = true;
  neww.stiffness *= 1.0;
  return neww;
}
Shape shapeRect(float X, float Y, float W, float H, float mass, int N) {
  Shape neww = new Shape(false);
  
  for (float n = 0; n <= N; n++) {
    neww.points.add(new PointMass(X+(W*(n/N)),   Y,   mass));
  }
  for (float n = 0; n <= N; n++) {
    neww.points.add(new PointMass(X+W, Y+(H*(n/N)),   mass));
  }
  for (float n = N; n >= 0; n--) {
    neww.points.add(new PointMass(X+(W*(n/N)),   Y+H,   mass));
  }
  for (float n = N; n >= 0; n--) {
    neww.points.add(new PointMass(X, Y+(H*(n/N)),   mass));
  }
  
  return neww;
}
class ClosestEdgeResult {
  int edgeStart;
  int edgeEnd;
  float t; // 0=start, 1=end

  ClosestEdgeResult(int edgeStart, int edgeEnd, float t) {
    this.edgeStart = edgeStart;
    this.edgeEnd = edgeEnd;
    this.t = t;
  }
}


public class Shape {
  public ArrayList<PointMass> points = new ArrayList<>();
  public boolean static_ = false;
  private PVector[] boundingBox = new PVector[]{ new PVector(0,0), new PVector(0,0) };
  public ArrayList<Float> springs = new ArrayList<>();
  public float stiffness = 100.0;
  public boolean smooth = false;
  public boolean solid = true;
  public color col;

  public void completeSprings() {
    for (int i = 0; i < points.size(); i++) {
      for (int j = 0; j < points.size(); j++) {
        if (i == j) continue;
        springs.add((float) i);
        springs.add((float) j);
        springs.add(points.get(i).position.dist(points.get(j).position));
      }
    }
  }
  public PVector[] getBoundingBox() { return boundingBox; }
  Shape(boolean static_) {
    this.static_ = static_;
    this.col = color((this.hashCode() & 0xFFFFFF) | 0x80000000);
  }
  public Shape makeStatic() {
    this.static_ = true;
    return this;
  }
  public void calcBoundingBox() {
    if (points.isEmpty()) {
      boundingBox[0].set(0,0);
      boundingBox[1].set(0,0);
    } else {
      float minX = Float.POSITIVE_INFINITY;
      float minY = Float.POSITIVE_INFINITY;
      float maxX = Float.NEGATIVE_INFINITY;
      float maxY = Float.NEGATIVE_INFINITY;
    
      // Can use parallelStream if you like, but regular loop is fine
      for (var point : points) {
        PVector p = point.position;
        if (p.x < minX) minX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.x > maxX) maxX = p.x;
        if (p.y > maxY) maxY = p.y;
      }

      boundingBox[0].set(minX,minY);
      boundingBox[1].set(maxX,maxY);
    }
  }
  
  ClosestEdgeResult findClosestEdgePoint(float x, float y) {
    PVector target = new PVector(x, y);
    int bestA = -1; int bestB = -1;
    float bestT = 0;
    float minDistSq = Float.POSITIVE_INFINITY;
  
    int size = points.size();
    for (int i = 0; i < size; i++) {
      PVector A = points.get(i).position;
      PVector B = points.get((i + 1) % size).position;
  
      PVector AB = PVector.sub(B, A);
      float t = PVector.sub(target, A).dot(AB) / AB.magSq();
      t = constrain(t, 0, 1);
  
      PVector proj = PVector.add(A, PVector.mult(AB, t));
      float distSq = PVector.sub(target, proj).magSq();
  
      if (distSq < minDistSq) {
        minDistSq = distSq;
        bestA = i;
        bestB = (i + 1) % size;
        bestT = t;
      }
    }
  
    return new ClosestEdgeResult(bestA, bestB, bestT);
  }
  
  boolean pointInBBox(float x, float y) {
    PVector minn = boundingBox[0];
    PVector maxx = boundingBox[1];
    
    return (x >= minn.x && x <= maxx.x &&
            y >= minn.y && y <= maxx.y);
  }
  boolean pointInShape(float x, float y) {
    if (!pointInBBox(x,y)) return false;
    float a,b;
    a = boundingBox[1].x+5.0f;
    b = boundingBox[1].y+5.0f;
    int count = countIntersections(a,b,x,y);
    if (count%2 == 0) return false;
    return true;
  }
  
  void drawBBox() {
    noFill(); stroke(255,0,255,200); strokeWeight(2.5f);
    rect(boundingBox[0].x, boundingBox[0].y, boundingBox[1].x-boundingBox[0].x, boundingBox[1].y-boundingBox[0].y);
  }
  void drawSprings() {
    noFill(); stroke(255,255,0,255); strokeWeight(1.5f);
    for (int i = 0; i < springs.size()/3; i++) {
      int a = springs.get(i*3 + 0).intValue();
      int b = springs.get(i*3 + 1).intValue();
      line(points.get(a).position.x, points.get(a).position.y, points.get(b).position.x, points.get(b).position.y);
    }
  }
  void draw() {
    if (solid) { fill(this.col); }
    else { noFill(); }
    stroke(0);
    strokeWeight(4);
    if (smooth && true) drawSmooth();
    else {
    strokeWeight(3);
    //fill(this.col); noStroke();
    //for (var point : points) point.draw();
    PVector curr,next;
    beginShape();
    curr = points.get(0).position;
    vertex(curr.x,curr.y);
    for (int i = 0; i < points.size(); i++) {
      next = points.get((i+1) % points.size()).position;
      vertex(next.x,next.y);
    }
    endShape();
    }
  }
  void drawMesh() {
    stroke(20,20,20,200);
    fill(22,22,22,200);
    strokeWeight(1);
    
    PVector curr,next;
    for (int i = 0; i < points.size(); i++) {
      curr = points.get(i).position; next = points.get((i+1) % points.size()).position;
      line(curr.x,curr.y, next.x,next.y);
      circle(curr.x, curr.y, 5.0f);
    }
    next = points.get(points.size()-1).position;
    circle(next.x, next.y, 5.0f);
  }
  void drawSmooth() {
    //fill(255); noStroke();
    //for (var point : points) point.draw();
    PVector curr,next;
    beginShape();
    strokeJoin(ROUND); strokeCap(ROUND);
    curr = points.get(0).position;
    curveVertex(curr.x,curr.y+0.01);
    curveVertex(curr.x,curr.y);
    for (int i = 0; i < points.size(); i++) {
      curr = points.get(i).position; next = points.get((i+1) % points.size()).position;
      curveVertex(next.x,next.y);
    }
    next = points.get(points.size()-1).position;
    curveVertex(next.x,next.y+0.01);
    endShape();
  }
  
  int countIntersections(float Ax, float Ay, float Bx, float By) {
  //int countIntersections(float Ax, float Bx, float y) { // HORIZONATAL(EAST) ASSUMED
    AtomicInteger count = new AtomicInteger(0);
    int size = points.size();
    
    IntStream.range(0, size).parallel().forEach(i -> {
      PVector P1 = points.get(i).position;
      PVector P2 = points.get((i + 1) % size).position;
      
      if (segmentsIntersect(Ax,Ay, Bx,By, P1.x,P1.y, P2.x,P2.y)) count.incrementAndGet();
      // Check if the edge crosses the horizontal line at y
      //if ((P1.y <= y && P2.y > y) || (P2.y <= y && P1.y > y)) {
        // Compute the X of intersection
        //float intersectX = P1.x + (y - P1.y) * (P2.x - P1.x) / (P2.y - P1.y);
        //if (intersectX >= Bx && intersectX <= Ax) count.incrementAndGet();
      //}
    });
    
    return count.get();
  }
  boolean segmentsIntersect(float p1x, float p1y, float p2x, float p2y, float q1x, float q1y, float q2x, float q2y) {
    float d = (p2x - p1x) * (q2y - q1y) - (p2y - p1y) * (q2x - q1x);
    if (abs(d) < 1e-6f) return false; // parallel
    
    float u = ((q1x - p1x) * (q2y - q1y) - (q1y - p1y) * (q2x - q1x)) / d;
    float v = ((q1x - p1x) * (p2y - p1y) - (q1y - p1y) * (p2x - p1x)) / d;
    
    return (u >= 0 && u <= 1 && v >= 0 && v <= 1);
  }
  
  void update() {
    if (!static_) {
      points.parallelStream().forEach(point -> point.force(0,(1.0f/DT)*9.8*point.mass));
      
      if (mousePressed && (mode==1 || mode==2)) {
        points.parallelStream().forEach(point -> {
          float TMx, TMy;
          TMx = mouseXw; TMy = mouseYw;
          TMx -= point.position.x; TMy -= point.position.y;
          if (mode == 2) { TMx *= -0.1; TMy *= -0.1; }
          point.position.add(TMx/40.0, TMy/40.0);
        });
      }
      for (int i = 0; i < springs.size()/3; i++) {
        int ai = springs.get(i*3 + 0).intValue(); PointMass a = points.get(ai);
        int bi = springs.get(i*3 + 1).intValue(); PointMass b = points.get(bi);
        PVector D = PVector.sub(b.position, a.position);
        //PVector C = PVector.div(D, 2.0).add(a.position);
        float d = D.mag(); D.normalize();
        float x = springs.get(i*3 + 2);
        D.setMag(SN * stiffness * (d-x)/2.0);
        
        if (abs(ai-bi) >= 2) D.mult(10.0);
        
        a.force(D.x, D.y);
        b.force(-D.x, -D.y);
      }
    }
    points.parallelStream().forEach(PointMass::update);
  }
}
