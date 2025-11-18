
int SN = 4;
float DT = (1.0f/60.0f)/((float) SN);

public class PointMass {
    public float mass;
    public PVector position;
    public PVector velocity;
    
    PointMass(float X, float Y, float mass) {
      this.mass = mass;
      this.position = new PVector(X,Y);
      this.velocity = new PVector(0,0);
    }
    
    void moveTo(float x, float y) {
      float rx,ry;
      rx = x - position.x;
      ry = y - position.y;
      velocity.x += rx * DT;
      velocity.y += ry * DT;
    }
    void force(float x, float y) {
      velocity.x += x * DT / mass;
      velocity.y += y * DT / mass;
    }
    void force(PVector p) { force(p.x, p.y); }
    
    void update() {
      velocity.mult(0.99);
      position.add(velocity.x*DT, velocity.y*DT);
    }
    
    void draw() {
      circle(position.x, position.y, 8.0f);
    }
}
