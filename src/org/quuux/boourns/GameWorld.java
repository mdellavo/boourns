package org.quuux.boourns;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.MassData;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.callbacks.ContactImpulse;

import android.util.Log;


// source http://code.google.com/p/jbox2d/source/browse/trunk/jbox2d-testbed/src/main/java/org/jbox2d/testbed/tests/LiquidTest.java?spec=svn517&r=517
// With a large number of bodies the gc very frequently.  this code isnt the best for android
class Liquid {

    private static final int SIZE = 40;

    /** A "close to zero" float epsilon value for use */
    public static final float EPSILON = 1.1920928955078125E-7f;

    private float rad = 0.6f;
    private float visc = 0.004f;

    private ArrayList<Integer>[][] index;
    
    public Liquid() {

        index = new ArrayList[SIZE][SIZE];

        for (int i = 0; i < SIZE; i++) {
            for (int j = 0; j < SIZE; j++) {
                index[i][j] = new ArrayList<Integer>();
            }
        }
    }

    public final static float map(final float val, final float fromMin, final float fromMax, final float toMin,
                                  final float toMax) {
        final float mult = (val - fromMin) / (fromMax - fromMin);
        final float res = toMin + mult * (toMax - toMin);
        return res;
    }
    
    private int hash(float val, float fromMin, float fromMax) {
        return (int)map(val, fromMin, fromMax, 0, SIZE - .001f);
    }

    public void updateIndex(Body[] bodies, int width, int height) {
        for (int i = 0; i < SIZE; i++) {
            for (int j = 0; j < SIZE; j++) {
                index[i][j].clear();
            }
        }

        for(int i = 0; i < bodies.length; i++) {
            if (bodies[i] == null) 
                continue;

            int hcell = hash(bodies[i].m_sweep.c.x, 0, width);
            int vcell = hash(bodies[i].m_sweep.c.y, 0, height);
            if(hcell > -1 && hcell < SIZE && vcell > -1 && vcell < SIZE)
                index[hcell][vcell].add(new Integer(i));
        }     
    }
    
    public void dampen(Body[] bodies) {
        for (int i=0; i<bodies.length; i++) {
            if (bodies[i] == null)
                continue;

            bodies[i].setLinearVelocity(bodies[i].getLinearVelocity().mul(0.995f));
        }
    }
    
    public void apply(Body[] bodies, int width, int height, float delta) {
        /*
         * Unfortunately, this simulation method is not actually scale
         * invariant, and it breaks down for rad < ~3 or so.  So we need
         * to scale everything to an ideal rad and then scale it back after.
         */
        final float idealRad = 50.0f;
        float multiplier = idealRad / rad;
                
        float[] xchange = new float[bodies.length];
        float[] ychange = new float[bodies.length];
        Arrays.fill(xchange,0.0f);
        Arrays.fill(ychange, 0.0f);
                
        float[] xs = new float[bodies.length];
        float[] ys = new float[bodies.length];
        float[] vxs = new float[bodies.length];
        float[] vys = new float[bodies.length];
        for (int i=0; i<bodies.length; ++i) {
            if (bodies[i] == null)
                continue;

            xs[i] = multiplier*bodies[i].m_sweep.c.x;
            ys[i] = multiplier*bodies[i].m_sweep.c.y;
            vxs[i] = multiplier*bodies[i].m_linearVelocity.x;
            vys[i] = multiplier*bodies[i].m_linearVelocity.y;
        }
                
        for(int i = 0; i < bodies.length; i++) {

            if (bodies[i] == null)
                continue;

            // Populate the neighbor list from the 9 proximate cells
            ArrayList<Integer> neighbors = new ArrayList<Integer>();
            int hcell = hash(bodies[i].m_sweep.c.x, 0, width);
            int vcell = hash(bodies[i].m_sweep.c.y, 0, height);
            for(int nx = -1; nx < 2; nx++) {
                for(int ny = -1; ny < 2; ny++) {
                    int xc = hcell + nx;
                    int yc = vcell + ny;
                    if(xc > -1 && xc < SIZE && yc > -1 && yc < SIZE && index[xc][yc].size() > 0) {
                        for(int a = 0; a < index[xc][yc].size(); a++) {
                            Integer ne = (Integer)index[xc][yc].get(a);
                            if(ne != null && ne.intValue() != i)
                                neighbors.add(ne);
                        }
                    }
                }
            }
                
            // Particle pressure calculated by particle proximity
            // Pressures = 0 iff all particles within range are idealRad distance away
            float[] vlen = new float[neighbors.size()];
            float p = 0.0f;
            float pnear = 0.0f;
            for(int a = 0; a < neighbors.size(); a++) {
                Integer n = (Integer)neighbors.get(a);
                int j = n.intValue();
                float vx = xs[j]-xs[i];//bodies[j].m_sweep.c.x - bodies[i].m_sweep.c.x;
                float vy = ys[j]-ys[i];//bodies[j].m_sweep.c.y - bodies[i].m_sweep.c.y;
                
                //early exit check
                if(vx > -idealRad && vx < idealRad && vy > -idealRad && vy < idealRad) {
                    float vlensqr = (vx * vx + vy * vy);
                    //within idealRad check
                    if(vlensqr < idealRad*idealRad) {
                        vlen[a] = (float)Math.sqrt(vlensqr);
                        if (vlen[a] < EPSILON) vlen[a] = idealRad-.01f;
                        float oneminusq = 1.0f-(vlen[a] / idealRad);
                        p = (p + oneminusq*oneminusq);
                        pnear = (pnear + oneminusq*oneminusq*oneminusq);
                    } else {
                        vlen[a] = Float.MAX_VALUE;
                    }
                }
            }
            
            // Now actually apply the forces
            //System.out.println(p);
            float pressure = (p - 5F) / 2.0F; //normal pressure term
            float presnear = pnear / 2.0F; //near particles term
            float changex = 0.0F;
            float changey = 0.0F;
            for(int a = 0; a < neighbors.size(); a++) {
                Integer n = (Integer)neighbors.get(a);
                int j = n.intValue();
                float vx = xs[j]-xs[i];//bodies[j].m_sweep.c.x - bodies[i].m_sweep.c.x;
                float vy = ys[j]-ys[i];//bodies[j].m_sweep.c.y - bodies[i].m_sweep.c.y;
                if(vx > -idealRad && vx < idealRad && vy > -idealRad && vy < idealRad) {
                    if(vlen[a] < idealRad) {
                        float q = vlen[a] / idealRad;
                        float oneminusq = 1.0f-q;
                        float factor = oneminusq * (pressure + presnear * oneminusq) / (2.0F*vlen[a]);
                        float dx = vx * factor;
                        float dy = vy * factor;
                        float relvx = vxs[j] - vxs[i];
                        float relvy = vys[j] - vys[i];
                        factor = visc * oneminusq * delta;
                        dx -= relvx * factor;
                        dy -= relvy * factor;
                        //bodies[j].m_xf.position.x += dx;//*delta*delta;
                        //bodies[j].m_xf.position.y += dy;//*delta*delta;
                        //bodies[j].m_linearVelocity.x += dx;///delta;//delta;
                        //bodies[j].m_linearVelocity.y += dy;///delta;//delta;
                        xchange[j] += dx;
                        ychange[j] += dy;
                        changex -= dx;
                        changey -= dy;
                    }
                }
            }
            //bodies[i].m_xf.position.x += changex;//*delta*delta;
            //bodies[i].m_xf.position.y += changey;//*delta*delta;
            //bodies[i].m_linearVelocity.x += changex;///delta;//delta;
            //bodies[i].m_linearVelocity.y += changey;///delta;//delta;
            xchange[i] += changex;
            ychange[i] += changey;
        }
        //multiplier *= delta;
        for (int i=0; i<bodies.length; ++i) {
            if (bodies[i] == null)
                continue;

            bodies[i].m_xf.position.x += xchange[i] / multiplier;
            bodies[i].m_xf.position.y += ychange[i] / multiplier;
            bodies[i].m_linearVelocity.x += xchange[i] / (multiplier*delta);
            bodies[i].m_linearVelocity.y += ychange[i] / (multiplier*delta);
        }              
    }
}

public class GameWorld 
    implements ContactListener {

    private static final String TAG = "GameWorld";

    public static final float DRAWFRAME_FRAMERATE = 60;
    public static final float PHYSIC_FRAMERATE = 100;
    public static final int SIM_COUNT = (int) Math.round(PHYSIC_FRAMERATE / DRAWFRAME_FRAMERATE);
    public static final int VEL_ITER = 3;
    public static final int POS_ITER = 8;

    public static final float PHYSIC_STEP_SEC = (1f / PHYSIC_FRAMERATE);
    public static final int DRAWFRAME_STEP_MS = Math.round(1000.0f / DRAWFRAME_FRAMERATE);

    private Body[] bodies;
    private World world;

    private int width, height;
    private float innerWidth, innerHeight;
    private float offsetWidth, offsetHeight;

    private Liquid liquid;

    // private float[][][] relativeEdges = {
    //     { { 0f, .25f   } , { .667f, .25f } },
    //     { { .333f, .5f } , { 1f, .5f } },
    //     { { 0f, .75f   } , { .667f, .75f } }
    // };

    private float[][][] relativeEdges = {
        { { 0f, .5f   } , { .5f, .25f } },
        { { .5f, .5f } , { 1f, .75f } },
        { { 0f, 1f   } , { .5f, .75f } }
    };

    private Vec2[][] edges = new Vec2[relativeEdges.length][2];

    public GameWorld(int size) {
        bodies = new Body[size];
        this.width = this.height = 100;
        create();
    }    

    // FIXME y is reversed
    public void create() {
        Log.d(TAG, "Creating world");

        Vec2 gravity = new Vec2(0.0f, -10.0f);
        
        world = new World(gravity, false);
        world.setContinuousPhysics(true);
        world.setWarmStarting(true);
        //world.setContactListener(this);
             
        BodyDef bd = new BodyDef();
        bd.position.set(0.0f, 0.0f);
        Body ground = world.createBody(bd);

        innerWidth = width * .5f;
        innerHeight = height * .5f;
        
        offsetWidth = width * .25f;
        offsetHeight = height * .25f;

        Log.d(TAG, "inner = "  + innerWidth + "x" + innerHeight);
        Log.d(TAG, "offset = "  + offsetWidth + "x" + offsetHeight);

        for (int i=0; i<relativeEdges.length; i++) {
            
            float x1 = relativeEdges[i][0][0] * innerWidth + offsetWidth;
            float y1 = relativeEdges[i][0][1] * innerHeight + offsetHeight;
            Vec2 a = new Vec2(x1, y1);

            float x2 = relativeEdges[i][1][0] * innerWidth + offsetWidth;
            float y2 = relativeEdges[i][1][1] * innerHeight + offsetHeight;
            Vec2 b = new Vec2(x2, y2);

            edges[i][0] = a;
            edges[i][1] = b;
            
            ground.createFixture(createEdge(a, b));
        }

        liquid = new Liquid();

        for (int i=0; i<bodies.length; i++) {
            bodies[i] = addBall(1f, 10f / bodies.length);
        }

     }

    private Body addBall(float radius, float mass) {
        
        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyType.DYNAMIC;
        bodyDef.position.set(new Vec2((offsetWidth + innerWidth/4) + (float)Math.random() * 4f, 
                                      height + (float)Math.random() * 4f));
        Body body = world.createBody(bodyDef);
        
        CircleShape shape = new CircleShape();
        shape.m_radius = radius;
        
        FixtureDef def = new FixtureDef();
        def.density = 1.0f;
        def.friction = 0;
        def.restitution = 0.8f;
        def.shape = shape;
        body.createFixture(def);

        MassData md = new MassData();
        md.mass = mass;
        md.I = 1.0f;
        body.setMassData(md);

        body.resetMassData();
        body.setSleepingAllowed(false);
        body.setFixedRotation(true);
        body.setUserData(Float.valueOf(radius));

        // Vec2 impulse = new Vec2((float)Math.random() * 25f, (float)Math.random() * -25f);
        // body.setLinearVelocity(impulse);
  
        return body;
    }

    private FixtureDef createEdge(Vec2 a, Vec2 b) {

        PolygonShape groundShapeDef = new PolygonShape();
        groundShapeDef.setAsEdge(a, b);

        FixtureDef def = new FixtureDef();
        def.density = 1.0f;
        def.friction = 0.9f;
        def.restitution = 0.7f;
        def.shape = groundShapeDef;

        return def;
    }

    private void checkBounds() {
        for (int i=0; i < bodies.length; i++) {
            if (bodies[i] == null)
                continue;

            if (bodies[i].getWorldCenter().y < -10.0f ||
                bodies[i].getWorldCenter().x < -10.0f ||
                bodies[i].getWorldCenter().y > (height + 10) ||
                bodies[i].getWorldCenter().x > (width + 10) ) {
                
                world.destroyBody(bodies[i]);

                bodies[i] = addBall(1f, 10f / bodies.length);
            }
        }
    }

    public void tick(long elapsed) {
        for (int i = 0; i < 1; i++) {
            world.step(PHYSIC_STEP_SEC, VEL_ITER, POS_ITER);

            liquid.updateIndex(bodies, width, height);
            liquid.apply(bodies, width, height, 1);
            liquid.dampen(bodies);

            checkBounds();
        }
    }

    public Body[] getBodies() {
        return bodies;
    }

    public Vec2[][] getEdges() {
        return edges;
    }

    public void beginContact(Contact contact)  {}
    public void endContact(Contact contact)  {}
    public void postSolve(Contact contact, ContactImpulse impulse)  {}
    public void preSolve(Contact contact, Manifold oldManifold)  {}
}

