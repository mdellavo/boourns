package org.quuux.boourns;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Bundle;
import android.view.Window;
import android.view.View;
import android.view.WindowManager;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.util.Log;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;

class PausableThread extends Thread {
    protected final String TAG;
    protected final int FPS;

    private boolean running, paused;
    private static final int FPS_STEP = 5;

    public PausableThread(String tag, int fps) {
        super();
        TAG = tag;
        FPS = fps;
        pauseRunning();
        paused = true;
        running = false;
    }

    @Override
    public void start() {
        super.start();
        startRunning();
    }

    public void startRunning() {
        running = true;
    }

    public synchronized boolean isRunning() {
        return running;
    }

    public void stopRunning() {
        running = false;
    }

    public synchronized void pauseRunning() {
        Log.d(TAG, "pausing...");
        paused = true;
        notifyAll();
    }

    public synchronized void resumeRunning() {
        Log.d(TAG, "resuming");
        paused = false;
        notifyAll();
    }

    public synchronized void waitForResume() {
        while(paused) {
            try {
                wait();
            } catch(InterruptedException e) {
            }
        }
    }

    public void throttle(int timeslice, long elapsed) {
        if(elapsed < timeslice) {
            try {
                Thread.sleep(timeslice - elapsed);
            } catch(InterruptedException e) {
            }
        }
    }

    public void update(long elapsed) {
    }

    @Override
    public void run() {

        int frames = 0;
        long last = System.currentTimeMillis();
        long last_frames = 0;
        long fps_tally = 0;

        while (isRunning()) {
            frames++;
            waitForResume();

            long now = System.currentTimeMillis();
            long elapsed = now - last;

            update(elapsed);

            now = System.currentTimeMillis();
            elapsed = now - last;

            last = now;

            fps_tally += elapsed;
            if (fps_tally > (FPS_STEP * 1000)) {
                long delta_frames = frames - last_frames;
                Log.d(TAG, "fps = " + (delta_frames / FPS_STEP));
                last_frames = frames;
                fps_tally = 0;
            }

            throttle(1000/FPS, elapsed);
        }
    }
}

enum BodyType {
    BALL,
    EDGE
};

class Item {
    private static final String TAG ="Item";
    
    protected BodyBuffer buffer;
    public BodyType bodyType;
    protected Paint paint;

    public Item(BodyType bodyType, BodyBuffer buffer, Paint paint) {
        this.buffer = buffer;
        this.bodyType = bodyType;
        this.paint = paint;
    }
}

class BallItem extends Item {
    private static final String TAG ="BallItem";

    protected Body body;

    public BallItem(BodyBuffer buffer, Paint paint) {
        super(BodyType.BALL, buffer, paint);
    }
 
    public void draw(Canvas c, long elapsed) {
        if (body == null) 
            return ;

        float x = body.getPosition().x * buffer.getScale();
        float y = buffer.getHeight() - body.getPosition().y * buffer.getScale();
        float radius = ((Float)body.getUserData()).floatValue() * buffer.getScale();        
        c.drawCircle(x, y, radius, paint);
    }

    // FIXME swapped or translated???
    // FIXME compute angle
    public void update(Body b) {
        body = b;
    }
}
 
class EdgeItem extends Item {
    private static final String TAG ="EdgeItem";

    protected Vec2 a, b;
    public EdgeItem(BodyBuffer buffer, Paint paint) {
        super(BodyType.EDGE, buffer, paint);
    }

    public void draw(Canvas c, long elapsed) {
        if (a == null || b == null) 
            return;

        float x1 = a.x * buffer.getScale();
        float y1 = buffer.getHeight() - a.y * buffer.getScale();
        float x2 = b.x * buffer.getScale();
        float y2 = buffer.getHeight() - b.y * buffer.getScale();

        c.drawLine(x1, y1, x2, y2, paint);
    }

    public void update(Vec2 a, Vec2 b) {
        this.a = a;
        this.b = b;
    }
}
   
class BodyBuffer {

    private static final String TAG = "BodyBuffer";

    private float scale = 10.0f;

    private int width, height;
    private EdgeItem[] edges;
    private BallItem[] front, back;

    private Paint ballPaint = new Paint();
    private Paint edgePaint = new Paint();

    public BodyBuffer(int sizeBalls, int sizeEdges) {
        ballPaint.setColor(Color.BLUE);
        ballPaint.setAntiAlias(true);
        ballPaint.setStrokeWidth(5);
        ballPaint.setStrokeCap(Paint.Cap.ROUND);
        ballPaint.setStyle(Paint.Style.FILL);

        edgePaint.setColor(Color.WHITE);
        edgePaint.setAntiAlias(true);
        edgePaint.setStrokeWidth(5);
        edgePaint.setStrokeCap(Paint.Cap.ROUND);
        edgePaint.setStyle(Paint.Style.FILL);

        front = allocBalls(sizeBalls, ballPaint);
        back = allocBalls(sizeBalls, ballPaint);
        edges = allocEdges(sizeEdges, edgePaint);

    }

    public void setSize(int width, int height) {
        this.width = width;
        this.height = height;
        this.scale = height / 100;
    }

    public void setScale(float s) {
        scale = s;
    }
    
    public float getScale() {
        return scale;
    }

    public int getHeight() { return height; }
    public int getWidth() { return width; }

    public int getLength() {
        return front.length;
    }

    public BallItem[] allocBalls(int size, Paint paint) {
        BallItem[] rv = new BallItem[size];
        
        for (int i=0; i<size; i++) {
            rv[i] = new BallItem(this, paint);
        }

        return rv;
    }

    public EdgeItem[] allocEdges(int size, Paint paint) {
        EdgeItem[] rv = new EdgeItem[size];
        
        for (int i=0; i<size; i++) {
            rv[i] = new EdgeItem(this, paint);
        }

        return rv;
    }
   
    public synchronized void swap() {
        BallItem[] tmp = front;
        front = back;
        back = tmp;
    }

    public void addEdges(Vec2[][] edges) {
        for (int i=0; i<edges.length; i++) {
            Vec2 a = edges[i][0];
            Vec2 b = edges[i][1];
            Log.d(TAG, "adding edge " + a + ", " + b);
                this.edges[i].update(a, b);
        }
    }

    public void update(Body[] bodies) {
        for (int i=0; i < back.length && i < bodies.length; i++) {
            back[i].update(bodies[i]);
        }
    }

    public synchronized void draw(Canvas c, long elapsed) {
        
        c.save();

        // FIXME set scale, translation rotation
        //c.scale(scale, scale);
        c.translate(width / 2 - 50 * scale, 0);
        c.rotate(0);

        c.drawColor(Color.BLACK);

        for (int i=0; i<edges.length; i++) {
            c.save();
            edges[i].draw(c, elapsed);
            c.restore();
        }

        for (int i=0; i<front.length; i++) {
            c.save();
            front[i].draw(c, elapsed);
            c.restore();
        }

        c.restore();
    }
}

public class DemoActivity
    extends Activity 
    implements SurfaceHolder.Callback{

    protected static final String TAG = "DemoActivity";

    class RenderThread extends PausableThread {

        private BodyBuffer buffer;
        private SurfaceHolder surfaceHolder;
 
        public RenderThread(BodyBuffer buffer) {
            super("RenderThread", 60);
            this.buffer = buffer;
        }

        public synchronized void setSurfaceHolder(SurfaceHolder s) {
            surfaceHolder = s;
        }
 
        public void update(long elapsed) {
            //Log.d(TAG, "update");
            
            Canvas c = surfaceHolder.lockCanvas();
            buffer.draw(c, elapsed);
            surfaceHolder.unlockCanvasAndPost(c);
        }
    }

    class SimulationThread extends PausableThread {
        private GameWorld world;
        private BodyBuffer buffer;

        public SimulationThread(BodyBuffer buffer) {
            super("SimulationThread", 200);
            this.buffer = buffer;
            world = new GameWorld(buffer.getLength());
            buffer.addEdges(world.getEdges());
        }

        public void update(long elapsed) {
            //Log.d(TAG, "update");

            world.tick(elapsed);
            buffer.update(world.getBodies());
            buffer.swap();
        }
    }

    private SurfaceView worldView;
    private BodyBuffer buffer;
    private RenderThread renderThread;
    private SimulationThread simulationThread;

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_HIDE_NAVIGATION);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, 
                             WindowManager.LayoutParams.FLAG_FULLSCREEN);

        worldView = new SurfaceView(this);
        worldView.getHolder().addCallback(this);

        setContentView(worldView);

        buffer = new BodyBuffer(100, 3);

        renderThread = new RenderThread(buffer);
        simulationThread = new SimulationThread(buffer);

        simulationThread.start();
    }

    @Override
    protected void onResume() {
        super.onResume();

    }
 
    @Override
    protected void onPause() {
        super.onPause();
        renderThread.pauseRunning();
        simulationThread.pauseRunning();
    }

    public void surfaceCreated(SurfaceHolder holder) {
        renderThread.setSurfaceHolder(holder);
        
        if (!renderThread.isRunning())
            renderThread.start();        

        Log.d(TAG, "Resuming ren thread");
        renderThread.resumeRunning();

        Log.d(TAG, "Resuming sim thread");
        simulationThread.resumeRunning();

    }

    public void killThread(PausableThread t) {
        t.stopRunning();
        // try {
        //     t.join();
        // } catch(InterruptedException e) {
        // }
    }

    public void surfaceDestroyed(SurfaceHolder holder) {
        killThread(renderThread);
        killThread(simulationThread);
    }

    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        Log.d(TAG, String.format("Surface changed - %dx%d", width, height));       
        buffer.setSize(width, height);
    }
}
