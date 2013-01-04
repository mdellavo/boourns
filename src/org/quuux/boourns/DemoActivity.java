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

class BodyBuffer {

    private static final String TAG = "BodyBuffer";

    private float scale = 10.0f;
    private Paint paint = new Paint();
    private int width, height;

    enum BodyType {
        BALL
    };

    class Item {
        public boolean alive;
        public BodyType bodyType;
        public float x, y, radius;
        
        public void draw(Canvas c, long elapsed) {
            //Log.d(TAG, "drawing at " + x + "," + y + " | r=" + radius);
            if (alive)
                c.drawCircle(x, height - y, radius, paint);
        }

        // FIXME swapped or translated???
        // FIXME compute angle
        public void update(Body b) {
            //Log.d(TAG, "updating to " + b.getPosition().x + "," + b.getPosition().y);
            
            alive = true;
            x = b.getPosition().x * scale;
            y = b.getPosition().y * scale;
            radius = ((Float)b.getUserData()).floatValue() * scale;
        }

        public void disable() {
            alive = false;
        }

    }

    private Item[] front, back;

    public BodyBuffer(int size) {
        front = alloc(size);
        back = alloc(size);

        paint.setColor(0xffffffff);
        paint.setAntiAlias(true);
        paint.setStrokeWidth(5);
        paint.setStrokeCap(Paint.Cap.ROUND);
        paint.setStyle(Paint.Style.FILL);
    }

    public void setSize(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public int getLength() {
        return front.length;
    }

    public Item[] alloc(int size) {
        Item[] rv = new Item[size];
        
        for (int i=0; i<size; i++) {
            rv[i] = new Item();
        }

        return rv;
    }
   
    public synchronized void swap() {
        Item[] tmp = front;
        front = back;
        back = tmp;
    }

    public void update(Body[] bodies, int numBodies) {
        for (int i=0; i<back.length && i < bodies.length; i++) {
            
            if (i < numBodies)
                back[i].update(bodies[i]);
            else
                back[i].disable();

        }
    }

    public synchronized void draw(Canvas c, long elapsed) {
        for (int i=0; i<front.length; i++) {
            if (front[i].alive) {
                front[i].draw(c, elapsed);
            }
        }
    }
}

public class DemoActivity
    extends Activity 
    implements SurfaceHolder.Callback{

    protected static final String TAG = "DemoActivity";

    class RenderThread extends PausableThread {

        private BodyBuffer buffer;
        private SurfaceHolder surfaceHolder;
        private int width, height;

        public RenderThread(BodyBuffer buffer) {
            super("RenderThread", 60);
            this.buffer = buffer;
        }

        public synchronized void setSurfaceHolder(SurfaceHolder s) {
            surfaceHolder = s;
        }

        public synchronized void setSize(int w, int h) {
            width = w;
            height = h;
        }

        public void update(long elapsed) {
            //Log.d(TAG, "update");
            
            Canvas c = surfaceHolder.lockCanvas();
            c.drawColor(Color.BLUE);
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
        }

        public synchronized void setSize(int w, int h) {
            world.setSize(w/10, h/10);
        }

        public void update(long elapsed) {
            //Log.d(TAG, "update");

            world.tick(elapsed);
            buffer.update(world.getBodies(), world.getNumBodies());
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

        buffer = new BodyBuffer(50);

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
        renderThread.setSize(width, height);
        simulationThread.setSize(width, height);
    }
}
