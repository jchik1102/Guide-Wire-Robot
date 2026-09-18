package com.robotcontroller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class JoystickView extends View {

    private final Paint basePaint       = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint baseRingPaint   = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint thumbPaint      = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint thumbBorderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint crosshairPaint  = new Paint(Paint.ANTI_ALIAS_FLAG);

    private float centerX, centerY;
    private float baseRadius;
    private float thumbRadius;
    private float thumbX, thumbY;

    private OnJoystickMoveListener listener;

    public interface OnJoystickMoveListener {
        void onJoystickMove(int xPercent, int yPercent, int angle, int strength);
    }

    public JoystickView(Context context) {
        super(context);
        init();
    }

    public JoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public JoystickView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        basePaint.setColor(Color.parseColor("#111111"));
        basePaint.setStyle(Paint.Style.FILL);

        baseRingPaint.setColor(Color.parseColor("#2C2C2E"));
        baseRingPaint.setStyle(Paint.Style.STROKE);
        baseRingPaint.setStrokeWidth(2f);

        crosshairPaint.setColor(Color.parseColor("#1C1C1E"));
        crosshairPaint.setStyle(Paint.Style.STROKE);
        crosshairPaint.setStrokeWidth(1f);

        thumbPaint.setColor(Color.parseColor("#3E8EDE"));
        thumbPaint.setStyle(Paint.Style.FILL);

        thumbBorderPaint.setColor(Color.parseColor("#5AA0E8"));
        thumbBorderPaint.setStyle(Paint.Style.STROKE);
        thumbBorderPaint.setStrokeWidth(2f);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        centerX = w / 2f;
        centerY = h / 2f;

        float maxRadius = Math.min(w, h) / 2f - 10f;
        baseRadius  = maxRadius * 0.85f;
        thumbRadius = maxRadius * 0.20f;

        thumbX = centerX;
        thumbY = centerY;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        canvas.drawCircle(centerX, centerY, baseRadius, basePaint);
        canvas.drawCircle(centerX, centerY, baseRadius, baseRingPaint);

        canvas.drawLine(centerX - baseRadius * 0.6f, centerY,
                centerX + baseRadius * 0.6f, centerY, crosshairPaint);
        canvas.drawLine(centerX, centerY - baseRadius * 0.6f,
                centerX, centerY + baseRadius * 0.6f, crosshairPaint);

        canvas.drawCircle(centerX, centerY, baseRadius * 0.45f, crosshairPaint);

        canvas.drawCircle(thumbX, thumbY, thumbRadius, thumbPaint);
        canvas.drawCircle(thumbX, thumbY, thumbRadius, thumbBorderPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
            case MotionEvent.ACTION_MOVE:
                float dx = event.getX() - centerX;
                float dy = event.getY() - centerY;
                float distance = (float) Math.sqrt(dx * dx + dy * dy);

                if (distance > baseRadius) {
                    dx = dx * baseRadius / distance;
                    dy = dy * baseRadius / distance;
                    distance = baseRadius;
                }

                thumbX = centerX + dx;
                thumbY = centerY + dy;

                int xPercent  = (int) (dx / baseRadius * 100);
                int yPercent  = (int) (dy / baseRadius * 100);
                int strength  = (int) (distance / baseRadius * 100);
                int angle     = (int) (Math.toDegrees(Math.atan2(-dy, dx)) + 360) % 360;

                if (listener != null) {
                    listener.onJoystickMove(xPercent, yPercent, angle, strength);
                }

                invalidate();
                return true;

            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_CANCEL:
                thumbX = centerX;
                thumbY = centerY;

                if (listener != null) {
                    listener.onJoystickMove(0, 0, 0, 0);
                }

                invalidate();
                return true;
        }

        return super.onTouchEvent(event);
    }

    public void setOnJoystickMoveListener(OnJoystickMoveListener listener) {
        this.listener = listener;
    }
}