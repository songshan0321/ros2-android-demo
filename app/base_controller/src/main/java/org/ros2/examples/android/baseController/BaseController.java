/* Copyright 2016-2017 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ros2.examples.android.baseController;

import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import org.ros2.rcljava.RCLJava;

import org.ros2.android.activity.ROSActivity;

import java.util.Date;

import io.github.controlwear.virtual.joystick.android.JoystickView;

public class BaseController extends ROSActivity {

  private BaseControllerNode baseControllerNode;
  private ListenerNode listenerNode;
  private TextView listenerView;
  private TextView dateView;
  private TextView directionView;
  private TextView limitView;

  private static String logtag = BaseController.class.getName();

  /** Called when the activity is first created. */
  @Override
  public final void onCreate(final Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);

    listenerView = (TextView)findViewById(R.id.listenerView);
//    listenerView.setMovementMethod(new ScrollingMovementMethod());
    dateView = (TextView)findViewById(R.id.dateView);
    directionView = (TextView)findViewById(R.id.directionView);
    limitView = (TextView)findViewById(R.id.limitView);
    Button connectButton = (Button)findViewById(R.id.connectButton);
    connectButton.setOnClickListener(connectListener);

    SeekBar maxSpeedBar = (SeekBar) findViewById(R.id.maxSpeedBar);

    maxSpeedBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
      int progressChangedValue = 500;

      public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
          progressChangedValue = progress;
          limitView.setText(String.valueOf(progressChangedValue));
          double maxLinearVel = progressChangedValue / 100.0;
          baseControllerNode.setMaxLinearVel(maxLinearVel);
          baseControllerNode.setMaxAngVel(maxLinearVel);
      }

      public void onStartTrackingTouch(SeekBar seekBar) {
          // TODO Auto-generated method stub
      }

      public void onStopTrackingTouch(SeekBar seekBar) {
          Toast.makeText(BaseController.this, "Max speed is updated:" + progressChangedValue,
                  Toast.LENGTH_SHORT).show();
      }
    });

    JoystickView joystick = (JoystickView) findViewById(R.id.joystickView);

    RCLJava.rclJavaInit();

    // Start a ROS2 publisher node
    baseControllerNode = new BaseControllerNode("teleop_twist_joy", "cmd_vel");

    // Start a ROS2 subscriber node
    listenerNode = new ListenerNode("ldr_listener", "ldr_value", listenerView, dateView);

    // Start listening to /ldr_value
    getExecutor().addNode(listenerNode);

    // Check connection
    Handler handler = new Handler();
    Runnable r1=new Runnable() {
        public void run() {
            Date currentDate = new Date();
            long updateTime = listenerNode.dateobj.getTime();
            long currentTime = currentDate.getTime();
            long diff = (currentTime - updateTime) / 1000 ;
            if (diff > 3){
                dateView.setText("Disconnected");
                listenerView.setText("Unknown");
            }
            handler.postDelayed(this, 500);
        }
    };
    handler.postDelayed(r1, 500);

    // Once joystick is being move
    joystick.setOnMoveListener(new JoystickView.OnMoveListener() {
      @Override
      public void onMove(int angle, int strength) {
        int initAng = 23;
        int deltaAng = 45;
        double maxLinear = baseControllerNode.getMaxLinearVel();
        double maxAng = baseControllerNode.getMaxAngVel();
        double linearVel = 0.0;
        double angVel = 0.0;

        if(angle >= initAng + 7*deltaAng && strength >= 25 || angle < initAng && strength >= 25){
          // R
          linearVel = 0.0;
          angVel = -strength/100.0 * maxAng;
        }
        else if(angle >= initAng && angle < initAng + deltaAng && strength >= 25){
          // RF
          linearVel = strength/100.0 * maxLinear;
          angVel = -strength/100.0 * maxAng;
        }
        else if(angle >= initAng + deltaAng && angle < initAng + 2*deltaAng && strength >= 25){
          // F
          linearVel = strength/100.0 * maxLinear;
          angVel = 0.0;
        }
        else if(angle >= initAng + 2*deltaAng && angle < initAng + 3*deltaAng && strength >= 25){
          // LF
          linearVel = strength/100.0 * maxLinear;
          angVel = strength/100.0 * maxAng;
        }
        else if(angle >= initAng + 3*deltaAng && angle < initAng + 4*deltaAng && strength >= 25){
          // L
          linearVel = 0.0;
          angVel = strength/100.0 * maxAng;
        }
        else if(angle >= initAng + 4*deltaAng && angle < initAng + 5*deltaAng && strength >= 25){
          // LB
          linearVel = -strength/100.0 * maxLinear;
          angVel = strength/100.0 * maxAng;
        }
        else if(angle >= initAng + 5*deltaAng && angle < initAng + 6*deltaAng && strength >= 25){
          // B
          linearVel = -strength/100.0 * maxLinear;
          angVel = 0.0;
        }
        else if(angle >= initAng + 6*deltaAng && angle < initAng + 7*deltaAng && strength >= 25){
          // RB
          linearVel = -strength/100.0 * maxLinear;
          angVel = -strength/100.0 * maxAng;
        }
        else if(strength < 25){
          // Stop
          linearVel = 0.0;
          angVel = 0.0;
        }
        baseControllerNode.setTwistValues(linearVel,angVel);
        directionView.setText(String.format("%.2f",linearVel) + " , " + String.format("%.2f",angVel));

//        getExecutor().addNode(baseControllerNode);
//        baseControllerNode.pubTwist();
//        getExecutor().removeNode(baseControllerNode);
        }
    });

    // Publish twist msg every 200 millisecond
    Handler pubRoutine = new Handler();
    Runnable r2=new Runnable() {
      public void run() {
        // publish twist msg
        getExecutor().addNode(baseControllerNode);
        baseControllerNode.pubTwist();
        getExecutor().removeNode(baseControllerNode);
        pubRoutine.postDelayed(this, 200);
      }
    };
    pubRoutine.postDelayed(r2, 200);
  }

  private OnClickListener connectListener = new OnClickListener() {
    public void onClick(final View view) {
      Log.d(logtag, "onClick() called - Reconnect");
      Toast
              .makeText(BaseController.this, "Trying to reconnect",
                      Toast.LENGTH_LONG)
              .show();

      // Kill nodes
      baseControllerNode = null;
      listenerNode = null;
      // Start a ROS2 publisher node
      baseControllerNode = new BaseControllerNode("teleop_twist_joy", "cmd_vel");
      // Start a ROS2 subscriber node
      listenerNode = new ListenerNode("ldr_listener", "ldr_value", listenerView, dateView);

      // Start listening to /ldr_value
      getExecutor().addNode(listenerNode);

    }
  };
}