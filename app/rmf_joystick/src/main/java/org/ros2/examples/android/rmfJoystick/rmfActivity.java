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

package org.ros2.examples.android.rmfJoystick;

import android.os.Bundle;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.widget.TextView;

import org.ros2.rcljava.RCLJava;

import org.ros2.android.activity.ROSActivity;

import java.util.Date;

import io.github.controlwear.virtual.joystick.android.JoystickView;

public class rmfActivity extends ROSActivity {

  private TalkerNode talkerNode;

  private ListenerNode listenerNode;

  private TextView listenerView;

  private TextView dateView;

  private static String logtag = rmfActivity.class.getName();

  private String preState;

  private String state;

  /** Called when the activity is first created. */
  @Override
  public final void onCreate(final Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);

    listenerView = (TextView)findViewById(R.id.listenerView);
    listenerView.setMovementMethod(new ScrollingMovementMethod());

    dateView = (TextView)findViewById(R.id.dateView);

    JoystickView joystick = (JoystickView) findViewById(R.id.joystickView);

    RCLJava.rclJavaInit();

    // Start a ROS2 publisher node
    talkerNode = new TalkerNode("teleop_twist_joy", "cmd_vel");

    // Start a ROS2 subscriber node
    listenerNode = new ListenerNode("ldr_listener", "ldr_value", listenerView, dateView);

    // Start listening to /ldr_value
    getExecutor().addNode(listenerNode);

    Handler handler = new Handler();

    Runnable r=new Runnable() {
        public void run() {
            Date currentDate = new Date();
            long updateTime = listenerNode.dateobj.getTime();
            long currentTime = currentDate.getTime();
            long diff = (currentTime - updateTime) / 1000 ;
            if (diff > 3){
                dateView.setText("Disconnected");
            }
            handler.postDelayed(this, 500);
        }
    };

    handler.postDelayed(r, 500);

    joystick.setOnMoveListener(new JoystickView.OnMoveListener() {
      @Override
      public void onMove(int angle, int strength) {
        preState = state;
        if(angle >= 315 && strength >= 25 || angle <45 && strength >= 25){
          state = "right";
        }
        else if(angle >= 45 && angle <135 && strength >= 25){
          state = "forward";
        }
        else if(angle >= 135 && angle < 225 && strength >= 25){
          state = "left";
        }
        else if(angle >= 225 && angle <315 && strength >= 25){
          state = "backward";
        }
        else if(strength < 25){
          state = "stop";
        }
        else{
          state = "stop";
        }

        if (preState != state){
          getExecutor().addNode(talkerNode);
          move(state);
          getExecutor().removeNode(talkerNode);
        }
      }
    });

  }
  private void move(String direction){
    if (direction == "left"){
      talkerNode.left();
    }
    else if (direction == "right"){
      talkerNode.right();
    }
    else if (direction == "forward"){
      talkerNode.forward();
    }
    else if (direction == "backward"){
      talkerNode.backward();
    }
    else if (direction == "stop"){
      talkerNode.stop();
    }
    else{
      Log.d(logtag, "invalid direction msg");
    }
  }
}