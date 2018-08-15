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
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.Button;
import android.widget.Toast;

import org.ros2.rcljava.RCLJava;

import org.ros2.android.activity.ROSActivity;

import io.github.controlwear.virtual.joystick.android.JoystickView;

public class rmfActivity extends ROSActivity {

  private rmfNode rmfNode;

  private static String logtag = rmfActivity.class.getName();

  private String preState;

  private String state;

  /** Called when the activity is first created. */
  @Override
  public final void onCreate(final Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);


    RCLJava.rclJavaInit();

    // Start a ROS2 node
    rmfNode = new rmfNode("android_rmf_node", "RMFChatter");

    JoystickView joystick = (JoystickView) findViewById(R.id.joystickView);
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
          getExecutor().addNode(rmfNode);
          move(state);
          getExecutor().removeNode(rmfNode);
        }
      }
    });

  }
  private void move(String direction){
    if (direction == "left"){
      rmfNode.left();
    }
    else if (direction == "right"){
      rmfNode.right();
    }
    else if (direction == "forward"){
      rmfNode.forward();
    }
    else if (direction == "backward"){
      rmfNode.backward();
    }
    else if (direction == "stop"){
      rmfNode.stop();
    }
    else{
      Log.d(logtag, "invalid direction msg");
    }
  }
  }