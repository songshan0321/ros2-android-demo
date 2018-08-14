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

package org.ros2.examples.android.rmfTool;

import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.Button;
import android.widget.Toast;

import org.ros2.rcljava.RCLJava;

import org.ros2.android.activity.ROSActivity;

public class rmfActivity extends ROSActivity {

  private rmfNode rmfNode;

  private static String logtag = rmfActivity.class.getName();

  /** Called when the activity is first created. */
  @Override
  public final void onCreate(final Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);

    // Create 4 button listener
    Button buttonLeft = (Button)findViewById(R.id.buttonLeft);
    buttonLeft.setOnTouchListener(leftListener);

    Button buttonRight = (Button)findViewById(R.id.buttonRight);
    buttonRight.setOnTouchListener(rightListener);

    Button buttonUp = (Button)findViewById(R.id.buttonUp);
    buttonUp.setOnTouchListener(upListener);

    Button buttonDown = (Button)findViewById(R.id.buttonDown);
    buttonDown.setOnTouchListener(downListener);

    RCLJava.rclJavaInit();

    // Start a ROS2 node
    rmfNode = new rmfNode("android_rmf_node", "RMFChatter");
  }

  // Create an anonymous implementation of onTouchListener
  private OnTouchListener leftListener = new OnTouchListener() {
    public boolean onTouch(final View view, MotionEvent event) {
        if(event.getAction() == MotionEvent.ACTION_DOWN){

            Log.d(logtag, "onTouch() called - left button");
            Toast
                    .makeText(rmfActivity.this, "The Turn Left button was touched.",
                            Toast.LENGTH_LONG)
                    .show();
            getExecutor().addNode(rmfNode);
            move("left");
            getExecutor().removeNode(rmfNode);
            return true;
        }
        else if(event.getAction() == MotionEvent.ACTION_UP){

            Log.d(logtag, "onTouch() ended - left button");
            Toast
                    .makeText(rmfActivity.this, "The Turn Left button was touched.",
                            Toast.LENGTH_LONG)
                    .show();
            getExecutor().addNode(rmfNode);
            move("stop");
            getExecutor().removeNode(rmfNode);
            return false;
        }
        return false;
    }
  };

    private OnTouchListener rightListener = new OnTouchListener() {
        public boolean onTouch(final View view, MotionEvent event) {
            if(event.getAction() == MotionEvent.ACTION_DOWN){

                Log.d(logtag, "onTouch() called - right button");
                Toast
                        .makeText(rmfActivity.this, "The Turn Right button was touched.",
                                Toast.LENGTH_LONG)
                        .show();
                getExecutor().addNode(rmfNode);
                move("right");
                getExecutor().removeNode(rmfNode);
                return true;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP){

                Log.d(logtag, "onTouch() ended - right button");
                Toast
                        .makeText(rmfActivity.this, "The Turn right button was touched.",
                                Toast.LENGTH_LONG)
                        .show();
                getExecutor().addNode(rmfNode);
                move("stop");
                getExecutor().removeNode(rmfNode);
                return false;
            }
            return false;
        }
    };

    private OnTouchListener upListener = new OnTouchListener() {
        public boolean onTouch(final View view, MotionEvent event) {
            if(event.getAction() == MotionEvent.ACTION_DOWN){

                Log.d(logtag, "onTouch() called - forward button");
                Toast
                        .makeText(rmfActivity.this, "The Forward button was touched.",
                                Toast.LENGTH_LONG)
                        .show();
                getExecutor().addNode(rmfNode);
                move("forward");
                getExecutor().removeNode(rmfNode);
                return true;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP){

                Log.d(logtag, "onTouch() ended - forward button");
                Toast
                        .makeText(rmfActivity.this, "The Forward button was touched.",
                                Toast.LENGTH_LONG)
                        .show();
                getExecutor().addNode(rmfNode);
                move("stop");
                getExecutor().removeNode(rmfNode);
                return false;
            }
            return false;
        }
    };

    private OnTouchListener downListener = new OnTouchListener() {
        public boolean onTouch(final View view, MotionEvent event) {
            if(event.getAction() == MotionEvent.ACTION_DOWN){

                Log.d(logtag, "onTouch() called - backward button");
                Toast
                        .makeText(rmfActivity.this, "The Backward button was touched.",
                                Toast.LENGTH_LONG)
                        .show();
                getExecutor().addNode(rmfNode);
                move("backward");
                getExecutor().removeNode(rmfNode);
                return true;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP){

                Log.d(logtag, "onTouch() ended - backward button");
                Toast
                        .makeText(rmfActivity.this, "The Backward button was touched.",
                                Toast.LENGTH_LONG)
                        .show();
                getExecutor().addNode(rmfNode);
                move("stop");
                getExecutor().removeNode(rmfNode);
                return false;
            }
            return false;
        }
    };

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
