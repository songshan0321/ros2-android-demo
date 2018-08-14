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
import android.view.View;
import android.view.View.OnClickListener;
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

    // Create two button listener
    Button buttonLeft = (Button)findViewById(R.id.buttonLeft);
    buttonLeft.setOnClickListener(leftListener);

    Button buttonRight = (Button)findViewById(R.id.buttonRight);
    buttonRight.setOnClickListener(rightListener);

    RCLJava.rclJavaInit();

    // Start a ROS2 node
    rmfNode = new rmfNode("android_rmf_node", "RMFChatter");
  }

  // Create an anonymous implementation of OnClickListener
  private OnClickListener leftListener = new OnClickListener() {
    public void onClick(final View view) {
      Log.d(logtag, "onClick() called - left button");
      Toast
          .makeText(rmfActivity.this, "The Go Left button was clicked.",
                    Toast.LENGTH_LONG)
          .show();
      Log.d(logtag, "onClick() ended - left button");

      getExecutor().addNode(rmfNode);
      move("left");
      getExecutor().removeNode(rmfNode);
    }
  };

  // Create an anonymous implementation of OnClickListener
  private OnClickListener rightListener = new OnClickListener() {
    public void onClick(final View view) {
      Log.d(logtag, "onClick() called - right button");
      Toast
          .makeText(rmfActivity.this, "The Go Right button was clicked.",
                    Toast.LENGTH_LONG)
          .show();
      Log.d(logtag, "onClick() ended - right button");

      getExecutor().addNode(rmfNode);
      move("right");
      getExecutor().removeNode(rmfNode);
    }
  };

  private void move(String direction){
      if (direction == "left"){
          rmfNode.left();
      }
      else if (direction == "right"){
          rmfNode.right();
      }
      else{
          Log.d(logtag, "invalid direction msg");
      }
  }

}
