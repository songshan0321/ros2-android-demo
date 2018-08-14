/* Copyright 2017 Esteve Fernandez <esteve@apache.org>
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

import java.util.concurrent.TimeUnit;

import android.util.Log;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.timer.WallTimer;

public class rmfNode extends BaseComposableNode {

  private static String logtag = rmfNode.class.getName();

  private final String topic;

  private Publisher<std_msgs.msg.String> publisher;

  public rmfNode(final String name, final String topic) {
    super(name);
    this.topic = topic;
    this.publisher = this.node.<std_msgs.msg.String>createPublisher(
        std_msgs.msg.String.class, this.topic);
  }

  public void left() {
    Log.d(logtag, "rmfNode::left()");
    std_msgs.msg.String msg = new std_msgs.msg.String();
    msg.setData("left");
    this.publisher.publish(msg);
  }

  public void right() {
    Log.d(logtag, "rmfNode::right()");
    std_msgs.msg.String msg = new std_msgs.msg.String();
    msg.setData("right");
    this.publisher.publish(msg);
  }
  public void forward() {
    Log.d(logtag, "rmfNode::forward()");
    std_msgs.msg.String msg = new std_msgs.msg.String();
    msg.setData("forward");
    this.publisher.publish(msg);
  }
  public void backward() {
    Log.d(logtag, "rmfNode::backward()");
    std_msgs.msg.String msg = new std_msgs.msg.String();
    msg.setData("backward");
    this.publisher.publish(msg);
  }
  public void stop() {
      Log.d(logtag, "rmfNode::stop()");
      std_msgs.msg.String msg = new std_msgs.msg.String();
      msg.setData("stop");
      this.publisher.publish(msg);
  }
}
