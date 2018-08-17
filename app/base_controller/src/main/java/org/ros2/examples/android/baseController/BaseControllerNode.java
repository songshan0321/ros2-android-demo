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

package org.ros2.examples.android.baseController;

import java.util.concurrent.TimeUnit;

import android.util.Log;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.timer.WallTimer;

public class BaseControllerNode extends BaseComposableNode {

  private double linearVelX = 0.0;
  private double angVelZ = 0.0;
  private geometry_msgs.msg.Vector3 linear;
  private geometry_msgs.msg.Vector3 angular;

  private final String topic;
//  private long lastUpdateTimeMs = 0;

  private static String logtag = BaseControllerNode.class.getName();
//  private static final long MAX_TIME_BETWEEN_UPDATES_MS = 1000;


  private Publisher<geometry_msgs.msg.Twist> publisher;

  public BaseControllerNode(final String name, final String topic) {
    super(name);
    this.topic = topic;
    this.publisher = this.node.<geometry_msgs.msg.Twist>createPublisher(
            geometry_msgs.msg.Twist.class, this.topic);
  }

  public void setTwistValues(double linearVelX, double angVelZ){
    this.linearVelX = linearVelX;
    this.angVelZ = angVelZ;
    Log.d(logtag, "synchronized setting: (" + this.linearVelX + "," + this.angVelZ + ")");
  }

  // Transforms twist speeds (linear and angular) to twist msg.
  public void pubTwist() {
    Log.d(logtag, "publish: (" + this.linearVelX + "," + this.angVelZ + ")");
    geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist();
    this.linear.setX(this.linearVelX);
    this.linear.setY(0.0);
    this.linear.setZ(0.0);
    this.angular.setX(0.0);
    this.angular.setY(0.0);
    this.angular.setZ(this.angVelZ);
    msg.setLinear(linear);
    msg.setAngular(angular);
    this.publisher.publish(msg);
  }
}
