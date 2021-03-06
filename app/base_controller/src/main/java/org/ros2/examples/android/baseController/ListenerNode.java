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

import android.widget.TextView;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.subscription.Subscription;

//import java.text.DateFormat;
//import java.text.SimpleDateFormat;
import java.util.Date;

public class ListenerNode extends BaseComposableNode {

  private final String topic;

  private final TextView listenerView;

  private final TextView dateView;

  public Date dateobj = new Date();

  private Subscription<std_msgs.msg.Int16> subscriber;

  public ListenerNode(final String name, final String topic,
                      final TextView listenerView, final TextView dateView) {
    super(name);
    this.topic = topic;
    this.listenerView = listenerView;
    this.dateView = dateView;
    this.subscriber = this.node.<std_msgs.msg.Int16>createSubscription(
        std_msgs.msg.Int16.class, this.topic, msg -> update(msg.getData())
    );
  }

  public void update(final int number){
//      DateFormat df = new SimpleDateFormat("dd/MM/yy HH:mm:ss");
      this.dateobj = new Date();
      this.listenerView.setText(String.valueOf(number));
//      this.dateView.setText("Active: " + df.format(dateobj));
      this.dateView.setText("Active");
  }


}
