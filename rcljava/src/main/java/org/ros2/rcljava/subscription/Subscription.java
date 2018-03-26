/* Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
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

package org.ros2.rcljava.subscription;

import java.lang.ref.WeakReference;

import org.ros2.rcljava.consumers.Consumer;
import org.ros2.rcljava.interfaces.Disposable;
import org.ros2.rcljava.interfaces.MessageDefinition;
import org.ros2.rcljava.node.Node;

/**
 * This class serves as a bridge between ROS2's rcl_subscription_t and RCLJava.
 * A Subscription must be created via
 * @{link Node#createSubscription(Class&lt;T&gt;, String, Consumer&lt;T&gt;)}
 *
 * @param <T> The type of the messages that this subscription will receive.
 */
public interface Subscription<T extends MessageDefinition> extends Disposable {
  /**
   * @return The type of the messages that this subscription may receive.
   */
  Class<T> getMessageType();

  /**
   * @return A @{link java.lang.ref.WeakReference} to the
   * @{link org.ros2.rcljava.Node}that created this subscription.
   */
  WeakReference<Node> getNodeReference();

  void executeCallback(T message);
}
