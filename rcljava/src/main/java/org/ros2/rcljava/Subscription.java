/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
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

package org.ros2.rcljava;

import org.ros2.rcljava.interfaces.Disposable;

/**
 * This class serves as a bridge between ROS2's rcl_subscription_t and RCLJava.
 * A Subscription must be created via
 * @{link Node#createSubscription(Class&lt;T&gt;, String, Consumer&lt;T&gt;)}
 *
 * @param <T> The type of the messages that this subscription will receive.
 */
public interface Subscription<T> extends Disposable {
  /**
   * @return The pointer to the underlying ROS2 subscription structure.
   */
  long getSubscriptionHandle();

  /**
   * @return The type of the messages that this subscription may receive.
   */
  Class<T> getMessageType();

  /**
   * @return The callback function that this subscription will trigger when
   *     a message is received.
   */
  Consumer<T> getCallback();

  /**
   * @return The pointer to the underlying ROS2 node structure.
   */
  long getNodeHandle();
}
