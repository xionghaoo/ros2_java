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
import org.ros2.rcljava.interfaces.MessageDefinition;

/**
 * This class serves as a bridge between ROS2's rcl_publisher_t and RCLJava.
 * A Publisher must be created via
 * @{link Node#createPublisher(Class&lt;T&gt;, String)}
 *
 * @param <T> The type of the messages that this publisher will publish.
 */
public interface Publisher<T extends MessageDefinition> extends Disposable {
  /**
   * Publish a message.
   *
   * @param message An instance of the &lt;T&gt; parameter.
   */
  void publish(final T message);

  /**
   * An integer that represents a pointer to the underlying ROS2 node
   * structure (rcl_node_t).
   */
  long getNodeHandle();

  /**
   * An integer that represents a pointer to the underlying ROS2 publisher
   * structure (rcl_publisher_t).
   */
  long getPublisherHandle();
}
