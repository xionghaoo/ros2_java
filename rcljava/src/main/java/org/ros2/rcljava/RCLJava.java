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

package org.ros2.rcljava;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Collection;
import java.util.Map;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros2.rcljava.client.Client;
import org.ros2.rcljava.common.JNIUtils;
import org.ros2.rcljava.executors.SingleThreadedExecutor;
import org.ros2.rcljava.interfaces.MessageDefinition;
import org.ros2.rcljava.interfaces.ServiceDefinition;
import org.ros2.rcljava.node.ComposableNode;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.NodeImpl;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.service.RMWRequestId;
import org.ros2.rcljava.service.Service;
import org.ros2.rcljava.subscription.Subscription;
import org.ros2.rcljava.timer.WallTimer;

/**
 * Entry point for the ROS2 Java API, similar to the rclcpp API.
 */
public final class RCLJava {
  private static final Logger logger = LoggerFactory.getLogger(RCLJava.class);

  /**
   * Private constructor so this cannot be instantiated.
   */
  private RCLJava() {}

  /**
   * All the @{link Node}s that have been created.
   */
  private static Collection<Node> nodes;

  private static void cleanup() {
    for (Node node : nodes) {
      for (Subscription subscription : node.getSubscriptions()) {
        subscription.dispose();
      }

      for (Publisher publisher : node.getPublishers()) {
        publisher.dispose();
      }

      for (WallTimer timer : node.getTimers()) {
        timer.dispose();
      }

      for (Service service : node.getServices()) {
        service.dispose();
      }

      for (Client client : node.getClients()) {
        client.dispose();
      }

      node.dispose();
    }
  }

  static {
    nodes = new LinkedBlockingQueue<Node>();

    RMW_TO_TYPESUPPORT = new ConcurrentSkipListMap<String, String>() {
      {
        put("rmw_fastrtps_cpp", "rosidl_typesupport_introspection_c");
        put("rmw_opensplice_cpp", "rosidl_typesupport_opensplice_c");
        put("rmw_connext_cpp", "rosidl_typesupport_connext_c");
        put("rmw_connext_dynamic_cpp", "rosidl_typesupport_introspection_c");
      }
    };

    // NOTE(esteve): disabling shutdown hook for now to avoid JVM crashes
    // Runtime.getRuntime().addShutdownHook(new Thread() {
    //   public void run() {
    //     cleanup();
    //   }
    // });
  }

  /**
   * The identifier of the currently active RMW implementation.
   */
  private static String rmwImplementation = null;

  /**
   * Flag to indicate if RCLJava has been fully initialized, with a valid RMW
   *   implementation.
   */
  private static boolean initialized = false;

  /**
   * A mapping between RMW implementations and their typesupports.
   */
  private static final Map<String, String> RMW_TO_TYPESUPPORT;

  /**
   * @return true if RCLJava has been fully initialized, false otherwise.
   */
  public static boolean isInitialized() {
    return RCLJava.initialized;
  }

  /**
   * Initialize the RCLJava API. If successful, a valid RMW implementation will
   *   be loaded and accessible, enabling the creating of ROS2 entities
   *   (@{link Node}s, @{link Publisher}s and @{link Subscription}s.
   */
  public static void rclJavaInit() {
    synchronized (RCLJava.class) {
      if (!initialized) {
        if (RCLJava.rmwImplementation == null) {
          for (Map.Entry<String, String> entry : RMW_TO_TYPESUPPORT.entrySet()) {
            try {
              setRMWImplementation(entry.getKey());
              break;
            } catch (UnsatisfiedLinkError ule) {
              logger.info("UnsatisfiedLinkError", ule);
            } catch (Exception exc) {
              logger.info("Exception", exc);
            }
          }
        }
        if (RCLJava.rmwImplementation == null) {
          logger.error("No RMW implementation found");
          System.exit(1);
        } else {
          nativeRCLJavaInit();
          logger.info("Using RMW implementation: {}", getRMWIdentifier());
          initialized = true;
        }
      }
    }
  }

  /**
   * Initialize the underlying rcl layer.
   */
  private static native void nativeRCLJavaInit();

  /**
   * Create a ROS2 node (rcl_node_t) and return a pointer to it as an integer.
   *
   * @param nodeName The name that will identify this node in a ROS2 graph.
   * @param namespace The namespace of the node.
   * @return A pointer to the underlying ROS2 node structure.
   */
  private static native long nativeCreateNodeHandle(String nodeName, String namespace);

  public static String getTypesupportIdentifier() {
    return RMW_TO_TYPESUPPORT.get(nativeGetRMWIdentifier());
  }

  public static void setRMWImplementation(final String rmwImplementation) throws Exception {
    synchronized (RCLJava.class) {
      JNIUtils.loadLibrary(RCLJava.class);
      RCLJava.rmwImplementation = RCLJava.getRMWIdentifier();
    }
  }

  /**
   * @return The identifier of the currently active RMW implementation via the
   *     native ROS2 API.
   */
  private static native String nativeGetRMWIdentifier();

  /**
   * @return The identifier of the currently active RMW implementation.
   */
  public static String getRMWIdentifier() {
    return nativeGetRMWIdentifier();
  }

  /**
   * Call the underlying ROS2 rcl mechanism to check if ROS2 has been shut
   *   down.
   *
   * @return true if RCLJava hasn't been shut down, false otherwise.
   */
  private static native boolean nativeOk();

  /**
   * @return true if RCLJava hasn't been shut down, false otherwise.
   */
  public static boolean ok() {
    return nativeOk();
  }

  /**
   * Create a @{link Node}.
   *
   * @param nodeName The name that will identify this node in a ROS2 graph.
   * @return A @{link Node} that represents the underlying ROS2 node
   *     structure.
   */
  public static Node createNode(final String nodeName) {
    return createNode(nodeName, "");
  }

  /**
   * Create a @{link Node}.
   *
   * @param nodeName The name that will identify this node in a ROS2 graph.
   * @param namespace The namespace of the node.
   * @return A @{link Node} that represents the underlying ROS2 node
   *     structure.
   */
  public static Node createNode(final String nodeName, final String namespace) {
    long nodeHandle = nativeCreateNodeHandle(nodeName, namespace);
    Node node = new NodeImpl(nodeHandle);
    nodes.add(node);
    return node;
  }

  public static void spin(final Node node) {
    SingleThreadedExecutor executor = new SingleThreadedExecutor();
    ComposableNode composableNode = new ComposableNode() {
      public Node getNode() {
        return node;
      }
    };
    executor.addNode(composableNode);
    executor.spin();
    executor.removeNode(composableNode);
  }

  public static void spin(final ComposableNode composableNode) {
    SingleThreadedExecutor executor = new SingleThreadedExecutor();
    executor.addNode(composableNode);
    executor.spin();
    executor.removeNode(composableNode);
  }

  public static void spinOnce(final Node node) {
    SingleThreadedExecutor executor = new SingleThreadedExecutor();
    ComposableNode composableNode = new ComposableNode() {
      public Node getNode() {
        return node;
      }
    };
    executor.addNode(composableNode);
    executor.spinOnce();
    executor.removeNode(composableNode);
  }

  public static void spinOnce(final ComposableNode composableNode) {
    SingleThreadedExecutor executor = new SingleThreadedExecutor();
    executor.addNode(composableNode);
    executor.spinOnce();
    executor.removeNode(composableNode);
  }

  public static void spinSome(final Node node) {
    SingleThreadedExecutor executor = new SingleThreadedExecutor();
    ComposableNode composableNode = new ComposableNode() {
      public Node getNode() {
        return node;
      }
    };
    executor.addNode(composableNode);
    executor.spinSome();
    executor.removeNode(composableNode);
  }

  public static void spinSome(final ComposableNode composableNode) {
    SingleThreadedExecutor executor = new SingleThreadedExecutor();
    executor.addNode(composableNode);
    executor.spinSome();
    executor.removeNode(composableNode);
  }

  private static native void nativeShutdown();

  public static void shutdown() {
    cleanup();
    nativeShutdown();
  }

  public static long convertQoSProfileToHandle(final QoSProfile qosProfile) {
    int history = qosProfile.getHistory().getValue();
    int depth = qosProfile.getDepth();
    int reliability = qosProfile.getReliability().getValue();
    int durability = qosProfile.getDurability().getValue();
    boolean avoidROSNamespaceConventions = qosProfile.getAvoidROSNamespaceConventions();

    return nativeConvertQoSProfileToHandle(
        history, depth, reliability, durability, avoidROSNamespaceConventions);
  }

  private static native long nativeConvertQoSProfileToHandle(int history, int depth,
      int reliability, int durability, boolean avoidROSNamespaceConventions);

  public static void disposeQoSProfile(final long qosProfileHandle) {
    nativeDisposeQoSProfile(qosProfileHandle);
  }

  private static native void nativeDisposeQoSProfile(long qosProfileHandle);
}
