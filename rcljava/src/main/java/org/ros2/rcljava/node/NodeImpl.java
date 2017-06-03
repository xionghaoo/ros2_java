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

package org.ros2.rcljava.node;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.client.Client;
import org.ros2.rcljava.client.ClientImpl;
import org.ros2.rcljava.common.JNIUtils;
import org.ros2.rcljava.consumers.Consumer;
import org.ros2.rcljava.consumers.TriConsumer;
import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.interfaces.MessageDefinition;
import org.ros2.rcljava.interfaces.ServiceDefinition;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.publisher.PublisherImpl;
import org.ros2.rcljava.service.RMWRequestId;
import org.ros2.rcljava.service.Service;
import org.ros2.rcljava.service.ServiceImpl;
import org.ros2.rcljava.subscription.Subscription;
import org.ros2.rcljava.subscription.SubscriptionImpl;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.ref.WeakReference;
import java.lang.reflect.Method;

import java.util.Collection;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * {@inheritDoc}
 */
public class NodeImpl implements Node {
  private static final Logger logger = LoggerFactory.getLogger(NodeImpl.class);

  static {
    try {
      JNIUtils.loadLibrary(NodeImpl.class);
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  /**
   * An integer that represents a pointer to the underlying ROS2 node
   * structure (rcl_node_t).
   */
  private long handle;

  /**
   * All the @{link Subscription}s that have been created through this instance.
   */
  private final Collection<Subscription> subscriptions;

  /**
   * All the @{link Publisher}s that have been created through this instance.
   */
  private final Collection<Publisher> publishers;

  /**
   * All the @{link Service}s that have been created through this instance.
   */
  private final Collection<Service> services;

  /**
   * All the @{link Client}s that have been created through this instance.
   */
  private final Collection<Client> clients;

  /**
   * Constructor.
   *
   * @param handle A pointer to the underlying ROS2 node structure. Must not
   *     be zero.
   */
  public NodeImpl(final long handle) {
    this.handle = handle;
    this.publishers = new LinkedBlockingQueue<Publisher>();
    this.subscriptions = new LinkedBlockingQueue<Subscription>();
    this.services = new LinkedBlockingQueue<Service>();
    this.clients = new LinkedBlockingQueue<Client>();
  }

  /**
   * Create a ROS2 publisher (rcl_publisher_t) and return a pointer to
   *     it as an integer.
   *
   * @param <T> The type of the messages that will be published by the
   *     created @{link Publisher}.
   * @param handle A pointer to the underlying ROS2 node structure.
   * @param messageType The class of the messages that will be published by the
   *     created @{link Publisher}.
   * @param topic The topic to which the created @{link Publisher} will
   *     publish messages.
   * @param qosProfileHandle A pointer to the underlying ROS2 QoS profile
   *     structure.
   * @return A pointer to the underlying ROS2 publisher structure.
   */
  private static native <T extends MessageDefinition> long nativeCreatePublisherHandle(
      long handle, Class<T> messageType, String topic, long qosProfileHandle);

  /**
   * Create a ROS2 subscription (rcl_subscription_t) and return a pointer to
   *     it as an integer.
   *
   * @param <T> The type of the messages that will be received by the
   *     created @{link Subscription}.
   * @param handle A pointer to the underlying ROS2 node structure.
   * @param messageType The class of the messages that will be received by the
   *     created @{link Subscription}.
   * @param topic The topic from which the created @{link Subscription} will
   *     receive messages.
   * @param qosProfileHandle A pointer to the underlying ROS2 QoS profile
   *     structure.
   * @return A pointer to the underlying ROS2 subscription structure.
   */
  private static native <T extends MessageDefinition> long nativeCreateSubscriptionHandle(
      long handle, Class<T> messageType, String topic, long qosProfileHandle);

  /**
   * {@inheritDoc}
   */
  public final <T extends MessageDefinition> Publisher<T> createPublisher(
      final Class<T> messageType, final String topic, final QoSProfile qosProfile) {
    long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
    long publisherHandle =
        nativeCreatePublisherHandle(this.handle, messageType, topic, qosProfileHandle);
    RCLJava.disposeQoSProfile(qosProfileHandle);

    Publisher<T> publisher =
        new PublisherImpl<T>(new WeakReference<Node>(this), publisherHandle, topic);
    this.publishers.add(publisher);

    return publisher;
  }

  public final <T extends MessageDefinition> Publisher<T> createPublisher(
      final Class<T> messageType, final String topic) {
    return this.<T>createPublisher(messageType, topic, QoSProfile.DEFAULT);
  }

  /**
   * {@inheritDoc}
   */
  public final <T extends MessageDefinition> Subscription<T> createSubscription(
      final Class<T> messageType, final String topic, final Consumer<T> callback,
      final QoSProfile qosProfile) {
    long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
    long subscriptionHandle =
        nativeCreateSubscriptionHandle(this.handle, messageType, topic, qosProfileHandle);
    RCLJava.disposeQoSProfile(qosProfileHandle);

    Subscription<T> subscription = new SubscriptionImpl<T>(
        new WeakReference<Node>(this), subscriptionHandle, messageType, topic, callback);

    this.subscriptions.add(subscription);

    return subscription;
  }

  public final <T extends MessageDefinition> Subscription<T> createSubscription(
      final Class<T> messageType, final String topic, final Consumer<T> callback) {
    return this.<T>createSubscription(messageType, topic, callback, QoSProfile.DEFAULT);
  }

  /**
   * {@inheritDoc}
   */
  public final Collection<Subscription> getSubscriptions() {
    return this.subscriptions;
  }

  /**
   * {@inheritDoc}
   */
  public final Collection<Publisher> getPublishers() {
    return this.publishers;
  }

  private static native <T extends ServiceDefinition> long nativeCreateServiceHandle(
      long handle, Class<T> cls, String serviceName, long qosProfileHandle);

  public final <T extends ServiceDefinition> Service<T> createService(final Class<T> serviceType,
      final String serviceName,
      final TriConsumer<RMWRequestId, ? extends MessageDefinition, ? extends MessageDefinition>
          callback,
      final QoSProfile qosProfile) throws NoSuchFieldException, IllegalAccessException {
    Class<MessageDefinition> requestType = (Class) serviceType.getField("RequestType").get(null);

    Class<MessageDefinition> responseType = (Class) serviceType.getField("ResponseType").get(null);

    long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
    long serviceHandle =
        nativeCreateServiceHandle(this.handle, serviceType, serviceName, qosProfileHandle);
    RCLJava.disposeQoSProfile(qosProfileHandle);

    Service<T> service = new ServiceImpl<T>(new WeakReference<Node>(this), serviceHandle,
        serviceName, callback, requestType, responseType);
    this.services.add(service);

    return service;
  }

  public <T extends ServiceDefinition> Service<T> createService(final Class<T> serviceType,
      final String serviceName,
      final TriConsumer<RMWRequestId, ? extends MessageDefinition, ? extends MessageDefinition>
          callback) throws NoSuchFieldException, IllegalAccessException {
    return this.<T>createService(serviceType, serviceName, callback, QoSProfile.SERVICES_DEFAULT);
  }

  /**
   * {@inheritDoc}
   */
  public final Collection<Service> getServices() {
    return this.services;
  }

  public final <T extends ServiceDefinition> Client<T> createClient(
      final Class<T> serviceType, final String serviceName, final QoSProfile qosProfile)
      throws NoSuchFieldException, IllegalAccessException {
    Class<MessageDefinition> requestType = (Class) serviceType.getField("RequestType").get(null);

    Class<MessageDefinition> responseType = (Class) serviceType.getField("ResponseType").get(null);

    long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
    long clientHandle =
        nativeCreateClientHandle(this.handle, serviceType, serviceName, qosProfileHandle);
    RCLJava.disposeQoSProfile(qosProfileHandle);

    Client<T> client = new ClientImpl<T>(
        new WeakReference<Node>(this), clientHandle, serviceName, requestType, responseType);
    this.clients.add(client);

    return client;
  }

  public <T extends ServiceDefinition> Client<T> createClient(final Class<T> serviceType,
      final String serviceName) throws NoSuchFieldException, IllegalAccessException {
    return this.<T>createClient(serviceType, serviceName, QoSProfile.SERVICES_DEFAULT);
  }

  private static native <T extends ServiceDefinition> long nativeCreateClientHandle(
      long handle, Class<T> cls, String serviceName, long qosProfileHandle);

  /**
   * {@inheritDoc}
   */
  public final Collection<Client> getClients() {
    return this.clients;
  }

  /**
   * Destroy a ROS2 node (rcl_node_t).
   *
   * @param handle A pointer to the underlying ROS2 node
   *     structure, as an integer. Must not be zero.
   */
  private static native void nativeDispose(long handle);

  /**
   * {@inheritDoc}
   */
  public final void dispose() {
    nativeDispose(this.handle);
    this.handle = 0;
  }

  /**
   * {@inheritDoc}
   */
  public final long getHandle() {
    return this.handle;
  }
}
