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

package org.ros2.rcljava.client;

import java.lang.ref.WeakReference;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Future;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.common.JNIUtils;
import org.ros2.rcljava.concurrent.RCLFuture;
import org.ros2.rcljava.interfaces.MessageDefinition;
import org.ros2.rcljava.interfaces.ServiceDefinition;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.service.RMWRequestId;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ClientImpl<T extends ServiceDefinition> implements Client<T> {
  private static final Logger logger = LoggerFactory.getLogger(ClientImpl.class);

  static {
    try {
      JNIUtils.loadLibrary(ClientImpl.class);
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  private final WeakReference<Node> nodeReference;
  private long handle;
  private final String serviceName;
  private long sequenceNumber = 0;
  private Map<Long, RCLFuture> pendingRequests;

  private final Class<MessageDefinition> requestType;
  private final Class<MessageDefinition> responseType;

  public ClientImpl(final WeakReference<Node> nodeReference, final long handle,
      final String serviceName, final Class<MessageDefinition> requestType,
      final Class<MessageDefinition> responseType) {
    this.nodeReference = nodeReference;
    this.handle = handle;
    this.serviceName = serviceName;
    this.requestType = requestType;
    this.responseType = responseType;
    this.pendingRequests = new HashMap<Long, RCLFuture>();
  }

  public final <U extends MessageDefinition, V extends MessageDefinition> Future<V> sendRequest(
      final U request) {
    synchronized (pendingRequests) {
      sequenceNumber++;
      nativeSendClientRequest(handle, sequenceNumber, request.getFromJavaConverterInstance(),
          request.getToJavaConverterInstance(), request.getDestructorInstance(), request);
      RCLFuture<V> future = new RCLFuture<V>(this.nodeReference);
      pendingRequests.put(sequenceNumber, future);
      return future;
    }
  }

  public final <U extends MessageDefinition> void handleResponse(
      final RMWRequestId header, final U response) {
    synchronized (pendingRequests) {
      long sequenceNumber = header.sequenceNumber;
      RCLFuture<U> future = pendingRequests.remove(sequenceNumber);
      future.set(response);
    }
  }

  private static native void nativeSendClientRequest(long handle, long sequenceNumber,
      long requestFromJavaConverterHandle, long requestToJavaConverterHandle,
      long requestDestructorHandle, MessageDefinition requestMessage);

  public final Class<MessageDefinition> getRequestType() {
    return this.requestType;
  }

  public final Class<MessageDefinition> getResponseType() {
    return this.responseType;
  }

  /**
   * Destroy a ROS2 client (rcl_client_t).
   *
   * @param nodeHandle A pointer to the underlying ROS2 node structure that
   *     created this client, as an integer. Must not be zero.
   * @param handle A pointer to the underlying ROS2 client
   *     structure, as an integer. Must not be zero.
   */
  private static native void nativeDispose(long nodeHandle, long handle);

  /**
   * {@inheritDoc}
   */
  public final void dispose() {
    Node node = this.nodeReference.get();
    if (node != null) {
      nativeDispose(node.getHandle(), this.handle);
      this.handle = 0;
    }
  }

  /**
   * {@inheritDoc}
   */
  public final long getHandle() {
    return this.handle;
  }
}
