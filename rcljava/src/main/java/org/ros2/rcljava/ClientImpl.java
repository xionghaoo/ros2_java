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

import org.ros2.rcljava.interfaces.MessageDefinition;
import org.ros2.rcljava.interfaces.ServiceDefinition;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.ref.WeakReference;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Future;

public class ClientImpl<T extends ServiceDefinition> implements Client<T> {
  private static final Logger logger = LoggerFactory.getLogger(ClientImpl.class);

  static {
    try {
      System.loadLibrary("rcljavaClientImpl__" + RCLJava.getRMWIdentifier());
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  private final WeakReference<Node> nodeReference;
  private final long nodeHandle;
  private final long clientHandle;
  private final String serviceName;
  private long sequenceNumber = 0;
  private Map<Long, RCLFuture> pendingRequests;

  private final Class<MessageDefinition> requestType;
  private final Class<MessageDefinition> responseType;

  public ClientImpl(final WeakReference<Node> nodeReference,
      final long nodeHandle, final long clientHandle,
      final String serviceName,
      final Class<MessageDefinition> requestType,
      final Class<MessageDefinition> responseType) {
    this.nodeReference = nodeReference;
    this.nodeHandle = nodeHandle;
    this.clientHandle = clientHandle;
    this.serviceName = serviceName;
    this.requestType = requestType;
    this.responseType = responseType;
    this.pendingRequests = new HashMap<Long, RCLFuture>();
  }

  public final <U extends MessageDefinition, V extends MessageDefinition>
      Future<V> sendRequest(final U request) {
    synchronized (pendingRequests) {
      sequenceNumber++;
      nativeSendClientRequest(clientHandle, sequenceNumber,
          request.getFromJavaConverterInstance(), request.getToJavaConverterInstance(),
          request);
      RCLFuture<V> future = new RCLFuture<V>(this.nodeReference);
      pendingRequests.put(sequenceNumber, future);
      return future;
    }
  }

  public final <U extends MessageDefinition> void handleResponse(final RMWRequestId header,
      final U response) {
    synchronized (pendingRequests) {
      long sequenceNumber = header.sequenceNumber;
      RCLFuture<U> future = pendingRequests.remove(sequenceNumber);
      future.set(response);
    }
  }

  public final long getClientHandle() {
    return clientHandle;
  }

  private static native void nativeSendClientRequest(long clientHandle,
      long sequenceNumber, long requestFromJavaConverterHandle,
      long requestToJavaConverterHandle, MessageDefinition requestMessage);

  public final Class<MessageDefinition> getRequestType() {
    return this.requestType;
  }

  public final Class<MessageDefinition> getResponseType() {
    return this.responseType;
  }

  /**
   * {@inheritDoc}
   */
  public final void dispose() {
    // TODO(esteve): implement
  }
}
