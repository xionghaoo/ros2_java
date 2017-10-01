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

package org.ros2.rcljava.executors;

import java.util.Map;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.client.Client;
import org.ros2.rcljava.common.JNIUtils;
import org.ros2.rcljava.executors.AnyExecutable;
import org.ros2.rcljava.executors.Executor;
import org.ros2.rcljava.interfaces.MessageDefinition;
import org.ros2.rcljava.interfaces.ServiceDefinition;
import org.ros2.rcljava.node.ComposableNode;
import org.ros2.rcljava.service.RMWRequestId;
import org.ros2.rcljava.service.Service;
import org.ros2.rcljava.subscription.Subscription;
import org.ros2.rcljava.timer.Timer;

public class BaseExecutor {
  private static final Logger logger = LoggerFactory.getLogger(BaseExecutor.class);

  static {
    try {
      JNIUtils.loadImplementation(BaseExecutor.class);
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  private BlockingQueue<ComposableNode> nodes = new LinkedBlockingQueue<ComposableNode>();

  private ConcurrentHashMap<Long, Subscription> subscriptionHandles =
      new ConcurrentHashMap<Long, Subscription>();

  private ConcurrentHashMap<Long, Timer> timerHandles = new ConcurrentHashMap<Long, Timer>();

  private ConcurrentHashMap<Long, Service> serviceHandles = new ConcurrentHashMap<Long, Service>();

  private ConcurrentHashMap<Long, Client> clientHandles = new ConcurrentHashMap<Long, Client>();

  protected void addNode(ComposableNode node) {
    this.nodes.add(node);
  }

  protected void removeNode(ComposableNode node) {
    this.nodes.remove(node);
  }

  protected void executeAnyExecutable(AnyExecutable anyExecutable) {
    if (anyExecutable.timer != null) {
      anyExecutable.timer.callTimer();
      anyExecutable.timer.executeCallback();
      timerHandles.remove(anyExecutable.timer.getHandle());
    }

    if (anyExecutable.subscription != null) {
      MessageDefinition message = nativeTake(
          anyExecutable.subscription.getHandle(), anyExecutable.subscription.getMessageType());
      if (message != null) {
        anyExecutable.subscription.executeCallback(message);
      }
      subscriptionHandles.remove(anyExecutable.subscription.getHandle());
    }

    if (anyExecutable.service != null) {
      Class<MessageDefinition> requestType = anyExecutable.service.getRequestType();
      Class<MessageDefinition> responseType = anyExecutable.service.getResponseType();

      MessageDefinition requestMessage = null;
      MessageDefinition responseMessage = null;

      try {
        requestMessage = requestType.newInstance();
        responseMessage = responseType.newInstance();
      } catch (InstantiationException ie) {
        ie.printStackTrace();
      } catch (IllegalAccessException iae) {
        iae.printStackTrace();
      }

      if (requestMessage != null && responseMessage != null) {
        long requestFromJavaConverterHandle = requestMessage.getFromJavaConverterInstance();
        long requestToJavaConverterHandle = requestMessage.getToJavaConverterInstance();
        long requestDestructorHandle = requestMessage.getDestructorInstance();
        long responseFromJavaConverterHandle = responseMessage.getFromJavaConverterInstance();
        long responseToJavaConverterHandle = responseMessage.getToJavaConverterInstance();
        long responseDestructorHandle = responseMessage.getDestructorInstance();

        RMWRequestId rmwRequestId =
            nativeTakeRequest(anyExecutable.service.getHandle(), requestFromJavaConverterHandle,
                requestToJavaConverterHandle, requestDestructorHandle, requestMessage);
        if (rmwRequestId != null) {
          anyExecutable.service.executeCallback(rmwRequestId, requestMessage, responseMessage);
          nativeSendServiceResponse(anyExecutable.service.getHandle(), rmwRequestId,
              responseFromJavaConverterHandle, responseToJavaConverterHandle,
              responseDestructorHandle, responseMessage);
        }
      }
      serviceHandles.remove(anyExecutable.service.getHandle());
    }

    if (anyExecutable.client != null) {
      Class<MessageDefinition> requestType = anyExecutable.client.getRequestType();
      Class<MessageDefinition> responseType = anyExecutable.client.getResponseType();

      MessageDefinition requestMessage = null;
      MessageDefinition responseMessage = null;

      try {
        requestMessage = requestType.newInstance();
        responseMessage = responseType.newInstance();
      } catch (InstantiationException ie) {
        ie.printStackTrace();
      } catch (IllegalAccessException iae) {
        iae.printStackTrace();
      }

      if (requestMessage != null && responseMessage != null) {
        long requestFromJavaConverterHandle = requestMessage.getFromJavaConverterInstance();
        long requestToJavaConverterHandle = requestMessage.getToJavaConverterInstance();
        long responseFromJavaConverterHandle = responseMessage.getFromJavaConverterInstance();
        long responseToJavaConverterHandle = responseMessage.getToJavaConverterInstance();
        long responseDestructorHandle = responseMessage.getDestructorInstance();

        RMWRequestId rmwRequestId =
            nativeTakeResponse(anyExecutable.client.getHandle(), responseFromJavaConverterHandle,
                responseToJavaConverterHandle, responseDestructorHandle, responseMessage);

        if (rmwRequestId != null) {
          anyExecutable.client.handleResponse(rmwRequestId, responseMessage);
        }
      }
      clientHandles.remove(anyExecutable.client.getHandle());
    }
  }

  protected void waitForWork(long timeout) {
    this.subscriptionHandles.clear();

    for (ComposableNode node : this.nodes) {
      for (Subscription<MessageDefinition> subscription : node.getNode().getSubscriptions()) {
        this.subscriptionHandles.put(subscription.getHandle(), subscription);
      }

      for (Timer timer : node.getNode().getTimers()) {
        this.timerHandles.put(timer.getHandle(), timer);
      }

      for (Service<ServiceDefinition> service : node.getNode().getServices()) {
        this.serviceHandles.put(service.getHandle(), service);
      }

      for (Client<ServiceDefinition> client : node.getNode().getClients()) {
        this.clientHandles.put(client.getHandle(), client);
      }
    }

    int subscriptionsSize = 0;
    int timersSize = 0;
    int clientsSize = 0;
    int servicesSize = 0;

    for (ComposableNode node : this.nodes) {
      subscriptionsSize += node.getNode().getSubscriptions().size();
      timersSize += node.getNode().getTimers().size();
      clientsSize += node.getNode().getClients().size();
      servicesSize += node.getNode().getServices().size();
    }

    if (subscriptionsSize == 0 && timersSize == 0 && clientsSize == 0 && servicesSize == 0) {
      return;
    }

    long waitSetHandle = nativeGetZeroInitializedWaitSet();

    nativeWaitSetInit(waitSetHandle, subscriptionsSize, 0, timersSize, clientsSize, servicesSize);

    nativeWaitSetClearSubscriptions(waitSetHandle);

    nativeWaitSetClearTimers(waitSetHandle);

    nativeWaitSetClearServices(waitSetHandle);

    nativeWaitSetClearClients(waitSetHandle);

    for (ComposableNode node : this.nodes) {
      for (Subscription<MessageDefinition> subscription : node.getNode().getSubscriptions()) {
        nativeWaitSetAddSubscription(waitSetHandle, subscription.getHandle());
      }

      for (Timer timer : node.getNode().getTimers()) {
        nativeWaitSetAddTimer(waitSetHandle, timer.getHandle());
      }

      for (Service<ServiceDefinition> service : node.getNode().getServices()) {
        nativeWaitSetAddService(waitSetHandle, service.getHandle());
      }

      for (Client<ServiceDefinition> client : node.getNode().getClients()) {
        nativeWaitSetAddClient(waitSetHandle, client.getHandle());
      }
    }

    nativeWait(waitSetHandle, timeout);
  }
  protected AnyExecutable getNextExecutable() {
    AnyExecutable anyExecutable = new AnyExecutable();

    for (ComposableNode node : this.nodes) {
      for (Timer timer : node.getNode().getTimers()) {
        if (timer.isReady()) {
          anyExecutable.timer = timer;
          return anyExecutable;
        }
      }

      for (Map.Entry<Long, Subscription> entry : subscriptionHandles.entrySet()) {
        anyExecutable.subscription = entry.getValue();
        return anyExecutable;
      }

      for (Service service : node.getNode().getServices()) {
        anyExecutable.service = service;
        return anyExecutable;
      }

      for (Client<ServiceDefinition> client : node.getNode().getClients()) {
        anyExecutable.client = client;
        return anyExecutable;
      }
    }
    return null;
  }

  protected void spinSome() {
    AnyExecutable anyExecutable = getNextExecutable();
    if (anyExecutable == null) {
      waitForWork(0);
      anyExecutable = getNextExecutable();
    }

    if (anyExecutable != null) {
      executeAnyExecutable(anyExecutable);
    }
  }

  protected void spinOnce(long timeout) {
    AnyExecutable anyExecutable = getNextExecutable();
    if (anyExecutable == null) {
      waitForWork(timeout);
      anyExecutable = getNextExecutable();
    }

    if (anyExecutable != null) {
      executeAnyExecutable(anyExecutable);
    }
  }

  private static native long nativeGetZeroInitializedWaitSet();

  private static native void nativeWaitSetInit(long waitSetHandle, int numberOfSubscriptions,
      int numberOfGuardConditions, int numberOfTimers, int numberOfClients, int numberOfServices);

  private static native void nativeWaitSetClearSubscriptions(long waitSetHandle);

  private static native void nativeWaitSetAddSubscription(
      long waitSetHandle, long subscriptionHandle);

  private static native void nativeWait(long waitSetHandle, long timeout);

  private static native MessageDefinition nativeTake(
      long subscriptionHandle, Class<MessageDefinition> messageType);

  private static native void nativeWaitSetClearTimers(long waitSetHandle);

  private static native void nativeWaitSetClearServices(long waitSetHandle);

  private static native void nativeWaitSetAddService(long waitSetHandle, long serviceHandle);

  private static native void nativeWaitSetClearClients(long waitSetHandle);

  private static native void nativeWaitSetAddClient(long waitSetHandle, long clientHandle);

  private static native void nativeWaitSetAddTimer(long waitSetHandle, long timerHandle);

  private static native RMWRequestId nativeTakeRequest(long serviceHandle,
      long requestFromJavaConverterHandle, long requestToJavaConverterHandle,
      long requestDestructorHandle, MessageDefinition requestMessage);

  private static native void nativeSendServiceResponse(long serviceHandle, RMWRequestId header,
      long responseFromJavaConverterHandle, long responseToJavaConverterHandle,
      long responseDestructorHandle, MessageDefinition responseMessage);

  private static native RMWRequestId nativeTakeResponse(long clientHandle,
      long responseFromJavaConverterHandle, long responseToJavaConverterHandle,
      long responseDestructorHandle, MessageDefinition responseMessage);
}
