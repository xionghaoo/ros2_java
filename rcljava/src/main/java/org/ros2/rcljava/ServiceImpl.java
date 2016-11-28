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

public class ServiceImpl<T extends ServiceDefinition> implements Service<T> {

  private final long nodeHandle;
  private final long serviceHandle;
  private final String serviceName;
  private final TriConsumer<RMWRequestId, ?, ?> callback;

  private final Class<MessageDefinition> requestType;
  private final Class<MessageDefinition> responseType;

  public ServiceImpl(final long nodeHandle, final long serviceHandle,
      final String serviceName,
      final TriConsumer<RMWRequestId, ?, ?> callback,
      final Class<MessageDefinition> requestType,
      final Class<MessageDefinition> responseType) {
    this.nodeHandle = nodeHandle;
    this.serviceHandle = serviceHandle;
    this.serviceName = serviceName;
    this.callback = callback;
    this.requestType = requestType;
    this.responseType = responseType;
  }

  public final TriConsumer<RMWRequestId, ?, ?> getCallback() {
    return callback;
  }

  public final long getServiceHandle() {
    return serviceHandle;
  }

  public final Class<MessageDefinition> getRequestType() {
    return this.requestType;
  }

  public final Class<MessageDefinition> getResponseType() {
    return this.responseType;
  }
}
