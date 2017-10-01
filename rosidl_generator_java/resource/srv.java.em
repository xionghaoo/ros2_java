package @(package_name).@(subfolder);

import org.ros2.rcljava.common.JNIUtils;
import org.ros2.rcljava.interfaces.ServiceDefinition;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class @(type_name) implements ServiceDefinition {

  private static final Logger logger = LoggerFactory.getLogger(@(type_name).class);

  static {
    try {
      JNIUtils.loadTypesupport(@(type_name).class);
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  public static native long getServiceTypeSupport();

  public static final Class<@(type_name)_Request> RequestType = @(type_name)_Request.class;

  public static final Class<@(type_name)_Response> ResponseType = @(type_name)_Response.class;
}
