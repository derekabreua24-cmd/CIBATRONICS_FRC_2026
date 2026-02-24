package org.wpilib.annotation;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Minimal stub of WPILib's NoDiscard annotation so external JAR metadata resolves during compile.
 * This fixes a compiler warning emitted when the WPILib command JAR references this annotation but
 * the annotation class is not present on the classpath.
 */
@Documented
@Target({ElementType.TYPE, ElementType.METHOD, ElementType.FIELD, ElementType.PARAMETER})
@Retention(RetentionPolicy.CLASS)
public @interface NoDiscard {
  boolean value() default true;
}
