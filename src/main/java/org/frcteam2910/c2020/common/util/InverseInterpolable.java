package org.frcteam2910.c2020.common.util;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}
