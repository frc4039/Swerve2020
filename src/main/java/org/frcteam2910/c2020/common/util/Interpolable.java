package org.frcteam2910.c2020.common.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
