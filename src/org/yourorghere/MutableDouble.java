package org.yourorghere;

public class MutableDouble {

    private double value;

    MutableDouble(double d) {
        this.value = d;
    }

    void add(double d) {
        this.value = this.value + d;
    }

    void sub(double d) {
        this.value = this.value - d;
    }

    public double getValue() {
        return value;
    }

    public void setValue(double value) {
        this.value = value;
    }

}
