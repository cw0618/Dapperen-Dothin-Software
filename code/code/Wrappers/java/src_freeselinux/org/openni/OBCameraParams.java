package org.openni;


public class OBCameraParams {

    //[fx,fy,cx,cy]
    private float[] d_intr_p;

    //[fx,fy,cx,cy]
    private float[] c_intr_p;

    //[r00,r01,r02;r10,r11,r12;r20,r21,r22]
    private float[] d2c_r;

    //[t1,t2,t3]
    private float[] d2c_t;

    //[k1,k2,k3,p1,p2]
    private float[] d_k;

    //[k1,k2,k3,p1,p2]
    private float[] c_k;

    public float[] getD_intr_p() {
        return d_intr_p;
    }

    public void setD_intr_p(float[] d_intr_p) {
        this.d_intr_p = d_intr_p;
    }

    public float[] getC_intr_p() {
        return c_intr_p;
    }

    public void setC_intr_p(float[] c_intr_p) {
        this.c_intr_p = c_intr_p;
    }

    public float[] getD2c_r() {
        return d2c_r;
    }

    public void setD2c_r(float[] d2c_r) {
        this.d2c_r = d2c_r;
    }

    public float[] getD2c_t() {
        return d2c_t;
    }

    public void setD2c_t(float[] d2c_t) {
        this.d2c_t = d2c_t;
    }

    public float[] getD_k() {
        return d_k;
    }

    public void setD_k(float[] d_k) {
        this.d_k = d_k;
    }

    public float[] getC_k() {
        return c_k;
    }

    public void setC_k(float[] c_k) {
        this.c_k = c_k;
    }

}
