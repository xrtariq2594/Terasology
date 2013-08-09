package org.terasology.math;

import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4f;

public class QuatUtil {
	   
    /*
     *Orients the Rotations of the Entity to face the given point
     */
    private void facePoint(Quat4f q, float x,float y, float z){
    	double rotx,roty;
    	rotx = Math.atan2( y, z );
    	 if (z >= 0) {
    	   roty = -Math.atan2( x * Math.cos(rotx), z );
    	 }else{
    	   roty = Math.atan2( x * Math.cos(rotx), -z );
    	 }
    }
    
    /**
	 * Transform vector by a quaternion.
	 * v' = q*v*q_inv
	 */
	static void transform_noopt( Quat4f q, Vector3f v )
	{
		Quat4f q_ = new Quat4f();
		q_.inverse( q );
		
		Quat4f qv = new Quat4f();
		qv.set( v.x, v.y, v.z, 1f );
		
		Quat4f res = new Quat4f( q );
		res.mul( qv );
		res.mul( q_ );
		
		v.x = res.x;
		v.y = res.y;
		v.z = res.z;
	}
	
	//This runs 1.4 times faster then the Standart Mathlibary solution.
	//Should give the same result //needs some review
	/**Transform/rotate vector by a quaternion.
	*	v' = q*v*q_inv
    *	@author Joerg 'Herkules' Plewe
	*/
	public final void transform( Vector3f v, Quat4f q ) {; 
		float w0 = q.w - v.x*q.x - q.y*v.y - q.z*v.z; 
		float x0 = q.w*v.x + q.x + q.y*v.z - q.z*v.y; 
		float y0 = q.w*v.y + q.y - q.x*v.z + q.z*v.x; 
		float z0 = q.w*v.z + q.z + q.x*v.y - q.y*v.x; 
		float norm = 1.0f/(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z); 
		float x1 = -norm * q.x; 
		float y1 = -norm * q.y; 
		float z1 = -norm * q.z; 
		float w1 = norm * q.w; 
		v.x = w0*x1 + w1*x0 + y0*z1 - z0*y1;
		v.y = w0*y1 + w1*y0 - x0*z1 + z0*x1;
		v.z = w0*z1 + w1*z0 + x0*y1 - y0*x1; 
	}
	

    private static Quat4f createQuaternionFromAxisAndAngle( Vector3f axis, double angle ) {
    	double sin_a = Math.sin( angle / 2 );
    	double cos_a = Math.cos( angle / 2 );
    	// Normalize the Vector (used to be done on Queternion 'q').
    	axis.normalize();
    	// Use a Vector so we can call normalize (replaced '+' with '*').
    	Vector4f q = new Vector4f();
    	q.x = (float) (axis.x * sin_a);
    	q.y = (float) (axis.y * sin_a);
    	q.z = (float) (axis.z * sin_a);
    	q.w = (float) cos_a;
    	// Convert to a Quat4f and return.
    	return new Quat4f( q );
    }

    /**
     * <code>lookAt</code> is a convienence method for auto-setting the
     * quaternion based on a direction and an up vector. It computes
     * the rotation to transform the z-axis to point into 'direction'
     * and the y-axis to 'up'.
     *
     * @param direction
     *            where to look at in terms of local coordinates
     * @param up
     *            a vector indicating the local up direction.
     */
    public static void lookAt(Quat4f quanterion, Vector3f direction, Vector3f up ) {
    	Vector3f yAxis = new Vector3f();
    	Vector3f zAxis = new Vector3f();
    	Vector3f xAxis = new Vector3f();
        zAxis.set(direction);
        zAxis.normalize();
        xAxis.cross(up, direction);
        xAxis.normalize();
        yAxis.cross(direction, xAxis);
        yAxis.normalize();
        Matrix3f rotationMatrix = new Matrix3f(xAxis.x,yAxis.x,zAxis.x,
											   xAxis.y,yAxis.y,zAxis.y,
											   xAxis.z,yAxis.z,zAxis.z);
        quanterion.set(rotationMatrix); 
    }
    
    public static double getPitch(Quat4f q)
    {
      return Math.atan2(2*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    }

    public static double getYaw(Quat4f q)
    {
      return Math.asin(-2*(q.x*q.z - q.w*q.y));
    }

    public static double getRoll(Quat4f q)
    {
      return Math.atan2(2*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    }
    
    public static Vector3f getForwardVector(Quat4f q) {
        return new Vector3f( 2 * (q.x * q.z + q.w * q.y), 
                        2 * (q.y * q.x - q.w * q.x),
                        1 - 2 * (q.x * q.x + q.y * q.y));
    }
     
    public static Vector3f getUpVector(Quat4f q){
        return new Vector3f( 2 * (q.x * q.y - q.w * q.z), 
                        1 - 2 * (q.x * q.x + q.z * q.z),
                        2 * (q.y * q.z + q.w * q.x));
    }
     
    public static Vector3f getRightVector(Quat4f q){
        return new Vector3f( 1 - 2 * (q.y * q.y + q.z * q.z),
                        2 * (q.x * q.y + q.w * q.z),
                        2 * (q.x * q.z - q.w * q.y));
    }
       
}
