/*
 * Copyright 2012 Benjamin Glatzel <benjamin.glatzel@me.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.terasology.physics;

import java.util.Arrays;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.terasology.world.block.BlockComponent;
import org.terasology.entitySystem.EntityRef;
import org.terasology.entitySystem.EventReceiver;
import org.terasology.entitySystem.EventSystem;
import org.terasology.game.CoreRegistry;
import org.terasology.math.AABB;
import org.terasology.math.QuatUtil;
import org.terasology.math.Vector3i;
import org.terasology.performanceMonitor.PerformanceMonitor;
import org.terasology.world.BlockChangedEvent;
import org.terasology.world.BlockEntityRegistry;
import org.terasology.world.WorldProvider;
import org.terasology.math.Vector3fUtil;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionFilterGroups;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.CollisionWorld.LocalConvexResult;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.dispatch.GhostPairCallback;
import com.bulletphysics.collision.dispatch.ManifoldResult;
import com.bulletphysics.collision.dispatch.PairCachingGhostObject;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.voxel.VoxelWorldShape;
import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.GLShapeDrawer;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.Transform;
import com.google.common.collect.Lists;
import com.sun.corba.se.impl.orbutil.concurrent.DebugMutex;

/**
 * Renders blocks using the Bullet physics library.
 *
 * @author Benjamin Glatzel <benjamin.glatzel@me.com>
 */
// TODO: Merge this with Physics System
public class BulletPhysics implements EventReceiver<BlockChangedEvent> {

    private static final Logger logger = LoggerFactory.getLogger(BulletPhysics.class);
    
    private final Deque<RigidBodyRequest> _insertionQueue = new LinkedList<RigidBodyRequest>();
    private final Deque<RigidBody> _removalQueue = new LinkedList<RigidBody>();

    private final CollisionDispatcher _dispatcher;
    private final BroadphaseInterface _broadphase;
    private final CollisionConfiguration _defaultCollisionConfiguration;
    private final SequentialImpulseConstraintSolver _sequentialImpulseConstraintSolver;
    private final DiscreteDynamicsWorld _discreteDynamicsWorld;
    private final BlockEntityRegistry blockEntityRegistry;
    private final CollisionGroupManager collisionGroupManager;

    private int step = 1;

    public BulletPhysics(WorldProvider world) {
        collisionGroupManager = CoreRegistry.get(CollisionGroupManager.class);
        _broadphase = new DbvtBroadphase();
        _broadphase.getOverlappingPairCache().setInternalGhostPairCallback(new GhostPairCallback());
        _defaultCollisionConfiguration = new DefaultCollisionConfiguration();
        _dispatcher = new CollisionDispatcher(_defaultCollisionConfiguration);
        _sequentialImpulseConstraintSolver = new SequentialImpulseConstraintSolver();
        _discreteDynamicsWorld = new DiscreteDynamicsWorld(_dispatcher, _broadphase, _sequentialImpulseConstraintSolver, _defaultCollisionConfiguration);
        _discreteDynamicsWorld.setGravity(new Vector3f(0f, -15f, 0f));
        blockEntityRegistry = CoreRegistry.get(BlockEntityRegistry.class);
        CoreRegistry.get(EventSystem.class).registerEventReceiver(this, BlockChangedEvent.class, BlockComponent.class);
        PhysicsWorldWrapper wrapper = new PhysicsWorldWrapper(world);
        VoxelWorldShape worldShape = new VoxelWorldShape(wrapper);
        Matrix3f rot = new Matrix3f();
        rot.setIdentity();
        DefaultMotionState blockMotionState = new DefaultMotionState(new Transform(new Matrix4f(rot, new Vector3f(0, 0, 0), 1.0f)));
        RigidBodyConstructionInfo blockConsInf = new RigidBodyConstructionInfo(0, blockMotionState, worldShape, new Vector3f());
        RigidBody rigidBody = new RigidBody(blockConsInf);
        rigidBody.setCollisionFlags(CollisionFlags.STATIC_OBJECT | rigidBody.getCollisionFlags());
        _discreteDynamicsWorld.addRigidBody(rigidBody, combineGroups(StandardCollisionGroup.WORLD), (short)(CollisionFilterGroups.ALL_FILTER ^ CollisionFilterGroups.STATIC_FILTER));
        //TODO Debugging
        IDebugDraw test = new GLDebugDrawer(LWJGL.getGL());
        test.setDebugMode(1);
        _discreteDynamicsWorld.setDebugDrawer(test);
    }

    public DynamicsWorld getWorld() {
        return _discreteDynamicsWorld;
    }

    // TODO: Wrap ghost object?
    public PairCachingGhostObject createCollider(Vector3f pos, ConvexShape shape, List<CollisionGroup> groups, List<CollisionGroup> filters) {
        return createCollider(pos, shape, groups, filters, 0);
    }

    public static short combineGroups(CollisionGroup ... groups) {
        return combineGroups(Arrays.asList(groups));
    }

    public static short combineGroups(Iterable<CollisionGroup> groups) {
        short flags = 0;
        for (CollisionGroup group : groups) {
            flags |= group.getFlag();
        }
        return flags;
    }

    public PairCachingGhostObject createCollider(Vector3f pos, ConvexShape shape, List<CollisionGroup> groups, List<CollisionGroup> filters, int collisionFlags) {
        return createCollider(pos, shape, combineGroups(groups), combineGroups(filters), collisionFlags);
    }

    private PairCachingGhostObject createCollider(Vector3f pos, ConvexShape shape, short groups, short filters, int collisionFlags) {
        Transform startTransform = new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), pos, 1.0f));
        PairCachingGhostObject result = new PairCachingGhostObject();
        result.setWorldTransform(startTransform);
        result.setCollisionShape(shape);
        result.setCollisionFlags(collisionFlags);
        _discreteDynamicsWorld.addCollisionObject(result, groups, filters);
        return result;
    }

    public void addRigidBody(RigidBody body) {
        _insertionQueue.add(new RigidBodyRequest(body, CollisionFilterGroups.DEFAULT_FILTER, (short)(CollisionFilterGroups.DEFAULT_FILTER | CollisionFilterGroups.STATIC_FILTER | CollisionFilterGroups.SENSOR_TRIGGER)));
    }

    public void addRigidBody(RigidBody body, List<CollisionGroup> groups, List<CollisionGroup> filter) {
        _insertionQueue.add(new RigidBodyRequest(body, combineGroups(groups), combineGroups(filter)));
    }

    public void addRigidBody(RigidBody body, short groups, short filter) {
        _insertionQueue.add(new RigidBodyRequest(body, groups, (short)(filter | CollisionFilterGroups.SENSOR_TRIGGER)));
    }

    public void removeRigidBody(RigidBody body) {
        _removalQueue.add(body);
    }

    public void removeCollider(GhostObject collider) {
        _discreteDynamicsWorld.removeCollisionObject(collider);
    }

    public Iterable<EntityRef> scanArea(AABB area, Iterable<CollisionGroup> collisionFilter) {
        // TODO: Add the aabbTest method from newer versions of bullet to TeraBullet, use that instead
        BoxShape shape = new BoxShape(area.getExtents());
        GhostObject scanObject = createCollider(area.getCenter(), shape, CollisionFilterGroups.SENSOR_TRIGGER, combineGroups(collisionFilter), CollisionFlags.NO_CONTACT_RESPONSE);
        // This in particular is overkill
        _broadphase.calculateOverlappingPairs(_dispatcher);
        List<EntityRef> result = Lists.newArrayList();
        for (int i = 0; i < scanObject.getNumOverlappingObjects(); ++i) {
            CollisionObject other = scanObject.getOverlappingObject(i);
            Object userObj = other.getUserPointer();
            if (userObj instanceof EntityRef) {
                result.add((EntityRef)userObj);
            }
        }
        removeCollider(scanObject);
        return result;
    }

    public HitResult rayTrace(Vector3f from, Vector3f direction, float distance) {
        Vector3f to = new Vector3f(direction);
        to.normalize();
        to.scale(distance);
        to.add(from);
        double test = Vector3fUtil.calcdist(from, to);
        if(distance != test){
        	System.out.println("Distance fo Raytracin ist not correct: distance:"+distance+" > "+"test:"+test);
        }
        CollisionWorld.ClosestRayResultWithUserDataCallback closest = new CollisionWorld.ClosestRayResultWithUserDataCallback(from, to);
        closest.collisionFilterGroup = CollisionFilterGroups.SENSOR_TRIGGER;
        _discreteDynamicsWorld.rayTest(from, to, closest);
        if (closest.userData instanceof Vector3i) {
            return new HitResult(blockEntityRegistry.getOrCreateEntityAt((Vector3i)closest.userData), closest.hitPointWorld, closest.hitNormalWorld);
        } else if (closest.userData instanceof EntityRef) {
            return new HitResult((EntityRef) closest.userData, closest.hitPointWorld, closest.hitNormalWorld);
        }
        return new HitResult();
    }
    
    public HitResult rayTrace(Vector3f from, Vector3f direction, float distance, short collisionFilter) {
        Vector3f to = new Vector3f(direction);
        to.normalize();
        to.scale(distance);
        to.add(from);
        CollisionWorld.ClosestRayResultWithUserDataCallback closest = new CollisionWorld.ClosestRayResultWithUserDataCallback(from, to);
        closest.collisionFilterGroup = collisionFilter;
        _discreteDynamicsWorld.rayTest(from, to, closest);
        if (closest.userData instanceof Vector3i) {
            return new HitResult(blockEntityRegistry.getOrCreateEntityAt((Vector3i)closest.userData), closest.hitPointWorld, closest.hitNormalWorld);
        } else if (closest.userData instanceof EntityRef) {
            return new HitResult((EntityRef) closest.userData, closest.hitPointWorld, closest.hitNormalWorld);
        }
        return new HitResult();
    }

    public HitResult rayTrace(Vector3f from, Vector3f direction, float distance, CollisionGroup ... collisionGroups) {
        Vector3f to = new Vector3f(direction);
        to.scale(distance);
        to.add(from);

        short filter = combineGroups(collisionGroups);

        CollisionWorld.ClosestRayResultWithUserDataCallback closest = new CollisionWorld.ClosestRayResultWithUserDataCallback(from, to);
        closest.collisionFilterGroup = CollisionFilterGroups.ALL_FILTER;
        closest.collisionFilterMask = filter;
        _discreteDynamicsWorld.rayTest(from, to, closest);
        if (closest.userData instanceof Vector3i) {
            return new HitResult(blockEntityRegistry.getOrCreateEntityAt((Vector3i)closest.userData), closest.hitPointWorld, closest.hitNormalWorld, (Vector3i)closest.userData);
        } else if (closest.userData instanceof EntityRef) {
            return new HitResult((EntityRef) closest.userData, closest.hitPointWorld, closest.hitNormalWorld);
        }
        return new HitResult();
    }

    @Override
    public void onEvent(BlockChangedEvent event, EntityRef entity) {
        Vector3f min = event.getBlockPosition().toVector3f();
        min.sub(new Vector3f(0.6f, 0.6f, 0.6f));
        Vector3f max = event.getBlockPosition().toVector3f();
        max.add(new Vector3f(0.6f, 0.6f, 0.6f));
        _discreteDynamicsWorld.awakenRigidBodiesInArea(min, max);
    }
    
   /** Checks and handles the Translation of the CollsionObject whose velocity within the next Frame (delta) is bigger than the radius of
    * its BoundingSphere  in order to prevent it from Tunneling through other CollisionObjects.
    * 
    * */
    public int calculateMinSteppingForCollisionObject(float collisionRadius, double hitDistance, double distanceInFrame){
    	int steps = 1;
    	if(hitDistance+collisionRadius < distanceInFrame){//collision in next frame and step calculation needed ?
    		System.out.println("distanceInFrame:"+distanceInFrame);
    		System.out.println("hitDistance:"+hitDistance);
    		System.out.println("radius:"+collisionRadius);
    		System.out.println("hitDistance < distanceInFrame: "+hitDistance+" < "+distanceInFrame);
    		if(hitDistance <= collisionRadius){
    			System.out.println("hitDistance <= collisionRadius: "+hitDistance+"<="+collisionRadius);
    			steps = (int)(distanceInFrame/collisionRadius)+1;
    		}else{
        		double difference = distanceInFrame-hitDistance;
    			double distanceToCollision = 0;
    			if(difference < hitDistance){ 
    				distanceToCollision = difference;
    			}else{
    				distanceToCollision = hitDistance;
    			}
    			System.out.println("distanceToCollision:"+distanceToCollision);
				//steps = (int)(distanceInFrame/distanceToCollision);
//    			while(distanceToCollision % (distanceInFrame/steps) > collisionRadius){
//        			System.out.println("distanceToCollision % (distanceInFrame/steps):="+((distanceInFrame/steps) % distanceToCollision));
//    				steps++;
//    			}
    			
    			while((distanceInFrame/steps) % distanceToCollision > collisionRadius){
        			System.out.println("distanceToCollision % (distanceInFrame/steps):="+((distanceInFrame/steps) % distanceToCollision));
    				steps++;
    			}
				System.out.println("steps with WHILE >collsion Rdius with modulo:"+steps);
				steps = (int)(distanceInFrame/(collisionRadius/2))+1;
				System.out.println("steps with division collsion Radius:"+steps);
				steps = 1;
				while(distanceInFrame/steps > (collisionRadius/2)){
    				steps++;
    			}
    			System.out.println("steps with WHILE >collsion Radius:"+steps);
				

    		}
			System.out.println("steps:"+steps);
    	}
    	return steps;
    }
    
    /**
     * Checks and handles the Translation of the CollsionObject whose velocity within the next Frame (delta) is bigger than the radius of
     * its BoundingSphere  in order to prevent it from Tunneling through other CollisionObjects.
     *  
     * @param delta 	time in ms since last tick
     * @param colObj	CollisionObject to check
     * 
     *  TODO use several raycasts to improve Obstacle Detection for bigger Objects.   
     *  TODO check the movement of the colliding entity and check if collision will still occur after the target(s) movement within the next frame 
     *  (if it is a moving object). If not the translation of the Collision Object is not needed.
     *  
     */
    private void handleFastMovement(float delta, CollisionObject colObj){
    	if(colObj.isActive() && !colObj.isStaticOrKinematicObject()){//Only check Moving Objects and CollisionObjects that can move
    		Vector3f velocity = new Vector3f();
    		colObj.getInterpolationLinearVelocity(velocity);
    		float distanceVelocity = velocity.length();
    		if(distanceVelocity > 0){
    			Vector3f center = new Vector3f();
    			float[] radius = new float[1];
    			colObj.getCollisionShape().getBoundingSphere(center, radius);
    			double distanceInFrame = distanceVelocity*delta;
    			if(distanceInFrame > radius[0]){
    				short collisionFilterMask = colObj.getBroadphaseHandle().collisionFilterMask;
    				Transform transform = new Transform();
    				colObj.getInterpolationWorldTransform(transform);
    				System.out.println("distanceInFrame > radius: Handle Possible Tunneling");
    				//TODO This may not be appropriate for big/bigger CollsionObjects. Edges of Bigger Objects can clip through other collisionObjects 
    				org.terasology.physics.HitResult hitResult = rayTrace(transform.origin, velocity, (float)distanceInFrame, collisionFilterMask);
    				if(hitResult.isHit()){
    	    			//System.out.println("distanceInFrame:"+distanceInFrame);
    					Vector3f hitPoint = hitResult.getHitPoint();
    					double hitDistance = Vector3fUtil.calcdist(transform.origin, hitPoint);
    					System.out.println("delta:"+delta);
    					int tempStep = calculateMinSteppingForCollisionObject(radius[0], hitDistance, distanceInFrame);
    					//Set Stepping for  Frame to guarantee Collision detection
    					if( tempStep > step ){ 
    						step = tempStep;
    					}
    					System.out.println("hitpoint:"+hitPoint);
    					transform.origin.set(hitPoint);
    					System.out.println("tranform.origin"+transform.origin);
    					colObj.setInterpolationWorldTransform(transform);
    					EntityRef entity = (EntityRef)colObj.getUserPointer();
    					entity.send(new CollideEvent(hitResult.getEntity(), hitPoint, hitResult.getHitNormal())); 
    					//Send Collision Event. Shouldn't be noticeable for very fast moving objects anyway
    				}	
    			}	
    		}
    	}
    }
    
    
    
    public void testAABBPlane(){
    	
    }
    
    public void testAABBAABB(){
    	
    }
    
    public void testOBBPlane(){
    	
    }
    
    public void testOBBOBB(){
    	
    }
    
    public void testSpherePlane(){
    	
    }
    
    public void testSphereSphere(){
    	
    }
    
    public void testSphereAABB(){
    	
    }
    
    

   /* * Performs a sweep collision test and returns the results as a list of HitResults<br/>
    * You have to use different Transforms for start and end (at least distance > 0.4f).
    * SweepTest will not see a collision if it starts INSIDE an object and is moving AWAY from its center.
    */
   public List<HitResult> sweepTest(CollisionShape shape, Transform start, Transform end, List<HitResult> results) {
       results.clear();
       if (!(shape instanceof ConvexShape)) {
           logger.error("ERROR: Trying to sweep test with incompatible mesh shape!");
           return results;
       }
       _discreteDynamicsWorld.convexSweepTest((ConvexShape) shape, start, end, new InternalSweepListener(results));
       return results;
   }
   
    private class InternalSweepListener extends CollisionWorld.ConvexResultCallback {
        private List<HitResult> results;

        public InternalSweepListener(List<HitResult> results) {
            this.results = results;
        }
        @Override
        public float addSingleResult(LocalConvexResult lcr, boolean bln) {
        	EntityRef entity = (EntityRef) lcr.hitCollisionObject.getUserPointer();
            HitResult hitResult = new HitResult(entity,lcr.hitPointLocal,lcr.hitNormalLocal); 
            results.add(hitResult);
            return lcr.hitFraction;
        }
    }
    
    
    /**TODO Use a Component (for example HitDetection) to define which CollisionObjects should be checked ??? 
     * This would improve the runtime since only the defined Object would be checked. */
    private HitResult checkCollision(float delta, CollisionObject colObj){
    	HitResult hitResult = null;
    	Vector3f center = new Vector3f();
		float[] radius = new float[1];
		colObj.getCollisionShape().getBoundingSphere(center, radius);
		Transform transform = new Transform();
		colObj.getInterpolationWorldTransform(transform);
		Vector3f velocity = new Vector3f();
		colObj.getInterpolationLinearVelocity(velocity);
		float distanceVelocity = velocity.length();
		double distanceInFrame = distanceVelocity*delta;
		short collisionFilterMask = colObj.getBroadphaseHandle().collisionFilterMask;
		hitResult = rayTrace(transform.origin, velocity, (float)distanceInFrame, collisionFilterMask);
		if(!hitResult.isHit()){
			if(radius[0] > 0.5){//TODO should this be a static value ? Should this be relative to the radius ?	
				Quat4f rotation = new Quat4f();
				transform.getRotation(rotation);
				//check Y-Axis
				Vector3f yAxis = QuatUtil.getUpVector(rotation);
				yAxis.normalize();
				yAxis.scale(radius[0]);
				Vector3f origin = transform.origin;
				origin.add(yAxis);
				hitResult = rayTrace(origin, velocity, (float)distanceInFrame, collisionFilterMask);
				if(!hitResult.isHit()){
					origin = transform.origin;
					origin.sub(yAxis);
					hitResult = rayTrace(origin, velocity, (float)distanceInFrame, collisionFilterMask);
					if(!hitResult.isHit()){
						//check X-Axis
						Vector3f xAxis = QuatUtil.getRightVector(rotation);
						xAxis.normalize();
						xAxis.scale(radius[0]);
						origin.add(xAxis);
						hitResult = rayTrace(origin, velocity, (float)distanceInFrame, collisionFilterMask);
						if(!hitResult.isHit()){
							origin = transform.origin;
							origin.sub(xAxis);
							hitResult = rayTrace(origin, velocity, (float)distanceInFrame, collisionFilterMask);
						}
					}
				}
			}
		}
    	return hitResult;
    }
    
    /**TODO Use a Component (for example HitDetection) to define which CollisionObjects should be checked ??? 
     * This would improve the runtime since only the defined Object would be checked. 
     * This may or may not be an optimal solution to check the collision between two Bodies*/
    private boolean checkCollision(CollisionObject colObj, CollisionObject otherColObj){
    	boolean result = false;
    	if( colObj.checkCollideWith(otherColObj)){
			PersistentManifold manifold = new PersistentManifold();
    		ManifoldResult manifoldResult = new ManifoldResult(colObj, otherColObj);
        	CollisionAlgorithm algorithm = _dispatcher.findAlgorithm(colObj, otherColObj,manifold);
        	algorithm.processCollision(colObj, otherColObj, _discreteDynamicsWorld.getDispatchInfo(), manifoldResult);
        	result = manifold.getNumContacts() > 0;
    	}
    	return result;
    }
    
    /**TODO Use a Component (for example HitDetection) to define which CollisionObjects should be checked ??? 
     * This would improve the runtime since only the defined Object would be checked. 
     * This may or may not be an optimal solution to check the collision between two Bodies*/
    private boolean checkCollisionInNextFrame(float delta ,CollisionObject colObj, CollisionObject otherColObj){
    	boolean result = false;
		HitResult hitResult = checkCollision(delta, colObj);
    	if(hitResult.isHit()){
    		
    		if(!otherColObj.isStaticObject() && otherColObj.isActive()){//&& colObj.checkCollideWith(otherColObj)
    			Vector3f positionInNextFrame = new Vector3f();
    			Vector3f center = new Vector3f();
    			float[] radius = new float[1];
    			colObj.getCollisionShape().getBoundingSphere(center, radius);
    			Vector3f centerOther = new Vector3f();
    			float[] radiusOther = new float[1];
    			otherColObj.getCollisionShape().getBoundingSphere(centerOther, radiusOther);
    			HitResult hitResultOther = checkCollision(delta, otherColObj);
    			if(hitResultOther.isHit()){
    				positionInNextFrame = hitResultOther.getHitPoint();
    			}else{
    				//calculate position in next frame
    				Vector3f velocityOther = new Vector3f();
    				colObj.getInterpolationLinearVelocity(velocityOther);
        			double distanceInFrame = velocityOther.length()*delta;
        			velocityOther.normalize();
        			velocityOther.scale((float)distanceInFrame);
    				centerOther.add(velocityOther);
    				positionInNextFrame = centerOther;
    			}
				//check if collision between the two objects can could occur during next frame
    			double distance = Vector3fUtil.calcdist(hitResult.getHitPoint(), positionInNextFrame);
    			result = distance < radius[0]+radiusOther[0];
    		}
    	}
    	return result;
    }
     
    /**TODO Use a Component (for example HitDetection) to define which CollisionObjects should be checked ??? 
     * This would improve the runtime since only the defined Object would be checked. */
    private void handleNextFrame(float delta){
		for(CollisionObject colObj : _discreteDynamicsWorld.getCollisionObjectArray()){
			handleFastMovement(delta, colObj);
		}
    }

    public void update(float delta) {
        processQueuedBodies();
        try {
            PerformanceMonitor.startActivity("Step Simulation");
           // handleNextFrame(delta);
           // _discreteDynamicsWorld.stepSimulation(delta, step, delta/step);
            _discreteDynamicsWorld.stepSimulation(delta, step);
            PerformanceMonitor.endActivity();
        } catch (Exception e) {
            logger.error("Error running simulation step.", e);
        }
    }

    private synchronized void processQueuedBodies() {
        while (!_insertionQueue.isEmpty()) {
            RigidBodyRequest request = _insertionQueue.poll();
            _discreteDynamicsWorld.addRigidBody(request.body, request.groups, request.filter);
        }
        while (!_removalQueue.isEmpty()) {
            RigidBody body = _removalQueue.poll();
            _discreteDynamicsWorld.removeRigidBody(body);
        }
    }

    private static class RigidBodyRequest
    {
        final RigidBody body;
        final short groups;
        final short filter;

        public RigidBodyRequest(RigidBody body, short groups, short filter) {
            this.body = body;
            this.groups = groups;
            this.filter = filter;
        }
    }

}
