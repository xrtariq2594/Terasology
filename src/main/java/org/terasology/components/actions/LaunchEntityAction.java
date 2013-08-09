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
package org.terasology.components.actions;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.terasology.componentSystem.items.InventorySystem;
import org.terasology.components.InventoryComponent;
import org.terasology.components.ItemComponent;
import org.terasology.components.combat.LaunchEntityComponent;
import org.terasology.components.LocalPlayerComponent;
import org.terasology.components.combat.ProjectileComponent;
import org.terasology.components.world.LocationComponent;
import org.terasology.entitySystem.EntityManager;
import org.terasology.entitySystem.EntityRef;
import org.terasology.entitySystem.EventHandlerSystem;
import org.terasology.entitySystem.Prefab;
import org.terasology.entitySystem.PrefabManager;
import org.terasology.entitySystem.ReceiveEvent;
import org.terasology.entitySystem.RegisterComponentSystem;
import org.terasology.events.ActivateEvent;
import org.terasology.game.CoreRegistry;
import org.terasology.math.AABB;
import org.terasology.math.TeraMath;
import org.terasology.physics.ImpulseEvent;
import org.terasology.physics.RigidBodyComponent;
import org.terasology.physics.character.CharacterMovementComponent;
import org.terasology.physics.shapes.BoxShapeComponent;
import org.terasology.physics.shapes.CapsuleShapeComponent;
import org.terasology.physics.shapes.CylinderShapeComponent;
import org.terasology.physics.shapes.HullShapeComponent;
import org.terasology.physics.shapes.SphereShapeComponent;
import org.terasology.rendering.logic.MeshComponent;
import org.terasology.world.block.BlockPickupComponent;

import com.bulletphysics.linearmath.QuaternionUtil;

/**
 * @author aherber 
 */
@RegisterComponentSystem
public class LaunchEntityAction implements EventHandlerSystem {

    private EntityManager entityManager;
    private InventorySystem inventorySystem;

    public void initialise() {
    	entityManager = CoreRegistry.get(EntityManager.class);
    	inventorySystem = CoreRegistry.get(InventorySystem.class);
    }

    @Override
    public void shutdown() {
    }
    
    @ReceiveEvent(components = {LaunchEntityComponent.class,ItemComponent.class})
    public void onActivate(ActivateEvent event, EntityRef entity){
    	EntityRef instigator = event.getInstigator();
    	LocationComponent instigatorLocation = instigator.getComponent(LocationComponent.class);
    	LaunchEntityComponent launchEntityComponent = entity.getComponent(LaunchEntityComponent.class);
    	ItemComponent itemComponent = entity.getComponent(ItemComponent.class);
    	Vector3f direction = new Vector3f(event.getDirection());
    	Prefab prefab = entityManager.getPrefabManager().getPrefab(launchEntityComponent.entity);
    	if (prefab != null) {
        	if(launchEntityComponent.useAmmunition){
        		ItemComponent item = prefab.getComponent(ItemComponent.class);
        		if(!inventorySystem.removeItemFromIventory(instigator, item, 1)){//Ammunition should not be fixed value
        			return;
        		}
        	}
        	LocationComponent projectileLocation = prefab.getComponent(LocationComponent.class);
            if(launchEntityComponent.useMouseLookForRotation){
        	 	Quat4f lookRotation = new Quat4f();
        		//TODO this should use the CameraComponent when ready
            	LocalPlayerComponent localPlayerComponent = instigator.getComponent(LocalPlayerComponent.class);
            	QuaternionUtil.setEuler(lookRotation, TeraMath.DEG_TO_RAD * localPlayerComponent.viewYaw, TeraMath.DEG_TO_RAD * localPlayerComponent.viewPitch, 0);
            	QuaternionUtil.quatRotate(lookRotation, new Vector3f(0,0,1), direction);
        		projectileLocation.setLocalRotation(lookRotation);
        		Vector3f spawnPosition = new Vector3f(instigatorLocation.getLocalPosition());
                spawnPosition.x += direction.x * launchEntityComponent.spawnDistance;
                spawnPosition.y += direction.y * launchEntityComponent.spawnDistance + 0.5f;//TODO this shouldn't be a fixed value
                spawnPosition.z += direction.z * launchEntityComponent.spawnDistance; 
                projectileLocation.setLocalPosition(spawnPosition);
        	}            			
            direction.scale(launchEntityComponent.distancePerSecond*(itemComponent.chargeTime/itemComponent.maxChargeTime)); 
            //TODO find a better way to copy entity;
            EntityRef temp = entityManager.create(prefab, projectileLocation.getLocalPosition());
        	EntityRef projectile  = entityManager.copy(temp);
        	temp.destroy();
        	ProjectileComponent projectileComponent = projectile.getComponent(ProjectileComponent.class);
        	projectileComponent.initialVelocity = direction;
        	projectile.saveComponent(projectileComponent);
//        	if(projectile.hasComponent(RigidBodyComponent.class)){
//        		RigidBodyComponent rigidBodyComponent = projectile.getComponent(RigidBodyComponent.class);
//        		if(!rigidBodyComponent.kinematic){
//                	projectile.saveComponent(copy);
//        			//projectile.send(new ImpulseEvent(direction));
//        		}else{
//                	copy.initialVelocity = direction;
//                	projectile.saveComponent(copy);
//        		}
//        	}

        }
    }
    
    public static Object clone(Object copyObject) {
        try {
          ByteArrayOutputStream baos = new ByteArrayOutputStream(4096);
          ObjectOutputStream oos = new ObjectOutputStream(baos);
          oos.writeObject(copyObject);
          ByteArrayInputStream bais = new ByteArrayInputStream(baos.toByteArray());
          ObjectInputStream ois = new ObjectInputStream(bais);
          Object deepCopy = ois.readObject();
          return deepCopy;
        } catch (IOException e) {
          e.printStackTrace();
        } catch(ClassNotFoundException e) {
          e.printStackTrace();
        }
        return null;
      }
    
   //recursive search for item in Inventory 
   public boolean removeItemFromIventory(EntityRef entity,ItemComponent item, int amount){
	   boolean result = false;
	   InventoryComponent inventory = entity.getComponent(InventoryComponent.class);
	   if(inventory != null){
		   for(EntityRef curItem: inventory.itemSlots){
			   if(curItem != null && curItem != EntityRef.NULL ){
				   if(curItem.hasComponent(InventoryComponent.class)){
					   if(removeItemFromIventory(curItem,item, amount)){
						   result = true;
						   break;
					   }
				   }
				   ItemComponent curItemComonent = curItem.getComponent(ItemComponent.class);
				   if(curItemComonent != null){
				   	    if(curItemComonent.name.equals(item.name)){
						   if(curItemComonent.stackCount >= amount ){
							   curItemComonent.stackCount-=amount;
							   curItem.saveComponent(curItemComonent);
							   result = true;
							   break;
				   			}	   
					    }
				   }	
			   }
		   }
	   }	   
	   return result;
   }
    
/**    private float calcSpawnDistance(ActivateEvent event, EntityRef entity) {
    	float distance = 0;
//    	EntityRef instigator = event.getInstigator();
//    	float collisionVolumeDistanceZ = getCollisionVolumeDistanceZ(instigator);
//    	float meshVolumeDisanceZ = getMeshVolumeDistanceZ(instigator);
//        distance += collisionVolumeDistanceZ > meshVolumeDisanceZ ? collisionVolumeDistanceZ : meshVolumeDisanceZ;
//    	collisionVolumeDistanceZ = getCollisionVolumeDistanceZ(entity);
//    	meshVolumeDisanceZ = getMeshVolumeDistanceZ(entity);
//        distance += collisionVolumeDistanceZ > meshVolumeDisanceZ ? collisionVolumeDistanceZ : meshVolumeDisanceZ;
//        return distance;
//    }
//    
//    private float getCollisionVolumeDistanceZ(EntityRef entity){
//    	float distance = 0;
//        BoxShapeComponent box = entity.getComponent(BoxShapeComponent.class);
//        if (box != null) {
//            Vector3f halfExtents = new Vector3f(box.extents);
//            halfExtents.scale(0.5f);
//            return halfExtents.z;
//        }
//        SphereShapeComponent sphere = entity.getComponent(SphereShapeComponent.class);
//        if (sphere != null) 
//        	return sphere.radius;
//        CapsuleShapeComponent capsule = entity.getComponent(CapsuleShapeComponent.class);
//        if (capsule != null) 
//            return capsule.radius;
//        CylinderShapeComponent cylinder = entity.getComponent(CylinderShapeComponent.class);
//        if (cylinder != null) 
//        	return cylinder.radius;
//        HullShapeComponent hull = entity.getComponent(HullShapeComponent.class);
//        if (hull != null) {
//            AABB aabb = hull.sourceMesh.getAABB();
//            Vector3f halfExtents = new Vector3f(aabb.getExtents());
//            halfExtents.scale(0.5f);
//            return halfExtents.z;
//        }
//        CharacterMovementComponent characterMovementComponent = entity.getComponent(CharacterMovementComponent.class);
//        if (characterMovementComponent != null) 
//            return characterMovementComponent.radius;
//        return distance;    
//    }
//    
//    private float getMeshVolumeDistanceZ(EntityRef entity){
//    	float distance = 0;
//    	MeshComponent meshComponent = entity.getComponent(MeshComponent.class);
//    	if(meshComponent != null){
//    		Vector3f halfExtents = new Vector3f(meshComponent.mesh.getAABB().getExtents());
//    		halfExtents.scale(0.5f);
//    		distance = halfExtents.z;
//    	}
//        return distance;    
/}*/    
}
