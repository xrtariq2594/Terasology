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
package org.terasology.componentSystem.common;

import javax.vecmath.Vector3f;

import org.terasology.components.HealthComponent;
import org.terasology.components.ItemComponent;
import org.terasology.components.world.LocationComponent;

import org.terasology.components.combat.ProjectileComponent;
import org.terasology.entityFactory.DeadEntityFactory;
import org.terasology.entitySystem.*;
import org.terasology.events.*;
import org.terasology.logic.manager.AudioManager;
import org.terasology.logic.manager.SoundManager;
import org.terasology.physics.PhysicsSystem;
/**
 * @author aherber 
 * TODO Define hit Zones like,legs,feet,arms,head,organs,hands, fingers  
 * TODO Handle hit event for non Projectiles collision volumes 
 */
@RegisterComponentSystem
public class DamageSystem implements EventHandlerSystem {
    
	
	DeadEntityFactory deadEntityFactory = new DeadEntityFactory();
	@In
	PhysicsSystem physicsSystem;
	
    @Override
    public void initialise() {
    }

    @Override
    public void shutdown() {
    }
           
    @ReceiveEvent(components =  {ProjectileComponent.class,ItemComponent.class})
    public void onProjectileHit(HitEvent event, EntityRef entity) {
    	ProjectileComponent projectileComponent = entity.getComponent(ProjectileComponent.class);
    	ItemComponent item = entity.getComponent(ItemComponent.class);
    	event.getOther().send(new DamageEvent(item.baseDamage,projectileComponent.owner));
    	event.cancel();
    	pushBack(event);
    }
    
    public void pushBack(HitEvent event){
    	EntityRef other = event.getOther();
        org.terasology.physics.character.CharacterMovementComponent characterMovement= other.getComponent( org.terasology.physics.character.CharacterMovementComponent.class);
    	Vector3f direction = new Vector3f(event.getHitNormal());
    	direction.scale(3);
    	direction.negate();
    	if(characterMovement != null){
    		characterMovement.getVelocity().add(direction);
    		other.saveComponent(characterMovement);
    	}
    }
    
    @ReceiveEvent(components = {HealthComponent.class})
    public void onDeath(NoHealthEvent event, EntityRef entity) {
	    if(!entity.hasComponent( org.terasology.world.block.BlockComponent.class)){
			EntityRef deadEntity = deadEntityFactory.newInstance(entity);
		    entity.destroy();
		    //TODO temporary "exagarated death animation" 
		    if(physicsSystem != null){
		    	while(physicsSystem.getRigidBodyForEntity(deadEntity) == null){};
		    	physicsSystem.getRigidBodyForEntity(deadEntity).applyImpulse(new Vector3f(0,20,10), new Vector3f(0,0.75f,2.5f));
		    }
	    }
	}
   
}
