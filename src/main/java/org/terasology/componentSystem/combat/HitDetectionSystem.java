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
package org.terasology.componentSystem.combat;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.xml.stream.events.EntityReference;

import org.terasology.componentSystem.UpdateSubscriberSystem;
import org.terasology.components.combat.HitDetectionComponent;
import org.terasology.components.world.LocationComponent;
import org.terasology.entitySystem.*;
import org.terasology.entitySystem.event.AddComponentEvent;
import org.terasology.entitySystem.event.RemovedComponentEvent;
import org.terasology.events.ActivateEvent;
import org.terasology.events.HitEvent;
import org.terasology.events.NoHealthEvent;
import org.terasology.game.CoreRegistry;
import org.terasology.physics.CollideEvent;
import org.terasology.physics.HitDetection;
import org.terasology.physics.HitDetectionContext;
import org.terasology.physics.HitDetectionContextImpl;
import org.terasology.physics.PhysicsSystem;
import org.terasology.physics.RigidBodyComponent;
import org.terasology.physics.TriggerComponent;

import com.bulletphysics.dynamics.RigidBody;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

/**
 * @author aherber 
 */
@RegisterComponentSystem
public class HitDetectionSystem implements EventHandlerSystem, UpdateSubscriberSystem {

	public HitDetectionContextImpl context;
	private volatile float delta = 0;
	@Override
	public void initialise() {
		context = new HitDetectionContextImpl();
        CoreRegistry.put(HitDetectionSystem.class, this);
	}

	@Override
	public void shutdown() {
	}
	
	/***
	 * @param event
	 * @param entity
	 * 
	 * 
	 */
	@ReceiveEvent(components = HitDetectionComponent.class,  priority = EventPriority.PRIORITY_NORMAL)
	public void onCollision(CollideEvent event, EntityRef entity) {
		EntityRef other = event.getOtherEntity();
		HitDetectionComponent hitDetection = entity.getComponent(HitDetectionComponent.class);
		if(hitDetection.hitBlocks || !other.hasComponent(org.terasology.world.block.BlockComponent.class)){
			if(hitDetection.trigger != HitDetection.DISABLED ){
				HitEvent hitEvent = new HitEvent(entity,other, event.getHitPoint(), event.getHitNormal());
				if(hitDetection.trigger == HitDetection.ONCE){//Collision Event doesnt need to be saved for later checks
					hitDetection.trigger = HitDetection.DISABLED;
					entity.saveComponent(hitDetection);
					entity.send(hitEvent);
				}else if(context.addHit(entity, other)){
					entity.send(hitEvent);
				}
			}
		}	
	}
	@ReceiveEvent(components = {HitDetectionComponent.class}, priority = EventPriority.PRIORITY_NORMAL)
	public void addHitDetection(AddComponentEvent event, EntityRef entity) {
	    context.getOncePerEntity().remove(entity);
	    context.getPeriodic().remove(entity);
	    context.getPeriodicPerEntity().remove(entity);  
	}
	
	@ReceiveEvent(components = {HitDetectionComponent.class})
	public void removeHitDetection(RemovedComponentEvent event, EntityRef entity) {
		HitDetectionComponent hitDetection = entity.getComponent(HitDetectionComponent.class);
	    switch (hitDetection.trigger) {
		case PERIODIC:
		    context.getPeriodic().remove(entity);
			break;
		case PERIODIC_PER_ENTITY:
		    context.getPeriodicPerEntity().remove(entity);
			break;
		case ONCE_PER_ENTITY:
			context.getOncePerEntity().remove(entity);
			break;
		default:
			break;
		}
	}
	
	/***
	 * @param event
	 * @param entity
	 * 
	 * 
	 */
	private synchronized void processHits(float delta){

		//Periodic
		HashMap<EntityRef,Float> removeMap = Maps.newHashMap();
		for(Entry<EntityRef,Float> entry:  context.getPeriodic().entrySet()){
			Float test =  entry.getValue();
			test = test-delta;
			entry.setValue(test);
			if(test <= 0){
				removeMap.put(entry.getKey(), test);
				entry = null;
			}
		}
		context.setPeriodic(Maps.newHashMap(Maps.difference(context.getPeriodic(),removeMap).entriesOnlyOnLeft()));
		//PeriodicPer Entity
		Set<Entry<EntityRef, HashMap<EntityRef, Float>>> entries = context.getPeriodicPerEntity().entrySet();
		for(Map.Entry<EntityRef,HashMap<EntityRef,Float>> entry : entries){
			Set<Entry<EntityRef, Float>> subEntries = entry.getValue().entrySet();
			if(entry != null && entry.getKey().exists()){
				for(Entry<EntityRef,Float> entryTemp :  subEntries){
					if(entryTemp != null && (entryTemp.getKey().exists())){
						Float test =  entryTemp.getValue();
						test = test-delta;
						entryTemp.setValue(test);
						if(test <= 0){
							entryTemp = null;//Eintrag entfernern
						}
					}
				}
			}else{
				entry = null;//Eintrag entfernern
			}
		}
	}
	
	@Override
	public void update(float delta) {
		this.delta += delta;//handle lags
		float consume = this.delta;
		processHits(consume);
		this.delta-=consume;
	}
}
