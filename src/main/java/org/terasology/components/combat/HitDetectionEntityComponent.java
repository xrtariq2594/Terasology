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
package org.terasology.components.combat;

import java.util.HashSet;
import java.util.Map;

import org.terasology.entitySystem.Component;
import org.terasology.entitySystem.EntityRef;
import org.terasology.game.Timer;
import org.terasology.physics.HitDetection;

import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

public final class HitDetectionEntityComponent implements Component {
	public boolean active = true;
	public boolean hit = false;
    public HitDetection trigger = HitDetection.ONCE;
    public float detectionPeriod = 0.0f;
    public float timeSinceLastHit = 0;
    public float timeBetweenHits = 0;
    public Map<EntityRef,Float> timedCollision = Maps.newHashMap();
    public HashSet<EntityRef> entityCollsion = Sets.newHashSet();
}
