/**
 * ============================================================================
 * ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE FOR AUTONOMOUS VEHICLES
 * ON UNSTRUCTURED INDIAN ROADS
 * 
 * MODULE 1: SIMULATION ENVIRONMENT (STATIC WORLD & CANVAS RENDERER)
 * MODULE 2: EGO VEHICLE (PHYSICS, TELEOP CONTROLS, TELEMETRY & COLLISION BOUNDS)
 * 
 * Clean, modular vanilla JavaScript architecture.
 * Designed for subsequent module integration (Traffic, Sensors, Collision
 * Detection, and Path Planning).
 * ============================================================================
 */

'use strict';

/* ============================================================================
   1. WORLD CONFIGURATION & CONSTANTS
   ============================================================================ */
const WORLD_CONFIG = {
  // Coordinate space bounds (pixels/meters reference)
  width: 1800,
  height: 1000,
  
  // Visual Theme Colors
  colors: {
    ground: '#181f2a',
    groundPatch: '#1c2433',
    dirtShoulder: '#42372b',
    dirtEdge: '#2e261e',
    roadAsphalt: '#333842',
    roadPatch: '#2a2e37',
    roadMarking: 'rgba(241, 245, 249, 0.45)',
    curbYellow: '#f59e0b',
    curbBlack: '#1e293b',
    sidewalk: '#273142',
    sidewalkTile: '#202837',
    buildingBase: '#1a2232',
    buildingRoof: '#243044',
    buildingBorder: '#3b4b66',
    potholeEdge: '#15171c',
    potholeWater: '#0c0e12',
    debrisSand: '#d97706',
    debrisBarricade: '#dc2626',
    gridLine: 'rgba(56, 189, 248, 0.07)',
    gridText: 'rgba(148, 163, 184, 0.4)',
    
    // Ego Vehicle & Waypoint Colors
    egoBody: '#ffffff',
    egoAccent: '#06b6d4',
    egoGlass: '#0f172a',
    egoLiDAR: '#38bdf8',
    egoHeadlight: 'rgba(254, 240, 138, 0.85)',
    egoLightCone: 'rgba(254, 240, 138, 0.12)',
    egoTaillightOff: '#7f1d1d',
    egoTaillightOn: '#ef4444',
    destinationEmerald: '#10b981',
    destinationGlow: 'rgba(16, 185, 129, 0.25)'
  },
  
  // Grid spacing for coordinate overlay
  gridStep: 100
};

/* ============================================================================
   2. STATIC ENVIRONMENT DATA MODEL (COORDINATE-BASED)
   ============================================================================ */
const EnvironmentData = {
  // Main and Intersecting Road Geometries
  roads: {
    // Main horizontal road (unstructured with irregular shoulder contours)
    main: {
      yCenter: 560,
      width: 190,
      startX: 0,
      endX: 1800,
      shoulderPoints: [
        { x: 0, top: 460, bottom: 655 },
        { x: 250, top: 465, bottom: 660 },
        { x: 500, top: 455, bottom: 650 },
        { x: 750, top: 460, bottom: 660 },
        { x: 900, top: 460, bottom: 665 },
        { x: 1080, top: 465, bottom: 655 },
        { x: 1350, top: 455, bottom: 660 },
        { x: 1600, top: 460, bottom: 650 },
        { x: 1800, top: 465, bottom: 655 }
      ]
    },
    // Intersecting side road (vertical T-junction approaching from top)
    side: {
      xCenter: 920,
      width: 150,
      startY: 0,
      endY: 460,
      shoulderPoints: [
        { y: 0, left: 840, right: 995 },
        { y: 150, left: 845, right: 1000 },
        { y: 300, left: 835, right: 990 },
        { y: 460, left: 820, right: 1010 }
      ]
    }
  },

  // Sidewalks & Curbstone Strips (Indian style yellow/black striped curbs)
  curbs: [
    { startX: 0, endX: 820, y: 450, side: 'top' },
    { startX: 1010, endX: 1800, y: 450, side: 'top' },
    { startX: 0, endX: 1800, y: 670, side: 'bottom' },
    { x: 830, startY: 0, endY: 440, side: 'vertical' },
    { x: 1010, startY: 0, endY: 440, side: 'vertical' }
  ],

  // Buildings, Shops & Compounds along the roads
  buildings: [
    { id: 'bld-n1', name: 'Sai Krupa Auto Spares', x: 60, y: 120, width: 200, height: 260, color: '#1e293b', roofType: 'shop' },
    { id: 'bld-n2', name: 'Metro Diagnostic Center', x: 300, y: 140, width: 230, height: 240, color: '#1e2638', roofType: 'commercial' },
    { id: 'bld-n3', name: 'Shree Ganesh Complex', x: 570, y: 110, width: 210, height: 270, color: '#252e3d', roofType: 'mall' },
    { id: 'bld-n4', name: 'Vikas Tech Hub', x: 1050, y: 110, width: 240, height: 270, color: '#1e293b', roofType: 'office' },
    { id: 'bld-n5', name: 'National Public School Compound', x: 1330, y: 90, width: 410, height: 290, color: '#1a2332', roofType: 'compound' },
    { id: 'bld-s1', name: 'Sharma Tea Stall & Snacks', x: 70, y: 730, width: 170, height: 210, color: '#272a33', roofType: 'stall' },
    { id: 'bld-s2', name: 'City Hospital & Trauma Care', x: 280, y: 720, width: 280, height: 230, color: '#1e2638', roofType: 'hospital' },
    { id: 'bld-s3', name: 'Pooja Kirana & General Store', x: 600, y: 740, width: 210, height: 200, color: '#222d3d', roofType: 'shop' },
    { id: 'bld-s4', name: 'A-One Car Care Workshop', x: 860, y: 730, width: 260, height: 220, color: '#20293a', roofType: 'workshop' },
    { id: 'bld-s5', name: 'Greenview Residential Enclave', x: 1160, y: 710, width: 320, height: 240, color: '#1e293b', roofType: 'apartment' },
    { id: 'bld-s6', name: 'Royal Food Plaza', x: 1520, y: 730, width: 220, height: 210, color: '#242e40', roofType: 'shop' }
  ],

  // Irregular Potholes (3 realistic road surface hazards)
  potholes: [
    {
      id: 'pothole-1',
      name: 'Main Lane Pothole 1',
      x: 430,
      y: 535,
      radius: 26,
      vertices: [
        [-24, -10], [-10, -22], [14, -18], [26, -4],
        [22, 14], [8, 24], [-16, 20], [-26, 6]
      ],
      depth: 0.8,
      hasWater: true
    },
    {
      id: 'pothole-2',
      name: 'Intersection Junction Pothole 2',
      x: 805,
      y: 605,
      radius: 32,
      vertices: [
        [-30, -12], [-14, -28], [12, -26], [32, -8],
        [28, 16], [10, 30], [-12, 28], [-32, 10]
      ],
      depth: 0.9,
      hasWater: true
    },
    {
      id: 'pothole-3',
      name: 'Eastbound Lane Pothole 3',
      x: 1390,
      y: 545,
      radius: 24,
      vertices: [
        [-22, -14], [-4, -22], [20, -16], [24, 2],
        [16, 20], [-2, 22], [-20, 14], [-26, -2]
      ],
      depth: 0.6,
      hasWater: false
    }
  ],

  // Construction Areas & Debris Obstacles (3 static hazards)
  debris: [
    {
      id: 'debris-1',
      type: 'sand_gravel_mound',
      name: 'Roadside Sand Mound & Cones',
      x: 640,
      y: 635,
      width: 70,
      height: 45,
      hasCones: true,
      label: 'Sand Pile / Debris'
    },
    {
      id: 'debris-2',
      type: 'roadwork_barricade',
      name: 'Striped Roadwork Barricade',
      x: 1140,
      y: 495,
      width: 95,
      height: 28,
      angle: -0.05,
      label: 'Roadwork Barricade'
    },
    {
      id: 'debris-3',
      type: 'brick_pile',
      name: 'Construction Brick Clutter',
      x: 980,
      y: 270,
      width: 55,
      height: 42,
      label: 'Brick Debris'
    }
  ],

  // Traffic Signal at the Intersection (North-West corner)
  trafficSignal: {
    id: 'signal-intersection',
    x: 815,
    y: 435,
    poleX: 815,
    poleY: 435,
    state: 'RED',
    aspects: ['RED', 'YELLOW', 'GREEN'],
    timer: 0,
    stopLineX: 790,
    stopLineY: 465,
    label: 'Intersection Signal (Static)'
  },

  // Parked / Static Objects (Indian road roadside objects)
  parkedObjects: [
    {
      id: 'parked-auto',
      type: 'auto_rickshaw',
      name: 'Parked Auto-Rickshaw',
      x: 290,
      y: 645,
      width: 48,
      height: 28,
      angle: 0.08,
      color: '#eab308'
    },
    {
      id: 'parked-scooter-1',
      type: 'two_wheeler',
      name: 'Parked Scooter',
      x: 350,
      y: 652,
      width: 26,
      height: 12,
      angle: 0.25,
      color: '#38bdf8'
    },
    {
      id: 'parked-thela',
      type: 'handcart_thela',
      name: 'Street Vendor Handcart (Thela)',
      x: 720,
      y: 640,
      width: 46,
      height: 26,
      angle: -0.04,
      color: '#b45309'
    },
    {
      id: 'parked-car',
      type: 'small_car',
      name: 'Parked Hatchback',
      x: 1280,
      y: 480,
      width: 68,
      height: 34,
      angle: 0.02,
      color: '#94a3b8'
    },
    {
      id: 'utility-pole-1',
      type: 'electric_pole',
      name: 'Utility Transformer Pole',
      x: 1025,
      y: 435,
      width: 20,
      height: 20
    }
  ],

  // Destination Goal Waypoint (Target marker along the main road)
  destination: {
    id: 'destination-goal',
    name: 'Target Destination',
    x: 1650,
    y: 560,
    radius: 24,
    label: 'DESTINATION GOAL'
  },

  // Trees and Green Foliage on Roadside Patches
  trees: [
    { x: 30, y: 410, r: 24 },
    { x: 260, y: 415, r: 20 },
    { x: 535, y: 415, r: 26 },
    { x: 1070, y: 415, r: 22 },
    { x: 1300, y: 410, r: 28 },
    { x: 1760, y: 415, r: 25 },
    { x: 40, y: 700, r: 22 },
    { x: 580, y: 705, r: 24 },
    { x: 1130, y: 695, r: 26 },
    { x: 1500, y: 700, r: 22 },
    { x: 1765, y: 700, r: 28 },
    { x: 805, y: 160, r: 22 },
    { x: 1030, y: 180, r: 25 }
  ]
};

/* ============================================================================
   3. STATIC COLLISION & ROAD ENVIRONMENT QUERY SYSTEM
   ============================================================================ */
const StaticCollisionSystem = {
  /**
   * Check if a circular footprint at (x, y) collides with any solid, non-drivable obstacle
   * (Buildings, construction barricades, parked vehicles/poles, traffic signal, roadside trees)
   */
  checkSolidCollision(x, y, radius = 14) {
    // 1. Buildings (Solid structures along roadsides)
    if (EnvironmentData.buildings) {
      for (const b of EnvironmentData.buildings) {
        const minX = b.x;
        const maxX = b.x + b.width;
        const minY = b.y;
        const maxY = b.y + b.height;

        const closestX = Math.max(minX, Math.min(x, maxX));
        const closestY = Math.max(minY, Math.min(y, maxY));
        const distX = x - closestX;
        const distY = y - closestY;
        const distSq = distX * distX + distY * distY;

        if (distSq < radius * radius) {
          return {
            collided: true,
            type: 'building',
            name: b.name,
            obstacle: b,
            overlap: radius - Math.sqrt(distSq || 0.001)
          };
        }
      }
    }

    // 2. Construction Barricades & Debris Clutter
    if (EnvironmentData.debris) {
      for (const d of EnvironmentData.debris) {
        const halfW = d.width / 2;
        const halfH = d.height / 2;
        const minX = d.x - halfW;
        const maxX = d.x + halfW;
        const minY = d.y - halfH;
        const maxY = d.y + halfH;

        const closestX = Math.max(minX, Math.min(x, maxX));
        const closestY = Math.max(minY, Math.min(y, maxY));
        const distX = x - closestX;
        const distY = y - closestY;
        const distSq = distX * distX + distY * distY;

        if (distSq < radius * radius) {
          return {
            collided: true,
            type: 'debris',
            name: d.name,
            obstacle: d,
            overlap: radius - Math.sqrt(distSq || 0.001)
          };
        }
      }
    }

    // 3. Parked Vehicles, Thelas, and Electric Poles
    if (EnvironmentData.parkedObjects) {
      for (const p of EnvironmentData.parkedObjects) {
        const halfW = p.width / 2;
        const halfH = p.height / 2;
        const minX = p.x - halfW;
        const maxX = p.x + halfW;
        const minY = p.y - halfH;
        const maxY = p.y + halfH;

        const closestX = Math.max(minX, Math.min(x, maxX));
        const closestY = Math.max(minY, Math.min(y, maxY));
        const distX = x - closestX;
        const distY = y - closestY;
        const distSq = distX * distX + distY * distY;

        if (distSq < radius * radius) {
          return {
            collided: true,
            type: 'parked_object',
            name: p.name,
            obstacle: p,
            overlap: radius - Math.sqrt(distSq || 0.001)
          };
        }
      }
    }

    // 4. Traffic Signal Base & Pole
    if (EnvironmentData.trafficSignal) {
      const sig = EnvironmentData.trafficSignal;
      const dist = Math.hypot(x - sig.x, y - sig.y);
      if (dist < radius + 12) {
        return {
          collided: true,
          type: 'signal_pole',
          name: sig.label,
          obstacle: sig,
          overlap: (radius + 12) - dist
        };
      }
    }

    // 5. Trees on roadside sidewalks
    if (EnvironmentData.trees) {
      for (const t of EnvironmentData.trees) {
        const dist = Math.hypot(x - t.x, y - t.y);
        if (dist < radius + (t.r * 0.5)) {
          return {
            collided: true,
            type: 'tree',
            name: 'Roadside Tree',
            obstacle: t,
            overlap: (radius + t.r * 0.5) - dist
          };
        }
      }
    }

    return { collided: false };
  },

  /**
   * Check if a position is inside any pothole hazard on the road
   */
  checkPothole(x, y, radius = 10) {
    if (!EnvironmentData.potholes) return { inPothole: false };

    for (const p of EnvironmentData.potholes) {
      const dist = Math.hypot(x - p.x, y - p.y);
      if (dist < p.radius + radius) {
        return {
          inPothole: true,
          pothole: p,
          depth: p.depth || 0.7,
          distance: dist
        };
      }
    }
    return { inPothole: false };
  },

  /**
   * Get all solid obstacle items formatted for Detection and Planning
   */
  getAllSolidObstacles() {
    const list = [];
    if (EnvironmentData.buildings) {
      EnvironmentData.buildings.forEach(b => {
        list.push({
          id: b.id,
          type: 'building',
          name: b.name,
          x: b.x + b.width / 2,
          y: b.y + b.height / 2,
          width: b.width,
          height: b.height,
          radius: Math.hypot(b.width, b.height) / 2,
          isSolid: true
        });
      });
    }
    if (EnvironmentData.debris) {
      EnvironmentData.debris.forEach(d => {
        list.push({
          id: d.id,
          type: 'debris',
          name: d.name,
          x: d.x,
          y: d.y,
          width: d.width,
          height: d.height,
          radius: Math.hypot(d.width, d.height) / 2,
          isSolid: true
        });
      });
    }
    if (EnvironmentData.parkedObjects) {
      EnvironmentData.parkedObjects.forEach(p => {
        list.push({
          id: p.id,
          type: 'parked_object',
          name: p.name,
          x: p.x,
          y: p.y,
          width: p.width,
          height: p.height,
          radius: Math.hypot(p.width, p.height) / 2,
          isSolid: true
        });
      });
    }
    if (EnvironmentData.trafficSignal) {
      list.push({
        id: EnvironmentData.trafficSignal.id,
        type: 'traffic_signal',
        name: EnvironmentData.trafficSignal.label,
        x: EnvironmentData.trafficSignal.x,
        y: EnvironmentData.trafficSignal.y,
        radius: 16,
        isSolid: true
      });
    }
    return list;
  }
};

/* ============================================================================
   4. CAMERA & VIEWPORT CONTROLLER
   ============================================================================ */
class SimulationCamera {
  constructor(worldWidth, worldHeight) {
    this.worldWidth = worldWidth;
    this.worldHeight = worldHeight;
    this.x = 0;
    this.y = 0;
    this.zoom = 1;
    this.minZoom = 0.4;
    this.maxZoom = 3.0;

    this.viewportWidth = 800;
    this.viewportHeight = 600;
    this.dpr = window.devicePixelRatio || 1;
  }

  updateViewport(width, height) {
    this.viewportWidth = width;
    this.viewportHeight = height;
  }

  fitToWorld() {
    const scaleX = this.viewportWidth / this.worldWidth;
    const scaleY = this.viewportHeight / this.worldHeight;
    this.zoom = Math.min(scaleX, scaleY) * 0.96;
    this.zoom = Math.max(this.minZoom, Math.min(this.maxZoom, this.zoom));

    this.x = (this.viewportWidth - this.worldWidth * this.zoom) / 2;
    this.y = (this.viewportHeight - this.worldHeight * this.zoom) / 2;
  }

  worldToScreen(wx, wy) {
    return {
      x: wx * this.zoom + this.x,
      y: wy * this.zoom + this.y
    };
  }

  screenToWorld(sx, sy) {
    return {
      x: (sx - this.x) / this.zoom,
      y: (sy - this.y) / this.zoom
    };
  }

  zoomAt(factor, screenCenterX, screenCenterY) {
    const oldZoom = this.zoom;
    const newZoom = Math.max(this.minZoom, Math.min(this.maxZoom, this.zoom * factor));
    if (newZoom === oldZoom) return;

    const worldPos = this.screenToWorld(screenCenterX, screenCenterY);
    this.zoom = newZoom;
    this.x = screenCenterX - worldPos.x * this.zoom;
    this.y = screenCenterY - worldPos.y * this.zoom;
  }

  pan(dx, dy) {
    this.x += dx;
    this.y += dy;
  }

  applyTransform(ctx) {
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.translate(this.x, this.y);
    ctx.scale(this.zoom, this.zoom);
  }
}

/* ============================================================================
   4. ENVIRONMENT RENDERER (STATIC WORLD & DESTINATION MARKER)
   ============================================================================ */
const EnvironmentRenderer = {
  render(ctx, data, options = {}) {
    const showGrid = options.showGrid !== false;
    const showLabels = options.showLabels !== false;

    // 1. Base Ground & Plots
    this.drawGround(ctx, data);

    // 2. Road Network (Main + Intersecting)
    this.drawRoads(ctx, data.roads);

    // 3. Sidewalks & Curbstones
    this.drawCurbsAndSidewalks(ctx, data.curbs);

    // 4. Buildings & Compounds
    this.drawBuildings(ctx, data.buildings);

    // 5. Environmental Foliage & Trees
    this.drawFoliage(ctx, data.trees);

    // 6. Destination Goal Waypoint Marker
    if (data.destination) {
      this.drawDestinationMarker(ctx, data.destination);
    }

    // 7. Static Obstacles: Potholes
    this.drawPotholes(ctx, data.potholes);

    // 8. Static Obstacles: Debris & Construction
    this.drawDebris(ctx, data.debris);

    // 9. Parked Vehicles & Street Objects
    this.drawParkedObjects(ctx, data.parkedObjects);

    // 10. Intersection Traffic Signal & Markings
    this.drawTrafficSignal(ctx, data.trafficSignal);

    // 11. Coordinate Grid & World Bounds
    if (showGrid) {
      this.drawCoordinateGrid(ctx, WORLD_CONFIG);
    }

    // 12. Feature Labels Overlay
    if (showLabels) {
      this.drawFeatureLabels(ctx, data);
    }
  },

  drawGround(ctx, data) {
    ctx.fillStyle = WORLD_CONFIG.colors.ground;
    ctx.fillRect(0, 0, WORLD_CONFIG.width, WORLD_CONFIG.height);

    ctx.fillStyle = WORLD_CONFIG.colors.groundPatch;
    ctx.fillRect(40, 80, 760, 320);
    ctx.fillRect(1030, 80, 730, 320);
    ctx.fillRect(40, 690, 1720, 270);

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
    ctx.lineWidth = 2;
    ctx.strokeRect(40, 80, 760, 320);
    ctx.strokeRect(1030, 80, 730, 320);
    ctx.strokeRect(40, 690, 1720, 270);
  },

  drawRoads(ctx, roads) {
    const main = roads.main;
    const side = roads.side;

    // Dirt Shoulders
    ctx.fillStyle = WORLD_CONFIG.colors.dirtShoulder;
    ctx.beginPath();
    ctx.moveTo(0, 440);
    ctx.lineTo(WORLD_CONFIG.width, 440);
    ctx.lineTo(WORLD_CONFIG.width, 675);
    ctx.lineTo(0, 675);
    ctx.closePath();
    ctx.fill();

    ctx.fillRect(side.xCenter - side.width / 2 - 25, 0, side.width + 50, 470);

    // Main Asphalt Road Surface
    ctx.fillStyle = WORLD_CONFIG.colors.roadAsphalt;
    ctx.beginPath();
    ctx.moveTo(main.shoulderPoints[0].x, main.shoulderPoints[0].top);
    for (let i = 1; i <= 3; i++) {
      ctx.lineTo(main.shoulderPoints[i].x, main.shoulderPoints[i].top);
    }
    ctx.quadraticCurveTo(820, 460, 835, 430);
    ctx.lineTo(840, 0);
    ctx.lineTo(995, 0);
    ctx.lineTo(1000, 430);
    ctx.quadraticCurveTo(1010, 460, 1080, main.shoulderPoints[5].top);

    for (let i = 6; i < main.shoulderPoints.length; i++) {
      ctx.lineTo(main.shoulderPoints[i].x, main.shoulderPoints[i].top);
    }
    ctx.lineTo(WORLD_CONFIG.width, main.shoulderPoints[main.shoulderPoints.length - 1].bottom);

    for (let i = main.shoulderPoints.length - 1; i >= 0; i--) {
      ctx.lineTo(main.shoulderPoints[i].x, main.shoulderPoints[i].bottom);
    }
    ctx.closePath();
    ctx.fill();

    ctx.strokeStyle = WORLD_CONFIG.colors.dirtEdge;
    ctx.lineWidth = 3;
    ctx.stroke();

    // Road Wear Patches
    ctx.fillStyle = WORLD_CONFIG.colors.roadPatch;
    ctx.beginPath();
    ctx.ellipse(360, 560, 180, 45, 0, 0, Math.PI * 2);
    ctx.fill();

    ctx.beginPath();
    ctx.ellipse(1250, 560, 240, 50, 0, 0, Math.PI * 2);
    ctx.fill();

    // Faded Center Markings
    ctx.strokeStyle = WORLD_CONFIG.colors.roadMarking;
    ctx.lineWidth = 4;
    ctx.setLineDash([24, 28]);

    ctx.beginPath();
    ctx.moveTo(20, main.yCenter);
    ctx.lineTo(760, main.yCenter);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(1080, main.yCenter);
    ctx.lineTo(1780, main.yCenter);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(side.xCenter, 20);
    ctx.lineTo(side.xCenter, 400);
    ctx.stroke();

    ctx.setLineDash([]);
  },

  drawCurbsAndSidewalks(ctx, curbs) {
    const curbHeight = 8;
    const stripeLength = 22;

    ctx.fillStyle = WORLD_CONFIG.colors.sidewalk;
    ctx.fillRect(0, 420, 820, 30);
    ctx.fillRect(1010, 420, 790, 30);
    ctx.fillRect(0, 665, 1800, 30);
    ctx.fillRect(810, 0, 25, 430);
    ctx.fillRect(1005, 0, 25, 430);

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.lineWidth = 1;
    for (let x = 0; x < WORLD_CONFIG.width; x += 30) {
      if (x < 820 || x > 1010) {
        ctx.beginPath();
        ctx.moveTo(x, 420);
        ctx.lineTo(x, 450);
        ctx.stroke();
      }
      ctx.beginPath();
      ctx.moveTo(x, 665);
      ctx.lineTo(x, 695);
      ctx.stroke();
    }

    curbs.forEach((curb) => {
      if (curb.side === 'top' || curb.side === 'bottom') {
        const length = curb.endX - curb.startX;
        const count = Math.ceil(length / stripeLength);
        for (let i = 0; i < count; i++) {
          const segX = curb.startX + i * stripeLength;
          const segW = Math.min(stripeLength, curb.endX - segX);
          ctx.fillStyle = i % 2 === 0 ? WORLD_CONFIG.colors.curbYellow : WORLD_CONFIG.colors.curbBlack;
          ctx.fillRect(segX, curb.y, segW, curbHeight);
        }
      } else if (curb.side === 'vertical') {
        const length = curb.endY - curb.startY;
        const count = Math.ceil(length / stripeLength);
        for (let i = 0; i < count; i++) {
          const segY = curb.startY + i * stripeLength;
          const segH = Math.min(stripeLength, curb.endY - segY);
          ctx.fillStyle = i % 2 === 0 ? WORLD_CONFIG.colors.curbYellow : WORLD_CONFIG.colors.curbBlack;
          ctx.fillRect(curb.x, segY, curbHeight, segH);
        }
      }
    });
  },

  drawBuildings(ctx, buildings) {
    buildings.forEach((bld) => {
      ctx.fillStyle = 'rgba(0, 0, 0, 0.45)';
      ctx.fillRect(bld.x + 6, bld.y + 6, bld.width, bld.height);

      ctx.fillStyle = bld.color || WORLD_CONFIG.colors.buildingBase;
      ctx.fillRect(bld.x, bld.y, bld.width, bld.height);

      ctx.strokeStyle = WORLD_CONFIG.colors.buildingBorder;
      ctx.lineWidth = 3;
      ctx.strokeRect(bld.x, bld.y, bld.width, bld.height);

      const inset = 10;
      ctx.fillStyle = WORLD_CONFIG.colors.buildingRoof;
      ctx.fillRect(bld.x + inset, bld.y + inset, bld.width - inset * 2, bld.height - inset * 2);

      ctx.fillStyle = '#0f172a';
      ctx.beginPath();
      ctx.arc(bld.x + bld.width - 26, bld.y + 26, 12, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      ctx.fillStyle = '#1e293b';
      ctx.fillRect(bld.x + 20, bld.y + 20, 36, 26);
      ctx.strokeStyle = '#475569';
      ctx.lineWidth = 1;
      ctx.strokeRect(bld.x + 20, bld.y + 20, 36, 26);

      ctx.fillStyle = '#475569';
      ctx.fillRect(bld.x + 24, bld.y + bld.height - 38, 22, 16);

      if (bld.y < 400) {
        ctx.fillStyle = '#0284c7';
        ctx.fillRect(bld.x + 6, bld.y + bld.height - 8, bld.width - 12, 8);
      } else {
        ctx.fillStyle = '#d97706';
        ctx.fillRect(bld.x + 6, bld.y, bld.width - 12, 8);
      }

      ctx.fillStyle = '#e2e8f0';
      ctx.font = '600 11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(bld.name, bld.x + bld.width / 2, bld.y + bld.height / 2);
    });
  },

  drawFoliage(ctx, trees) {
    trees.forEach((tree) => {
      ctx.fillStyle = 'rgba(0, 0, 0, 0.35)';
      ctx.beginPath();
      ctx.arc(tree.x + 4, tree.y + 4, tree.r, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#14532d';
      ctx.beginPath();
      ctx.arc(tree.x, tree.y, tree.r, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#15803d';
      ctx.beginPath();
      ctx.arc(tree.x - 3, tree.y - 3, tree.r * 0.75, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#22c55e';
      ctx.beginPath();
      ctx.arc(tree.x - 6, tree.y - 6, tree.r * 0.4, 0, Math.PI * 2);
      ctx.fill();
    });
  },

  /**
   * Draw visible destination marker along the main road
   */
  drawDestinationMarker(ctx, dest) {
    ctx.save();
    ctx.translate(dest.x, dest.y);

    const now = Date.now() / 1000;
    const pulse = (Math.sin(now * 3) + 1) / 2; // 0 to 1

    // Outer radiant pulse ring
    ctx.fillStyle = WORLD_CONFIG.colors.destinationGlow;
    ctx.beginPath();
    ctx.arc(0, 0, dest.radius + 10 + pulse * 8, 0, Math.PI * 2);
    ctx.fill();

    // Concentric target ring 1
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.arc(0, 0, dest.radius + pulse * 3, 0, Math.PI * 2);
    ctx.stroke();

    // Inner chequered base ring
    ctx.fillStyle = 'rgba(16, 185, 129, 0.2)';
    ctx.beginPath();
    ctx.arc(0, 0, dest.radius, 0, Math.PI * 2);
    ctx.fill();

    // Center waypoint bullseye
    ctx.fillStyle = '#10b981';
    ctx.beginPath();
    ctx.arc(0, 0, 6, 0, Math.PI * 2);
    ctx.fill();

    // Waypoint Flag Icon
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 6);
    ctx.lineTo(0, -22);
    ctx.stroke();

    // Chequered Goal Flag
    ctx.fillStyle = '#34d399';
    ctx.beginPath();
    ctx.moveTo(0, -22);
    ctx.lineTo(16, -16);
    ctx.lineTo(0, -10);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = '#065f46';
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.restore();
  },

  drawPotholes(ctx, potholes) {
    potholes.forEach((pot) => {
      ctx.save();
      ctx.translate(pot.x, pot.y);

      ctx.fillStyle = WORLD_CONFIG.colors.potholeEdge;
      ctx.beginPath();
      const v = pot.vertices;
      ctx.moveTo(v[0][0] * 1.15, v[0][1] * 1.15);
      for (let i = 1; i < v.length; i++) {
        ctx.lineTo(v[i][0] * 1.15, v[i][1] * 1.15);
      }
      ctx.closePath();
      ctx.fill();

      ctx.strokeStyle = '#57534e';
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.strokeStyle = 'rgba(41, 37, 36, 0.7)';
      ctx.lineWidth = 1.5;
      for (let i = 0; i < v.length; i += 2) {
        ctx.beginPath();
        ctx.moveTo(v[i][0], v[i][1]);
        ctx.lineTo(v[i][0] * 1.5, v[i][1] * 1.5);
        ctx.stroke();
      }

      ctx.fillStyle = WORLD_CONFIG.colors.potholeWater;
      ctx.beginPath();
      ctx.moveTo(v[0][0], v[0][1]);
      for (let i = 1; i < v.length; i++) {
        ctx.lineTo(v[i][0], v[i][1]);
      }
      ctx.closePath();
      ctx.fill();

      if (pot.hasWater) {
        ctx.fillStyle = 'rgba(14, 116, 144, 0.35)';
        ctx.beginPath();
        ctx.ellipse(0, 0, pot.radius * 0.55, pot.radius * 0.35, 0.2, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.beginPath();
        ctx.ellipse(-pot.radius * 0.15, -pot.radius * 0.1, pot.radius * 0.25, pot.radius * 0.1, -0.2, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.restore();
    });
  },

  drawDebris(ctx, debrisList) {
    debrisList.forEach((deb) => {
      ctx.save();
      ctx.translate(deb.x, deb.y);
      if (deb.angle) ctx.rotate(deb.angle);

      if (deb.type === 'sand_gravel_mound') {
        ctx.fillStyle = 'rgba(0,0,0,0.3)';
        ctx.beginPath();
        ctx.ellipse(3, 3, deb.width / 2, deb.height / 2, 0, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#b45309';
        ctx.beginPath();
        ctx.ellipse(0, 0, deb.width / 2, deb.height / 2, 0, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#d97706';
        ctx.beginPath();
        ctx.ellipse(-4, -3, deb.width / 2.8, deb.height / 2.8, -0.2, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#78350f';
        for (let i = 0; i < 6; i++) {
          const gx = (Math.sin(i * 1.7) * deb.width) / 2.5;
          const gy = (Math.cos(i * 2.3) * deb.height) / 2.5;
          ctx.fillRect(gx, gy, 4, 4);
        }

        this.drawTrafficCone(ctx, -deb.width / 2 + 5, -deb.height / 2);
        this.drawTrafficCone(ctx, deb.width / 2 - 5, deb.height / 2 - 2);

      } else if (deb.type === 'roadwork_barricade') {
        const w = deb.width;
        const h = deb.height;

        ctx.fillStyle = 'rgba(0,0,0,0.35)';
        ctx.fillRect(-w / 2 + 4, -h / 2 + 4, w, h);

        ctx.fillStyle = '#1e293b';
        ctx.fillRect(-w / 2, -h / 2, w, h);
        ctx.strokeStyle = '#dc2626';
        ctx.lineWidth = 2;
        ctx.strokeRect(-w / 2, -h / 2, w, h);

        const stripeW = 14;
        ctx.save();
        ctx.beginPath();
        ctx.rect(-w / 2, -h / 2, w, h);
        ctx.clip();

        for (let sx = -w / 2 - h; sx < w / 2 + h; sx += stripeW * 2) {
          ctx.fillStyle = '#f59e0b';
          ctx.beginPath();
          ctx.moveTo(sx, -h / 2);
          ctx.lineTo(sx + stripeW, -h / 2);
          ctx.lineTo(sx + stripeW - h, h / 2);
          ctx.lineTo(sx - h, h / 2);
          ctx.closePath();
          ctx.fill();
        }
        ctx.restore();

        ctx.fillStyle = '#ef4444';
        ctx.fillRect(-w / 2 - 4, -h / 2 - 2, 6, h + 4);
        ctx.fillRect(w / 2 - 2, -h / 2 - 2, 6, h + 4);

      } else if (deb.type === 'brick_pile') {
        const w = deb.width;
        const h = deb.height;

        ctx.fillStyle = '#7c2d12';
        ctx.fillRect(-w / 2, -h / 2, w, h);

        ctx.strokeStyle = '#b91c1c';
        ctx.lineWidth = 1;
        const bw = 12;
        const bh = 7;
        for (let bx = -w / 2; bx < w / 2; bx += bw) {
          for (let by = -h / 2; by < h / 2; by += bh) {
            ctx.strokeRect(bx, by, bw, bh);
          }
        }
      }

      ctx.restore();
    });
  },

  drawTrafficCone(ctx, x, y) {
    ctx.save();
    ctx.translate(x, y);

    ctx.fillStyle = '#ea580c';
    ctx.fillRect(-7, -7, 14, 14);

    ctx.fillStyle = '#f8fafc';
    ctx.beginPath();
    ctx.arc(0, 0, 4.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#c2410c';
    ctx.beginPath();
    ctx.arc(0, 0, 2.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore();
  },

  drawTrafficSignal(ctx, signal) {
    ctx.strokeStyle = 'rgba(241, 245, 249, 0.4)';
    ctx.lineWidth = 8;
    ctx.setLineDash([]);

    ctx.beginPath();
    ctx.moveTo(signal.stopLineX, 460);
    ctx.lineTo(signal.stopLineX, 660);
    ctx.stroke();

    ctx.lineWidth = 4;
    ctx.setLineDash([12, 10]);
    ctx.beginPath();
    ctx.moveTo(signal.stopLineX + 16, 465);
    ctx.lineTo(signal.stopLineX + 16, 655);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.save();
    ctx.translate(signal.x, signal.y);

    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(24, 24);
    ctx.stroke();

    const boxW = 18;
    const boxH = 46;
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(18, 14, boxW, boxH);
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(18, 14, boxW, boxH);

    ctx.fillStyle = signal.state === 'RED' ? '#ef4444' : '#7f1d1d';
    ctx.beginPath();
    ctx.arc(27, 22, 5, 0, Math.PI * 2);
    ctx.fill();
    if (signal.state === 'RED') {
      ctx.shadowColor = '#ef4444';
      ctx.shadowBlur = 10;
      ctx.stroke();
      ctx.shadowBlur = 0;
    }

    ctx.fillStyle = signal.state === 'YELLOW' ? '#eab308' : '#713f12';
    ctx.beginPath();
    ctx.arc(27, 37, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = signal.state === 'GREEN' ? '#22c55e' : '#14532d';
    ctx.beginPath();
    ctx.arc(27, 52, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#334155';
    ctx.beginPath();
    ctx.arc(0, 0, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.restore();
  },

  drawParkedObjects(ctx, objects) {
    objects.forEach((obj) => {
      ctx.save();
      ctx.translate(obj.x, obj.y);
      if (obj.angle) ctx.rotate(obj.angle);

      if (obj.type === 'auto_rickshaw') {
        const w = obj.width;
        const h = obj.height;

        ctx.fillStyle = 'rgba(0,0,0,0.4)';
        ctx.fillRect(-w / 2 + 3, -h / 2 + 3, w, h);

        ctx.fillStyle = '#15803d';
        ctx.fillRect(-w / 2, -h / 2, w, h);

        ctx.fillStyle = '#eab308';
        ctx.fillRect(-w / 2 + 6, -h / 2 + 2, w - 16, h - 4);

        ctx.beginPath();
        ctx.moveTo(w / 2 - 10, -h / 2);
        ctx.lineTo(w / 2, -h / 4);
        ctx.lineTo(w / 2, h / 4);
        ctx.lineTo(w / 2 - 10, h / 2);
        ctx.closePath();
        ctx.fill();

        ctx.fillStyle = '#38bdf8';
        ctx.fillRect(w / 2 - 12, -h / 2 + 4, 4, h - 8);

        ctx.fillStyle = '#0f172a';
        ctx.fillRect(w / 2 - 10, -h / 2 - 3, 3, 3);
        ctx.fillRect(w / 2 - 10, h / 2, 3, 3);

        ctx.fillRect(w / 2 - 4, -2, 5, 4);
        ctx.fillRect(-w / 2 + 6, -h / 2 - 2, 8, 3);
        ctx.fillRect(-w / 2 + 6, h / 2 - 1, 8, 3);

      } else if (obj.type === 'two_wheeler') {
        const w = obj.width;
        const h = obj.height;

        ctx.fillStyle = 'rgba(0,0,0,0.3)';
        ctx.fillRect(-w / 2 + 2, -h / 2 + 2, w, h);

        ctx.fillStyle = obj.color || '#38bdf8';
        ctx.beginPath();
        ctx.ellipse(0, 0, w / 2, h / 2, 0, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#1e293b';
        ctx.fillRect(-w / 2 + 4, -h / 2 + 2, w * 0.5, h - 4);

        ctx.strokeStyle = '#0f172a';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(w / 2 - 4, -h / 2 - 2);
        ctx.lineTo(w / 2 - 4, h / 2 + 2);
        ctx.stroke();

      } else if (obj.type === 'handcart_thela') {
        const w = obj.width;
        const h = obj.height;

        ctx.fillStyle = 'rgba(0,0,0,0.35)';
        ctx.fillRect(-w / 2 + 3, -h / 2 + 3, w, h);

        ctx.fillStyle = '#92400e';
        ctx.fillRect(-w / 2, -h / 2, w, h);
        ctx.strokeStyle = '#451a03';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(-w / 2, -h / 2, w, h);

        for (let px = -w / 2 + 8; px < w / 2; px += 8) {
          ctx.beginPath();
          ctx.moveTo(px, -h / 2);
          ctx.lineTo(px, h / 2);
          ctx.stroke();
        }

        ctx.fillStyle = '#22c55e';
        ctx.fillRect(-w / 2 + 4, -h / 2 + 3, 14, 9);
        ctx.fillStyle = '#f97316';
        ctx.fillRect(-w / 2 + 20, -h / 2 + 3, 12, 9);
        ctx.fillStyle = '#eab308';
        ctx.fillRect(-w / 2 + 6, 2, 16, 8);

        ctx.fillStyle = '#0f172a';
        ctx.fillRect(-w / 2 + 4, -h / 2 - 3, 6, 3);
        ctx.fillRect(w / 2 - 10, -h / 2 - 3, 6, 3);
        ctx.fillRect(-w / 2 + 4, h / 2, 6, 3);
        ctx.fillRect(w / 2 - 10, h / 2, 6, 3);

      } else if (obj.type === 'small_car') {
        const w = obj.width;
        const h = obj.height;

        ctx.fillStyle = 'rgba(0,0,0,0.45)';
        ctx.fillRect(-w / 2 + 4, -h / 2 + 4, w, h);

        ctx.fillStyle = obj.color || '#64748b';
        ctx.beginPath();
        ctx.roundRect(-w / 2, -h / 2, w, h, 6);
        ctx.fill();

        ctx.fillStyle = '#0f172a';
        ctx.fillRect(w / 2 - 18, -h / 2 + 3, 7, h - 6);
        ctx.fillRect(-w / 2 + 10, -h / 2 + 3, 6, h - 6);
        ctx.fillStyle = '#475569';
        ctx.fillRect(-w / 2 + 18, -h / 2 + 4, w - 38, h - 8);

        ctx.fillStyle = '#334155';
        ctx.fillRect(w / 2 - 18, -h / 2 - 3, 4, 3);
        ctx.fillRect(w / 2 - 18, h / 2, 4, 3);

      } else if (obj.type === 'electric_pole') {
        ctx.fillStyle = '#475569';
        ctx.beginPath();
        ctx.arc(0, 0, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-4, -4);
        ctx.lineTo(4, 4);
        ctx.moveTo(4, -4);
        ctx.lineTo(-4, 4);
        ctx.stroke();
      }

      ctx.restore();
    });
  },

  drawCoordinateGrid(ctx, config) {
    const step = config.gridStep;
    ctx.strokeStyle = config.colors.gridLine;
    ctx.lineWidth = 1;
    ctx.fillStyle = config.colors.gridText;
    ctx.font = '500 10px ui-monospace, SFMono-Regular, Menlo, Monaco, monospace';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';

    for (let x = 0; x <= config.width; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, config.height);
      ctx.stroke();

      if (x % 200 === 0) {
        ctx.fillText(`X:${x}`, x + 4, 6);
      }
    }

    for (let y = 0; y <= config.height; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(config.width, y);
      ctx.stroke();

      if (y % 200 === 0) {
        ctx.fillText(`Y:${y}`, 6, y + 4);
      }
    }

    ctx.strokeStyle = 'rgba(56, 189, 248, 0.4)';
    ctx.lineWidth = 2;
    ctx.strokeRect(0, 0, config.width, config.height);
  },

  drawFeatureLabels(ctx, data) {
    ctx.font = '600 10px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';

    data.potholes.forEach((pot) => {
      this.drawPillBadge(ctx, pot.x, pot.y - pot.radius - 4, 'POTHOLE', '#f43f5e', '#ffffff');
    });

    data.debris.forEach((deb) => {
      this.drawPillBadge(ctx, deb.x, deb.y - deb.height / 2 - 6, deb.label, '#f59e0b', '#0f172a');
    });

    if (data.destination) {
      this.drawPillBadge(ctx, data.destination.x, data.destination.y - data.destination.radius - 8, 'DESTINATION GOAL', '#10b981', '#ffffff');
    }

    this.drawPillBadge(ctx, data.trafficSignal.x + 30, data.trafficSignal.y - 4, 'TRAFFIC SIGNAL', '#22c55e', '#ffffff');
    this.drawPillBadge(ctx, 450, 485, 'MAIN ROAD (UNSTRUCTURED)', '#38bdf8', '#0f172a');
    this.drawPillBadge(ctx, 920, 60, 'SIDE ROAD JUNCTION', '#38bdf8', '#0f172a');
  },

  drawPillBadge(ctx, x, y, text, bgColor, textColor) {
    const metrics = ctx.measureText(text);
    const paddingX = 7;
    const paddingY = 3;
    const badgeW = metrics.width + paddingX * 2;
    const badgeH = 16;

    ctx.fillStyle = bgColor;
    ctx.beginPath();
    ctx.roundRect(x - badgeW / 2, y - badgeH, badgeW, badgeH, 4);
    ctx.fill();

    ctx.fillStyle = textColor;
    ctx.fillText(text, x, y - 2);
  }
};

/* ============================================================================
   5. MODULE 2: EGO VEHICLE MODEL & CONTROLLER (2D KINEMATIC PHYSICS)
   ============================================================================ */
class EgoVehicle {
  constructor(startX = 140, startY = 580, startHeading = 0) {
    // Initial spawn location on main road
    this.startX = startX;
    this.startY = startY;
    this.startHeading = startHeading;

    // World Coordinates & Geometry
    this.x = startX;
    this.y = startY;
    this.length = 52;  // Vehicle length (px)
    this.width = 26;   // Vehicle width (px)
    this.wheelbase = 34; // Wheelbase L for kinematic steering (px)

    // Motion State
    this.heading = startHeading; // Yaw angle (radians, 0 = East / Right)
    this.speed = 0;              // Velocity (px/s, positive = forward, negative = reverse)
    this.steeringAngle = 0;      // Front wheels steering angle (radians)
    
    // Physics Limits & Tuning
    this.maxForwardSpeed = 220;  // Max forward velocity (~55 km/h)
    this.maxReverseSpeed = 65;   // Max reverse velocity (~16 km/h)
    this.acceleration = 160;     // Throttle acceleration (px/s²)
    this.brakingDecel = 280;     // Footbrake deceleration (px/s²)
    this.handbrakeDecel = 450;   // Emergency stop deceleration (px/s²)
    this.frictionDecel = 50;     // Rolling friction / air drag (px/s²)
    
    this.maxSteeringAngle = 0.58; // Max steer lock (~33 degrees)
    this.steeringSpeed = 3.2;     // Steering rate (rad/s)
    this.steeringReturnSpeed = 4.5; // Self-centering rate (rad/s)

    // User Input States
    this.inputs = {
      throttle: 0,   // +1 = forward (W/Up), -1 = brake/reverse (S/Down)
      steer: 0,      // -1 = left (A/Left), +1 = right (D/Right)
      brake: 0,      // 0 to 1 = progressive footbrake deceleration
      handbrake: false // Space = immediate stop
    };

    // State Classification for HUD
    this.state = 'STOPPED'; // 'STOPPED', 'ACCELERATING', 'CRUISING', 'BRAKING', 'REVERSING'
    this.isBraking = false;
    this.isReversing = false;
    this.isCollided = false;
    this.inPothole = false;
    this.potholeSeverity = 0;
    this.potholeVibration = 0;

    // Sensor / LiDAR animation state
    this.lidarRotation = 0;

    // Bind Keyboard Listeners
    this.bindControls();
  }

  /**
   * Bind WASD, Arrow Keys, Spacebar, and Reset (P)
   */
  bindControls() {
    const keysDown = {};

    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      const code = e.code;
      keysDown[code] = true;

      // Handle Immediate Stop / Handbrake
      if (code === 'Space') {
        e.preventDefault();
        this.inputs.handbrake = true;
      }

      // Reset Car Position with 'P'
      if (code === 'KeyP') {
        this.reset();
      }

      this.processKeyStates(keysDown);
    });

    window.addEventListener('keyup', (e) => {
      const code = e.code;
      delete keysDown[code];

      if (code === 'Space') {
        this.inputs.handbrake = false;
      }

      this.processKeyStates(keysDown);
    });
  }

  processKeyStates(keys) {
    // Throttle / Brake
    const forward = keys['KeyW'] || keys['ArrowUp'];
    const backward = keys['KeyS'] || keys['ArrowDown'];
    
    if (forward && !backward) {
      this.inputs.throttle = 1;
    } else if (backward && !forward) {
      this.inputs.throttle = -1;
    } else {
      this.inputs.throttle = 0;
    }

    // Steering
    const left = keys['KeyA'] || keys['ArrowLeft'];
    const right = keys['KeyD'] || keys['ArrowRight'];

    if (left && !right) {
      this.inputs.steer = -1;
    } else if (right && !left) {
      this.inputs.steer = 1;
    } else {
      this.inputs.steer = 0;
    }
  }

  /**
   * Reset vehicle to initial position on main road
   */
  reset(x = this.startX, y = this.startY, heading = this.startHeading) {
    this.x = x;
    this.y = y;
    this.heading = heading;
    this.speed = 0;
    this.steeringAngle = 0;
    this.inputs.throttle = 0;
    this.inputs.steer = 0;
    this.inputs.brake = 0;
    this.inputs.handbrake = false;
    this.state = 'STOPPED';
    this.isBraking = false;
    this.isReversing = false;
    this.isCollided = false;
    this.inPothole = false;
    this.potholeVibration = 0;
    console.log('[EgoVehicle] Reset to starting position.');
  }

  /**
   * Smooth 2D Vehicle Kinematics Physics Integration with Solid Collision and Pothole Drag
   * @param {number} dt - Delta time in seconds
   */
  update(dt) {
    if (dt <= 0 || dt > 0.1) dt = 0.016; // Clamp against delta spike

    // --- 1. Steering Dynamics (Smooth rate & self-centering) ---
    if (this.inputs.steer !== 0) {
      this.steeringAngle += this.inputs.steer * this.steeringSpeed * dt;
      this.steeringAngle = Math.max(-this.maxSteeringAngle, Math.min(this.maxSteeringAngle, this.steeringAngle));
    } else {
      // Auto-center steering when key is released
      if (Math.abs(this.steeringAngle) > 0.005) {
        const returnDir = -Math.sign(this.steeringAngle);
        this.steeringAngle += returnDir * this.steeringReturnSpeed * dt;
        if (Math.sign(this.steeringAngle) !== -returnDir) {
          this.steeringAngle = 0;
        }
      } else {
        this.steeringAngle = 0;
      }
    }

    // --- 2. Longitudinal Acceleration, Braking & Friction ---
    this.isBraking = false;
    this.isReversing = false;

    if (this.inputs.handbrake) {
      // Emergency stop / handbrake
      this.isBraking = true;
      if (Math.abs(this.speed) > 2) {
        const brakeDir = -Math.sign(this.speed);
        this.speed += brakeDir * this.handbrakeDecel * dt;
        if (Math.sign(this.speed) !== -brakeDir) this.speed = 0;
      } else {
        this.speed = 0;
      }
      this.state = 'STOPPED';

    } else if (this.inputs.brake > 0) {
      // Autonomous / Progressive Footbrake
      this.isBraking = true;
      if (Math.abs(this.speed) > 1.5) {
        const brakeDir = -Math.sign(this.speed);
        this.speed += brakeDir * this.brakingDecel * this.inputs.brake * dt;
        if (Math.sign(this.speed) !== -brakeDir) this.speed = 0;
      } else {
        this.speed = 0;
      }
      this.state = (this.speed === 0) ? 'STOPPED' : 'BRAKING';

    } else if (this.inputs.throttle > 0) {
      // Forward Acceleration
      if (this.speed < 0) {
        // Braking while in reverse
        this.isBraking = true;
        this.speed += this.brakingDecel * dt;
        if (this.speed > 0) this.speed = 0;
        this.state = 'BRAKING';
      } else {
        this.speed += this.acceleration * dt;
        this.speed = Math.min(this.maxForwardSpeed, this.speed);
        this.state = this.speed >= this.maxForwardSpeed * 0.95 ? 'CRUISING' : 'ACCELERATING';
      }

    } else if (this.inputs.throttle < 0) {
      // Braking / Reverse
      if (this.speed > 2) {
        // Braking forward motion
        this.isBraking = true;
        this.speed -= this.brakingDecel * dt;
        if (this.speed < 0) this.speed = 0;
        this.state = 'BRAKING';
      } else {
        // Reversing
        this.isReversing = true;
        this.speed -= this.acceleration * 0.7 * dt;
        this.speed = Math.max(-this.maxReverseSpeed, this.speed);
        this.state = 'REVERSING';
      }

    } else {
      // Coasting / Rolling Friction drag
      if (Math.abs(this.speed) > 1) {
        const frictionDir = -Math.sign(this.speed);
        this.speed += frictionDir * this.frictionDecel * dt;
        if (Math.sign(this.speed) !== -frictionDir) this.speed = 0;
        this.state = 'CRUISING';
      } else {
        this.speed = 0;
        this.state = 'STOPPED';
      }
    }

    // --- 3. Kinematic Bicycle Yaw Rate & Heading Integration ---
    if (Math.abs(this.speed) > 0.05) {
      const angularVelocity = (this.speed / this.wheelbase) * Math.tan(this.steeringAngle);
      this.heading += angularVelocity * dt;
      this.heading = Math.atan2(Math.sin(this.heading), Math.cos(this.heading));
    }

    // --- 4. Position Integration with Solid Obstacle Collision Prevention ---
    const proposedX = this.x + this.speed * Math.cos(this.heading) * dt;
    const proposedY = this.y + this.speed * Math.sin(this.heading) * dt;

    const solidCol = StaticCollisionSystem.checkSolidCollision(proposedX, proposedY, 14);

    if (solidCol.collided) {
      // Solid collision resolution: vehicle cannot drive through buildings, barricades, or roadside objects
      this.speed = 0;
      this.state = 'STOPPED';
      this.isCollided = true;
    } else {
      this.x = proposedX;
      this.y = proposedY;
      this.isCollided = false;
    }

    // --- 5. Pothole Hazard Consequence (Speed Reduction & Road Vibration) ---
    const potholeCheck = StaticCollisionSystem.checkPothole(this.x, this.y, 12);
    if (potholeCheck.inPothole) {
      this.inPothole = true;
      this.potholeSeverity = potholeCheck.depth;
      // Heavy friction/drag when driving over pothole
      this.speed *= (1 - 2.8 * dt);
      if (Math.abs(this.speed) < 1) this.speed = 0;
      this.potholeVibration = Math.sin(performance.now() * 0.05) * 2;
    } else {
      this.inPothole = false;
      this.potholeVibration = 0;
    }

    // --- 6. Lidar Sensor Rotation for Animation ---
    this.lidarRotation += 8.0 * dt;
    if (this.lidarRotation > Math.PI * 2) this.lidarRotation -= Math.PI * 2;
  }

  /**
   * Get 4 rotated corner coordinates in World space for rectangular collision detection
   * Order: [Front-Left, Front-Right, Rear-Right, Rear-Left]
   */
  getCollisionCorners() {
    const halfL = this.length / 2;
    const halfW = this.width / 2;
    const cosH = Math.cos(this.heading);
    const sinH = Math.sin(this.heading);

    const relativeCorners = [
      { dx: halfL, dy: -halfW },  // Front-Left
      { dx: halfL, dy: halfW },   // Front-Right
      { dx: -halfL, dy: halfW },  // Rear-Right
      { dx: -halfL, dy: -halfW }  // Rear-Left
    ];

    return relativeCorners.map(pt => ({
      x: this.x + pt.dx * cosH - pt.dy * sinH,
      y: this.y + pt.dx * sinH + pt.dy * cosH
    }));
  }

  /**
   * Get Axis-Aligned Bounding Box (AABB) in World Coordinates
   */
  getBoundingBox() {
    const corners = this.getCollisionCorners();
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    corners.forEach(c => {
      minX = Math.min(minX, c.x);
      minY = Math.min(minY, c.y);
      maxX = Math.max(maxX, c.x);
      maxY = Math.max(maxY, c.y);
    });
    return { minX, minY, maxX, maxY, width: maxX - minX, height: maxY - minY };
  }

  /**
   * Get Oriented Bounding Box (OBB) description
   */
  getOrientedBox() {
    return {
      x: this.x,
      y: this.y,
      length: this.length,
      width: this.width,
      heading: this.heading,
      corners: this.getCollisionCorners()
    };
  }

  /**
   * Render the Ego Autonomous Vehicle in Canvas 2D
   */
  render(ctx) {
    ctx.save();
    ctx.translate(this.x, this.y);
    ctx.rotate(this.heading);

    const halfL = this.length / 2;
    const halfW = this.width / 2;

    // 1. Headlight Lighting Cones (Illuminating the road ahead)
    if (this.speed >= 0) {
      const grad = ctx.createRadialGradient(halfL + 2, 0, 4, halfL + 60, 0, 75);
      grad.addColorStop(0, 'rgba(254, 240, 138, 0.28)');
      grad.addColorStop(1, 'rgba(254, 240, 138, 0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.moveTo(halfL, -halfW + 3);
      ctx.lineTo(halfL + 80, -halfW - 30);
      ctx.lineTo(halfL + 80, halfW + 30);
      ctx.lineTo(halfL, halfW - 3);
      ctx.closePath();
      ctx.fill();
    }

    // 2. Drop Shadow under vehicle
    ctx.fillStyle = 'rgba(0, 0, 0, 0.45)';
    ctx.beginPath();
    ctx.roundRect(-halfL + 3, -halfW + 3, this.length, this.width, 6);
    ctx.fill();

    // 3. Four Wheels / Tires
    const wheelL = 12;
    const wheelW = 5;
    const frontAxleX = halfL - 10;
    const rearAxleX = -halfL + 10;
    const wheelOffsetY = halfW - 1;

    // Front Left Wheel (Steerable)
    this.drawWheel(ctx, frontAxleX, -wheelOffsetY, wheelL, wheelW, this.steeringAngle);
    // Front Right Wheel (Steerable)
    this.drawWheel(ctx, frontAxleX, wheelOffsetY, wheelL, wheelW, this.steeringAngle);
    // Rear Left Wheel (Fixed)
    this.drawWheel(ctx, rearAxleX, -wheelOffsetY, wheelL, wheelW, 0);
    // Rear Right Wheel (Fixed)
    this.drawWheel(ctx, rearAxleX, wheelOffsetY, wheelL, wheelW, 0);

    // 4. Main Vehicle Chassis Body
    ctx.fillStyle = WORLD_CONFIG.colors.egoBody;
    ctx.beginPath();
    ctx.roundRect(-halfL, -halfW, this.length, this.width, 7);
    ctx.fill();

    // Outer Chassis Stroke Border
    ctx.strokeStyle = '#0284c7';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // 5. Front Hood Accent Stripe (Cybernetic AV cyan badge)
    ctx.fillStyle = WORLD_CONFIG.colors.egoAccent;
    ctx.beginPath();
    ctx.moveTo(halfL - 14, -halfW + 6);
    ctx.lineTo(halfL - 4, -halfW + 8);
    ctx.lineTo(halfL - 4, halfW - 8);
    ctx.lineTo(halfL - 14, halfW - 6);
    ctx.closePath();
    ctx.fill();

    // 6. Tinted Glass Cabin (Windshield, Roof, Windows)
    ctx.fillStyle = WORLD_CONFIG.colors.egoGlass;
    // Front Windshield
    ctx.beginPath();
    ctx.roundRect(halfL - 24, -halfW + 3, 9, this.width - 6, 2);
    ctx.fill();

    // Rear Window Glass
    ctx.beginPath();
    ctx.roundRect(-halfL + 8, -halfW + 3, 7, this.width - 6, 2);
    ctx.fill();

    // Roof Center Panel
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(-halfL + 16, -halfW + 4, this.length - 34, this.width - 8);

    // 7. Roof Autonomous Sensor LiDAR Puck
    ctx.fillStyle = '#0f172a';
    ctx.beginPath();
    ctx.arc(0, 0, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = WORLD_CONFIG.colors.egoLiDAR;
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Spinning LiDAR Beam Indicator
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.8)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(Math.cos(this.lidarRotation) * 5.5, Math.sin(this.lidarRotation) * 5.5);
    ctx.stroke();

    // 8. Front LED Headlights
    ctx.fillStyle = WORLD_CONFIG.colors.egoHeadlight;
    ctx.fillRect(halfL - 3, -halfW + 2, 3, 5);
    ctx.fillRect(halfL - 3, halfW - 7, 3, 5);

    // 9. Rear LED Taillights (Bright Neon Red when braking/reversing)
    const brakeActive = this.isBraking || this.isReversing || this.inputs.handbrake;
    ctx.fillStyle = brakeActive ? WORLD_CONFIG.colors.egoTaillightOn : WORLD_CONFIG.colors.egoTaillightOff;
    
    if (brakeActive) {
      ctx.shadowColor = '#ef4444';
      ctx.shadowBlur = 8;
    }
    ctx.fillRect(-halfL, -halfW + 2, 3, 5);
    ctx.fillRect(-halfL, halfW - 7, 3, 5);
    ctx.shadowBlur = 0;

    // 10. Side Mirrors
    ctx.fillStyle = '#0284c7';
    ctx.fillRect(halfL - 22, -halfW - 3, 4, 3);
    ctx.fillRect(halfL - 22, halfW, 4, 3);

    // 11. Subtle Directional Arrow
    ctx.strokeStyle = 'rgba(6, 182, 212, 0.7)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-6, 0);
    ctx.lineTo(8, 0);
    ctx.lineTo(5, -3);
    ctx.moveTo(8, 0);
    ctx.lineTo(5, 3);
    ctx.stroke();

    // 12. Pothole / Collision Alert Badge
    if (this.inPothole) {
      ctx.fillStyle = '#f59e0b';
      ctx.font = 'bold 9px ui-monospace, monospace';
      ctx.textAlign = 'center';
      ctx.fillText('POTHOLE IMPACT', 0, -halfW - 10);
    } else if (this.isCollided) {
      ctx.fillStyle = '#ef4444';
      ctx.font = 'bold 9px ui-monospace, monospace';
      ctx.textAlign = 'center';
      ctx.fillText('BLOCKED (OBSTACLE)', 0, -halfW - 10);
    }

    ctx.restore();
  }

  /**
   * Helper: Draw an individual car wheel with steering rotation
   */
  drawWheel(ctx, x, y, length, width, steerAngle) {
    ctx.save();
    ctx.translate(x, y);
    if (steerAngle) ctx.rotate(steerAngle);

    ctx.fillStyle = '#0f172a';
    ctx.beginPath();
    ctx.roundRect(-length / 2, -width / 2, length, width, 2);
    ctx.fill();

    // Silver rim hubcap
    ctx.fillStyle = '#94a3b8';
    ctx.fillRect(-length / 4, -width / 4, length / 2, width / 2);

    ctx.restore();
  }
}

/* ============================================================================
   6. MODULE 3: DYNAMIC OBJECT & TRAFFIC SYSTEM
   ============================================================================ */

/**
 * Reusable Dynamic Entity Representation for simulation world
 * Types: 'car', 'motorcycle', 'auto_rickshaw', 'pedestrian', 'animal'
 */
class DynamicObject {
  constructor(config = {}) {
    this.id = config.id || `dyn-${Math.random().toString(36).substr(2, 6)}`;
    this.type = config.type || 'car';
    this.subType = config.subType || 'default';
    this.name = config.name || `${this.type}_${this.id}`;

    // World Space Coordinates
    this.x = config.x || 0;
    this.y = config.y || 0;
    this.heading = config.heading !== undefined ? config.heading : 0; // Radians
    this.speed = config.speed !== undefined ? config.speed : 60;       // px/s
    this.baseSpeed = this.speed;

    // Dimensions
    this.length = config.length || 44;
    this.width = config.width || 22;
    this.radius = config.radius || Math.max(this.length, this.width) / 2;

    // Visual Palette
    this.color = config.color || '#dc2626';
    this.accentColor = config.accentColor || '#1e293b';

    // Route / Persistent Destination Configuration
    this.route = config.route || {
      road: 'main',
      laneY: this.y,
      minX: 0,
      maxX: 1800,
      minY: 0,
      maxY: 1000
    };
    this.initPersistentDestination();

    // Local Navigation & Planning State
    this.isBraking = false;
    this.isStopped = false;
    this.state = 'CRUISING'; // 'CRUISING', 'AVOIDING', 'BRAKING', 'STOPPED', 'CROSSING', 'WANDERING'
    this.selectedPath = null;
    this.speedJitter = 0;
    this.stateTimer = Math.random() * 3 + 2;
    this.pauseTimer = 0;
    this.swayPhase = Math.random() * Math.PI * 2;
    this.lateralOffset = 0;
    this.targetLateralOffset = 0;
    this.crossingDirection = config.crossingDirection || 1; // +1 down, -1 up
    this.active = true;
  }

  /**
   * Initialize persistent road network destinations
   */
  initPersistentDestination() {
    if (this.type === 'pedestrian' || this.type === 'animal') return;

    if (this.route.road === 'side') {
      this.routePhase = 'APPROACH_JUNCTION';
      this.destination = { x: this.route.laneX || 895, y: 470 };
    } else {
      const isEastbound = (this.heading === 0 || Math.abs(this.heading) < 0.3);
      if (isEastbound) {
        this.routePhase = 'EASTBOUND';
        this.destination = { x: 1720, y: this.route.laneY || 600 };
      } else {
        this.routePhase = 'WESTBOUND';
        this.destination = { x: 80, y: this.route.laneY || 525 };
      }
    }
  }

  /**
   * Update persistent destination when reaching road waypoint (Never disappears / wraps abruptly)
   */
  updatePersistentDestination() {
    if (this.type === 'pedestrian' || this.type === 'animal') return;

    const distToGoal = Math.hypot(this.destination.x - this.x, this.destination.y - this.y);

    if (distToGoal < 75) {
      if (this.routePhase === 'EASTBOUND') {
        // At East end of main road -> smoothly switch to Westbound lane
        this.routePhase = 'WESTBOUND';
        const targetY = 515 + (Math.random() * 18 - 9);
        this.route.laneY = targetY;
        this.destination = { x: 80, y: targetY };
      } else if (this.routePhase === 'WESTBOUND') {
        // At West end of main road -> smoothly switch to Eastbound lane
        this.routePhase = 'EASTBOUND';
        const targetY = 595 + (Math.random() * 18 - 9);
        this.route.laneY = targetY;
        this.destination = { x: 1720, y: targetY };
      } else if (this.routePhase === 'APPROACH_JUNCTION') {
        // Side road vehicle arrived at T-junction -> merge onto Main Road
        const chooseEast = Math.random() < 0.5;
        if (chooseEast) {
          this.routePhase = 'EASTBOUND';
          this.route.road = 'main';
          this.route.laneY = 600;
          this.destination = { x: 1720, y: 600 };
        } else {
          this.routePhase = 'WESTBOUND';
          this.route.road = 'main';
          this.route.laneY = 525;
          this.destination = { x: 80, y: 525 };
        }
      }
    }
  }

  /**
   * Update kinematics, intelligent local path planning and collision avoidance
   * @param {number} dt - Delta time in seconds
   * @param {EgoVehicle} egoVehicle - Ego vehicle reference
   * @param {Array<DynamicObject>} allEntities - All active traffic entities
   * @param {Object} environmentData - Static environment definition
   */
  update(dt, egoVehicle, allEntities, environmentData) {
    if (!this.active) return;
    if (dt <= 0 || dt > 0.1) dt = 0.016;

    this.swayPhase += 6.0 * dt;
    if (this.swayPhase > Math.PI * 2) this.swayPhase -= Math.PI * 2;
    this.stateTimer -= dt;

    switch (this.type) {
      case 'car':
      case 'motorcycle':
      case 'auto_rickshaw':
        this.navigateTrafficVehicle(dt, egoVehicle, allEntities, environmentData);
        break;
      case 'pedestrian':
        this.updatePedestrian(dt);
        break;
      case 'animal':
        this.updateAnimal(dt);
        break;
      default:
        this.updateGeneric(dt);
        break;
    }
  }

  // --- Intelligent Local Traffic Planner & Collision Prevention ---

  /**
   * Perceive surrounding dynamic threats within local sensor range
   */
  perceiveThreats(egoVehicle, allEntities) {
    const threats = [];

    // 1. Ego Vehicle
    if (egoVehicle) {
      const dx = egoVehicle.x - this.x;
      const dy = egoVehicle.y - this.y;
      const dist = Math.hypot(dx, dy);
      if (dist < 180) {
        threats.push({
          id: 'ego',
          type: 'ego_vehicle',
          x: egoVehicle.x,
          y: egoVehicle.y,
          radius: 18,
          speed: egoVehicle.speed || 0,
          heading: egoVehicle.heading || 0
        });
      }
    }

    // 2. Other Dynamic Traffic Entities
    if (Array.isArray(allEntities)) {
      for (const ent of allEntities) {
        if (ent.id === this.id || !ent.active) continue;
        const dx = ent.x - this.x;
        const dy = ent.y - this.y;
        const dist = Math.hypot(dx, dy);
        if (dist < 180) {
          threats.push({
            id: ent.id,
            type: ent.type,
            x: ent.x,
            y: ent.y,
            radius: ent.radius || 14,
            speed: ent.speed || 0,
            heading: ent.heading || 0
          });
        }
      }
    }

    return threats;
  }

  /**
   * Generate short candidate trajectories for local navigation
   */
  generateLocalPaths() {
    const isBike = (this.type === 'motorcycle');
    const lookaheadDist = isBike ? 65 : 80;
    const numPoints = 6;
    const stepSize = lookaheadDist / numPoints;

    // Desired base heading toward persistent destination
    const destAngle = Math.atan2(this.destination.y - this.y, this.destination.x - this.x);
    let baseAngleDiff = destAngle - this.heading;
    while (baseAngleDiff > Math.PI) baseAngleDiff -= Math.PI * 2;
    while (baseAngleDiff < -Math.PI) baseAngleDiff += Math.PI * 2;

    const baseOffset = Math.sign(baseAngleDiff) * Math.min(Math.abs(baseAngleDiff), 0.28);
    const avoidAngle = isBike ? 0.40 : 0.32;

    const steerAngles = [
      { id: 'STRAIGHT',     offset: baseOffset },
      { id: 'SLIGHT_LEFT',  offset: baseOffset - 0.15 },
      { id: 'SLIGHT_RIGHT', offset: baseOffset + 0.15 },
      { id: 'AVOID_LEFT',   offset: baseOffset - avoidAngle },
      { id: 'AVOID_RIGHT',  offset: baseOffset + avoidAngle }
    ];

    return steerAngles.map(def => {
      const waypoints = [];
      let curX = this.x;
      let curY = this.y;
      const targetHeading = this.heading + def.offset;

      for (let i = 1; i <= numPoints; i++) {
        curX += stepSize * Math.cos(targetHeading);
        curY += stepSize * Math.sin(targetHeading);
        waypoints.push({ x: curX, y: curY });
      }

      return {
        id: def.id,
        steerOffset: def.offset,
        targetHeading: targetHeading,
        waypoints: waypoints,
        endPoint: waypoints[waypoints.length - 1],
        feasible: true,
        cost: 0
      };
    });
  }

  /**
   * Score candidate path against collision threats, obstacles, potholes, destination progress, and road bounds
   */
  scoreLocalPath(path, threats, environmentData) {
    let cost = 0;
    const myRadius = this.radius;

    // 1. Evaluate Waypoints against Threats, Obstacles, Potholes & Road Bounds
    for (let i = 0; i < path.waypoints.length; i++) {
      const wp = path.waypoints[i];

      // A. Dynamic threats (Ego, other cars, bikes, autos, pedestrians, animals)
      for (const threat of threats) {
        const dist = Math.hypot(wp.x - threat.x, wp.y - threat.y);
        const clearance = dist - (myRadius + threat.radius);

        if (clearance <= 2) {
          path.feasible = false;
          return 10000;
        }

        if (clearance < 35) {
          cost += 350 / (clearance + 2);
        }
      }

      // B. Solid static collision obstacles (buildings, barricades, parked cars, trees, signal)
      if (window.StaticCollisionSystem) {
        const solidCol = window.StaticCollisionSystem.checkSolidCollision(wp.x, wp.y, myRadius);
        if (solidCol.collided) {
          path.feasible = false;
          return 10000;
        }

        // C. Pothole Road Surface Hazard
        const pot = window.StaticCollisionSystem.checkPothole(wp.x, wp.y, myRadius * 0.75);
        if (pot.inPothole) {
          cost += 1800; // Strong penalty to steer around potholes if clear road exists
        }
      }

      // D. Drivable Road Constraints
      const onMain = (wp.y >= 470 && wp.y <= 650) && (wp.x >= 0 && wp.x <= 1800);
      const onSide = (wp.x >= 840 && wp.x <= 990) && (wp.y >= 0 && wp.y <= 580);
      if (!onMain && !onSide) {
        path.feasible = false;
        return 8000;
      }
    }

    // 2. Goal Progress toward Persistent Destination
    const startDist = Math.hypot(this.destination.x - this.x, this.destination.y - this.y);
    const endDist = Math.hypot(this.destination.x - path.endPoint.x, this.destination.y - path.endPoint.y);
    const deltaGoal = endDist - startDist;

    if (deltaGoal < 0) {
      cost += deltaGoal * 2.5; // Bonus for progress towards goal
    } else {
      cost += 400 + deltaGoal * 3.0;
    }

    // 3. Smoothness & Steering Effort
    cost += Math.pow(Math.abs(path.steerOffset), 1.6) * 35;

    return cost;
  }

  /**
   * Check if a pothole is directly in front of this dynamic vehicle
   */
  detectPotholeAhead(lookahead = 65) {
    if (!EnvironmentData.potholes) return null;
    const cosH = Math.cos(this.heading);
    const sinH = Math.sin(this.heading);

    for (const p of EnvironmentData.potholes) {
      const dx = p.x - this.x;
      const dy = p.y - this.y;
      const forwardDist = dx * cosH + dy * sinH;
      const lateralDist = Math.abs(dy * cosH - dx * sinH);

      if (forwardDist > 5 && forwardDist < (lookahead + p.radius) && lateralDist < (p.radius + this.width / 2 + 4)) {
        return { pothole: p, forwardDist, lateralDist, pY: p.y, pX: p.x };
      }
    }
    return null;
  }

  /**
   * Master navigation and control loop for non-ego road vehicles (Cars, Bikes, Autos)
   */
  navigateTrafficVehicle(dt, egoVehicle, allEntities, environmentData) {
    this.updatePersistentDestination();

    // 1. Gather all obstacles and collision threats
    const threats = this.perceiveThreats(egoVehicle, allEntities);

    // 2. Generate local candidate rollout paths
    const candidatePaths = this.generateLocalPaths();

    // 3. Score candidate paths
    let bestPath = null;
    let minCost = Infinity;

    for (const path of candidatePaths) {
      const cost = this.scoreLocalPath(path, threats, environmentData);
      path.cost = cost;
      if (path.feasible && cost < minCost) {
        minCost = cost;
        bestPath = path;
      }
    }

    this.selectedPath = bestPath;

    // 4. Calculate Forward Lookahead Threat & Target Speed
    let targetSpeed = this.baseSpeed;
    this.isBraking = false;
    let obstacleDirectlyAhead = false;
    let closestObstacleDist = Infinity;

    // Check direct forward corridor for obstacles and vehicles
    const cosH = Math.cos(this.heading);
    const sinH = Math.sin(this.heading);

    for (const threat of threats) {
      const dx = threat.x - this.x;
      const dy = threat.y - this.y;
      const fwd = dx * cosH + dy * sinH;
      const lat = Math.abs(dy * cosH - dx * sinH);

      // In forward corridor
      if (fwd > 0 && fwd < 110 && lat < (this.radius + (threat.radius || 12) + 6)) {
        if (fwd < closestObstacleDist) {
          closestObstacleDist = fwd;
        }

        // Relative speed & TTC
        const relVx = (threat.speed || 0) * Math.cos(threat.heading || 0) - this.speed * cosH;
        const relVy = (threat.speed || 0) * Math.sin(threat.heading || 0) - this.speed * sinH;
        const closingSpeed = -(relVx * (dx / (fwd || 1)) + relVy * (dy / (fwd || 1)));

        // Critical safety gap
        const safeGap = this.length * 0.75 + (threat.radius || 12);

        if (fwd < safeGap + 12 || (closingSpeed > 5 && (fwd - safeGap) / closingSpeed < 1.6)) {
          obstacleDirectlyAhead = true;
        }
      }
    }

    // Check Potholes in forward path
    const potholeCheck = this.detectPotholeAhead(65);

    // 5. Intelligent Speed & Braking Resolution
    if (!bestPath || obstacleDirectlyAhead) {
      // Path is completely blocked or imminent collision -> FULL STOP
      targetSpeed = 0;
      this.isBraking = true;
      this.state = 'STOPPED';
    } else if (closestObstacleDist < 60) {
      // Developing proximity -> SLOW down significantly
      targetSpeed = this.baseSpeed * 0.35;
      this.isBraking = true;
      this.state = 'BRAKING';
    } else if (bestPath.steerOffset !== 0) {
      // Executing avoidance maneuver around obstacle/pothole
      targetSpeed = this.baseSpeed * 0.7;
      this.state = 'AVOIDING';
    } else if (potholeCheck) {
      // Approaching pothole without alternate path -> SLOW
      targetSpeed = this.baseSpeed * 0.4;
      this.isBraking = true;
      this.state = 'BRAKING';
    } else {
      // Normal clear road cruising with organic speed variation
      if (this.stateTimer <= 0) {
        this.stateTimer = Math.random() * 3 + 2;
        this.speedJitter = (Math.random() * 8 - 4);
      }
      targetSpeed = this.baseSpeed + (this.speedJitter || 0);
      this.state = 'CRUISING';
    }

    // 6. Smooth Acceleration & Deceleration Physics
    const maxAccel = this.type === 'motorcycle' ? 180 : 120;
    const maxDecel = 280;

    if (this.speed < targetSpeed) {
      this.speed = Math.min(targetSpeed, this.speed + maxAccel * dt);
    } else if (this.speed > targetSpeed) {
      this.speed = Math.max(targetSpeed, this.speed - maxDecel * dt);
    }

    if (this.speed < 1 && targetSpeed === 0) {
      this.speed = 0;
      this.state = 'STOPPED';
    }

    // 7. Steering & Kinematics Update
    if (bestPath && this.speed > 0) {
      const turnRate = this.type === 'motorcycle' ? 4.5 : 3.0;
      let targetHeading = bestPath.targetHeading;
      let headingDiff = targetHeading - this.heading;
      while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
      while (headingDiff < -Math.PI) headingDiff += Math.PI * 2;

      this.heading += Math.sign(headingDiff) * Math.min(Math.abs(headingDiff), turnRate * dt);
      this.heading = Math.atan2(Math.sin(this.heading), Math.cos(this.heading));
    }

    // Position integration with solid obstacle collision check
    const proposedX = this.x + this.speed * Math.cos(this.heading) * dt;
    const proposedY = this.y + this.speed * Math.sin(this.heading) * dt;

    if (window.StaticCollisionSystem) {
      const solidCol = window.StaticCollisionSystem.checkSolidCollision(proposedX, proposedY, this.radius);
      if (solidCol.collided) {
        // Stop before solid object
        this.speed = 0;
        this.isBraking = true;
        this.state = 'STOPPED';
      } else {
        this.x = proposedX;
        this.y = proposedY;
      }
    } else {
      this.x = proposedX;
      this.y = proposedY;
    }
  }

  updatePedestrian(dt) {
    if (this.pauseTimer > 0) {
      this.pauseTimer -= dt;
      return;
    }

    if (this.behavior === 'CROSSING') {
      // Crossing from one sidewalk to the other
      this.y += this.speed * this.crossingDirection * dt;
      this.heading = this.crossingDirection > 0 ? Math.PI / 2 : -Math.PI / 2;

      // When crossed fully
      if (this.crossingDirection > 0 && this.y >= 680) {
        this.y = 680;
        this.pauseTimer = Math.random() * 3 + 2;
        this.crossingDirection = -1;
        this.behavior = 'SIDEWALK';
        this.heading = Math.PI; // Walk along sidewalk
      } else if (this.crossingDirection < 0 && this.y <= 435) {
        this.y = 435;
        this.pauseTimer = Math.random() * 3 + 2;
        this.crossingDirection = 1;
        this.behavior = 'SIDEWALK';
        this.heading = 0; // Walk along sidewalk
      }
    } else {
      // Walking along sidewalk
      this.x += this.speed * Math.cos(this.heading) * dt;

      if (this.stateTimer <= 0) {
        this.stateTimer = Math.random() * 6 + 4;
        // Chance to start crossing when near middle zones
        if (this.x > 300 && this.x < 1500 && Math.random() < 0.45) {
          this.behavior = 'CROSSING';
        }
      }

      // Turn around on sidewalk ends
      if (this.x > 1780) {
        this.heading = Math.PI;
      } else if (this.x < 40) {
        this.heading = 0;
      }
    }
  }

  updateAnimal(dt) {
    if (this.subType === 'cow') {
      if (this.pauseTimer > 0) {
        this.pauseTimer -= dt;
        this.behavior = 'PAUSED';
        return;
      }

      this.behavior = 'WANDERING';
      this.x += this.speed * Math.cos(this.heading) * dt;
      this.y += this.speed * Math.sin(this.heading) * dt;

      // Keep within road/shoulder boundaries (Y: 480 - 640)
      if (this.y < 480 || this.y > 640) {
        this.heading = -this.heading;
      }

      if (this.stateTimer <= 0) {
        this.stateTimer = Math.random() * 5 + 3;
        // Cows frequently stop on the road (classic Indian road simulation feature)
        if (Math.random() < 0.4) {
          this.pauseTimer = Math.random() * 4 + 2.5;
        } else {
          // Slow wandering turn
          this.heading += (Math.random() * 0.8 - 0.4);
        }
      }

      // Turn around on world edges
      if (this.x > 1800) this.heading = Math.PI;
      if (this.x < 20) this.heading = 0;

    } else if (this.subType === 'dog') {
      if (this.pauseTimer > 0) {
        this.pauseTimer -= dt;
        return;
      }

      this.x += this.speed * Math.cos(this.heading) * dt;
      this.y += this.speed * Math.sin(this.heading) * dt;

      if (this.y < 440 || this.y > 670) {
        this.heading = -this.heading + (Math.random() * 0.4 - 0.2);
      }

      if (this.stateTimer <= 0) {
        this.stateTimer = Math.random() * 3 + 2;
        if (Math.random() < 0.3) {
          this.pauseTimer = Math.random() * 2 + 1; // Sniff / pause
        } else {
          // Quick turn
          this.heading += (Math.random() * 1.6 - 0.8);
        }
      }

      if (this.x > 1800) this.heading = Math.PI;
      if (this.x < 20) this.heading = 0;
    }
  }

  updateGeneric(dt) {
    this.x += this.speed * Math.cos(this.heading) * dt;
    this.y += this.speed * Math.sin(this.heading) * dt;
  }

  // --- Visual Render Pipeline ---

  render(ctx) {
    ctx.save();
    ctx.translate(this.x, this.y);
    ctx.rotate(this.heading);

    switch (this.type) {
      case 'car':
        this.renderCar(ctx);
        break;
      case 'motorcycle':
        this.renderMotorcycle(ctx);
        break;
      case 'auto_rickshaw':
        this.renderAutoRickshaw(ctx);
        break;
      case 'pedestrian':
        this.renderPedestrian(ctx);
        break;
      case 'animal':
        this.renderAnimal(ctx);
        break;
      default:
        this.renderGeneric(ctx);
        break;
    }

    ctx.restore();
  }

  /**
   * Render dynamic traffic car
   */
  renderCar(ctx) {
    const halfL = this.length / 2;
    const halfW = this.width / 2;

    // Drop Shadow
    ctx.fillStyle = 'rgba(0, 0, 0, 0.35)';
    ctx.beginPath();
    ctx.roundRect(-halfL + 2, -halfW + 2, this.length, this.width, 5);
    ctx.fill();

    // Wheels
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(halfL - 10, -halfW - 1, 9, 3);
    ctx.fillRect(halfL - 10, halfW - 2, 9, 3);
    ctx.fillRect(-halfL + 3, -halfW - 1, 9, 3);
    ctx.fillRect(-halfL + 3, halfW - 2, 9, 3);

    // Car Body
    ctx.fillStyle = this.color;
    ctx.beginPath();
    ctx.roundRect(-halfL, -halfW, this.length, this.width, 5);
    ctx.fill();

    // Chassis Stroke
    ctx.strokeStyle = 'rgba(0,0,0,0.3)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Windshield Glass & Windows
    ctx.fillStyle = '#0f172a';
    // Front windshield
    ctx.beginPath();
    ctx.roundRect(halfL - 16, -halfW + 3, 6, this.width - 6, 2);
    ctx.fill();

    // Rear window
    ctx.beginPath();
    ctx.roundRect(-halfL + 7, -halfW + 3, 5, this.width - 6, 2);
    ctx.fill();

    // Roof Top Plate
    ctx.fillStyle = this.accentColor || '#1e293b';
    ctx.fillRect(-halfL + 13, -halfW + 3, this.length - 28, this.width - 6);

    // Front Headlights
    ctx.fillStyle = '#fef08a';
    ctx.fillRect(halfL - 2, -halfW + 2, 2, 4);
    ctx.fillRect(halfL - 2, halfW - 6, 2, 4);

    // Rear Taillights (glow when braking or stopped)
    if (this.isBraking || this.speed < 5) {
      ctx.fillStyle = '#f43f5e';
      ctx.shadowColor = '#ef4444';
      ctx.shadowBlur = 8;
    } else {
      ctx.fillStyle = '#991b1b';
      ctx.shadowBlur = 0;
    }
    ctx.fillRect(-halfL, -halfW + 2, 2, 4);
    ctx.fillRect(-halfL, halfW - 6, 2, 4);
    ctx.shadowBlur = 0;
  }

  /**
   * Render dynamic motorcycle / scooter
   */
  renderMotorcycle(ctx) {
    const halfL = this.length / 2;
    const halfW = this.width / 2;

    // Drop shadow
    ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
    ctx.beginPath();
    ctx.roundRect(-halfL + 1, -halfW + 1, this.length, this.width, 3);
    ctx.fill();

    // Front & Rear Rubber Wheels
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(halfL - 6, -2, 6, 4);
    ctx.fillRect(-halfL, -2, 6, 4);

    // Bike Frame & Fuel Tank
    ctx.fillStyle = this.color;
    ctx.beginPath();
    ctx.roundRect(-halfL + 5, -3, this.length - 10, 6, 2);
    ctx.fill();

    // Handlebars
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(halfL - 7, -halfW + 1);
    ctx.lineTo(halfL - 7, halfW - 1);
    ctx.stroke();

    // Rider Body (Top-down view)
    ctx.fillStyle = '#1e293b';
    ctx.beginPath();
    ctx.ellipse(-2, 0, 6.5, 5, 0, 0, Math.PI * 2);
    ctx.fill();

    // Rider Helmet
    ctx.fillStyle = this.color;
    ctx.beginPath();
    ctx.arc(-2, 0, 4.2, 0, Math.PI * 2);
    ctx.fill();

    // Helmet Visor
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, -3, 2, 6);

    // Rear Taillight
    if (this.isBraking || this.speed < 5) {
      ctx.fillStyle = '#f43f5e';
      ctx.shadowColor = '#ef4444';
      ctx.shadowBlur = 6;
    } else {
      ctx.fillStyle = '#991b1b';
      ctx.shadowBlur = 0;
    }
    ctx.fillRect(-halfL, -1.5, 2, 3);
    ctx.shadowBlur = 0;
  }

  /**
   * Render dynamic Indian Auto-Rickshaw (3-Wheeler)
   */
  renderAutoRickshaw(ctx) {
    const halfL = this.length / 2;
    const halfW = this.width / 2;

    // Drop shadow
    ctx.fillStyle = 'rgba(0, 0, 0, 0.35)';
    ctx.fillRect(-halfL + 2, -halfW + 2, this.length, this.width);

    // Green Lower Chassis
    ctx.fillStyle = this.color || '#15803d';
    ctx.fillRect(-halfL, -halfW, this.length, this.width);

    // Tapered Front Nose
    ctx.beginPath();
    ctx.moveTo(halfL - 8, -halfW);
    ctx.lineTo(halfL, -halfW / 3);
    ctx.lineTo(halfL, halfW / 3);
    ctx.lineTo(halfL - 8, halfW);
    ctx.closePath();
    ctx.fill();

    // Wheels (1 front, 2 rear)
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(halfL - 3, -2, 4, 4);
    ctx.fillRect(-halfL + 5, -halfW - 2, 7, 3);
    ctx.fillRect(-halfL + 5, halfW - 1, 7, 3);

    // Iconic Yellow Canopy Roof
    ctx.fillStyle = '#eab308';
    ctx.beginPath();
    ctx.roundRect(-halfL + 5, -halfW + 2, this.length - 14, this.width - 4, 3);
    ctx.fill();

    // Front Windshield
    ctx.fillStyle = '#38bdf8';
    ctx.fillRect(halfL - 10, -halfW + 3, 3, this.width - 6);

    // Rear Black Bumper & Taillights
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(-halfL, -halfW + 2, 3, this.width - 4);

    if (this.isBraking || this.speed < 5) {
      ctx.fillStyle = '#f43f5e';
      ctx.shadowColor = '#ef4444';
      ctx.shadowBlur = 8;
    } else {
      ctx.fillStyle = '#991b1b';
      ctx.shadowBlur = 0;
    }
    ctx.fillRect(-halfL, -halfW + 3, 2, 4);
    ctx.fillRect(-halfL, halfW - 7, 2, 4);
    ctx.shadowBlur = 0;
  }

  /**
   * Render dynamic pedestrian (Top-down walking human)
   */
  renderPedestrian(ctx) {
    // Drop shadow
    ctx.fillStyle = 'rgba(0, 0, 0, 0.25)';
    ctx.beginPath();
    ctx.ellipse(1, 1, 6, 5, 0, 0, Math.PI * 2);
    ctx.fill();

    // Animated swinging feet during walking
    const footOffset = Math.sin(this.swayPhase) * 3.5;
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(-3, -4 + footOffset, 3, 2.5);
    ctx.fillRect(-3, 1.5 - footOffset, 3, 2.5);

    // Shoulders & Colorful Indian Attire
    ctx.fillStyle = this.color;
    ctx.beginPath();
    ctx.ellipse(0, 0, 6, 4.5, 0, 0, Math.PI * 2);
    ctx.fill();

    // Skin-tone Head
    ctx.fillStyle = '#fed7aa';
    ctx.beginPath();
    ctx.arc(0, 0, 3.4, 0, Math.PI * 2);
    ctx.fill();

    // Dark Hair
    ctx.fillStyle = '#0f172a';
    ctx.beginPath();
    ctx.arc(-0.8, 0, 3.2, -Math.PI / 2, Math.PI / 2);
    ctx.fill();
  }

  /**
   * Render dynamic animals (Stray Cow & Dog)
   */
  renderAnimal(ctx) {
    const halfL = this.length / 2;
    const halfW = this.width / 2;

    if (this.subType === 'cow') {
      // --- Indian Zebu Cow ---
      // Shadow
      ctx.fillStyle = 'rgba(0, 0, 0, 0.25)';
      ctx.beginPath();
      ctx.ellipse(2, 2, halfL, halfW * 0.9, 0, 0, Math.PI * 2);
      ctx.fill();

      // Main Torso
      ctx.fillStyle = this.color || '#f8fafc';
      ctx.beginPath();
      ctx.ellipse(0, 0, halfL - 4, halfW * 0.8, 0, 0, Math.PI * 2);
      ctx.fill();

      // Zebu Shoulder Hump (Distinctive Indian cattle feature)
      ctx.fillStyle = '#cbd5e1';
      ctx.beginPath();
      ctx.arc(halfL - 10, 0, 5, 0, Math.PI * 2);
      ctx.fill();

      // Head
      ctx.fillStyle = this.color || '#f8fafc';
      ctx.beginPath();
      ctx.ellipse(halfL - 2, 0, 6, 4.5, 0, 0, Math.PI * 2);
      ctx.fill();

      // Horns
      ctx.strokeStyle = '#475569';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(halfL - 4, -4);
      ctx.lineTo(halfL - 2, -8);
      ctx.moveTo(halfL - 4, 4);
      ctx.lineTo(halfL - 2, 8);
      ctx.stroke();

      // Ears
      ctx.fillStyle = '#cbd5e1';
      ctx.beginPath();
      ctx.ellipse(halfL - 4, -5, 3, 1.5, -0.4, 0, Math.PI * 2);
      ctx.ellipse(halfL - 4, 5, 3, 1.5, 0.4, 0, Math.PI * 2);
      ctx.fill();

      // Tail
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(-halfL + 4, 0);
      ctx.quadraticCurveTo(-halfL - 4, Math.sin(this.swayPhase) * 3, -halfL - 6, Math.sin(this.swayPhase) * 4);
      ctx.stroke();

    } else if (this.subType === 'dog') {
      // --- Stray Dog ---
      // Shadow
      ctx.fillStyle = 'rgba(0, 0, 0, 0.2)';
      ctx.beginPath();
      ctx.ellipse(1, 1, halfL, halfW * 0.8, 0, 0, Math.PI * 2);
      ctx.fill();

      // Body
      ctx.fillStyle = this.color || '#b45309';
      ctx.beginPath();
      ctx.ellipse(0, 0, halfL - 3, halfW * 0.7, 0, 0, Math.PI * 2);
      ctx.fill();

      // Head & Snout
      ctx.fillStyle = this.color || '#b45309';
      ctx.beginPath();
      ctx.ellipse(halfL - 2, 0, 4, 3, 0, 0, Math.PI * 2);
      ctx.fill();

      // Ears
      ctx.fillStyle = '#78350f';
      ctx.fillRect(halfL - 4, -3.5, 2, 2);
      ctx.fillRect(halfL - 4, 1.5, 2, 2);

      // Wagging Tail
      ctx.strokeStyle = this.color || '#b45309';
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.moveTo(-halfL + 3, 0);
      ctx.lineTo(-halfL - 4, Math.sin(this.swayPhase * 1.5) * 4);
      ctx.stroke();
    }
  }

  renderGeneric(ctx) {
    ctx.fillStyle = this.color;
    ctx.fillRect(-this.length / 2, -this.width / 2, this.length, this.width);
  }

  // --- Collision Boundary Queries ---

  /**
   * Get 4 rotated corner coordinates in World space for collision detection
   */
  getCollisionCorners() {
    const halfL = this.length / 2;
    const halfW = this.width / 2;
    const cosH = Math.cos(this.heading);
    const sinH = Math.sin(this.heading);

    const relativeCorners = [
      { dx: halfL, dy: -halfW },
      { dx: halfL, dy: halfW },
      { dx: -halfL, dy: halfW },
      { dx: -halfL, dy: -halfW }
    ];

    return relativeCorners.map(pt => ({
      x: this.x + pt.dx * cosH - pt.dy * sinH,
      y: this.y + pt.dx * sinH + pt.dy * cosH
    }));
  }

  /**
   * Get Axis-Aligned Bounding Box (AABB) in World Coordinates
   */
  getBoundingBox() {
    const corners = this.getCollisionCorners();
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    corners.forEach(c => {
      minX = Math.min(minX, c.x);
      minY = Math.min(minY, c.y);
      maxX = Math.max(maxX, c.x);
      maxY = Math.max(maxY, c.y);
    });
    return { minX, minY, maxX, maxY, width: maxX - minX, height: maxY - minY };
  }

  /**
   * Get Oriented Bounding Box (OBB) description
   */
  getOrientedBox() {
    return {
      x: this.x,
      y: this.y,
      length: this.length,
      width: this.width,
      heading: this.heading,
      corners: this.getCollisionCorners()
    };
  }

  /**
   * Unified collision boundary interface for future modules
   */
  getCollisionBounds() {
    return {
      id: this.id,
      type: this.type,
      subType: this.subType,
      x: this.x,
      y: this.y,
      radius: this.radius,
      length: this.length,
      width: this.width,
      heading: this.heading,
      speed: this.speed,
      corners: this.getCollisionCorners(),
      aabb: this.getBoundingBox(),
      obb: this.getOrientedBox()
    };
  }
}

/**
 * Traffic Manager: Coordinates dynamic traffic objects and lifecycle
 */
class TrafficManager {
  constructor() {
    this.entities = [];
    this.isEnabled = true;
    this.initDefaultEntities();
  }

  initDefaultEntities() {
    this.entities = [
      // --- 1. Dynamic Cars (4) ---
      new DynamicObject({
        id: 'car-1',
        type: 'car',
        subType: 'sedan',
        x: 350,
        y: 595,
        length: 50,
        width: 25,
        heading: 0, // Eastbound
        speed: 85,
        color: '#dc2626',
        route: { road: 'main', laneY: 595 }
      }),
      new DynamicObject({
        id: 'car-2',
        type: 'car',
        subType: 'hatchback',
        x: 1350,
        y: 525,
        length: 46,
        width: 23,
        heading: Math.PI, // Westbound
        speed: 90,
        color: '#2563eb',
        route: { road: 'main', laneY: 525 }
      }),
      new DynamicObject({
        id: 'car-3',
        type: 'car',
        subType: 'suv',
        x: 580,
        y: 605,
        length: 52,
        width: 26,
        heading: 0, // Eastbound
        speed: 75,
        color: '#e2e8f0',
        route: { road: 'main', laneY: 605 }
      }),
      new DynamicObject({
        id: 'car-4',
        type: 'car',
        subType: 'taxi',
        x: 895,
        y: 100,
        length: 46,
        width: 23,
        heading: Math.PI / 2, // Southbound on side road
        speed: 65,
        color: '#f59e0b',
        route: { road: 'side', laneX: 895 }
      }),

      // --- 2. Motorcycles & Bikes (4) ---
      new DynamicObject({
        id: 'bike-1',
        type: 'motorcycle',
        subType: 'sport_bike',
        x: 480,
        y: 575,
        length: 28,
        width: 12,
        heading: 0, // Eastbound
        speed: 120,
        color: '#ea580c',
        route: { road: 'main', laneY: 575 }
      }),
      new DynamicObject({
        id: 'bike-2',
        type: 'motorcycle',
        subType: 'commuter',
        x: 1450,
        y: 515,
        length: 27,
        width: 11,
        heading: Math.PI, // Westbound
        speed: 105,
        color: '#1e293b',
        route: { road: 'main', laneY: 515 }
      }),
      new DynamicObject({
        id: 'bike-3',
        type: 'motorcycle',
        subType: 'scooter',
        x: 180,
        y: 615,
        length: 25,
        width: 12,
        heading: 0, // Eastbound
        speed: 80,
        color: '#06b6d4',
        route: { road: 'main', laneY: 615 }
      }),
      new DynamicObject({
        id: 'bike-4',
        type: 'motorcycle',
        subType: 'scooter',
        x: 940,
        y: 200,
        length: 25,
        width: 12,
        heading: Math.PI / 2, // Southbound on side road
        speed: 85,
        color: '#ec4899',
        route: { road: 'side', laneX: 940 }
      }),

      // --- 3. Auto-Rickshaws (3) ---
      new DynamicObject({
        id: 'auto-1',
        type: 'auto_rickshaw',
        subType: 'cng_auto',
        x: 1020,
        y: 605,
        length: 44,
        width: 26,
        heading: 0, // Eastbound
        speed: 70,
        color: '#15803d',
        route: { road: 'main', laneY: 605 }
      }),
      new DynamicObject({
        id: 'auto-2',
        type: 'auto_rickshaw',
        subType: 'cng_auto',
        x: 620,
        y: 525,
        length: 44,
        width: 26,
        heading: Math.PI, // Westbound
        speed: 65,
        color: '#15803d',
        route: { road: 'main', laneY: 525 }
      }),
      new DynamicObject({
        id: 'auto-3',
        type: 'auto_rickshaw',
        subType: 'cng_auto',
        x: 955,
        y: 60,
        length: 44,
        width: 26,
        heading: Math.PI / 2, // Southbound on side road
        speed: 60,
        color: '#15803d',
        route: { road: 'side', laneX: 955 }
      }),

      // --- 4. Pedestrians (4) ---
      new DynamicObject({
        id: 'ped-1',
        type: 'pedestrian',
        subType: 'commuter',
        x: 220,
        y: 435,
        length: 12,
        width: 10,
        radius: 6,
        heading: 0,
        speed: 25,
        color: '#f97316',
        behavior: 'SIDEWALK'
      }),
      new DynamicObject({
        id: 'ped-2',
        type: 'pedestrian',
        subType: 'commuter',
        x: 1420,
        y: 680,
        length: 12,
        width: 10,
        radius: 6,
        heading: Math.PI,
        speed: 28,
        color: '#38bdf8',
        behavior: 'SIDEWALK'
      }),
      new DynamicObject({
        id: 'ped-3',
        type: 'pedestrian',
        subType: 'crosser',
        x: 720,
        y: 680,
        length: 12,
        width: 10,
        radius: 6,
        heading: -Math.PI / 2,
        speed: 22,
        color: '#ec4899',
        crossingDirection: -1,
        behavior: 'CROSSING'
      }),
      new DynamicObject({
        id: 'ped-4',
        type: 'pedestrian',
        subType: 'crosser',
        x: 1240,
        y: 435,
        length: 12,
        width: 10,
        radius: 6,
        heading: Math.PI / 2,
        speed: 24,
        color: '#eab308',
        crossingDirection: 1,
        behavior: 'CROSSING'
      }),

      // --- 5. Animals (3) ---
      new DynamicObject({
        id: 'cow-1',
        type: 'animal',
        subType: 'cow',
        x: 530,
        y: 505,
        length: 36,
        width: 18,
        radius: 12,
        heading: 0.1,
        speed: 10,
        color: '#f8fafc',
        behavior: 'WANDERING'
      }),
      new DynamicObject({
        id: 'dog-1',
        type: 'animal',
        subType: 'dog',
        x: 870,
        y: 460,
        length: 20,
        width: 10,
        radius: 7,
        heading: -0.3,
        speed: 40,
        color: '#b45309',
        behavior: 'WANDERING'
      }),
      new DynamicObject({
        id: 'dog-2',
        type: 'animal',
        subType: 'dog',
        x: 1350,
        y: 660,
        length: 20,
        width: 10,
        radius: 7,
        heading: Math.PI,
        speed: 35,
        color: '#d97706',
        behavior: 'WANDERING'
      })
    ];
  }

  update(dt, egoVehicle, environmentData) {
    if (!this.isEnabled) return;
    const ego = egoVehicle || (window.SimulationEngine ? window.SimulationEngine.getEgoVehicle() : null);
    const env = environmentData || window.EnvironmentData;
    this.entities.forEach(entity => entity.update(dt, ego, this.entities, env));
  }

  render(ctx) {
    this.entities.forEach(entity => entity.render(ctx));
  }

  getEntities() {
    return this.entities;
  }

  getDynamicObjects() {
    return this.entities;
  }

  toggle(state) {
    this.isEnabled = state !== undefined ? state : !this.isEnabled;
    return this.isEnabled;
  }

  reset() {
    this.initDefaultEntities();
  }
}

/* ============================================================================
   7. MODULE 4: DETECTION & COLLISION RISK ASSESSMENT SYSTEM
   ============================================================================ */

/**
 * Detection & Risk Assessment Manager
 * Performs simulated perception, tracking, closing speed calculation,
 * Time-To-Collision (TTC) estimation, and danger/caution risk classification.
 */
class DetectionManager {
  constructor(options = {}) {
    // Perception Range (World units / px, scale: 1 px ~ 0.25 m, 230 px ~ 57.5 m)
    this.perceptionRadius = options.perceptionRadius || 230;
    this.forwardSectorAngle = options.forwardSectorAngle || (140 * Math.PI / 180); // 140 deg forward cone
    this.closeProximityRadius = options.closeProximityRadius || 110;

    // Safety Thresholds
    this.dangerTTC = 2.0;       // <= 2.0s is DANGER
    this.cautionTTC = 4.5;      // 2.0s - 4.5s is CAUTION
    this.criticalDistance = 48; // Critical proximity trigger (px)
    this.cautionDistance = 95;  // Proximity warning trigger (px)

    // Current Perception State
    this.detectedObjects = [];
    this.overallRisk = 'SAFE'; // 'SAFE', 'CAUTION', 'DANGER'
    this.minTTC = null;
    this.dangerCount = 0;
    this.cautionCount = 0;
    this.safeCount = 0;

    // Visualisation / Sensor Options
    this.showSensors = true;
    this.radarAngle = 0;
  }

  /**
   * Continuous perception and risk update loop
   * Evaluates relative positions, relative velocities, closing speeds, and TTC
   */
  update(egoVehicle, staticObstacles, dynamicObjects, dt) {
    if (dt <= 0 || dt > 0.1) dt = 0.016;

    // Rotate radar scan sweep
    this.radarAngle += 4.5 * dt;
    if (this.radarAngle > Math.PI * 2) this.radarAngle -= Math.PI * 2;

    if (!egoVehicle) {
      this.detectedObjects = [];
      this.overallRisk = 'SAFE';
      this.minTTC = null;
      return;
    }

    // Ego Kinematics in World Coordinates
    const egoX = egoVehicle.x;
    const egoY = egoVehicle.y;
    const egoHeading = egoVehicle.heading;
    const egoSpeed = egoVehicle.speed; // px/s (signed)
    const egoVx = egoSpeed * Math.cos(egoHeading);
    const egoVy = egoSpeed * Math.sin(egoHeading);
    const egoRadius = 18;

    const detected = [];
    let minTTC = Infinity;
    let highestThreat = 'SAFE';
    let dCount = 0, cCount = 0, sCount = 0;

    // 1. Gather all candidates (Dynamic traffic entities + Static obstacles)
    const candidates = [];

    // Add dynamic traffic entities
    if (Array.isArray(dynamicObjects)) {
      dynamicObjects.forEach(dyn => {
        candidates.push({
          id: dyn.id,
          type: dyn.type,
          subType: dyn.subType,
          name: dyn.name,
          x: dyn.x,
          y: dyn.y,
          heading: dyn.heading,
          speed: dyn.speed,
          length: dyn.length,
          width: dyn.width,
          radius: dyn.radius || Math.max(dyn.length, dyn.width) / 2,
          isDynamic: true,
          bounds: dyn.getCollisionCorners ? dyn.getCollisionCorners() : null,
          rawObject: dyn
        });
      });
    }

    // Add static obstacles (potholes, debris, parked vehicles)
    if (staticObstacles) {
      // Potholes
      if (Array.isArray(staticObstacles.potholes)) {
        staticObstacles.potholes.forEach(pot => {
          candidates.push({
            id: pot.id,
            type: 'pothole',
            subType: 'road_hazard',
            name: pot.name,
            x: pot.x,
            y: pot.y,
            heading: 0,
            speed: 0,
            length: pot.radius * 2,
            width: pot.radius * 2,
            radius: pot.radius,
            isDynamic: false,
            rawObject: pot
          });
        });
      }

      // Construction Debris
      if (Array.isArray(staticObstacles.debris)) {
        staticObstacles.debris.forEach(deb => {
          candidates.push({
            id: deb.id,
            type: 'debris',
            subType: deb.type,
            name: deb.name || deb.label,
            x: deb.x,
            y: deb.y,
            heading: deb.angle || 0,
            speed: 0,
            length: deb.width,
            width: deb.height,
            radius: Math.max(deb.width, deb.height) / 2,
            isDynamic: false,
            rawObject: deb
          });
        });
      }

      // Parked Objects
      if (Array.isArray(staticObstacles.parkedObjects)) {
        staticObstacles.parkedObjects.forEach(parked => {
          candidates.push({
            id: parked.id,
            type: 'parked_object',
            subType: parked.type,
            name: parked.name,
            x: parked.x,
            y: parked.y,
            heading: parked.angle || 0,
            speed: 0,
            length: parked.width,
            width: parked.height,
            radius: Math.max(parked.width, parked.height) / 2,
            isDynamic: false,
            rawObject: parked
          });
        });
      }
    }

    // 2. Evaluate Each Obstacle for Detection & Risk
    candidates.forEach(cand => {
      // Relative Position Vector (World Space)
      const dx = cand.x - egoX;
      const dy = cand.y - egoY;
      const distance = Math.hypot(dx, dy);

      // Early perception range check
      if (distance > this.perceptionRadius) return;

      // Bearing relative to ego heading [-PI, PI]
      const worldAngleToCand = Math.atan2(dy, dx);
      let bearing = worldAngleToCand - egoHeading;
      while (bearing > Math.PI) bearing -= Math.PI * 2;
      while (bearing < -Math.PI) bearing += Math.PI * 2;

      // Check perception sector (360 close proximity OR within forward sensor cone)
      const inCloseProximity = distance <= this.closeProximityRadius;
      const inForwardSector = Math.abs(bearing) <= (this.forwardSectorAngle / 2);

      if (!inCloseProximity && !inForwardSector) return;

      // Candidate Velocity Vector
      const candVx = cand.speed * Math.cos(cand.heading);
      const candVy = cand.speed * Math.sin(cand.heading);

      // Relative Velocity: V_rel = V_cand - V_ego
      const vRelX = candVx - egoVx;
      const vRelY = candVy - egoVy;
      const relSpeed = Math.hypot(vRelX, vRelY);

      // Unit vector from Ego to Candidate
      const uX = dx / (distance || 1);
      const uY = dy / (distance || 1);

      // Closing Speed: V_closing = - (V_rel . u)
      // Positive = distance is closing/decreasing; Negative = opening/moving away
      const closingSpeed = -(vRelX * uX + vRelY * uY);

      // Edge-to-edge approximate clearance distance
      const clearance = Math.max(0, distance - (egoRadius + cand.radius));

      // Time To Collision (TTC) Estimation
      let ttc = Infinity;
      if (closingSpeed > 1.0) {
        ttc = clearance / closingSpeed;
      }

      // Trajectory and Collision Risk Assessment
      let riskLevel = 'SAFE';

      // Forward corridor alignment: is candidate directly in ego vehicle's travel path?
      const forwardCorridorDist = dx * Math.cos(egoHeading) + dy * Math.sin(egoHeading);
      const lateralCorridorDist = Math.abs(dy * Math.cos(egoHeading) - dx * Math.sin(egoHeading));
      const inEgoTravelCorridor = (forwardCorridorDist > 0 && lateralCorridorDist < (cand.radius + 14));
      const isApproaching = closingSpeed > 2.0;

      // Potholes: Road surface hazards (assessed as CAUTION when approaching in corridor)
      if (cand.type === 'pothole') {
        if (inEgoTravelCorridor && forwardCorridorDist < 120 && egoSpeed > 10) {
          riskLevel = 'CAUTION';
          cCount++;
        } else {
          riskLevel = 'SAFE';
          sCount++;
        }
      }
      // Solid Obstacles & Dynamic Entities
      // Danger Condition: Imminent collision trajectory or critical proximity in corridor
      else if ((ttc <= this.dangerTTC && isApproaching && (inEgoTravelCorridor || inCloseProximity)) ||
               (distance <= this.criticalDistance && isApproaching && (inEgoTravelCorridor || inCloseProximity))) {
        riskLevel = 'DANGER';
        dCount++;
        if (ttc < minTTC) minTTC = ttc;
      }
      // Caution Condition: Potential collision path or proximity alert in travel corridor
      else if (((ttc <= this.cautionTTC && isApproaching) || (distance <= this.cautionDistance && isApproaching) || (!cand.isDynamic && distance < 120 && egoSpeed > 15)) && (inEgoTravelCorridor || inCloseProximity)) {
        riskLevel = 'CAUTION';
        cCount++;
        if (ttc < minTTC) minTTC = ttc;
      }
      // Safe Condition: Perceived without immediate threat
      else {
        riskLevel = 'SAFE';
        sCount++;
      }

      // Track highest overall threat level
      if (riskLevel === 'DANGER') {
        highestThreat = 'DANGER';
      } else if (riskLevel === 'CAUTION' && highestThreat !== 'DANGER') {
        highestThreat = 'CAUTION';
      }

      detected.push({
        id: cand.id,
        name: cand.name,
        type: cand.type,
        subType: cand.subType,
        x: cand.x,
        y: cand.y,
        heading: cand.heading !== undefined ? cand.heading : 0,
        speed: cand.speed !== undefined ? cand.speed : 0,
        dx: dx,
        dy: dy,
        distance: distance,
        clearance: clearance,
        bearingDeg: (bearing * 180 / Math.PI),
        relativeSpeed: relSpeed,
        closingSpeed: closingSpeed,
        ttc: ttc === Infinity ? null : ttc,
        riskLevel: riskLevel,
        isDynamic: cand.isDynamic,
        length: cand.length,
        width: cand.width,
        radius: cand.radius,
        bounds: cand.bounds
      });
    });

    // Sort detected objects: DANGER first, then CAUTION, then closest distance
    detected.sort((a, b) => {
      const priority = { DANGER: 0, CAUTION: 1, SAFE: 2 };
      if (priority[a.riskLevel] !== priority[b.riskLevel]) {
        return priority[a.riskLevel] - priority[b.riskLevel];
      }
      return a.distance - b.distance;
    });

    this.detectedObjects = detected;
    this.overallRisk = highestThreat;
    this.minTTC = minTTC === Infinity ? null : minTTC;
    this.dangerCount = dCount;
    this.cautionCount = cCount;
    this.safeCount = sCount;
  }

  /**
   * Render Perception Radar Field and Object Highlighting in World Space
   */
  render(ctx, egoVehicle) {
    if (!this.showSensors || !egoVehicle) return;

    ctx.save();
    const egoX = egoVehicle.x;
    const egoY = egoVehicle.y;
    const egoHeading = egoVehicle.heading;

    // 1. Draw Radar Perception Field & Range Rings
    this.drawPerceptionField(ctx, egoX, egoY, egoHeading);

    // 2. Highlight Detected Objects with Target Brackets & Threat Badges
    this.drawDetectedObjectHighlights(ctx, egoX, egoY);

    ctx.restore();
  }

  drawPerceptionField(ctx, x, y, heading) {
    ctx.save();
    ctx.translate(x, y);

    const rad = this.perceptionRadius;

    // Concentric Range Rings (20m, 35m, 55m scale)
    const rings = [70, 140, rad];
    rings.forEach((r, idx) => {
      ctx.strokeStyle = idx === rings.length - 1 ? 'rgba(56, 189, 248, 0.35)' : 'rgba(56, 189, 248, 0.15)';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 6]);
      ctx.beginPath();
      ctx.arc(0, 0, r, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);

      // Range labels
      ctx.fillStyle = 'rgba(148, 163, 184, 0.5)';
      ctx.font = '500 9px ui-monospace, SFMono-Regular, monospace';
      ctx.fillText(`${Math.round(r * 0.25)}m`, r - 16, -4);
    });

    // Outer Perception Field Soft Fill
    const grad = ctx.createRadialGradient(0, 0, 10, 0, 0, rad);
    grad.addColorStop(0, 'rgba(6, 182, 212, 0.08)');
    grad.addColorStop(0.7, 'rgba(6, 182, 212, 0.03)');
    grad.addColorStop(1, 'rgba(6, 182, 212, 0)');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(0, 0, rad, 0, Math.PI * 2);
    ctx.fill();

    // Forward Perception Sector FOV Wedge (140 deg)
    const halfAngle = this.forwardSectorAngle / 2;
    ctx.fillStyle = 'rgba(56, 189, 248, 0.06)';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, rad, heading - halfAngle, heading + halfAngle);
    ctx.closePath();
    ctx.fill();

    // Sector boundary rays
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.25)';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(Math.cos(heading - halfAngle) * rad, Math.sin(heading - halfAngle) * rad);
    ctx.moveTo(0, 0);
    ctx.lineTo(Math.cos(heading + halfAngle) * rad, Math.sin(heading + halfAngle) * rad);
    ctx.stroke();

    // Spinning Radar Sweep Line
    ctx.strokeStyle = 'rgba(6, 182, 212, 0.55)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(Math.cos(this.radarAngle) * rad, Math.sin(this.radarAngle) * rad);
    ctx.stroke();

    ctx.restore();
  }

  drawDetectedObjectHighlights(ctx, egoX, egoY) {
    this.detectedObjects.forEach(obj => {
      ctx.save();
      ctx.translate(obj.x, obj.y);

      const isDanger = obj.riskLevel === 'DANGER';
      const isCaution = obj.riskLevel === 'CAUTION';

      // Colors by threat
      let color = '#06b6d4';     // SAFE (Cyan)
      let glowColor = 'rgba(6, 182, 212, 0.4)';
      if (isDanger) {
        color = '#ef4444';       // DANGER (Neon Red)
        glowColor = 'rgba(239, 68, 68, 0.8)';
      } else if (isCaution) {
        color = '#f59e0b';       // CAUTION (Amber)
        glowColor = 'rgba(245, 158, 11, 0.7)';
      }

      // 1. Connection / Tracking Vector to Ego for Caution/Danger
      if (isDanger || isCaution) {
        ctx.restore();
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = isDanger ? 2 : 1.2;
        if (!isDanger) ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(egoX, egoY);
        ctx.lineTo(obj.x, obj.y);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
        ctx.save();
        ctx.translate(obj.x, obj.y);
      }

      // 2. Corner Bracket Target Reticle [  ]
      const boxW = Math.max(obj.length || 30, obj.radius * 2 || 30) + 10;
      const boxH = Math.max(obj.width || 20, obj.radius * 2 || 20) + 10;
      const halfW = boxW / 2;
      const halfH = boxH / 2;
      const arm = 6;

      ctx.strokeStyle = color;
      ctx.lineWidth = isDanger ? 2.5 : 1.5;
      if (isDanger) {
        ctx.shadowColor = glowColor;
        ctx.shadowBlur = 8;
      }

      // Top-Left
      ctx.beginPath();
      ctx.moveTo(-halfW, -halfH + arm);
      ctx.lineTo(-halfW, -halfH);
      ctx.lineTo(-halfW + arm, -halfH);
      // Top-Right
      ctx.moveTo(halfW - arm, -halfH);
      ctx.lineTo(halfW, -halfH);
      ctx.lineTo(halfW, -halfH + arm);
      // Bottom-Right
      ctx.moveTo(halfW, halfH - arm);
      ctx.lineTo(halfW, halfH);
      ctx.lineTo(halfW - arm, halfH);
      // Bottom-Left
      ctx.moveTo(-halfW + arm, halfH);
      ctx.lineTo(-halfW, halfH);
      ctx.lineTo(-halfW, halfH - arm);
      ctx.stroke();
      ctx.shadowBlur = 0;

      // 3. Floating Telemetry Callout Badge
      this.drawObjectBadge(ctx, 0, -halfH - 8, obj, color);

      ctx.restore();
    });
  }

  drawObjectBadge(ctx, x, y, obj, color) {
    let text = `${obj.type.toUpperCase()} | ${Math.round(obj.distance)}px`;
    if (obj.riskLevel === 'DANGER') {
      const ttcText = obj.ttc && obj.ttc < 10 ? `${obj.ttc.toFixed(1)}s` : '--';
      text = `! DANGER | TTC: ${ttcText}`;
    } else if (obj.riskLevel === 'CAUTION') {
      const ttcText = obj.ttc && obj.ttc < 10 ? `${obj.ttc.toFixed(1)}s` : `${Math.round(obj.distance)}px`;
      text = `CAUTION | ${ttcText}`;
    }

    ctx.font = 'bold 9px ui-monospace, SFMono-Regular, monospace';
    const metrics = ctx.measureText(text);
    const badgeW = metrics.width + 10;
    const badgeH = 14;

    // Badge Background
    ctx.fillStyle = obj.riskLevel === 'DANGER' ? '#ef4444' : (obj.riskLevel === 'CAUTION' ? '#f59e0b' : 'rgba(15, 23, 42, 0.85)');
    ctx.beginPath();
    ctx.roundRect(x - badgeW / 2, y - badgeH, badgeW, badgeH, 3);
    ctx.fill();

    // Badge Border
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.stroke();

    // Badge Text
    ctx.fillStyle = obj.riskLevel === 'SAFE' ? '#e2e8f0' : '#0f172a';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, x, y - badgeH / 2);
  }

  toggleSensors(state) {
    this.showSensors = state !== undefined ? state : !this.showSensors;
    return this.showSensors;
  }

  getPerceptionData() {
    return {
      detectedObjects: this.detectedObjects,
      overallRisk: this.overallRisk,
      minTTC: this.minTTC,
      detectedCount: this.detectedObjects.length,
      dangerCount: this.dangerCount,
      cautionCount: this.cautionCount,
      safeCount: this.safeCount,
      perceptionRadius: this.perceptionRadius
    };
  }

  getRiskAssessment() {
    return {
      overallRisk: this.overallRisk,
      minTTC: this.minTTC,
      threatObjects: this.detectedObjects.filter(o => o.riskLevel !== 'SAFE')
    };
  }
}

/* ============================================================================
   8. MODULE 5: ADAPTIVE PATH PLANNING SYSTEM
   ============================================================================ */

/**
 * Adaptive Local Path Planner
 * Evaluates candidate rollout trajectories from Ego Vehicle pose,
 * scores each trajectory against dynamic threats, static obstacles, road bounds,
 * destination progress, and steering smoothness, selecting the optimal safe path.
 */
class PathPlanner {
  constructor(options = {}) {
    // Lookahead and Rollout Parameters
    this.horizonDistance = options.horizonDistance || 175; // px (approx 43.7 m)
    this.numWaypoints = options.numWaypoints || 18;        // steps along trajectory
    this.wheelbase = 26;                                   // Ego vehicle wheelbase (px)

    // Steering definitions for 7 candidate actions
    this.candidateDefinitions = [
      { id: 'HARD_LEFT',    label: 'Hard Left',     steerAngle: -0.55 },
      { id: 'MOD_LEFT',     label: 'Mod Left',      steerAngle: -0.32 },
      { id: 'SLIGHT_LEFT',  label: 'Slight Left',   steerAngle: -0.15 },
      { id: 'STRAIGHT',     label: 'Straight',      steerAngle:  0.00 },
      { id: 'SLIGHT_RIGHT', label: 'Slight Right',  steerAngle:  0.15 },
      { id: 'MOD_RIGHT',    label: 'Mod Right',     steerAngle:  0.32 },
      { id: 'HARD_RIGHT',   label: 'Hard Right',    steerAngle:  0.55 }
    ];

    // Multi-objective Cost Weights
    this.weights = {
      collision: 5000,    // Immediate collision penalty
      clearance: 700,     // Obstacle clearance weight
      threatScale: 2.4,   // Extra penalty multiplier for DANGER/CAUTION perceived objects
      roadBoundary: 2500, // Off-road deviation penalty
      goalProgress: 1.6,  // Distance to target waypoint weight
      goalHeading: 85,    // Alignment with target vector weight
      steerMagnitude: 70, // Effort penalty for large steering
      steerRate: 35       // Continuity penalty from current steer angle
    };

    // Runtime State
    this.candidatePaths = [];
    this.selectedPath = null;
    this.plannerState = 'OPTIMAL'; // 'OPTIMAL', 'AVOIDANCE', 'EMERGENCY_STOP'
    this.showPaths = true;
    this.chevronAnimPhase = 0;
  }

  /**
   * Main planning cycle: generates trajectories, scores them, and selects the optimal path
   */
  update(egoVehicle, environmentData, perceptionData, dt) {
    if (dt <= 0 || dt > 0.1) dt = 0.016;

    // Advance directional chevron animation
    this.chevronAnimPhase = (this.chevronAnimPhase + 3.0 * dt) % 1.0;

    if (!egoVehicle) {
      this.candidatePaths = [];
      this.selectedPath = null;
      this.plannerState = 'IDLE';
      return;
    }

    const egoX = egoVehicle.x;
    const egoY = egoVehicle.y;
    const egoHeading = egoVehicle.heading;
    const currentSteer = egoVehicle.steeringAngle || 0;
    const destination = (environmentData && environmentData.destination) ? environmentData.destination : { x: 1650, y: 560 };

    // Compile dynamic obstacles from Module 4
    const obstacles = this.gatherObstacles(environmentData, perceptionData);

    // 1. Generate and Score each candidate trajectory
    const evaluatedPaths = this.candidateDefinitions.map(def => {
      const path = this.generateRollout(egoX, egoY, egoHeading, def.steerAngle, def.id, def.label);
      this.scoreTrajectory(path, obstacles, destination, currentSteer, environmentData, egoX, egoY);
      return path;
    });

    // 2. Select Optimal Path
    const feasiblePaths = evaluatedPaths.filter(p => p.status === 'FEASIBLE');

    let best = null;
    let state = 'OPTIMAL';

    if (feasiblePaths.length > 0) {
      // Find lowest cost feasible path
      feasiblePaths.sort((a, b) => a.totalCost - b.totalCost);
      best = feasiblePaths[0];

      // Check if the best path is actively avoiding an obstacle or pothole
      if (best.id !== 'STRAIGHT' && best.obstacleCost > 40) {
        state = 'AVOIDANCE';
      } else {
        state = 'OPTIMAL';
      }
    } else {
      // Emergency Fallback: If all paths have collision/off-road, choose path with farthest collision point
      evaluatedPaths.sort((a, b) => {
        if (a.collisionDistance !== b.collisionDistance) {
          return b.collisionDistance - a.collisionDistance;
        }
        return a.totalCost - b.totalCost;
      });
      best = evaluatedPaths[0];
      state = 'EMERGENCY_STOP';
    }

    this.candidatePaths = evaluatedPaths;
    this.selectedPath = best;
    this.plannerState = state;
  }

  /**
   * Generates a kinematically smooth bicycle rollout trajectory
   */
  generateRollout(startX, startY, startHeading, steerAngle, id, label) {
    const waypoints = [];
    const stepSize = this.horizonDistance / this.numWaypoints;
    const curvature = Math.tan(steerAngle) / this.wheelbase;

    let x = startX;
    let y = startY;
    let heading = startHeading;
    let dist = 0;

    waypoints.push({ x, y, heading, s: dist });

    for (let i = 1; i <= this.numWaypoints; i++) {
      heading += curvature * stepSize;
      x += stepSize * Math.cos(heading);
      y += stepSize * Math.sin(heading);
      dist += stepSize;
      waypoints.push({ x, y, heading, s: dist });
    }

    return {
      id,
      label,
      steerAngle,
      waypoints,
      endPoint: waypoints[waypoints.length - 1],
      totalCost: 0,
      collisionCost: 0,
      obstacleCost: 0,
      roadCost: 0,
      goalCost: 0,
      steerCost: 0,
      status: 'FEASIBLE', // 'FEASIBLE' | 'COLLISION' | 'OFF_ROAD'
      hasCollision: false,
      collisionIndex: -1,
      collisionDistance: Infinity,
      collisionObstacle: null,
      minClearance: Infinity
    };
  }

  /**
   * Evaluates and scores a trajectory against all objective criteria:
   * Solid obstacle collisions, dynamic threats, potholes, destination progress, road bounds, steering effort
   */
  scoreTrajectory(path, obstacles, destination, currentSteer, envData, egoX, egoY) {
    const egoRadius = 14; // Vehicle clearance radius (px)
    let totalObstacleCost = 0;
    let potholePenalty = 0;
    let minClearance = Infinity;
    let hasCollision = false;
    let collisionIdx = -1;
    let collisionDist = Infinity;
    let collisionObs = null;
    let roadCost = 0;
    let isOffRoad = false;

    const startDistToGoal = Math.hypot(destination.x - egoX, destination.y - egoY);
    const endWp = path.endPoint;
    const endDistToGoal = Math.hypot(destination.x - endWp.x, destination.y - endWp.y);

    // --- 1. Evaluate Waypoints for Solid Obstacles, Dynamic Vehicles, Potholes & Road Bounds ---
    for (let i = 0; i < path.waypoints.length; i++) {
      const wp = path.waypoints[i];

      // A. Solid Obstacle Collision Check (Buildings, barricades, parked cars, utility poles, trees, traffic signal)
      const solidCheck = StaticCollisionSystem.checkSolidCollision(wp.x, wp.y, egoRadius);
      if (solidCheck.collided) {
        hasCollision = true;
        if (collisionIdx === -1) {
          collisionIdx = i;
          collisionDist = wp.s;
          collisionObs = solidCheck.obstacle;
        }
        break;
      }

      // B. Dynamic Objects Collision & Clearance Check
      for (let j = 0; j < obstacles.length; j++) {
        const obs = obstacles[j];
        if (obs.type === 'pothole') continue; // Potholes are scored in Step C as road surface hazards

        const dist = Math.hypot(wp.x - obs.x, wp.y - obs.y);
        const clearance = dist - (egoRadius + (obs.radius || 12));

        if (clearance < minClearance) {
          minClearance = clearance;
        }

        // Direct Collision Check (clearance <= 2px)
        if (clearance <= 2) {
          hasCollision = true;
          if (collisionIdx === -1) {
            collisionIdx = i;
            collisionDist = wp.s;
            collisionObs = obs;
          }
          break;
        }

        // Proximity Soft Penalty (clearance < 50px)
        if (clearance < 50) {
          const threatMult = obs.riskLevel === 'DANGER' ? 2.5 : (obs.riskLevel === 'CAUTION' ? 1.6 : 1.0);
          const penalty = threatMult * (this.weights.clearance / (clearance + 4));
          totalObstacleCost += penalty;
        }
      }

      if (hasCollision) break;

      // C. Pothole Road Surface Hazard Check
      const potholeCheck = StaticCollisionSystem.checkPothole(wp.x, wp.y, egoRadius * 0.8);
      if (potholeCheck.inPothole) {
        potholePenalty += 2000; // Heavy penalty: planner strongly prefers clean tarmac around potholes
      }

      // D. Drivable Road Envelope Check
      const onMainRoad = (wp.y >= 465 && wp.y <= 655) && (wp.x >= 0 && wp.x <= 1800);
      const onSideRoad = (wp.x >= 860 && wp.x <= 980) && (wp.y >= 0 && wp.y <= 560);

      if (!onMainRoad && !onSideRoad) {
        isOffRoad = true;
        roadCost += 5000;
      } else {
        // Preferred lane centering: gently penalize running too close to road shoulder
        if (onMainRoad) {
          const distFromCenter = Math.abs(wp.y - 560);
          if (distFromCenter > 70) {
            roadCost += (distFromCenter - 70) * 3.0;
          }
        }
      }
    }

    // --- 2. Destination Progress & Alignment (STRONG Goal-Aware Scoring) ---
    const deltaGoal = endDistToGoal - startDistToGoal;
    let goalProgressCost = 0;

    if (deltaGoal < 0) {
      // Moving closer to destination -> Significant reward (negative cost bonus)
      goalProgressCost = deltaGoal * 4.2;
    } else {
      // Moving away or sideways -> Heavy penalty
      goalProgressCost = 1400 + deltaGoal * 6.5;
    }

    // Alignment angle toward destination
    const targetHeading = Math.atan2(destination.y - endWp.y, destination.x - endWp.x);
    let headingDiff = targetHeading - endWp.heading;
    while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
    while (headingDiff < -Math.PI) headingDiff += Math.PI * 2;
    const goalHeadingCost = Math.abs(headingDiff) * 200;

    const totalGoalCost = goalProgressCost + goalHeadingCost;

    // --- 3. Steering Smoothness & Curvature Effort ---
    const steerEffortCost = Math.pow(Math.abs(path.steerAngle), 1.8) * 85;
    const steerRateCost = Math.abs(path.steerAngle - currentSteer) * 35;
    const totalSteerCost = steerEffortCost + steerRateCost;

    // --- 4. Final Status & Total Cost ---
    const collisionCost = hasCollision ? 10000 : 0;
    path.hasCollision = hasCollision;
    path.collisionIndex = collisionIdx;
    path.collisionDistance = collisionDist;
    path.collisionObstacle = collisionObs;
    path.minClearance = minClearance;
    path.collisionCost = collisionCost;
    path.obstacleCost = totalObstacleCost + potholePenalty;
    path.roadCost = roadCost;
    path.goalCost = totalGoalCost;
    path.steerCost = totalSteerCost;

    if (hasCollision) {
      path.status = 'COLLISION';
    } else if (isOffRoad) {
      path.status = 'OFF_ROAD';
    } else {
      path.status = 'FEASIBLE';
    }

    path.totalCost = collisionCost + path.obstacleCost + roadCost + totalGoalCost + totalSteerCost;
  }

  /**
   * Gathers all dynamic traffic obstacles from perception data
   */
  gatherObstacles(envData, perceptionData) {
    const list = [];

    // 1. Detected objects from Perception Data (includes dynamic entities)
    if (perceptionData && Array.isArray(perceptionData.detectedObjects)) {
      perceptionData.detectedObjects.forEach(det => {
        list.push({
          id: det.id,
          name: det.name || det.type,
          type: det.type,
          x: det.x,
          y: det.y,
          radius: det.radius || 15,
          riskLevel: det.riskLevel || 'SAFE'
        });
      });
    }

    return list;
  }

  /**
   * Render Candidate Paths and the Selected Optimal Path in World Coordinates
   */
  render(ctx) {
    if (!this.showPaths || this.candidatePaths.length === 0) return;

    ctx.save();

    // 1. Render Unselected Candidate Paths (Subtle dashed lines)
    this.candidatePaths.forEach(path => {
      if (path === this.selectedPath) return; // Drawn separately in step 2

      ctx.save();
      const isInvalid = path.status !== 'FEASIBLE';

      if (isInvalid) {
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.45)';
        ctx.lineWidth = 1.2;
        ctx.setLineDash([3, 4]);
      } else {
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
      }

      ctx.beginPath();
      path.waypoints.forEach((wp, idx) => {
        if (idx === 0) ctx.moveTo(wp.x, wp.y);
        else ctx.lineTo(wp.x, wp.y);
      });
      ctx.stroke();

      // If collision, draw small (X) hazard marker at collision waypoint
      if (isInvalid && path.collisionIndex >= 0) {
        const cWp = path.waypoints[path.collisionIndex];
        ctx.setLineDash([]);
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 2;
        const s = 4;
        ctx.beginPath();
        ctx.moveTo(cWp.x - s, cWp.y - s);
        ctx.lineTo(cWp.x + s, cWp.y + s);
        ctx.moveTo(cWp.x + s, cWp.y - s);
        ctx.lineTo(cWp.x - s, cWp.y + s);
        ctx.stroke();
      }

      ctx.restore();
    });

    // 2. Render Selected / Optimal Path (Bold Glowing Ribbon + Chevrons)
    if (this.selectedPath) {
      const best = this.selectedPath;
      const isEmergency = this.plannerState === 'EMERGENCY_STOP';
      const pathColor = isEmergency ? '#ef4444' : (this.plannerState === 'AVOIDANCE' ? '#f59e0b' : '#10b981');
      const glowColor = isEmergency ? 'rgba(239, 68, 68, 0.6)' : (this.plannerState === 'AVOIDANCE' ? 'rgba(245, 158, 11, 0.6)' : 'rgba(16, 185, 129, 0.6)');

      // A. Safety Clearance Swath (Corridor Ribbon)
      ctx.save();
      ctx.strokeStyle = isEmergency ? 'rgba(239, 68, 68, 0.12)' : 'rgba(16, 185, 129, 0.12)';
      ctx.lineWidth = 28;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.beginPath();
      best.waypoints.forEach((wp, idx) => {
        if (idx === 0) ctx.moveTo(wp.x, wp.y);
        else ctx.lineTo(wp.x, wp.y);
      });
      ctx.stroke();
      ctx.restore();

      // B. Centerline Path with Neon Glow
      ctx.save();
      ctx.strokeStyle = pathColor;
      ctx.lineWidth = 3.5;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.shadowColor = glowColor;
      ctx.shadowBlur = 10;
      ctx.beginPath();
      best.waypoints.forEach((wp, idx) => {
        if (idx === 0) ctx.moveTo(wp.x, wp.y);
        else ctx.lineTo(wp.x, wp.y);
      });
      ctx.stroke();
      ctx.restore();

      // C. Directional Animated Chevrons Along Selected Path
      ctx.save();
      const numChevrons = 4;
      for (let c = 1; c <= numChevrons; c++) {
        const progress = (c / (numChevrons + 1) + this.chevronAnimPhase / (numChevrons + 1)) % 1.0;
        const targetIndex = Math.min(Math.floor(progress * (best.waypoints.length - 1)), best.waypoints.length - 2);
        const wp = best.waypoints[targetIndex];
        const nextWp = best.waypoints[targetIndex + 1];
        const angle = Math.atan2(nextWp.y - wp.y, nextWp.x - wp.x);

        ctx.save();
        ctx.translate(wp.x, wp.y);
        ctx.rotate(angle);
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-4, -4);
        ctx.lineTo(2, 0);
        ctx.lineTo(-4, 4);
        ctx.stroke();
        ctx.restore();
      }
      ctx.restore();

      // D. Lookahead Target Endpoint Marker
      ctx.save();
      const endWp = best.endPoint;
      ctx.translate(endWp.x, endWp.y);
      ctx.fillStyle = pathColor;
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.shadowColor = glowColor;
      ctx.shadowBlur = 8;
      ctx.beginPath();
      ctx.arc(0, 0, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Outer pulse ring
      ctx.strokeStyle = pathColor;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(0, 0, 8 + Math.sin(this.chevronAnimPhase * Math.PI * 2) * 2, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }

    ctx.restore();
  }

  togglePaths(state) {
    this.showPaths = state !== undefined ? state : !this.showPaths;
    return this.showPaths;
  }
}

/* ============================================================================
   9. MODULE 6A: CONTEXT-AWARE DECISION MAKING SYSTEM
   ============================================================================ */

/**
 * Context-Aware Decision Manager
 * Evaluates real-time perception, collision risk, traffic stream dynamics,
 * candidate path feasibility, road geometry, and obstacle blockages to determine
 * the high-level tactical behavior decision for the Ego Vehicle.
 *
 * Possible Decisions:
 *  - 'GO'      : Path is clear, safe speed, progression toward destination.
 *  - 'SLOW'    : Developing risk, proximity hazard, approaching pothole/junction, or avoidance steering.
 *  - 'STOP'    : Imminent collision threat, emergency brake condition, stationary obstacle ahead.
 *  - 'WAIT'    : Traffic gridlock in front, waiting for bottleneck/intersection to clear.
 *  - 'YIELD'   : Cross traffic incursion, yielding to vehicles on conflicting trajectories.
 *  - 'REVERSE' : Trapped/blocked in front with safe clear rear space to unblock.
 *  - 'REPLAN'  : Selected path infeasible/blocked, alternative trajectory computation required.
 */
class DecisionManager {
  constructor(options = {}) {
    this.currentDecision = 'GO';
    this.reason = 'Clear path toward destination';
    this.decisionHistory = [];
    this.historyLength = 10;
    
    // Configurable thresholds for context evaluation
    this.config = {
      imminentTTC: 1.8,            // Seconds (TTC <= 1.8s triggers STOP)
      cautionTTC: 4.0,             // Seconds (TTC <= 4.0s triggers SLOW)
      criticalDistance: 32,        // Pixels (Immediate threat proximity)
      corridorLookahead: 120,      // Pixels (Corridor forward zone)
      rearSafetyBuffer: 65,        // Pixels (Clearance behind ego for safe REVERSE)
      intersectionZone: {          // Intersection bounds
        minX: 780,
        maxX: 1040,
        minY: 420,
        maxY: 650
      }
    };
    
    // Telemetry state
    this.state = {
      crossTrafficDetected: false,
      frontBlocked: false,
      rearClear: true,
      pathFeasible: true,
      imminentDanger: false,
      developingRisk: false,
      approachingHazard: false
    };
  }

  /**
   * Main evaluation cycle: computes the single primary decision for the current frame
   * @param {EgoVehicle} egoVehicle
   * @param {Object} environmentData
   * @param {Object} perceptionData
   * @param {Object} pathPlannerData
   * @param {TrafficManager} trafficManager
   * @param {number} dt
   */
  update(egoVehicle, environmentData, perceptionData, pathPlannerData, trafficManager, dt) {
    if (!egoVehicle) {
      this.currentDecision = 'WAIT';
      this.reason = 'Ego vehicle not initialized';
      return;
    }

    const egoX = egoVehicle.x;
    const egoY = egoVehicle.y;
    const egoHeading = egoVehicle.heading;
    const egoSpeed = egoVehicle.speed || 0;
    const isEgoCollided = egoVehicle.isCollided || false;
    const isEgoInPothole = egoVehicle.inPothole || false;

    const detectedObjects = (perceptionData && Array.isArray(perceptionData.detectedObjects)) ? perceptionData.detectedObjects : [];
    const overallRisk = perceptionData ? perceptionData.overallRisk : 'SAFE';
    const minTTC = (perceptionData && perceptionData.minTTC !== null) ? perceptionData.minTTC : Infinity;

    const selectedPath = pathPlannerData ? pathPlannerData.selectedPath : null;
    const plannerState = pathPlannerData ? pathPlannerData.plannerState : 'OPTIMAL';
    const candidatePaths = pathPlannerData ? (pathPlannerData.candidatePaths || []) : [];

    const dynamicEntities = trafficManager ? trafficManager.getEntities() : [];

    // --- 1. Analyze Environmental Context & Traffic States ---
    const context = this.analyzeContext({
      egoX,
      egoY,
      egoHeading,
      egoSpeed,
      isEgoCollided,
      isEgoInPothole,
      detectedObjects,
      overallRisk,
      minTTC,
      selectedPath,
      plannerState,
      candidatePaths,
      dynamicEntities,
      environmentData
    });

    this.state = context;

    // --- 2. Execute Deterministic Decision Priority Pipeline ---
    const decisionResult = this.evaluatePriority(context);

    this.currentDecision = decisionResult.decision;
    this.reason = decisionResult.reason;

    // Record telemetry history
    this.decisionHistory.push({
      decision: this.currentDecision,
      reason: this.reason,
      timestamp: performance.now()
    });
    if (this.decisionHistory.length > this.historyLength) {
      this.decisionHistory.shift();
    }
  }

  /**
   * Analyze all sensory and spatial factors
   */
  analyzeContext(data) {
    const {
      egoX, egoY, egoHeading, egoSpeed, isEgoCollided, isEgoInPothole,
      detectedObjects, overallRisk, minTTC, selectedPath, plannerState,
      candidatePaths, dynamicEntities, environmentData
    } = data;

    const cosH = Math.cos(egoHeading);
    const sinH = Math.sin(egoHeading);

    // A. Check Imminent Danger / Emergency Stop Condition
    let imminentDanger = false;
    let dangerReason = '';

    if (isEgoCollided) {
      imminentDanger = true;
      dangerReason = 'Obstacle collision';
    } else if (overallRisk === 'DANGER' && minTTC <= this.config.imminentTTC) {
      imminentDanger = true;
      dangerReason = `Imminent collision threat (TTC ${minTTC.toFixed(1)}s)`;
    } else {
      // Check for objects critically close in forward trajectory corridor
      for (const obj of detectedObjects) {
        const dx = obj.x - egoX;
        const dy = obj.y - egoY;
        const forwardDist = dx * cosH + dy * sinH;
        const lateralDist = Math.abs(dy * cosH - dx * sinH);

        if (forwardDist > 0 && forwardDist < this.config.criticalDistance && lateralDist < 20) {
          imminentDanger = true;
          dangerReason = `Obstacle ahead (${obj.type || 'hazard'} ${Math.round(forwardDist)}px)`;
          break;
        }
      }
    }

    // B. Check Selected Path Feasibility
    let pathFeasible = true;
    let replanReason = '';

    if (!selectedPath || selectedPath.status === 'COLLISION' || selectedPath.status === 'OFF_ROAD' || plannerState === 'EMERGENCY_STOP') {
      pathFeasible = false;
      replanReason = selectedPath ? `Path blocked (${selectedPath.status})` : 'No safe path available';
    }

    // C. Check Front Traffic Blockage / Bottleneck
    let frontBlocked = false;
    let frontBlockReason = '';
    let slowFrontVehicles = 0;

    for (const obj of detectedObjects) {
      const dx = obj.x - egoX;
      const dy = obj.y - egoY;
      const forwardDist = dx * cosH + dy * sinH;
      const lateralDist = Math.abs(dy * cosH - dx * sinH);

      if (forwardDist > 5 && forwardDist < 65 && lateralDist < 26) {
        if ((obj.speed !== undefined && obj.speed < 15) || !obj.isDynamic || obj.type === 'debris' || obj.type === 'parked_object') {
          slowFrontVehicles++;
        }
      }
    }

    if (slowFrontVehicles >= 1 && (imminentDanger || !pathFeasible || egoSpeed < 10)) {
      frontBlocked = true;
      frontBlockReason = 'Traffic blocked ahead';
    }

    // D. Check Rear Clearance for Safe Reverse Movement
    let rearClear = true;
    let rearBlocker = null;

    // Check all dynamic entities
    for (const dyn of dynamicEntities) {
      const dx = dyn.x - egoX;
      const dy = dyn.y - egoY;
      const forwardDist = dx * cosH + dy * sinH;
      const lateralDist = Math.abs(dy * cosH - dx * sinH);

      // In rear zone directly behind ego
      if (forwardDist < -4 && forwardDist > -this.config.rearSafetyBuffer && lateralDist < 26) {
        rearClear = false;
        rearBlocker = dyn;
        break;
      }
    }

    // Check solid static collision obstacles in rear
    if (rearClear && window.StaticCollisionSystem) {
      const rearCheckX = egoX - cosH * 35;
      const rearCheckY = egoY - sinH * 35;
      const solidRear = window.StaticCollisionSystem.checkSolidCollision(rearCheckX, rearCheckY, 16);
      if (solidRear.collided) {
        rearClear = false;
        rearBlocker = solidRear.obstacle;
      }
    }

    // E. Check Cross-Traffic / Intersection Conflict
    let crossTrafficDetected = false;
    let crossTrafficReason = '';

    const inIntersectionZone = (
      egoX >= this.config.intersectionZone.minX - 50 &&
      egoX <= this.config.intersectionZone.maxX &&
      egoY >= this.config.intersectionZone.minY &&
      egoY <= this.config.intersectionZone.maxY
    );

    for (const obj of detectedObjects) {
      if (obj.isDynamic) {
        let headingDiff = Math.abs((obj.heading || 0) - egoHeading);
        while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
        headingDiff = Math.abs(headingDiff);

        const isCrossHeading = headingDiff > (30 * Math.PI / 180) && headingDiff < (150 * Math.PI / 180);
        const dx = obj.x - egoX;
        const dy = obj.y - egoY;
        const dist = Math.hypot(dx, dy);

        if (isCrossHeading && dist < 125) {
          crossTrafficDetected = true;
          crossTrafficReason = `Cross traffic approaching (${obj.type || 'vehicle'})`;
          break;
        }
      }

      // Check pedestrian or animal crossing actively across road corridor
      if ((obj.type === 'pedestrian' || obj.type === 'animal') && obj.distance < 90) {
        const dx = obj.x - egoX;
        const dy = obj.y - egoY;
        const forwardDist = dx * cosH + dy * sinH;
        const lateralDist = Math.abs(dy * cosH - dx * sinH);
        if (forwardDist > 0 && forwardDist < 90 && lateralDist < 40) {
          crossTrafficDetected = true;
          crossTrafficReason = `Crossing ${obj.type} ahead`;
          break;
        }
      }
    }

    // F. Check Developing Risk / Hazard Caution
    let developingRisk = false;
    let cautionReason = '';

    if (overallRisk === 'CAUTION' || (minTTC > this.config.imminentTTC && minTTC <= this.config.cautionTTC)) {
      developingRisk = true;
      cautionReason = `Developing collision risk (TTC ${minTTC < 10 ? minTTC.toFixed(1) + 's' : 'caution'})`;
    } else if (plannerState === 'AVOIDANCE') {
      developingRisk = true;
      cautionReason = 'Executing path avoidance maneuver';
    } else if (isEgoInPothole) {
      developingRisk = true;
      cautionReason = 'Pothole impact - reducing speed';
    } else {
      // Check approaching pothole in lane within lookahead
      if (window.StaticCollisionSystem) {
        for (let step = 15; step <= 70; step += 15) {
          const checkX = egoX + cosH * step;
          const checkY = egoY + sinH * step;
          const potCheck = window.StaticCollisionSystem.checkPothole(checkX, checkY, 12);
          if (potCheck.inPothole) {
            developingRisk = true;
            cautionReason = 'Approaching road pothole';
            break;
          }
        }
      }
    }

    return {
      imminentDanger,
      dangerReason,
      pathFeasible,
      replanReason,
      frontBlocked,
      frontBlockReason,
      rearClear,
      rearBlocker,
      crossTrafficDetected,
      crossTrafficReason,
      developingRisk,
      cautionReason,
      egoSpeed,
      inIntersectionZone
    };
  }

  /**
   * Deterministic Priority Hierarchy Decision Resolution
   */
  evaluatePriority(ctx) {
    // 1. HIGHEST PRIORITY: Imminent Collision Threat -> STOP
    if (ctx.imminentDanger) {
      return {
        decision: 'STOP',
        reason: ctx.dangerReason || 'Imminent collision threat'
      };
    }

    // 2. Trajectory Blockage / Infeasible Path -> REPLAN
    if (!ctx.pathFeasible) {
      return {
        decision: 'REPLAN',
        reason: ctx.replanReason || 'Selected path blocked'
      };
    }

    // 3. Traffic Blockage Resolution -> REVERSE vs WAIT
    if (ctx.frontBlocked) {
      if (ctx.rearClear) {
        return {
          decision: 'REVERSE',
          reason: 'Reversing to clear blockage'
        };
      } else {
        return {
          decision: 'WAIT',
          reason: 'Traffic blocked - rear occupied'
        };
      }
    }

    // 4. Cross Traffic / Intersection Incursion -> YIELD or WAIT
    if (ctx.crossTrafficDetected) {
      if (ctx.inIntersectionZone) {
        return {
          decision: 'WAIT',
          reason: ctx.crossTrafficReason || 'Waiting at busy intersection'
        };
      } else {
        return {
          decision: 'YIELD',
          reason: ctx.crossTrafficReason || 'Yielding to cross traffic'
        };
      }
    }

    // 5. Developing Risk / Hazard Caution / Avoidance Maneuver -> SLOW
    if (ctx.developingRisk) {
      return {
        decision: 'SLOW',
        reason: ctx.cautionReason || 'Developing risk - reducing speed'
      };
    }

    // 6. DEFAULT: Normal Clear Driving -> GO
    return {
      decision: 'GO',
      reason: 'Clear path toward destination'
    };
  }

  /**
   * Accessor for Decision Telemetry
   */
  getDecisionData() {
    return {
      decision: this.currentDecision,
      reason: this.reason,
      state: this.state,
      history: this.decisionHistory
    };
  }

  getDecision() {
    return this.currentDecision;
  }
}

/* ============================================================================
   10. MODULE 6B: AUTONOMOUS EGO VEHICLE CONTROL SYSTEM
   ============================================================================ */

/**
 * Autonomous Vehicle Controller
 * Closes the control loop between Perception, Collision Risk, Adaptive Path Planning,
 * Context-Aware Decision Making, and Physical Vehicle Actuation.
 *
 * Operational Modes:
 *  - MANUAL     : Teleoperated with WASD / Arrow keys / Spacebar.
 *  - AUTONOMOUS : Actuation driven deterministically by DecisionManager & PathPlanner.
 *
 * Safety Priority Hierarchy:
 *  DANGER / STOP -> WAIT / YIELD -> REPLAN -> SLOW -> GO
 */
class AutonomousVehicleController {
  constructor(options = {}) {
    this.isAutonomous = false;
    this.controlState = 'MANUAL'; // 'MANUAL', 'CRUISING', 'AVOIDING', 'BRAKING', 'STOPPED', 'WAITING', 'YIELDING', 'REVERSING', 'REPLANNING', 'ARRIVED'
    this.targetSpeed = 0;
    this.targetHeading = 0;
    this.hasArrived = false;
    this.arrivalDistance = 45; // px

    // Actuation parameters
    this.config = {
      cruiseSpeed: 95,          // px/s (~24 km/h)
      cautionSpeed: 42,         // px/s (~10 km/h)
      replanSpeed: 25,          // px/s (~6 km/h)
      reverseSpeed: -26,        // px/s (controlled backward speed)
      lookaheadWaypointIdx: 3,  // Lookahead point index along 18-point trajectory spline (~30-40px)
      steerGain: 2.2,           // Proportional lateral tracking gain
      steerRateLimit: 4.5,      // rad/s (smooth steering rate limit)
      speedDecelGain: 25,       // Longitudinal braking sensitivity
      speedAccelGain: 35,       // Longitudinal throttle sensitivity
      rearSafetyBuffer: 60      // px safety buffer required for reversing
    };
  }

  /**
   * Toggle between Manual and Autonomous driving modes
   * @param {boolean} [forceState]
   * @param {EgoVehicle} [egoVehicle]
   * @returns {boolean} Current autonomous mode state
   */
  toggleMode(forceState, egoVehicle) {
    this.isAutonomous = (forceState !== undefined) ? forceState : !this.isAutonomous;
    
    if (!this.isAutonomous) {
      this.controlState = 'MANUAL';
      if (egoVehicle) {
        egoVehicle.inputs.throttle = 0;
        egoVehicle.inputs.brake = 0;
        egoVehicle.inputs.steer = 0;
        egoVehicle.inputs.handbrake = false;
      }
    } else {
      this.controlState = 'CRUISING';
      this.hasArrived = false;
    }

    console.log(`[AutonomousController] Mode: ${this.isAutonomous ? 'AUTONOMOUS' : 'MANUAL'}`);
    return this.isAutonomous;
  }

  /**
   * Main autonomous actuation step
   * @param {EgoVehicle} egoVehicle
   * @param {Object} destination
   * @param {Object} decisionData
   * @param {Object} pathPlannerData
   * @param {Object} perceptionData
   * @param {TrafficManager} trafficManager
   * @param {number} dt
   */
  update(egoVehicle, destination, decisionData, pathPlannerData, perceptionData, trafficManager, dt) {
    if (!egoVehicle) return;

    // 1. If in Manual Mode: Leave ego inputs to manual teleop
    if (!this.isAutonomous) {
      this.controlState = 'MANUAL';
      return;
    }

    const egoX = egoVehicle.x;
    const egoY = egoVehicle.y;
    const egoHeading = egoVehicle.heading;
    const currentSpeed = egoVehicle.speed || 0;

    // 2. Check Destination Arrival (Destination Goal at x: 1650, y: 560)
    const destX = destination ? destination.x : 1650;
    const destY = destination ? destination.y : 560;
    const distToGoal = Math.hypot(destX - egoX, destY - egoY);

    if (distToGoal <= this.arrivalDistance || this.hasArrived) {
      this.hasArrived = true;
      this.controlState = 'ARRIVED';
      this.targetSpeed = 0;
      egoVehicle.inputs.throttle = 0;
      egoVehicle.inputs.brake = 1.0;
      egoVehicle.inputs.steer = 0;
      egoVehicle.inputs.handbrake = (Math.abs(currentSpeed) < 1.0);
      return;
    }

    // 3. Extract Decision and Trajectory
    const decision = decisionData ? decisionData.decision : 'GO';
    const selectedPath = pathPlannerData ? pathPlannerData.selectedPath : null;
    const plannerState = pathPlannerData ? pathPlannerData.plannerState : 'OPTIMAL';

    // 4. Execute Safety Priority Mapping: Decision -> Target Speed & Control State
    switch (decision) {
      case 'STOP':
        this.controlState = 'STOPPED';
        this.targetSpeed = 0;
        egoVehicle.inputs.throttle = 0;
        egoVehicle.inputs.brake = 1.0;
        egoVehicle.inputs.steer = 0;
        return;

      case 'WAIT':
        this.controlState = 'WAITING';
        this.targetSpeed = 0;
        egoVehicle.inputs.throttle = 0;
        egoVehicle.inputs.brake = 1.0;
        egoVehicle.inputs.steer = 0;
        return;

      case 'YIELD':
        this.controlState = 'YIELDING';
        this.targetSpeed = 0;
        egoVehicle.inputs.throttle = 0;
        egoVehicle.inputs.brake = 0.8;
        return;

      case 'REVERSE': {
        // Verify rear safety buffer in real-time before reversing
        const rearSafe = this.checkRearSafety(egoVehicle, perceptionData, trafficManager);
        if (rearSafe) {
          this.controlState = 'REVERSING';
          this.targetSpeed = this.config.reverseSpeed; // -26 px/s
          egoVehicle.inputs.throttle = -0.6;
          egoVehicle.inputs.brake = 0;
          egoVehicle.inputs.steer = 0;
          return;
        } else {
          // If rear occupied, fallback to WAITING
          this.controlState = 'WAITING';
          this.targetSpeed = 0;
          egoVehicle.inputs.throttle = 0;
          egoVehicle.inputs.brake = 1.0;
          egoVehicle.inputs.steer = 0;
          return;
        }
      }

      case 'REPLAN':
        this.controlState = 'REPLANNING';
        this.targetSpeed = this.config.replanSpeed; // 25 px/s
        break;

      case 'SLOW':
        this.controlState = (plannerState === 'AVOIDANCE') ? 'AVOIDING' : 'BRAKING';
        this.targetSpeed = this.config.cautionSpeed; // 42 px/s
        break;

      case 'GO':
      default:
        this.controlState = 'CRUISING';
        this.targetSpeed = this.config.cruiseSpeed; // 95 px/s
        break;
    }

    // 5. Lateral Path Tracking (Smooth Steering Control)
    let desiredSteerAngle = 0;

    if (selectedPath && selectedPath.waypoints && selectedPath.waypoints.length > 0) {
      // Pick lookahead point along spline trajectory
      const lookaheadIdx = Math.min(this.config.lookaheadWaypointIdx, selectedPath.waypoints.length - 1);
      const targetWp = selectedPath.waypoints[lookaheadIdx] || selectedPath.endPoint;

      // Target heading from ego position to target waypoint
      const targetHeading = Math.atan2(targetWp.y - egoY, targetWp.x - egoX);
      let headingError = targetHeading - egoHeading;
      while (headingError > Math.PI) headingError -= Math.PI * 2;
      while (headingError < -Math.PI) headingError += Math.PI * 2;

      // Calculate desired steer input [-1.0, 1.0]
      desiredSteerAngle = Math.max(-1.0, Math.min(1.0, headingError * this.config.steerGain));

      // Speed reduction on sharp turns to maintain stability
      if (Math.abs(desiredSteerAngle) > 0.4) {
        this.targetSpeed = Math.min(this.targetSpeed, this.config.cautionSpeed);
      }
    } else {
      // Direct waypoint heading to destination if planner has no path
      const directHeading = Math.atan2(destY - egoY, destX - egoX);
      let headingError = directHeading - egoHeading;
      while (headingError > Math.PI) headingError -= Math.PI * 2;
      while (headingError < -Math.PI) headingError += Math.PI * 2;
      desiredSteerAngle = Math.max(-1.0, Math.min(1.0, headingError * 1.5));
    }

    // Smooth steering rate transition
    egoVehicle.inputs.steer += (desiredSteerAngle - egoVehicle.inputs.steer) * this.config.steerRateLimit * dt;
    egoVehicle.inputs.steer = Math.max(-1.0, Math.min(1.0, egoVehicle.inputs.steer));

    // 6. Longitudinal Speed Control (Throttle & Progressive Braking)
    if (currentSpeed < this.targetSpeed) {
      const speedDiff = this.targetSpeed - currentSpeed;
      egoVehicle.inputs.throttle = Math.min(1.0, Math.max(0.15, speedDiff / this.config.speedAccelGain));
      egoVehicle.inputs.brake = 0;
      egoVehicle.inputs.handbrake = false;
    } else if (currentSpeed > this.targetSpeed + 3) {
      const excessSpeed = currentSpeed - this.targetSpeed;
      egoVehicle.inputs.throttle = 0;
      egoVehicle.inputs.brake = Math.min(1.0, excessSpeed / this.config.speedDecelGain);
      egoVehicle.inputs.handbrake = false;
    } else {
      // Maintain speed / coast
      egoVehicle.inputs.throttle = 0.1;
      egoVehicle.inputs.brake = 0;
      egoVehicle.inputs.handbrake = false;
    }
  }

  /**
   * Verify real-time rear safety clearance buffer before allowing REVERSE
   */
  checkRearSafety(egoVehicle, perceptionData, trafficManager) {
    const cosH = Math.cos(egoVehicle.heading);
    const sinH = Math.sin(egoVehicle.heading);

    const dynamicEntities = trafficManager ? trafficManager.getEntities() : [];
    for (const dyn of dynamicEntities) {
      const dx = dyn.x - egoVehicle.x;
      const dy = dyn.y - egoVehicle.y;
      const forwardDist = dx * cosH + dy * sinH;
      const lateralDist = Math.abs(dy * cosH - dx * sinH);

      if (forwardDist < -4 && forwardDist > -this.config.rearSafetyBuffer && lateralDist < 26) {
        return false;
      }
    }

    if (window.StaticCollisionSystem) {
      const rearX = egoVehicle.x - cosH * 35;
      const rearY = egoVehicle.y - sinH * 35;
      const solidRear = window.StaticCollisionSystem.checkSolidCollision(rearX, rearY, 16);
      if (solidRear.collided) return false;
    }

    return true;
  }

  getTelemetry() {
    return {
      isAutonomous: this.isAutonomous,
      controlState: this.controlState,
      targetSpeed: this.targetSpeed,
      hasArrived: this.hasArrived
    };
  }
}

/* ============================================================================
   11. SIMULATION CORE APPLICATION CONTROLLER & GAME LOOP
   ============================================================================ */
class SimulationAppController {
  constructor() {
    this.canvas = null;
    this.ctx = null;
    this.camera = new SimulationCamera(WORLD_CONFIG.width, WORLD_CONFIG.height);

    // View Options
    this.options = {
      showGrid: true,
      showLabels: true
    };

    // Module 2: Ego Vehicle Instance
    this.egoVehicle = new EgoVehicle(140, 580, 0);

    // Module 3: Dynamic Traffic Manager
    this.trafficManager = new TrafficManager();

    // Module 4: Detection & Collision Risk Assessment System
    this.detectionManager = new DetectionManager();

    // Module 5: Adaptive Path Planning System
    this.pathPlanner = new PathPlanner();

    // Module 6A: Context-Aware Decision Manager
    this.decisionManager = new DecisionManager();

    // Module 6B: Autonomous Vehicle Controller
    this.autonomousController = new AutonomousVehicleController();

    // Registered Submodules
    this.modules = {};

    // Pointer / Pan State
    this.isDragging = false;
    this.dragStart = { x: 0, y: 0 };
    this.mouseWorldPos = { x: 0, y: 0 };

    // Timing Loop
    this.lastTime = 0;
    this.isRunning = true;

    // DOM Element References
    this.dom = {};
  }

  init() {
    this.cacheDom();
    this.setupCanvas();
    this.bindEvents();
    this.camera.fitToWorld();

    // Start 60 FPS Continuous Simulation Loop
    this.lastTime = performance.now();
    requestAnimationFrame((time) => this.loop(time));

    console.log('[SimulationEngine] Modules 1-6B Active: Environment, Ego Vehicle, Traffic Intelligence, Perception Risk, Path Planner, Decision Maker & Autonomous Control loaded.');
  }

  cacheDom() {
    this.dom = {
      canvas: document.getElementById('sim-canvas'),
      viewport: document.getElementById('viewport-container'),
      coordX: document.getElementById('coord-x'),
      coordY: document.getElementById('coord-y'),
      
      // Telemetry Readouts
      egoSpeed: document.getElementById('ego-speed'),

      // Module 6B Mode & Control State Elements
      btnMode: document.getElementById('btn-mode'),
      btnModeText: document.getElementById('btn-mode-text'),
      modePill: document.getElementById('mode-pill'),
      modeDot: document.getElementById('mode-dot'),
      modeValue: document.getElementById('mode-value'),
      controlStatePill: document.getElementById('control-state-pill'),
      controlStateValue: document.getElementById('control-state-value'),
      badgeMode: document.getElementById('badge-mode'),
      hintBanner: document.getElementById('hint-banner'),

      // Module 6A Decision Telemetry Elements
      decisionDot: document.getElementById('decision-dot'),
      decisionValue: document.getElementById('decision-value'),
      decisionReasonText: document.getElementById('decision-reason-text'),
      
      // Module 4 Detection & Risk Elements
      riskDot: document.getElementById('risk-dot'),
      riskLevelText: document.getElementById('risk-level-text'),
      minTtcDisplay: document.getElementById('min-ttc-display'),
      radarDot: document.getElementById('radar-dot'),

      // Module 5 Planner Telemetry Elements
      plannerDot: document.getElementById('planner-dot'),
      plannerDecision: document.getElementById('planner-decision'),
      selectedPathLabel: document.getElementById('selected-path-label'),
      pathCost: document.getElementById('path-cost'),

      // Controls
      btnPaths: document.getElementById('btn-paths'),
      btnSensors: document.getElementById('btn-sensors'),
      btnTraffic: document.getElementById('btn-traffic'),
      btnResetCar: document.getElementById('btn-reset-car'),
      btnGrid: document.getElementById('btn-grid'),
      btnLabels: document.getElementById('btn-labels'),
      btnZoomIn: document.getElementById('btn-zoom-in'),
      btnZoomOut: document.getElementById('btn-zoom-out'),
      btnResetView: document.getElementById('btn-reset-view'),
      btnLegend: document.getElementById('btn-legend'),
      legendPanel: document.getElementById('legend-panel'),
      legendCloseBtn: document.getElementById('legend-close-btn')
    };

    this.canvas = this.dom.canvas;
    this.ctx = this.canvas.getContext('2d');
  }

  setupCanvas() {
    const resize = () => {
      const rect = this.dom.viewport.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;

      this.canvas.width = rect.width * dpr;
      this.canvas.height = rect.height * dpr;
      this.camera.dpr = dpr;
      this.camera.updateViewport(rect.width, rect.height);
    };

    window.addEventListener('resize', resize);
    resize();
  }

  bindEvents() {
    // Canvas Pan (Drag)
    this.canvas.addEventListener('mousedown', (e) => {
      if (e.button === 0) {
        this.isDragging = true;
        this.dragStart = { x: e.clientX, y: e.clientY };
        this.canvas.style.cursor = 'grabbing';
      }
    });

    window.addEventListener('mousemove', (e) => {
      const rect = this.canvas.getBoundingClientRect();
      const screenX = e.clientX - rect.left;
      const screenY = e.clientY - rect.top;

      const worldPos = this.camera.screenToWorld(screenX, screenY);
      this.mouseWorldPos = {
        x: Math.round(worldPos.x),
        y: Math.round(worldPos.y)
      };

      if (this.dom.coordX && this.dom.coordY) {
        this.dom.coordX.textContent = this.mouseWorldPos.x;
        this.dom.coordY.textContent = this.mouseWorldPos.y;
      }

      if (this.isDragging) {
        const dx = e.clientX - this.dragStart.x;
        const dy = e.clientY - this.dragStart.y;
        this.dragStart = { x: e.clientX, y: e.clientY };
        this.camera.pan(dx, dy);
      }
    });

    window.addEventListener('mouseup', () => {
      if (this.isDragging) {
        this.isDragging = false;
        this.canvas.style.cursor = 'crosshair';
      }
    });

    // Mouse Wheel Zoom
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const rect = this.canvas.getBoundingClientRect();
      const screenX = e.clientX - rect.left;
      const screenY = e.clientY - rect.top;
      const factor = e.deltaY < 0 ? 1.12 : 0.88;

      this.camera.zoomAt(factor, screenX, screenY);
    }, { passive: false });

    // Mode Toggle Button (Manual / Autonomous)
    if (this.dom.btnMode) {
      this.dom.btnMode.addEventListener('click', () => {
        this.toggleAutonomousMode();
      });
    }

    // Path Planner Toggle Button
    if (this.dom.btnPaths) {
      this.dom.btnPaths.addEventListener('click', () => {
        const active = this.pathPlanner.togglePaths();
        this.dom.btnPaths.classList.toggle('active', active);
      });
    }

    // Perception / Radar Sensors Button
    if (this.dom.btnSensors) {
      this.dom.btnSensors.addEventListener('click', () => {
        const active = this.detectionManager.toggleSensors();
        this.dom.btnSensors.classList.toggle('active', active);
        if (this.dom.radarDot) {
          this.dom.radarDot.style.opacity = active ? '1' : '0.3';
        }
      });
    }

    // Traffic Toggle Button
    if (this.dom.btnTraffic) {
      this.dom.btnTraffic.addEventListener('click', () => {
        const active = this.trafficManager.toggle();
        this.dom.btnTraffic.classList.toggle('active', active);
      });
    }

    // Reset Car Button
    if (this.dom.btnResetCar) {
      this.dom.btnResetCar.addEventListener('click', () => {
        this.egoVehicle.reset();
        if (this.autonomousController) {
          this.autonomousController.hasArrived = false;
        }
      });
    }

    // Grid Toggle
    if (this.dom.btnGrid) {
      this.dom.btnGrid.addEventListener('click', () => {
        this.options.showGrid = !this.options.showGrid;
        this.dom.btnGrid.classList.toggle('active', this.options.showGrid);
      });
    }

    // Labels Toggle
    if (this.dom.btnLabels) {
      this.dom.btnLabels.addEventListener('click', () => {
        this.options.showLabels = !this.options.showLabels;
        this.dom.btnLabels.classList.toggle('active', this.options.showLabels);
      });
    }

    // Zoom Buttons
    if (this.dom.btnZoomIn) {
      this.dom.btnZoomIn.addEventListener('click', () => {
        this.camera.zoomAt(1.2, this.camera.viewportWidth / 2, this.camera.viewportHeight / 2);
      });
    }

    if (this.dom.btnZoomOut) {
      this.dom.btnZoomOut.addEventListener('click', () => {
        this.camera.zoomAt(0.83, this.camera.viewportWidth / 2, this.camera.viewportHeight / 2);
      });
    }

    // Fit View
    if (this.dom.btnResetView) {
      this.dom.btnResetView.addEventListener('click', () => {
        this.camera.fitToWorld();
      });
    }

    // Legend Toggle
    if (this.dom.btnLegend && this.dom.legendPanel) {
      this.dom.btnLegend.addEventListener('click', () => {
        this.dom.legendPanel.classList.toggle('hidden');
      });
    }

    if (this.dom.legendCloseBtn && this.dom.legendPanel) {
      this.dom.legendCloseBtn.addEventListener('click', () => {
        this.dom.legendPanel.classList.add('hidden');
      });
    }

    // Keyboard Shortcuts (M: Mode, Space: Stop/Manual, G: Grid, L: Labels, R: Reset View, T: Toggle Traffic, V: Toggle Radar, C: Toggle Paths)
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      const key = e.key.toLowerCase();
      if (key === 'm') {
        this.toggleAutonomousMode();
      } else if (e.code === 'Space') {
        // Emergency Stop & Return to Manual Teleoperation
        if (this.autonomousController && this.autonomousController.isAutonomous) {
          this.autonomousController.toggleMode(false, this.egoVehicle);
          this.updateModeUI(false);
        }
      } else if (key === 'g' && this.dom.btnGrid) {
        this.dom.btnGrid.click();
      } else if (key === 'l' && this.dom.btnLabels) {
        this.dom.btnLabels.click();
      } else if (key === 'r' && this.dom.btnResetView) {
        this.dom.btnResetView.click();
      } else if (key === 't' && this.dom.btnTraffic) {
        this.dom.btnTraffic.click();
      } else if (key === 'v' && this.dom.btnSensors) {
        this.dom.btnSensors.click();
      } else if (key === 'c' && this.dom.btnPaths) {
        this.dom.btnPaths.click();
      }
    });
  }

  /**
   * Toggle between Manual and Autonomous driving mode
   * @param {boolean} [forceState]
   * @returns {boolean}
   */
  toggleAutonomousMode(forceState) {
    if (!this.autonomousController) return false;
    const isAuto = this.autonomousController.toggleMode(forceState, this.egoVehicle);
    this.updateModeUI(isAuto);
    return isAuto;
  }

  updateModeUI(isAuto) {
    if (this.dom.btnMode) {
      this.dom.btnMode.classList.toggle('autonomous', isAuto);
      if (this.dom.btnModeText) {
        this.dom.btnModeText.textContent = isAuto ? 'Manual Mode' : 'Autonomous';
      }
    }
    if (this.dom.badgeMode) {
      this.dom.badgeMode.textContent = isAuto ? 'Autonomous AI' : 'Manual Teleop';
      this.dom.badgeMode.classList.toggle('autonomous', isAuto);
    }
    if (this.dom.modePill) {
      this.dom.modePill.classList.toggle('autonomous', isAuto);
    }
    if (this.dom.modeDot) {
      this.dom.modeDot.className = 'pill-dot mode-dot ' + (isAuto ? 'autonomous' : 'manual');
    }
    if (this.dom.modeValue) {
      this.dom.modeValue.textContent = isAuto ? 'AUTONOMOUS' : 'MANUAL';
      this.dom.modeValue.className = 'pill-value ' + (isAuto ? 'accent-emerald' : 'accent-cyan');
    }
  }

  /**
   * Main Simulation Loop (Updates Physics, Traffic, Perception, Path Planner, Decision, Controller, Telemetry, Renders Scene)
   */
  loop(currentTime) {
    const dt = Math.min((currentTime - this.lastTime) / 1000, 0.1);
    this.lastTime = currentTime;

    // 1. Update Perception & Risk Assessment (Module 4)
    if (this.detectionManager) {
      this.detectionManager.update(
        this.egoVehicle,
        EnvironmentData,
        this.trafficManager ? this.trafficManager.getEntities() : [],
        dt
      );
    }

    // 2. Update Adaptive Path Planner (Module 5)
    if (this.pathPlanner) {
      this.pathPlanner.update(
        this.egoVehicle,
        EnvironmentData,
        this.getPerceptionData(),
        dt
      );
    }

    // 3. Update Context-Aware Decision Manager (Module 6A)
    if (this.decisionManager) {
      this.decisionManager.update(
        this.egoVehicle,
        EnvironmentData,
        this.getPerceptionData(),
        this.getPathPlannerData(),
        this.trafficManager,
        dt
      );
    }

    // 4. Update Autonomous Vehicle Controller (Module 6B)
    if (this.autonomousController) {
      this.autonomousController.update(
        this.egoVehicle,
        EnvironmentData.destination,
        this.getDecisionData(),
        this.getPathPlannerData(),
        this.getPerceptionData(),
        this.trafficManager,
        dt
      );
    }

    // 5. Update Ego Vehicle Physics (Module 2)
    if (this.egoVehicle) {
      this.egoVehicle.update(dt);
    }

    // 6. Update Dynamic Traffic & Intelligent Non-Ego Vehicles (Module 3 + Local Traffic Planner)
    if (this.trafficManager) {
      this.trafficManager.update(dt, this.egoVehicle, EnvironmentData);
    }

    // 7. Update Submodules (Future Extensions)
    Object.keys(this.modules).forEach((name) => {
      const mod = this.modules[name];
      if (typeof mod.update === 'function') {
        mod.update(dt);
      }
    });

    // 8. Update Real-time HUD Telemetry
    this.updateHUD();

    // 9. Render Scene
    this.render();

    // Request next animation frame
    if (this.isRunning) {
      requestAnimationFrame((time) => this.loop(time));
    }
  }

  /**
   * Update HUD Telemetry elements with Ego Vehicle, Perception, Path Planner, Decision, and Autonomous Controller telemetry
   */
  updateHUD() {
    if (this.egoVehicle) {
      // Convert speed (px/s) to scaled km/h
      const speedKmH = (Math.abs(this.egoVehicle.speed) * 0.25).toFixed(1);
      if (this.dom.egoSpeed) {
        this.dom.egoSpeed.textContent = speedKmH;
      }
    }

    // Update Module 6B Autonomous Control State Telemetry
    if (this.autonomousController) {
      const isAuto = this.autonomousController.isAutonomous;
      const state = this.autonomousController.controlState;
      const arrived = this.autonomousController.hasArrived;

      if (this.dom.controlStateValue) {
        this.dom.controlStateValue.textContent = arrived ? 'ARRIVED' : state;
        this.dom.controlStateValue.className = 'pill-value';
        if (arrived) {
          this.dom.controlStateValue.classList.add('accent-emerald');
        } else if (state === 'STOPPED' || state === 'BRAKING') {
          this.dom.controlStateValue.classList.add('accent-rose');
        } else if (state === 'AVOIDING' || state === 'REPLANNING') {
          this.dom.controlStateValue.classList.add('accent-amber');
        } else if (state === 'WAITING' || state === 'YIELDING') {
          this.dom.controlStateValue.classList.add('accent-indigo');
        } else if (state === 'REVERSING') {
          this.dom.controlStateValue.classList.add('accent-pink');
        } else {
          this.dom.controlStateValue.classList.add('accent-cyan');
        }
      }

      if (arrived && this.dom.hintBanner) {
        this.dom.hintBanner.innerHTML = '<span>&#127881; <strong>DESTINATION REACHED!</strong> Vehicle safely stopped at destination goal. [P] Reset Car</span>';
      }
    }

    // Update Module 6A Context-Aware Decision Telemetry
    if (this.decisionManager) {
      const dec = this.decisionManager.currentDecision;
      const reason = this.decisionManager.reason;

      if (this.dom.decisionValue) {
        this.dom.decisionValue.textContent = dec;
        this.dom.decisionValue.className = 'pill-value';
        if (dec === 'STOP') {
          this.dom.decisionValue.classList.add('accent-rose');
        } else if (dec === 'SLOW') {
          this.dom.decisionValue.classList.add('accent-amber');
        } else if (dec === 'WAIT') {
          this.dom.decisionValue.classList.add('accent-indigo');
        } else if (dec === 'YIELD') {
          this.dom.decisionValue.classList.add('accent-orange');
        } else if (dec === 'REVERSE') {
          this.dom.decisionValue.classList.add('accent-pink');
        } else if (dec === 'REPLAN') {
          this.dom.decisionValue.classList.add('accent-cyan');
        } else {
          this.dom.decisionValue.classList.add('accent-emerald');
        }
      }

      if (this.dom.decisionDot) {
        this.dom.decisionDot.className = 'pill-dot decision-dot ' + dec.toLowerCase();
      }

      if (this.dom.decisionReasonText) {
        this.dom.decisionReasonText.textContent = reason;
      }
    }

    // Update Module 4 Collision Risk Telemetry
    if (this.detectionManager) {
      const overall = this.detectionManager.overallRisk;

      if (this.dom.riskLevelText) {
        this.dom.riskLevelText.textContent = overall;
        this.dom.riskLevelText.className = 'pill-value';
        if (overall === 'DANGER') {
          this.dom.riskLevelText.classList.add('accent-rose');
        } else if (overall === 'CAUTION') {
          this.dom.riskLevelText.classList.add('accent-amber');
        } else {
          this.dom.riskLevelText.classList.add('accent-emerald');
        }
      }

      if (this.dom.riskDot) {
        this.dom.riskDot.className = 'pill-dot risk-dot';
        this.dom.riskDot.classList.add(overall.toLowerCase());
      }

      if (this.dom.minTtcDisplay) {
        if (this.detectionManager.minTTC !== null && this.detectionManager.minTTC < 10) {
          this.dom.minTtcDisplay.textContent = `${this.detectionManager.minTTC.toFixed(1)} s`;
          this.dom.minTtcDisplay.className = 'pill-value ' + (overall === 'DANGER' ? 'accent-rose' : 'accent-amber');
        } else {
          this.dom.minTtcDisplay.textContent = '-- s';
          this.dom.minTtcDisplay.className = 'pill-value accent-cyan';
        }
      }
    }

    // Update Module 5 Path Planner Telemetry
    if (this.pathPlanner) {
      const state = this.pathPlanner.plannerState;
      const best = this.pathPlanner.selectedPath;

      if (this.dom.plannerDecision) {
        this.dom.plannerDecision.textContent = state;
        this.dom.plannerDecision.className = 'pill-value';
        if (state === 'EMERGENCY_STOP') {
          this.dom.plannerDecision.classList.add('accent-rose');
        } else if (state === 'AVOIDANCE') {
          this.dom.plannerDecision.classList.add('accent-amber');
        } else {
          this.dom.plannerDecision.classList.add('accent-emerald');
        }
      }

      if (this.dom.plannerDot) {
        this.dom.plannerDot.className = 'pill-dot planner-dot';
        if (state === 'EMERGENCY_STOP') {
          this.dom.plannerDot.classList.add('emergency');
        } else if (state === 'AVOIDANCE') {
          this.dom.plannerDot.classList.add('avoiding');
        }
      }

      if (this.dom.selectedPathLabel) {
        this.dom.selectedPathLabel.textContent = best ? best.label.toUpperCase() : '--';
      }

      if (this.dom.pathCost) {
        this.dom.pathCost.textContent = best ? best.totalCost.toFixed(0) : '0';
      }
    }
  }

  /**
   * Render Canvas pipeline
   */
  render() {
    if (!this.ctx) return;

    // Clear Screen Canvas
    this.ctx.setTransform(1, 0, 0, 1, 0, 0);
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // Apply Camera World Transform
    this.camera.applyTransform(this.ctx);

    // Render Static Environment (Roads, buildings, potholes, debris, destination, signals)
    EnvironmentRenderer.render(this.ctx, EnvironmentData, this.options);

    // Render Dynamic Traffic Objects (Module 3)
    if (this.trafficManager) {
      this.trafficManager.render(this.ctx);
    }

    // Render Perception Radar & Collision Risk Highlights (Module 4)
    if (this.detectionManager) {
      this.detectionManager.render(this.ctx, this.egoVehicle);
    }

    // Render Adaptive Candidate Paths & Selected Path (Module 5)
    if (this.pathPlanner) {
      this.pathPlanner.render(this.ctx);
    }

    // Render Ego Vehicle (Module 2)
    if (this.egoVehicle) {
      this.egoVehicle.render(this.ctx);
    }

    // Render Future Submodules
    Object.keys(this.modules).forEach((name) => {
      const mod = this.modules[name];
      if (typeof mod.render === 'function') {
        mod.render(this.ctx, this.camera);
      }
    });
  }

  /* ==========================================================================
     PUBLIC API & EXTENSION HOOKS FOR FUTURE MODULES
     ========================================================================== */

  getEgoVehicle() {
    return this.egoVehicle;
  }

  getAutonomousController() {
    return this.autonomousController;
  }

  isAutonomous() {
    return this.autonomousController ? this.autonomousController.isAutonomous : false;
  }

  getDestination() {
    return EnvironmentData.destination;
  }

  getDynamicObjects() {
    return this.trafficManager ? this.trafficManager.getEntities() : [];
  }

  /**
   * Accessor for Perception & Detection Data (Module 4)
   */
  getPerceptionData() {
    return {
      detectedObjects: this.detectionManager ? this.detectionManager.detectedObjects : [],
      overallRisk: this.detectionManager ? this.detectionManager.overallRisk : 'SAFE',
      minTTC: this.detectionManager ? this.detectionManager.minTTC : null,
      detectedCount: this.detectionManager ? this.detectionManager.detectedObjects.length : 0,
      dangerCount: this.detectionManager ? this.detectionManager.dangerCount : 0,
      cautionCount: this.detectionManager ? this.detectionManager.cautionCount : 0,
      safeCount: this.detectionManager ? this.detectionManager.safeCount : 0,
      perceptionRadius: this.detectionManager ? this.detectionManager.perceptionRadius : 230
    };
  }

  /**
   * Accessor for Threat Summary & Prioritized Risk Objects
   */
  getRiskAssessment() {
    return {
      overallRisk: this.detectionManager ? this.detectionManager.overallRisk : 'SAFE',
      minTTC: this.detectionManager ? this.detectionManager.minTTC : null,
      threatObjects: this.detectionManager ? this.detectionManager.detectedObjects.filter(o => o.riskLevel !== 'SAFE') : []
    };
  }

  /**
   * Accessor for Adaptive Path Planner Data (Module 5 -> Module 6 Vehicle Control)
   */
  getPathPlannerData() {
    return {
      candidatePaths: this.pathPlanner ? this.pathPlanner.candidatePaths : [],
      selectedPath: this.pathPlanner ? this.pathPlanner.selectedPath : null,
      plannerState: this.pathPlanner ? this.pathPlanner.plannerState : 'OPTIMAL',
      totalCandidates: this.pathPlanner ? this.pathPlanner.candidatePaths.length : 0,
      feasibleCount: this.pathPlanner ? this.pathPlanner.candidatePaths.filter(p => p.status === 'FEASIBLE').length : 0
    };
  }

  getSelectedPath() {
    return this.pathPlanner ? this.pathPlanner.selectedPath : null;
  }

  getAllCandidatePaths() {
    return this.pathPlanner ? this.pathPlanner.candidatePaths : [];
  }

  /**
   * Accessor for Context-Aware Decision Maker (Module 6A)
   */
  getDecisionData() {
    return this.decisionManager ? this.decisionManager.getDecisionData() : { decision: 'GO', reason: 'Normal Driving' };
  }

  getDecision() {
    return this.decisionManager ? this.decisionManager.getDecision() : 'GO';
  }

  getControlState() {
    return this.autonomousController ? this.autonomousController.controlState : 'MANUAL';
  }

  getObstacles() {
    return {
      potholes: EnvironmentData.potholes,
      debris: EnvironmentData.debris,
      parkedObjects: EnvironmentData.parkedObjects,
      buildings: EnvironmentData.buildings,
      signal: EnvironmentData.trafficSignal,
      dynamicObjects: this.getDynamicObjects(),
      perception: this.getPerceptionData(),
      planner: this.getPathPlannerData(),
      decision: this.getDecisionData(),
      controller: this.autonomousController ? this.autonomousController.getTelemetry() : null
    };
  }

  getRoadNetwork() {
    return EnvironmentData.roads;
  }

  registerModule(name, moduleInstance) {
    this.modules[name] = moduleInstance;
    console.log(`[Module Registered] "${name}" connected to simulation engine.`);
    if (typeof moduleInstance.init === 'function') {
      moduleInstance.init(this);
    }
  }

  worldToScreen(wx, wy) {
    return this.camera.worldToScreen(wx, wy);
  }

  screenToWorld(sx, sy) {
    return this.camera.screenToWorld(sx, sy);
  }
}

// Global Singleton Instance
const SimulationEngine = new SimulationAppController();

// Mount on Global Window for modular inter-script communication
window.SimulationEngine = SimulationEngine;
window.EnvironmentData = EnvironmentData;
window.WORLD_CONFIG = WORLD_CONFIG;
window.EgoVehicle = EgoVehicle;
window.DynamicObject = DynamicObject;
window.TrafficManager = TrafficManager;
window.DetectionManager = DetectionManager;
window.PathPlanner = PathPlanner;
window.StaticCollisionSystem = StaticCollisionSystem;
window.DecisionManager = DecisionManager;
window.AutonomousVehicleController = AutonomousVehicleController;

// Boot on DOM Ready
document.addEventListener('DOMContentLoaded', () => {
  SimulationEngine.init();
});





