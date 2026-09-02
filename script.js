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
   3. CAMERA & VIEWPORT CONTROLLER
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
      handbrake: false // Space = immediate stop
    };

    // State Classification for HUD
    this.state = 'STOPPED'; // 'STOPPED', 'ACCELERATING', 'CRUISING', 'BRAKING', 'REVERSING'
    this.isBraking = false;
    this.isReversing = false;

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
    this.state = 'STOPPED';
    this.isBraking = false;
    this.isReversing = false;
    console.log('[EgoVehicle] Reset to starting position.');
  }

  /**
   * Smooth 2D Vehicle Kinematics Physics Integration
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
    // Angular velocity omega = (v / L) * tan(steeringAngle)
    if (Math.abs(this.speed) > 0.05) {
      const angularVelocity = (this.speed / this.wheelbase) * Math.tan(this.steeringAngle);
      this.heading += angularVelocity * dt;
      
      // Normalize heading angle to [-PI, PI]
      this.heading = Math.atan2(Math.sin(this.heading), Math.cos(this.heading));
    }

    // --- 4. Position Integration in World Coordinates ---
    this.x += this.speed * Math.cos(this.heading) * dt;
    this.y += this.speed * Math.sin(this.heading) * dt;

    // --- 5. Lidar Sensor Rotation for Animation ---
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
    this.radius = config.radius || 8; // For circular / pedestrian bounds

    // Visual Palette
    this.color = config.color || '#dc2626';
    this.accentColor = config.accentColor || '#1e293b';

    // Route / Waypoint / Path config
    this.route = config.route || {
      road: 'main',
      laneY: this.y,
      minX: -80,
      maxX: 1880,
      minY: -80,
      maxY: 1080
    };

    // State and Controlled Randomness
    this.behavior = config.behavior || 'CRUISING'; // 'CRUISING', 'CROSSING', 'WANDERING', 'PAUSED'
    this.stateTimer = Math.random() * 3 + 2;        // Countdown to next random behavior shift
    this.pauseTimer = 0;
    this.swayPhase = Math.random() * Math.PI * 2;
    this.lateralOffset = 0;
    this.targetLateralOffset = 0;
    this.crossingDirection = config.crossingDirection || 1; // +1 down, -1 up
    this.active = true;
  }

  /**
   * Update kinematics and controlled randomness
   * @param {number} dt - Delta time in seconds
   */
  update(dt) {
    if (!this.active) return;
    if (dt <= 0 || dt > 0.1) dt = 0.016;

    this.swayPhase += 6.0 * dt;
    if (this.swayPhase > Math.PI * 2) this.swayPhase -= Math.PI * 2;

    this.stateTimer -= dt;

    switch (this.type) {
      case 'car':
        this.updateCar(dt);
        break;
      case 'motorcycle':
        this.updateMotorcycle(dt);
        break;
      case 'auto_rickshaw':
        this.updateAutoRickshaw(dt);
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

  // --- Dynamic Behaviors & Controlled Randomness ---

  updateCar(dt) {
    // Controlled randomness: subtle speed fluctuation and lateral drift
    if (this.stateTimer <= 0) {
      this.stateTimer = Math.random() * 4 + 3;
      this.speed = this.baseSpeed + (Math.random() * 18 - 9);
      this.targetLateralOffset = (Math.random() * 16 - 8);
    }

    // Smooth lateral ease
    this.lateralOffset += (this.targetLateralOffset - this.lateralOffset) * 1.5 * dt;

    if (this.route.road === 'main') {
      this.x += this.speed * Math.cos(this.heading) * dt;
      const nominalY = this.route.laneY || 560;
      this.y = nominalY + this.lateralOffset;

      // Wrap around world edges
      if (this.heading === 0 && this.x > this.route.maxX) {
        this.x = this.route.minX;
        this.y = (this.route.laneY || 595) + (Math.random() * 14 - 7);
      } else if (Math.abs(this.heading - Math.PI) < 0.1 && this.x < this.route.minX) {
        this.x = this.route.maxX;
        this.y = (this.route.laneY || 525) + (Math.random() * 14 - 7);
      }
    } else if (this.route.road === 'side') {
      this.y += this.speed * Math.sin(this.heading) * dt;
      const nominalX = this.route.laneX || 895;
      this.x = nominalX + this.lateralOffset;

      if (this.y > (this.route.maxY || 470)) {
        this.y = this.route.minY || -60;
      }
    }
  }

  updateMotorcycle(dt) {
    // Motorcycles: Fast, slight weaving/swerving between lanes
    if (this.stateTimer <= 0) {
      this.stateTimer = Math.random() * 2.5 + 1.5;
      this.speed = this.baseSpeed + (Math.random() * 26 - 13);
      this.targetLateralOffset = (Math.random() * 22 - 11);
    }

    this.lateralOffset += (this.targetLateralOffset - this.lateralOffset) * 2.5 * dt;

    if (this.route.road === 'main') {
      this.x += this.speed * Math.cos(this.heading) * dt;
      const nominalY = this.route.laneY || 570;
      this.y = nominalY + this.lateralOffset + Math.sin(this.swayPhase * 0.6) * 2.0;

      if (this.heading === 0 && this.x > this.route.maxX) {
        this.x = this.route.minX;
      } else if (Math.abs(this.heading - Math.PI) < 0.1 && this.x < this.route.minX) {
        this.x = this.route.maxX;
      }
    } else if (this.route.road === 'side') {
      this.y += this.speed * Math.sin(this.heading) * dt;
      const nominalX = this.route.laneX || 940;
      this.x = nominalX + this.lateralOffset;

      if (this.y > (this.route.maxY || 470)) {
        this.y = this.route.minY || -60;
      }
    }
  }

  updateAutoRickshaw(dt) {
    // Auto-rickshaws: Moderate speed, occasional drift towards shoulder
    if (this.stateTimer <= 0) {
      this.stateTimer = Math.random() * 3.5 + 2.5;
      this.speed = this.baseSpeed + (Math.random() * 12 - 6);
      this.targetLateralOffset = (Math.random() * 16 - 8);
    }

    this.lateralOffset += (this.targetLateralOffset - this.lateralOffset) * 1.8 * dt;

    if (this.route.road === 'main') {
      this.x += this.speed * Math.cos(this.heading) * dt;
      const nominalY = this.route.laneY || 610;
      this.y = nominalY + this.lateralOffset;

      if (this.heading === 0 && this.x > this.route.maxX) {
        this.x = this.route.minX;
      } else if (Math.abs(this.heading - Math.PI) < 0.1 && this.x < this.route.minX) {
        this.x = this.route.maxX;
      }
    } else if (this.route.road === 'side') {
      this.y += this.speed * Math.sin(this.heading) * dt;
      const nominalX = this.route.laneX || 955;
      this.x = nominalX + this.lateralOffset;

      if (this.y > (this.route.maxY || 470)) {
        this.y = this.route.minY || -60;
      }
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

      // Wrap around sidewalk ends
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

      // Wrap on world edges
      if (this.x > 1820) this.x = -20;
      if (this.x < -20) this.x = 1820;

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

      if (this.x > 1820) this.x = -20;
      if (this.x < -20) this.x = 1820;
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

    // Rear Taillights
    ctx.fillStyle = '#ef4444';
    ctx.fillRect(-halfL, -halfW + 2, 2, 4);
    ctx.fillRect(-halfL, halfW - 6, 2, 4);
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

    // Rear Black Bumper
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(-halfL, -halfW + 2, 3, this.width - 4);
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
        route: { road: 'main', laneY: 595, minX: -80, maxX: 1880 }
      }),
      new DynamicObject({
        id: 'car-2',
        type: 'car',
        subType: 'hatchback',
        x: 1200,
        y: 525,
        length: 46,
        width: 23,
        heading: Math.PI, // Westbound
        speed: 90,
        color: '#2563eb',
        route: { road: 'main', laneY: 525, minX: -80, maxX: 1880 }
      }),
      new DynamicObject({
        id: 'car-3',
        type: 'car',
        subType: 'suv',
        x: 750,
        y: 615,
        length: 52,
        width: 26,
        heading: 0, // Eastbound
        speed: 75,
        color: '#e2e8f0',
        route: { road: 'main', laneY: 615, minX: -80, maxX: 1880 }
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
        route: { road: 'side', laneX: 895, minY: -60, maxY: 470 }
      }),

      // --- 2. Motorcycles & Bikes (4) ---
      new DynamicObject({
        id: 'bike-1',
        type: 'motorcycle',
        subType: 'sport_bike',
        x: 520,
        y: 580,
        length: 28,
        width: 12,
        heading: 0, // Eastbound
        speed: 120,
        color: '#ea580c',
        route: { road: 'main', laneY: 580, minX: -80, maxX: 1880 }
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
        route: { road: 'main', laneY: 515, minX: -80, maxX: 1880 }
      }),
      new DynamicObject({
        id: 'bike-3',
        type: 'motorcycle',
        subType: 'scooter',
        x: 180,
        y: 635,
        length: 25,
        width: 12,
        heading: 0, // Eastbound
        speed: 80,
        color: '#06b6d4',
        route: { road: 'main', laneY: 635, minX: -80, maxX: 1880 }
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
        route: { road: 'side', laneX: 940, minY: -60, maxY: 470 }
      }),

      // --- 3. Auto-Rickshaws (3) ---
      new DynamicObject({
        id: 'auto-1',
        type: 'auto_rickshaw',
        subType: 'cng_auto',
        x: 980,
        y: 625,
        length: 44,
        width: 26,
        heading: 0, // Eastbound
        speed: 70,
        color: '#15803d',
        route: { road: 'main', laneY: 625, minX: -80, maxX: 1880 }
      }),
      new DynamicObject({
        id: 'auto-2',
        type: 'auto_rickshaw',
        subType: 'cng_auto',
        x: 600,
        y: 535,
        length: 44,
        width: 26,
        heading: Math.PI, // Westbound
        speed: 65,
        color: '#15803d',
        route: { road: 'main', laneY: 535, minX: -80, maxX: 1880 }
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
        route: { road: 'side', laneX: 955, minY: -60, maxY: 470 }
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

  update(dt) {
    if (!this.isEnabled) return;
    this.entities.forEach(entity => entity.update(dt));
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

      // Forward corridor alignment: is candidate in ego's direct moving path?
      const inEgoTravelCorridor = Math.abs(bearing) < (35 * Math.PI / 180);
      const isApproaching = closingSpeed > 2.0;

      // Danger Condition: Imminent collision trajectory or critical proximity
      if ((ttc <= this.dangerTTC && isApproaching && (inEgoTravelCorridor || inCloseProximity)) ||
          (distance <= this.criticalDistance && isApproaching)) {
        riskLevel = 'DANGER';
        dCount++;
        if (ttc < minTTC) minTTC = ttc;
      }
      // Caution Condition: Potential collision path or proximity alert
      else if ((ttc <= this.cautionTTC && isApproaching) ||
               (distance <= this.cautionDistance && (isApproaching || inEgoTravelCorridor)) ||
               (!cand.isDynamic && inEgoTravelCorridor && distance < 140 && egoSpeed > 15)) {
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
}

/* ============================================================================
   8. SIMULATION CORE APPLICATION CONTROLLER & GAME LOOP
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

    console.log('[SimulationEngine] Module 1, 2, 3 & 4 Active: Environment, Ego Vehicle, Traffic & Detection Risk loaded.');
  }

  cacheDom() {
    this.dom = {
      canvas: document.getElementById('sim-canvas'),
      viewport: document.getElementById('viewport-container'),
      coordX: document.getElementById('coord-x'),
      coordY: document.getElementById('coord-y'),
      
      // Telemetry Readouts
      egoSpeed: document.getElementById('ego-speed'),
      egoHeading: document.getElementById('ego-heading'),
      egoSteer: document.getElementById('ego-steer'),
      
      // Module 4 Detection & Risk Elements
      riskDot: document.getElementById('risk-dot'),
      riskLevelText: document.getElementById('risk-level-text'),
      minTtcDisplay: document.getElementById('min-ttc-display'),
      detectedCount: document.getElementById('detected-count'),
      radarDot: document.getElementById('radar-dot'),

      // Controls
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

    // Keyboard Shortcuts (G: Grid, L: Labels, R: Reset View, T: Toggle Traffic, V: Toggle Radar)
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      const key = e.key.toLowerCase();
      if (key === 'g' && this.dom.btnGrid) {
        this.dom.btnGrid.click();
      } else if (key === 'l' && this.dom.btnLabels) {
        this.dom.btnLabels.click();
      } else if (key === 'r' && this.dom.btnResetView) {
        this.dom.btnResetView.click();
      } else if (key === 't' && this.dom.btnTraffic) {
        this.dom.btnTraffic.click();
      } else if (key === 'v' && this.dom.btnSensors) {
        this.dom.btnSensors.click();
      }
    });
  }

  /**
   * Main Simulation Loop (Updates Physics, Traffic, Perception, Telemetry, Renders Scene)
   */
  loop(currentTime) {
    const dt = Math.min((currentTime - this.lastTime) / 1000, 0.1);
    this.lastTime = currentTime;

    // 1. Update Ego Vehicle Physics (Module 2)
    if (this.egoVehicle) {
      this.egoVehicle.update(dt);
    }

    // 2. Update Dynamic Traffic & Entities (Module 3)
    if (this.trafficManager) {
      this.trafficManager.update(dt);
    }

    // 3. Update Perception & Risk Assessment (Module 4)
    if (this.detectionManager) {
      this.detectionManager.update(
        this.egoVehicle,
        EnvironmentData,
        this.trafficManager.getEntities(),
        dt
      );
    }

    // 4. Update Submodules (Future Module 5+)
    Object.keys(this.modules).forEach((name) => {
      const mod = this.modules[name];
      if (typeof mod.update === 'function') {
        mod.update(dt);
      }
    });

    // 5. Update Real-time HUD Telemetry
    this.updateHUD();

    // 6. Render Scene
    this.render();

    // Request next animation frame
    if (this.isRunning) {
      requestAnimationFrame((time) => this.loop(time));
    }
  }

  /**
   * Update HUD Telemetry elements with Ego Vehicle and Perception risk telemetry
   */
  updateHUD() {
    if (this.egoVehicle) {
      // Convert speed (px/s) to scaled km/h
      const speedKmH = (Math.abs(this.egoVehicle.speed) * 0.25).toFixed(1);
      if (this.dom.egoSpeed) {
        this.dom.egoSpeed.textContent = speedKmH;
      }

      // Convert heading from radians to degrees [0, 360)
      let headingDeg = (this.egoVehicle.heading * (180 / Math.PI)) % 360;
      if (headingDeg < 0) headingDeg += 360;
      if (this.dom.egoHeading) {
        this.dom.egoHeading.textContent = `${headingDeg.toFixed(1)}°`;
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

      if (this.dom.detectedCount) {
        this.dom.detectedCount.textContent = this.detectionManager.detectedObjects.length;
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

    // Render Ego Vehicle (Module 2)
    if (this.egoVehicle) {
      this.egoVehicle.render(this.ctx);
    }

    // Render Future Submodules (Module 5+)
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

  getDestination() {
    return EnvironmentData.destination;
  }

  getDynamicObjects() {
    return this.trafficManager ? this.trafficManager.getEntities() : [];
  }

  /**
   * Accessor for Perception & Detection Data (Module 4 -> Module 5 Path Planning)
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

  getObstacles() {
    return {
      potholes: EnvironmentData.potholes,
      debris: EnvironmentData.debris,
      parkedObjects: EnvironmentData.parkedObjects,
      buildings: EnvironmentData.buildings,
      signal: EnvironmentData.trafficSignal,
      dynamicObjects: this.getDynamicObjects(),
      perception: this.getPerceptionData()
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

// Boot on DOM Ready
document.addEventListener('DOMContentLoaded', () => {
  SimulationEngine.init();
});




