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
   6. SIMULATION CORE APPLICATION CONTROLLER & GAME LOOP
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

    console.log('[SimulationEngine] Module 1 & 2 Active: Environment and Ego Vehicle loaded.');
  }

  cacheDom() {
    this.dom = {
      canvas: document.getElementById('sim-canvas'),
      viewport: document.getElementById('viewport-container'),
      coordX: document.getElementById('coord-x'),
      coordY: document.getElementById('coord-y'),
      
      // Telemetry Readouts
      egoStateDot: document.getElementById('ego-state-dot'),
      egoStateText: document.getElementById('ego-state-text'),
      egoSpeed: document.getElementById('ego-speed'),
      egoHeading: document.getElementById('ego-heading'),
      egoSteer: document.getElementById('ego-steer'),

      // Controls
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

    // Keyboard Shortcuts (G: Grid, L: Labels, R: Reset View)
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      const key = e.key.toLowerCase();
      if (key === 'g' && this.dom.btnGrid) {
        this.dom.btnGrid.click();
      } else if (key === 'l' && this.dom.btnLabels) {
        this.dom.btnLabels.click();
      } else if (key === 'r' && this.dom.btnResetView) {
        this.dom.btnResetView.click();
      }
    });
  }

  /**
   * Main Simulation Loop (Updates Physics, Updates Telemetry, Renders Scene)
   */
  loop(currentTime) {
    const dt = Math.min((currentTime - this.lastTime) / 1000, 0.1);
    this.lastTime = currentTime;

    // 1. Update Ego Vehicle Physics
    if (this.egoVehicle) {
      this.egoVehicle.update(dt);
    }

    // 2. Update Submodules
    Object.keys(this.modules).forEach((name) => {
      const mod = this.modules[name];
      if (typeof mod.update === 'function') {
        mod.update(dt);
      }
    });

    // 3. Update Real-time HUD Telemetry
    this.updateHUD();

    // 4. Render Scene
    this.render();

    // Request next animation frame
    if (this.isRunning) {
      requestAnimationFrame((time) => this.loop(time));
    }
  }

  /**
   * Update HUD Telemetry elements with Ego Vehicle telemetry data
   */
  updateHUD() {
    if (!this.egoVehicle) return;

    // Convert speed (px/s) to scaled km/h (1 px/s approx 0.25 km/h)
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

    // Steer angle in degrees
    const steerDeg = (this.egoVehicle.steeringAngle * (180 / Math.PI)).toFixed(1);
    if (this.dom.egoSteer) {
      this.dom.egoSteer.textContent = `${steerDeg > 0 ? '+' : ''}${steerDeg}°`;
    }

    // State text & indicator dot
    if (this.dom.egoStateText && this.dom.egoStateDot) {
      const state = this.egoVehicle.state;
      this.dom.egoStateText.textContent = state;

      this.dom.egoStateDot.className = 'pill-dot';
      switch (state) {
        case 'ACCELERATING':
          this.dom.egoStateDot.classList.add('accelerating');
          break;
        case 'BRAKING':
          this.dom.egoStateDot.classList.add('braking');
          break;
        case 'REVERSING':
          this.dom.egoStateDot.classList.add('reversing');
          break;
        case 'STOPPED':
          this.dom.egoStateDot.classList.add('stopped');
          break;
        default:
          break;
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

    // Render Ego Vehicle (Module 2)
    if (this.egoVehicle) {
      this.egoVehicle.render(this.ctx);
    }

    // Render Future Submodules (Module 3+)
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

  getObstacles() {
    return {
      potholes: EnvironmentData.potholes,
      debris: EnvironmentData.debris,
      parkedObjects: EnvironmentData.parkedObjects,
      buildings: EnvironmentData.buildings,
      signal: EnvironmentData.trafficSignal
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

// Boot on DOM Ready
document.addEventListener('DOMContentLoaded', () => {
  SimulationEngine.init();
});


