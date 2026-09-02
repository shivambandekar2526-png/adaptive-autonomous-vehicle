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
  },

  /**
   * Determine if a point is within drivable road surfaces
   */
  isDrivableRoad(x, y) {
    const onMain = (y >= 468 && y <= 652) && (x >= -120 && x <= 1920);
    const onSide = (x >= 840 && x <= 990) && (y >= -120 && y <= 560);
    return onMain || onSide;
  },

  /**
   * Determine if a point is on pedestrian sidewalks / shoulders
   */
  isSidewalk(x, y) {
    if (this.isDrivableRoad(x, y)) return false;
    const northSidewalk = (y >= 405 && y < 468) && !(x >= 840 && x <= 990);
    const southSidewalk = (y > 652 && y <= 715) && (x >= -120 && x <= 1920);
    const westSidewalk = (x >= 775 && x < 840) && (y >= -120 && y < 468);
    const eastSidewalk = (x > 990 && x <= 1055) && (y >= -120 && y < 468);
    return northSidewalk || southSidewalk || westSidewalk || eastSidewalk;
  },

  /**
   * Determine if an entity on a sidewalk is actively heading toward the road
   */
  isMovingTowardRoad(x, y, heading, speed) {
    if (speed < 2.0) return false;
    const vx = speed * Math.cos(heading);
    const vy = speed * Math.sin(heading);

    // North sidewalk (y < 468): moving South toward road -> vy > 3
    if (y < 468 && vy > 3.0) return true;
    // South sidewalk (y > 652): moving North toward road -> vy < -3
    if (y > 652 && vy < -3.0) return true;
    // Side road West sidewalk (x in [775, 840]): moving East toward road -> vx > 3
    if (x >= 775 && x < 840 && y < 468 && vx > 3.0) return true;
    // Side road East sidewalk (x in [990, 1055]): moving West toward road -> vx < -3
    if (x > 990 && x <= 1055 && y < 468 && vx < -3.0) return true;

    return false;
  }
};

/* ============================================================================
   3B. ROAD NETWORK GRAPH & TOPOLOGY SYSTEM (DETERMINISTIC ROAD NETWORK)
   ============================================================================ */

/**
 * Road Network Node representation
 * Represents a discrete, topologically connected spatial waypoint on the road network
 */
class RoadNode {
  constructor(config = {}) {
    this.id = config.id || `rn-${Math.random().toString(36).substr(2, 6)}`;
    this.x = config.x || 0;
    this.y = config.y || 0;
    this.road = config.road || 'main'; // 'main', 'side', 'intersection'
    this.lane = config.lane || 'center'; // 'eastbound', 'westbound', 'center', 'southbound', 'northbound', 'junction'
    this.heading = config.heading !== undefined ? config.heading : 0; // Nominal road orientation in radians
    this.speedLimit = config.speedLimit || 95;
    this.edges = []; // Array of outgoing RoadSegment instances
    this.tags = config.tags || [];
  }

  addEdge(edge) {
    if (!this.edges.some(e => e.id === edge.id)) {
      this.edges.push(edge);
    }
  }
}

/**
 * Directed Road Segment (Edge) connecting two nodes
 */
class RoadSegment {
  constructor(config = {}) {
    this.id = config.id || `seg-${config.fromNode.id}->${config.toNode.id}`;
    this.fromNode = config.fromNode;
    this.toNode = config.toNode;
    this.road = config.road || config.fromNode.road;
    this.lane = config.lane || config.fromNode.lane;
    this.isDrivable = config.isDrivable !== undefined ? config.isDrivable : true;
    this.speedLimit = config.speedLimit || 95;
    this.length = Math.hypot(this.toNode.x - this.fromNode.x, this.toNode.y - this.fromNode.y);
    
    // Direction vector & heading
    const dx = this.toNode.x - this.fromNode.x;
    const dy = this.toNode.y - this.fromNode.y;
    this.heading = Math.atan2(dy, dx);
    this.dirX = this.length > 0 ? dx / this.length : 1;
    this.dirY = this.length > 0 ? dy / this.length : 0;

    // Optional intermediate waypoints for smooth curve geometry
    this.waypoints = config.waypoints || [
      { x: this.fromNode.x, y: this.fromNode.y },
      { x: this.toNode.x, y: this.toNode.y }
    ];
  }
}

/**
 * Reusable Road Network Graph & Geometry System
 */
class RoadNetworkGraph {
  constructor(environmentData) {
    this.env = environmentData || (typeof EnvironmentData !== 'undefined' ? EnvironmentData : null);
    this.nodes = new Map();
    this.segments = new Map();
    this.showDebug = false;

    // Build the deterministic graph structure
    this.buildGraph();
  }

  buildGraph() {
    this.nodes.clear();
    this.segments.clear();

    // 1. Create Main Road Nodes:
    // Eastbound Lane (y = 600, heading = 0)
    const eastX = [50, 200, 350, 500, 650, 800, 920, 1050, 1200, 1350, 1500, 1650, 1750];
    eastX.forEach((x, idx) => {
      const isJunction = (x === 920);
      this.addNode(new RoadNode({
        id: `node-me-${idx}`,
        x: x,
        y: 600,
        road: isJunction ? 'intersection' : 'main',
        lane: 'eastbound',
        heading: 0,
        speedLimit: 95,
        tags: ['main', 'eastbound', isJunction ? 'junction' : 'lane']
      }));
    });

    // Westbound Lane (y = 520, heading = PI)
    const westX = [1750, 1650, 1500, 1350, 1200, 1050, 920, 800, 650, 500, 350, 200, 50];
    westX.forEach((x, idx) => {
      const isJunction = (x === 920);
      this.addNode(new RoadNode({
        id: `node-mw-${idx}`,
        x: x,
        y: 520,
        road: isJunction ? 'intersection' : 'main',
        lane: 'westbound',
        heading: Math.PI,
        speedLimit: 95,
        tags: ['main', 'westbound', isJunction ? 'junction' : 'lane']
      }));
    });

    // Center / Median Corridor (y = 560, bidirectional)
    const centerX = [50, 200, 350, 500, 650, 800, 920, 1050, 1200, 1350, 1500, 1650, 1750];
    centerX.forEach((x, idx) => {
      const isJunction = (x === 920);
      const isGoal = (x === 1650);
      this.addNode(new RoadNode({
        id: `node-mc-${idx}`,
        x: x,
        y: 560,
        road: isJunction ? 'intersection' : 'main',
        lane: 'center',
        heading: 0,
        speedLimit: 95,
        tags: ['main', 'center', isJunction ? 'junction' : 'median', isGoal ? 'destination' : 'corridor']
      }));
    });

    // 2. Create Side Road (Vertical) Nodes:
    // Southbound Lane (x = 880, heading = PI / 2, approaching T-junction from top)
    const southY = [50, 150, 270, 380, 460];
    southY.forEach((y, idx) => {
      const isJunction = (y === 460);
      this.addNode(new RoadNode({
        id: `node-ss-${idx}`,
        x: 880,
        y: y,
        road: isJunction ? 'intersection' : 'side',
        lane: 'southbound',
        heading: Math.PI / 2,
        speedLimit: 60,
        tags: ['side', 'southbound', isJunction ? 'junction_entry' : 'lane']
      }));
    });

    // Northbound Lane (x = 950, heading = -PI / 2, departing T-junction to top)
    const northY = [460, 380, 270, 150, 50];
    northY.forEach((y, idx) => {
      const isJunction = (y === 460);
      this.addNode(new RoadNode({
        id: `node-sn-${idx}`,
        x: 950,
        y: y,
        road: isJunction ? 'intersection' : 'side',
        lane: 'northbound',
        heading: -Math.PI / 2,
        speedLimit: 60,
        tags: ['side', 'northbound', isJunction ? 'junction_exit' : 'lane']
      }));
    });

    // 3. Connect Main Road Eastbound Segments:
    for (let i = 0; i < eastX.length - 1; i++) {
      this.connectNodes(`node-me-${i}`, `node-me-${i + 1}`, { road: 'main', lane: 'eastbound' });
    }

    // 4. Connect Main Road Westbound Segments:
    for (let i = 0; i < westX.length - 1; i++) {
      this.connectNodes(`node-mw-${i}`, `node-mw-${i + 1}`, { road: 'main', lane: 'westbound' });
    }

    // 5. Connect Main Center Segments (Bidirectional):
    for (let i = 0; i < centerX.length - 1; i++) {
      this.connectNodes(`node-mc-${i}`, `node-mc-${i + 1}`, { road: 'main', lane: 'center' });
      this.connectNodes(`node-mc-${i + 1}`, `node-mc-${i}`, { road: 'main', lane: 'center' });
    }

    // 6. Connect Lateral & Diagonal Transitions between Main Lanes (Lane switching & Overtaking paths):
    for (let i = 0; i < centerX.length; i++) {
      // Eastbound <-> Center
      this.connectNodes(`node-me-${i}`, `node-mc-${i}`, { road: 'main', lane: 'transition', speedLimit: 60 });
      this.connectNodes(`node-mc-${i}`, `node-me-${i}`, { road: 'main', lane: 'transition', speedLimit: 60 });

      // Forward Diagonal transitions:
      // Eastbound -> Center Forward: node-me-i -> node-mc-(i+1)
      if (i < centerX.length - 1) {
        this.connectNodes(`node-me-${i}`, `node-mc-${i + 1}`, { road: 'main', lane: 'transition', speedLimit: 75 });
        this.connectNodes(`node-mc-${i}`, `node-me-${i + 1}`, { road: 'main', lane: 'transition', speedLimit: 75 });
      }

      // Center <-> Westbound (matched by coordinates)
      const westIdx = westX.indexOf(centerX[i]);
      if (westIdx !== -1) {
        this.connectNodes(`node-mw-${westIdx}`, `node-mc-${i}`, { road: 'main', lane: 'transition', speedLimit: 60 });
        this.connectNodes(`node-mc-${i}`, `node-mw-${westIdx}`, { road: 'main', lane: 'transition', speedLimit: 60 });

        // Westbound forward diagonal transition
        if (westIdx < westX.length - 1) {
          const nextCenterIdx = centerX.indexOf(westX[westIdx + 1]);
          if (nextCenterIdx !== -1) {
            this.connectNodes(`node-mw-${westIdx}`, `node-mc-${nextCenterIdx}`, { road: 'main', lane: 'transition', speedLimit: 75 });
            this.connectNodes(`node-mc-${nextCenterIdx}`, `node-mw-${westIdx}`, { road: 'main', lane: 'transition', speedLimit: 75 });
          }
        }
      }
    }

    // 7. Connect Side Road Southbound & Northbound:
    for (let i = 0; i < southY.length - 1; i++) {
      this.connectNodes(`node-ss-${i}`, `node-ss-${i + 1}`, { road: 'side', lane: 'southbound', speedLimit: 60 });
    }
    for (let i = 0; i < northY.length - 1; i++) {
      this.connectNodes(`node-sn-${i}`, `node-sn-${i + 1}`, { road: 'side', lane: 'northbound', speedLimit: 60 });
    }

    // 8. Connect T-Junction Intersecting Segments (Deterministic Turn Arcs):
    // A. Southbound Side Road (node-ss-4: 880, 460) -> Turn Left onto Eastbound Main (node-me-7: 1050, 600) via junction center (node-mc-6: 920, 560)
    this.connectNodes('node-ss-4', 'node-mc-6', { road: 'intersection', lane: 'turn_left', speedLimit: 45 });
    this.connectNodes('node-mc-6', 'node-me-7', { road: 'intersection', lane: 'turn_left', speedLimit: 45 });

    // B. Southbound Side Road (node-ss-4: 880, 460) -> Turn Right onto Westbound Main (node-mw-7: 800, 520)
    const mw800Idx = westX.indexOf(800);
    this.connectNodes('node-ss-4', `node-mw-${mw800Idx}`, { road: 'intersection', lane: 'turn_right', speedLimit: 45 });

    // C. Eastbound Main (node-me-5: 800, 600) -> Turn Left onto Northbound Side Road (node-sn-0: 950, 460) via junction center
    this.connectNodes('node-me-5', 'node-mc-6', { road: 'intersection', lane: 'turn_left', speedLimit: 45 });
    this.connectNodes('node-mc-6', 'node-sn-0', { road: 'intersection', lane: 'turn_left', speedLimit: 45 });

    // D. Westbound Main (node-mw-5: 1050, 520) -> Turn Right onto Northbound Side Road (node-sn-0: 950, 460)
    const mw1050Idx = westX.indexOf(1050);
    this.connectNodes(`node-mw-${mw1050Idx}`, 'node-sn-0', { road: 'intersection', lane: 'turn_right', speedLimit: 45 });

    // 9. Connect Terminal Turnaround Transitions (Closed-loop continuous circulation):
    // East End Turnaround: Eastbound end (node-me-12: 1750, 600) -> Westbound start (node-mw-0: 1750, 520)
    this.connectNodes(`node-me-${eastX.length - 1}`, 'node-mw-0', { road: 'main', lane: 'turnaround', speedLimit: 45 });
    // West End Turnaround: Westbound end (node-mw-12: 50, 520) -> Eastbound start (node-me-0: 50, 600)
    this.connectNodes(`node-mw-${westX.length - 1}`, 'node-me-0', { road: 'main', lane: 'turnaround', speedLimit: 45 });
    // Side Road Top Turnaround: Northbound top (node-sn-4: 950, 50) -> Southbound top (node-ss-0: 880, 50)
    this.connectNodes(`node-sn-${northY.length - 1}`, 'node-ss-0', { road: 'side', lane: 'turnaround', speedLimit: 40 });
  }

  addNode(node) {
    this.nodes.set(node.id, node);
    return node;
  }

  connectNodes(fromId, toId, options = {}) {
    const fromNode = this.nodes.get(fromId);
    const toNode = this.nodes.get(toId);
    if (!fromNode || !toNode) return null;

    const segment = new RoadSegment({
      id: options.id || `seg-${fromId}->${toId}`,
      fromNode,
      toNode,
      road: options.road || fromNode.road,
      lane: options.lane || fromNode.lane,
      speedLimit: options.speedLimit || fromNode.speedLimit,
      isDrivable: options.isDrivable !== undefined ? options.isDrivable : true,
      waypoints: options.waypoints
    });

    this.segments.set(segment.id, segment);
    fromNode.addEdge(segment);
    return segment;
  }

  getGraph() {
    return {
      nodes: Array.from(this.nodes.values()),
      segments: Array.from(this.segments.values()),
      nodeCount: this.nodes.size,
      segmentCount: this.segments.size
    };
  }

  getNearestNode(pos, filter = null) {
    if (!pos || typeof pos.x !== 'number' || typeof pos.y !== 'number') return null;
    let closestNode = null;
    let minDistance = Infinity;

    for (const node of this.nodes.values()) {
      if (filter && typeof filter === 'function' && !filter(node)) continue;
      if (filter && typeof filter === 'object') {
        if (filter.road && node.road !== filter.road) continue;
        if (filter.lane && node.lane !== filter.lane) continue;
      }

      const dist = Math.hypot(node.x - pos.x, node.y - pos.y);
      if (dist < minDistance) {
        minDistance = dist;
        closestNode = node;
      }
    }

    return closestNode ? { node: closestNode, distance: minDistance } : null;
  }

  getNearestSegment(pos, filter = null) {
    if (!pos || typeof pos.x !== 'number' || typeof pos.y !== 'number') return null;
    let closestSeg = null;
    let minDistance = Infinity;
    let bestProj = null;
    let bestT = 0;

    for (const seg of this.segments.values()) {
      if (filter && typeof filter === 'function' && !filter(seg)) continue;
      if (filter && typeof filter === 'object') {
        if (filter.road && seg.road !== filter.road) continue;
        if (filter.lane && seg.lane !== filter.lane) continue;
      }

      const ax = seg.fromNode.x;
      const ay = seg.fromNode.y;
      const bx = seg.toNode.x;
      const by = seg.toNode.y;

      const segLenSq = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
      if (segLenSq === 0) continue;

      // Project point P onto segment AB: t = ((P - A) . (B - A)) / |AB|^2
      let t = ((pos.x - ax) * (bx - ax) + (pos.y - ay) * (by - ay)) / segLenSq;
      t = Math.max(0, Math.min(1, t));

      const projX = ax + t * (bx - ax);
      const projY = ay + t * (by - ay);
      const dist = Math.hypot(pos.x - projX, pos.y - projY);

      if (dist < minDistance) {
        minDistance = dist;
        closestSeg = seg;
        bestProj = { x: projX, y: projY };
        bestT = t;
      }
    }

    return closestSeg ? {
      segment: closestSeg,
      distance: minDistance,
      projectedPoint: bestProj,
      t: bestT
    } : null;
  }

  getConnectedNodes(nodeOrId) {
    const node = typeof nodeOrId === 'string' ? this.nodes.get(nodeOrId) : nodeOrId;
    if (!node || !Array.isArray(node.edges)) return [];
    return node.edges.map(edge => edge.toNode);
  }

  isOnRoad(pos) {
    if (!pos || typeof pos.x !== 'number' || typeof pos.y !== 'number') return false;
    const x = pos.x;
    const y = pos.y;

    // Check main road corridor (including irregular shoulders)
    const onMain = (y >= 460 && y <= 665) && (x >= 0 && x <= 1800);
    // Check side road corridor
    const onSide = (x >= 835 && x <= 1005) && (y >= 0 && y <= 465);

    if (!onMain && !onSide) return false;

    // Reject sidewalk areas
    if (window.StaticCollisionSystem && window.StaticCollisionSystem.isSidewalk(x, y)) {
      return false;
    }

    // Reject buildings and solid obstacles
    if (window.StaticCollisionSystem) {
      const col = window.StaticCollisionSystem.checkSolidCollision(x, y, 6);
      if (col.collided && col.type === 'building') {
        return false;
      }
    }

    return true;
  }

  toggleDebug(forceState) {
    this.showDebug = forceState !== undefined ? forceState : !this.showDebug;
    return this.showDebug;
  }

  renderDebug(ctx, camera) {
    if (!this.showDebug || !ctx) return;

    ctx.save();

    // 1. Draw Drivable Road Boundary Overlay (Subtle emerald outline)
    ctx.strokeStyle = 'rgba(16, 185, 129, 0.55)';
    ctx.lineWidth = 2.5;
    ctx.setLineDash([8, 6]);

    // Main Road boundary
    ctx.beginPath();
    ctx.rect(0, 460, 1800, 205);
    ctx.stroke();

    // Side Road boundary
    ctx.beginPath();
    ctx.rect(835, 0, 170, 465);
    ctx.stroke();
    ctx.setLineDash([]);

    // 2. Draw Directed Segments and Direction Arrows
    for (const seg of this.segments.values()) {
      const from = seg.fromNode;
      const to = seg.toNode;

      let strokeColor = 'rgba(56, 189, 248, 0.65)'; // Default cyan for main road
      if (seg.lane === 'eastbound') strokeColor = 'rgba(16, 185, 129, 0.7)'; // Emerald
      else if (seg.lane === 'westbound') strokeColor = 'rgba(56, 189, 248, 0.7)'; // Cyan
      else if (seg.lane === 'center') strokeColor = 'rgba(234, 179, 8, 0.5)'; // Amber
      else if (seg.lane === 'southbound' || seg.lane === 'northbound') strokeColor = 'rgba(249, 115, 22, 0.75)'; // Orange
      else if (seg.road === 'intersection') strokeColor = 'rgba(168, 85, 247, 0.85)'; // Purple for junction turns

      ctx.strokeStyle = strokeColor;
      ctx.lineWidth = seg.road === 'intersection' ? 2.5 : 1.8;

      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();

      // Draw direction arrowhead at mid-point
      const midX = (from.x + to.x) / 2;
      const midY = (from.y + to.y) / 2;
      const arrowLen = 9;
      const arrowAngle = 0.45;

      ctx.fillStyle = strokeColor;
      ctx.beginPath();
      ctx.moveTo(midX, midY);
      ctx.lineTo(
        midX - arrowLen * Math.cos(seg.heading - arrowAngle),
        midY - arrowLen * Math.sin(seg.heading - arrowAngle)
      );
      ctx.lineTo(
        midX - arrowLen * Math.cos(seg.heading + arrowAngle),
        midY - arrowLen * Math.sin(seg.heading + arrowAngle)
      );
      ctx.closePath();
      ctx.fill();
    }

    // 3. Draw Road Nodes
    for (const node of this.nodes.values()) {
      let nodeColor = '#38bdf8';
      if (node.lane === 'eastbound') nodeColor = '#10b981';
      else if (node.lane === 'westbound') nodeColor = '#06b6d4';
      else if (node.lane === 'center') nodeColor = '#eab308';
      else if (node.lane === 'southbound' || node.lane === 'northbound') nodeColor = '#f97316';
      if (node.tags.includes('destination')) nodeColor = '#ec4899'; // Pink for destination node

      // Outer glow circle
      ctx.fillStyle = nodeColor;
      ctx.beginPath();
      ctx.arc(node.x, node.y, 4.5, 0, Math.PI * 2);
      ctx.fill();

      ctx.strokeStyle = '#0f172a';
      ctx.lineWidth = 1.2;
      ctx.stroke();

      // Small node label
      ctx.fillStyle = '#f8fafc';
      ctx.font = '9px monospace';
      ctx.fillText(node.id.replace('node-', ''), node.x + 6, node.y - 4);
    }

    ctx.restore();
  }
}

// Global Singleton Road Network Instance
const RoadNetworkSystem = new RoadNetworkGraph(EnvironmentData);
window.RoadNetworkSystem = RoadNetworkSystem;
window.RoadNode = RoadNode;
window.RoadSegment = RoadSegment;
window.RoadNetworkGraph = RoadNetworkGraph;

// Global Accessor Functions
window.getRoadGraph = () => RoadNetworkSystem.getGraph();
window.getNearestRoadNode = (pos, filter) => RoadNetworkSystem.getNearestNode(pos, filter);
window.getNearestRoadSegment = (pos, filter) => RoadNetworkSystem.getNearestSegment(pos, filter);
window.getConnectedRoadNodes = (node) => RoadNetworkSystem.getConnectedNodes(node);
window.isOnRoad = (pos) => RoadNetworkSystem.isOnRoad(pos);

/* ============================================================================
   3C. GLOBAL ROUTE PLANNER (A* SEARCH OVER ROAD NETWORK)
   ============================================================================ */

/**
 * Global Route Planner Class
 * Computes deterministic, topologically valid global routes from ego vehicle to destination
 * using A* Search across the RoadNetworkGraph.
 */
class GlobalRoutePlanner {
  constructor(roadNetwork) {
    this.network = roadNetwork || (typeof RoadNetworkSystem !== 'undefined' ? RoadNetworkSystem : null);
    this.currentRoute = null;
    this.blockedLocations = [];
    this.replanThreshold = 50; // Replan if ego drifts > 50px from route polyline
    this.lastEgoPos = { x: 0, y: 0 };
    this.activeWaypointIndex = 0;

    // Immediately compute initial route from default start (140, 580, 0) to default destination (1650, 560)
    if (this.network && typeof EnvironmentData !== 'undefined' && EnvironmentData.destination) {
      this.planRoute({ x: 140, y: 580, heading: 0 }, EnvironmentData.destination);
    }
  }

  /**
   * Mark a temporary/persistent failed route location to force alternate routing
   */
  markBlockedLocation(x, y, radius = 45, durationMs = 15000) {
    if (!this.blockedLocations) this.blockedLocations = [];
    this.blockedLocations.push({
      x,
      y,
      radius,
      expiry: Date.now() + durationMs
    });
  }

  /**
   * Smooth corner transitions with C1 Bezier fillets for progressive curve steering
   */
  smoothCornerWaypoints(rawWps) {
    if (!rawWps || rawWps.length < 3) return rawWps;
    const smoothed = [rawWps[0]];

    for (let i = 1; i < rawWps.length - 1; i++) {
      const prev = rawWps[i - 1];
      const curr = rawWps[i];
      const next = rawWps[i + 1];

      const d1x = curr.x - prev.x;
      const d1y = curr.y - prev.y;
      const len1 = Math.hypot(d1x, d1y);

      const d2x = next.x - curr.x;
      const d2y = next.y - curr.y;
      const len2 = Math.hypot(d2x, d2y);

      if (len1 < 10 || len2 < 10) {
        smoothed.push(curr);
        continue;
      }

      const h1 = Math.atan2(d1y, d1x);
      const h2 = Math.atan2(d2y, d2x);
      let diff = Math.abs(h2 - h1);
      while (diff > Math.PI) diff -= Math.PI * 2;
      diff = Math.abs(diff);

      if (diff > 0.18) {
        // Corner fillet smoothing
        const filletDist = Math.min(32, len1 * 0.35, len2 * 0.35);
        const startX = curr.x - (d1x / len1) * filletDist;
        const startY = curr.y - (d1y / len1) * filletDist;
        const endX = curr.x + (d2x / len2) * filletDist;
        const endY = curr.y + (d2y / len2) * filletDist;

        for (let t = 0.25; t <= 0.75; t += 0.25) {
          const u = 1 - t;
          const bx = u * u * startX + 2 * u * t * curr.x + t * t * endX;
          const by = u * u * startY + 2 * u * t * curr.y + t * t * endY;
          smoothed.push({
            x: bx,
            y: by,
            road: curr.road,
            lane: curr.lane,
            heading: Math.atan2(endY - startY, endX - startX),
            speedLimit: curr.speedLimit,
            nodeId: curr.nodeId
          });
        }
      } else {
        smoothed.push(curr);
      }
    }

    smoothed.push(rawWps[rawWps.length - 1]);
    return smoothed;
  }

  /**
   * Plan an optimal global route from start to destination using A* Search
   * @param {Object} startPos - { x, y, heading }
   * @param {Object} destPos - { x, y }
   * @param {Object} [options] - Additional routing constraints
   * @returns {Object|null} Planned route object
   */
  planRoute(startPos, destPos, options = {}) {
    if (!this.network || !startPos || !destPos) return null;

    // 1. Find nearest valid unobstructed road nodes for start and destination
    let startNode = null;
    if (startPos.heading !== undefined) {
      // Find candidate nodes that are clear of static obstacles and align with vehicle heading
      const candidates = Array.from(this.network.nodes.values()).filter(node => {
        if (typeof StaticCollisionSystem !== 'undefined') {
          if (StaticCollisionSystem.checkSolidCollision(node.x, node.y, 16).collided) return false;
        }
        return true;
      }).map(node => {
        const dist = Math.hypot(node.x - startPos.x, node.y - startPos.y);
        const angleToNode = Math.atan2(node.y - startPos.y, node.x - startPos.x);
        let headingDiff = Math.abs(angleToNode - startPos.heading);
        while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
        headingDiff = Math.abs(headingDiff);

        // Moderate penalty for nodes behind vehicle
        const forwardPenalty = headingDiff > (Math.PI / 2 + 0.2) ? 120 : 0;
        return { node, score: dist + forwardPenalty, distance: dist };
      }).sort((a, b) => a.score - b.score);

      startNode = candidates.length > 0 ? candidates[0].node : null;
    }

    if (!startNode) {
      const nearestStart = this.network.getNearestNode(startPos, (node) => {
        if (typeof StaticCollisionSystem !== 'undefined') {
          return !StaticCollisionSystem.checkSolidCollision(node.x, node.y, 16).collided;
        }
        return true;
      });
      startNode = nearestStart ? nearestStart.node : null;
    }

    const nearestGoal = this.network.getNearestNode(destPos, (node) => {
      if (typeof StaticCollisionSystem !== 'undefined') {
        return !StaticCollisionSystem.checkSolidCollision(node.x, node.y, 16).collided;
      }
      return true;
    });
    const goalNode = nearestGoal ? nearestGoal.node : null;

    if (!startNode || !goalNode) {
      console.warn('[GlobalRoutePlanner] Could not find start or goal road nodes.');
      return null;
    }

    // 2. Execute A* Search on the directed Road Network graph
    const openSet = new Set([startNode.id]);
    const closedSet = new Set();
    const cameFrom = new Map(); // node.id -> { fromNode, edge }

    const gScore = new Map(); // node.id -> cost from start
    const fScore = new Map(); // node.id -> estimated total cost

    this.network.nodes.forEach(node => {
      gScore.set(node.id, Infinity);
      fScore.set(node.id, Infinity);
    });

    gScore.set(startNode.id, 0);
    fScore.set(startNode.id, this.heuristic(startNode, goalNode));

    let foundGoal = false;

    while (openSet.size > 0) {
      // Find node in openSet with lowest fScore
      let currentId = null;
      let lowestF = Infinity;

      for (const id of openSet) {
        const f = fScore.get(id);
        if (f < lowestF) {
          lowestF = f;
          currentId = id;
        }
      }

      if (currentId === goalNode.id) {
        foundGoal = true;
        break;
      }

      openSet.delete(currentId);
      closedSet.add(currentId);

      const currentNode = this.network.nodes.get(currentId);
      if (!currentNode || !Array.isArray(currentNode.edges)) continue;

      // Expand outgoing directed edges
      for (const edge of currentNode.edges) {
        if (!edge.isDrivable) continue;
        const neighbor = edge.toNode;
        if (!neighbor || closedSet.has(neighbor.id)) continue;

        // Check if edge intersects static solid obstacles (buildings, debris, parked cars)
        if (typeof StaticCollisionSystem !== 'undefined') {
          let edgeBlocked = false;
          const samples = 6;
          for (let s = 0; s <= samples; s++) {
            const sx = currentNode.x + (s / samples) * (neighbor.x - currentNode.x);
            const sy = currentNode.y + (s / samples) * (neighbor.y - currentNode.y);
            if (StaticCollisionSystem.checkSolidCollision(sx, sy, 15).collided) {
              edgeBlocked = true;
              break;
            }
          }
          if (edgeBlocked) continue; // Skip edge obstructed by static obstacle
        }

        // Base edge cost = edge length
        let edgeCost = edge.length;

        // Check pothole hazard on edge: add penalty to prefer clean tarmac corridors
        if (typeof StaticCollisionSystem !== 'undefined') {
          const midX = (currentNode.x + neighbor.x) / 2;
          const midY = (currentNode.y + neighbor.y) / 2;
          if (StaticCollisionSystem.checkPothole(midX, midY, 18).inPothole) {
            edgeCost += 500;
          }
        }

        // Check persistent/temporary blocked locations from recovery system
        if (this.blockedLocations && this.blockedLocations.length > 0) {
          const now = Date.now();
          for (const b of this.blockedLocations) {
            if (b.expiry > now) {
              const dNode = Math.hypot(neighbor.x - b.x, neighbor.y - b.y);
              const dMid = Math.hypot(((currentNode.x + neighbor.x) / 2) - b.x, ((currentNode.y + neighbor.y) / 2) - b.y);
              if (dNode < (b.radius || 45) || dMid < (b.radius || 45)) {
                edgeCost += 5000;
                break;
              }
            }
          }
        }

        // Preference weighting:
        // - Main flow lanes preferred over lateral transitions
        if (edge.lane === 'transition') {
          edgeCost *= 1.25;
        }
        // - Junction turns speed adjusted
        if (edge.road === 'intersection') {
          edgeCost *= 1.1;
        }

        const tentativeG = gScore.get(currentId) + edgeCost;

        if (!openSet.has(neighbor.id)) {
          openSet.add(neighbor.id);
        } else if (tentativeG >= gScore.get(neighbor.id)) {
          continue; // Not a better path
        }

        cameFrom.set(neighbor.id, { fromNode: currentNode, edge: edge });
        gScore.set(neighbor.id, tentativeG);
        fScore.set(neighbor.id, tentativeG + this.heuristic(neighbor, goalNode));
      }
    }

    if (!foundGoal && startNode.id !== goalNode.id) {
      console.warn('[GlobalRoutePlanner] No connected path found between', startNode.id, 'and', goalNode.id);
      return null;
    }

    // 3. Reconstruct Route Path
    const pathNodes = [];
    const pathEdges = [];
    let curr = goalNode.id;

    pathNodes.unshift(goalNode);

    while (cameFrom.has(curr)) {
      const step = cameFrom.get(curr);
      pathEdges.unshift(step.edge);
      pathNodes.unshift(step.fromNode);
      curr = step.fromNode.id;
    }

    // 4. Generate Ordered Waypoint Sequence
    const rawWaypoints = [];
    let totalDist = 0;

    // Add start position as initial waypoint
    rawWaypoints.push({
      x: startPos.x,
      y: startPos.y,
      road: startNode.road,
      lane: startNode.lane,
      heading: startNode.heading,
      speedLimit: startNode.speedLimit,
      nodeId: startNode.id
    });

    for (let i = 0; i < pathNodes.length; i++) {
      const node = pathNodes[i];
      rawWaypoints.push({
        x: node.x,
        y: node.y,
        road: node.road,
        lane: node.lane,
        heading: node.heading,
        speedLimit: node.speedLimit,
        nodeId: node.id
      });

      if (i > 0) {
        totalDist += Math.hypot(node.x - pathNodes[i - 1].x, node.y - pathNodes[i - 1].y);
      }
    }

    // Add final destination marker waypoint
    rawWaypoints.push({
      x: destPos.x,
      y: destPos.y,
      road: goalNode.road,
      lane: goalNode.lane,
      heading: goalNode.heading,
      speedLimit: 0,
      nodeId: 'destination-goal'
    });
    totalDist += Math.hypot(destPos.x - goalNode.x, destPos.y - goalNode.y);

    // Apply C1 Bezier corner fillet smoothing to eliminate sharp waypoint corners
    const waypoints = this.smoothCornerWaypoints(rawWaypoints);

    const route = {
      startNode,
      goalNode,
      nodes: pathNodes,
      segments: pathEdges,
      rawWaypoints,
      waypoints,
      totalDistance: totalDist,
      estimatedTime: totalDist / 85, // Estimated seconds at average cruising speed
      status: 'FOUND',
      timestamp: Date.now()
    };

    this.currentRoute = route;
    this.activeWaypointIndex = 0;
    return route;
  }

  /**
   * Euclidean distance heuristic for A*
   */
  heuristic(nodeA, nodeB) {
    return Math.hypot(nodeB.x - nodeA.x, nodeB.y - nodeA.y);
  }

  /**
   * Continuous update / Re-routing / Recovery check
   */
  update(ego, destination) {
    if (!ego || !destination) return this.currentRoute;

    // Check if initial route exists
    if (!this.currentRoute || this.currentRoute.status !== 'FOUND') {
      return this.planRoute(ego, destination);
    }

    // Check distance to current route polyline
    const distToRoute = this.getDistanceToRoute(ego);

    // If vehicle has drifted off-route or is too far (> replanThreshold), recover by replanning
    if (distToRoute > this.replanThreshold) {
      return this.planRoute(ego, destination);
    }

    // Advance active waypoint index
    if (this.currentRoute.waypoints && this.currentRoute.waypoints.length > 0) {
      while (this.activeWaypointIndex < this.currentRoute.waypoints.length - 1) {
        const wpCurr = this.currentRoute.waypoints[this.activeWaypointIndex];
        const wpNext = this.currentRoute.waypoints[this.activeWaypointIndex + 1];

        const distToWp = Math.hypot(wpCurr.x - ego.x, wpCurr.y - ego.y);

        // Vector from curr to next waypoint
        const segDx = wpNext.x - wpCurr.x;
        const segDy = wpNext.y - wpCurr.y;
        const segLenSq = segDx * segDx + segDy * segDy;

        let passed = false;
        if (segLenSq > 0) {
          const t = ((ego.x - wpCurr.x) * segDx + (ego.y - wpCurr.y) * segDy) / segLenSq;
          if (t > 0.4) passed = true; // Passed towards next waypoint
        }

        if (distToWp < 45 || passed) {
          this.activeWaypointIndex++;
        } else {
          break;
        }
      }
    }

    return this.currentRoute;
  }

  /**
   * Calculate minimum distance from vehicle position to current route polyline
   */
  getDistanceToRoute(pos) {
    if (!this.currentRoute || !this.currentRoute.waypoints || this.currentRoute.waypoints.length < 2) {
      return Infinity;
    }

    let minDistance = Infinity;
    const wps = this.currentRoute.waypoints;

    for (let i = 0; i < wps.length - 1; i++) {
      const p1 = wps[i];
      const p2 = wps[i + 1];

      const dx = p2.x - p1.x;
      const dy = p2.y - p1.y;
      const lenSq = dx * dx + dy * dy;

      if (lenSq === 0) {
        const d = Math.hypot(pos.x - p1.x, pos.y - p1.y);
        if (d < minDistance) minDistance = d;
        continue;
      }

      let t = ((pos.x - p1.x) * dx + (pos.y - p1.y) * dy) / lenSq;
      t = Math.max(0, Math.min(1, t));

      const projX = p1.x + t * dx;
      const projY = p1.y + t * dy;
      const d = Math.hypot(pos.x - projX, pos.y - projY);

      if (d < minDistance) {
        minDistance = d;
      }
    }

    return minDistance;
  }

  /**
   * Computes comprehensive route-following navigation target telemetry:
   * Finds the active segment on the global route, calculates signed cross-track error,
   * traces forward along the polyline to find the lookahead target point,
   * detects upcoming turns / intersections, and calculates progress.
   *
   * @param {Object} ego - { x, y, heading, speed }
   * @param {number} [lookaheadDistance=50] - Forward lookahead distance (px)
   * @returns {Object} Navigation target & telemetry
   */
  getNavigationTarget(ego, lookaheadDistance = 50) {
    if (!this.currentRoute || !this.currentRoute.waypoints || this.currentRoute.waypoints.length < 2 || !ego) {
      return {
        hasRoute: false,
        targetLookahead: { x: ego ? ego.x + 50 : 0, y: ego ? ego.y : 0 },
        targetHeading: ego ? ego.heading : 0,
        segmentHeading: ego ? ego.heading : 0,
        crossTrackError: 0,
        progressPercent: 0,
        distanceToGoal: 0,
        distanceRemainingAlongRoute: 0,
        navState: 'NO_ROUTE',
        currentNode: null,
        targetWaypoint: null,
        isTurn: false,
        targetSpeedLimit: 90
      };
    }

    const wps = this.currentRoute.waypoints;
    const n = wps.length;

    // 1. Find closest segment on the route polyline
    let minDistance = Infinity;
    let closestSegIdx = 0;
    let closestProj = { x: wps[0].x, y: wps[0].y };

    for (let i = 0; i < n - 1; i++) {
      const p1 = wps[i];
      const p2 = wps[i + 1];

      const dx = p2.x - p1.x;
      const dy = p2.y - p1.y;
      const lenSq = dx * dx + dy * dy;

      if (lenSq === 0) continue;

      let t = ((ego.x - p1.x) * dx + (ego.y - p1.y) * dy) / lenSq;
      t = Math.max(0, Math.min(1, t));

      const projX = p1.x + t * dx;
      const projY = p1.y + t * dy;
      const d = Math.hypot(ego.x - projX, ego.y - projY);

      if (d < minDistance) {
        minDistance = d;
        closestSegIdx = i;
        closestProj = { x: projX, y: projY };
      }
    }

    // 2. Calculate signed cross-track error
    const segP1 = wps[closestSegIdx];
    const segP2 = wps[closestSegIdx + 1];
    const segDx = segP2.x - segP1.x;
    const segDy = segP2.y - segP1.y;
    const segLen = Math.hypot(segDx, segDy) || 1;
    const segHeading = Math.atan2(segDy, segDx);

    // Normal vector pointing to the left of the segment
    const normX = -segDy / segLen;
    const normY = segDx / segLen;

    // Signed cross-track error: positive = left of route, negative = right of route
    const crossTrackError = (ego.x - closestProj.x) * normX + (ego.y - closestProj.y) * normY;

    // 3. Trace forward along polyline from closestProj by lookaheadDistance
    let remainingLookahead = lookaheadDistance;
    let currIdx = closestSegIdx;
    let currPt = { x: closestProj.x, y: closestProj.y };
    let lookaheadPt = { x: segP2.x, y: segP2.y };
    let lookaheadHeading = segHeading;
    let lookaheadSpeedLimit = segP2.speedLimit || 90;

    while (currIdx < n - 1 && remainingLookahead > 0) {
      const nextWp = wps[currIdx + 1];
      const distToNext = Math.hypot(nextWp.x - currPt.x, nextWp.y - currPt.y);

      if (remainingLookahead <= distToNext) {
        const ratio = remainingLookahead / (distToNext || 1);
        lookaheadPt = {
          x: currPt.x + ratio * (nextWp.x - currPt.x),
          y: currPt.y + ratio * (nextWp.y - currPt.y)
        };
        lookaheadHeading = Math.atan2(nextWp.y - currPt.y, nextWp.x - currPt.x);
        lookaheadSpeedLimit = nextWp.speedLimit || 90;
        remainingLookahead = 0;
        break;
      } else {
        remainingLookahead -= distToNext;
        currPt = { x: nextWp.x, y: nextWp.y };
        currIdx++;
        if (currIdx < n - 1) {
          const nextNext = wps[currIdx + 1];
          lookaheadHeading = Math.atan2(nextNext.y - nextWp.y, nextNext.x - nextWp.x);
          lookaheadSpeedLimit = nextNext.speedLimit || 90;
        }
      }
    }

    if (remainingLookahead > 0) {
      lookaheadPt = { x: wps[n - 1].x, y: wps[n - 1].y };
      lookaheadHeading = wps[n - 1].heading || segHeading;
    }

    // 4. Calculate remaining distance along route to destination
    let distRemaining = Math.hypot(wps[closestSegIdx + 1].x - closestProj.x, wps[closestSegIdx + 1].y - closestProj.y);
    for (let i = closestSegIdx + 1; i < n - 1; i++) {
      distRemaining += Math.hypot(wps[i + 1].x - wps[i].x, wps[i + 1].y - wps[i].y);
    }
    const directDistToGoal = Math.hypot(wps[n - 1].x - ego.x, wps[n - 1].y - ego.y);

    const totalDist = this.currentRoute.totalDistance || 1;
    const progressDone = Math.max(0, totalDist - distRemaining);
    const progressPercent = Math.min(100, Math.max(0, (progressDone / totalDist) * 100));

    // 5. Detect Turns / Heading Changes ahead
    let headingDiff = Math.abs(lookaheadHeading - segHeading);
    while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
    headingDiff = Math.abs(headingDiff);

    const isTurn = (headingDiff > 0.35) || (segP1.road === 'intersection') || (segP2.road === 'intersection');

    // 6. Determine Navigation State
    let navState = 'TRACKING';
    if (directDistToGoal <= 45 || distRemaining <= 35) {
      navState = 'ARRIVED';
    } else if (directDistToGoal <= 140 || distRemaining <= 130) {
      navState = 'ARRIVING';
    } else if (Math.abs(crossTrackError) > 35) {
      navState = 'RECOVERING';
    } else if (isTurn) {
      navState = headingDiff > 0.6 ? 'TURNING' : 'APPROACHING_TURN';
    }

    const activeNode = wps[closestSegIdx].nodeId || `wp-${closestSegIdx}`;
    const targetWpNode = wps[Math.min(closestSegIdx + 1, n - 1)].nodeId || `wp-${closestSegIdx + 1}`;

    this.lastNavTarget = {
      hasRoute: true,
      currentNode: activeNode,
      targetWaypoint: targetWpNode,
      targetLookahead: lookaheadPt,
      targetHeading: lookaheadHeading,
      segmentHeading: segHeading,
      crossTrackError: crossTrackError,
      progressPercent: Math.round(progressPercent),
      distanceToGoal: Math.round(directDistToGoal),
      distanceRemainingAlongRoute: Math.round(distRemaining),
      navState: navState,
      isTurn: isTurn,
      targetSpeedLimit: lookaheadSpeedLimit,
      closestSegmentIndex: closestSegIdx
    };

    return this.lastNavTarget;
  }

  getRoute() {
    return this.currentRoute;
  }

  getNavTelemetry() {
    return this.lastNavTarget || {
      hasRoute: false,
      currentNode: '--',
      targetWaypoint: '--',
      progressPercent: 0,
      distanceToGoal: 0,
      navState: 'IDLE'
    };
  }

  toggleDebug(forceState) {
    this.showDebug = forceState !== undefined ? forceState : !this.showDebug;
    return this.showDebug;
  }

  renderDebug(ctx, camera) {
    if (!this.showDebug || !this.currentRoute || !this.currentRoute.waypoints || this.currentRoute.waypoints.length < 2 || !ctx) {
      return;
    }

    ctx.save();

    const wps = this.currentRoute.waypoints;

    // 1. Draw glowing background corridor for the route
    ctx.strokeStyle = 'rgba(234, 179, 8, 0.28)'; // Amber glow
    ctx.lineWidth = 14;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    ctx.beginPath();
    ctx.moveTo(wps[0].x, wps[0].y);
    for (let i = 1; i < wps.length; i++) {
      ctx.lineTo(wps[i].x, wps[i].y);
    }
    ctx.stroke();

    // 2. Draw solid sharp golden route line
    ctx.strokeStyle = '#eab308'; // Bright Gold
    ctx.lineWidth = 3.5;
    ctx.setLineDash([12, 6]);

    ctx.beginPath();
    ctx.moveTo(wps[0].x, wps[0].y);
    for (let i = 1; i < wps.length; i++) {
      ctx.lineTo(wps[i].x, wps[i].y);
    }
    ctx.stroke();
    ctx.setLineDash([]);

    // 3. Draw route waypoint nodes
    for (let i = 0; i < wps.length; i++) {
      const wp = wps[i];
      const isStart = (i === 0);
      const isGoal = (i === wps.length - 1);
      const isActive = (i === this.activeWaypointIndex);

      ctx.beginPath();
      ctx.arc(wp.x, wp.y, isStart || isGoal ? 6 : (isActive ? 5 : 3.5), 0, Math.PI * 2);

      if (isStart) {
        ctx.fillStyle = '#10b981'; // Green for route start
      } else if (isGoal) {
        ctx.fillStyle = '#ec4899'; // Pink for route goal
      } else if (isActive) {
        ctx.fillStyle = '#38bdf8'; // Cyan for current target
      } else {
        ctx.fillStyle = '#fbbf24'; // Amber for route waypoints
      }
      ctx.fill();

      ctx.strokeStyle = '#0f172a';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // 4. Render Active Pure Pursuit Lookahead Reticle & Leader
    if (this.lastNavTarget && this.lastNavTarget.targetLookahead) {
      const tgt = this.lastNavTarget.targetLookahead;

      // Cyan Glowing Target Lookahead Crosshair
      ctx.save();
      ctx.strokeStyle = '#06b6d4'; // Cyan
      ctx.fillStyle = 'rgba(6, 182, 212, 0.35)';
      ctx.lineWidth = 2;
      ctx.shadowColor = '#06b6d4';
      ctx.shadowBlur = 8;

      ctx.beginPath();
      ctx.arc(tgt.x, tgt.y, 8, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Crosshairs
      ctx.beginPath();
      ctx.moveTo(tgt.x - 12, tgt.y);
      ctx.lineTo(tgt.x + 12, tgt.y);
      ctx.moveTo(tgt.x, tgt.y - 12);
      ctx.lineTo(tgt.x, tgt.y + 12);
      ctx.stroke();

      ctx.restore();
    }

    ctx.restore();
  }
}

// Global Singleton Global Route Planner Instance
const GlobalRouteSystem = new GlobalRoutePlanner(RoadNetworkSystem);
window.GlobalRoutePlanner = GlobalRoutePlanner;
window.GlobalRouteSystem = GlobalRouteSystem;
window.getGlobalRoute = () => GlobalRouteSystem.getRoute();

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
    this.maxSteeringRate = 2.8;   // Configurable steering actuator angular rate limit (rad/s)
    this.steeringSpeed = 3.2;     // Steering rate (rad/s)
    this.steeringReturnSpeed = 4.5; // Self-centering rate (rad/s)
    this.angularVelocity = 0;     // Heading angular velocity (rad/s)
    this.lateralVelocity = 0;     // Lateral velocity (px/s)

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
    this.angularVelocity = 0;
    this.lateralVelocity = 0;
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

    // --- 1. Steering Actuator Dynamics (First-Order Rate Limiter) ---
    // Continuous steering target [-1, 1] mapped to physical steering angle limit
    const targetSteerAngle = Math.max(-this.maxSteeringAngle, Math.min(this.maxSteeringAngle, this.inputs.steer * this.maxSteeringAngle));
    const maxDelta = this.maxSteeringRate * dt;
    const steerDiff = targetSteerAngle - this.steeringAngle;
    const clampedDelta = Math.max(-maxDelta, Math.min(maxDelta, steerDiff));
    this.steeringAngle += clampedDelta;
    this.steeringAngle = Math.max(-this.maxSteeringAngle, Math.min(this.maxSteeringAngle, this.steeringAngle));

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
 * Separate Traffic Agent Navigation System for non-ego road vehicles
 * (Cars, Motorcycles/Bikes, Auto-Rickshaws)
 *
 * Architecture:
 * TRAFFIC DESTINATION -> GLOBAL ROUTE -> WAYPOINT FOLLOWING -> BASIC FOLLOWING -> DESTINATION
 */
class TrafficAgentNavigator {
  constructor(vehicle, roadNetwork) {
    this.vehicle = vehicle;
    this.network = roadNetwork || (typeof window !== 'undefined' && window.RoadNetworkSystem ? window.RoadNetworkSystem : (typeof RoadNetworkSystem !== 'undefined' ? RoadNetworkSystem : null));
    
    this.destination = null;
    this.route = null;
    this.routeWaypoints = [];
    this.currentWaypointIndex = 0;
    this.navState = 'CRUISING'; // 'CRUISING', 'FOLLOWING', 'YIELDING', 'AVOIDING', 'OVERTAKING', 'STOPPED'
    
    // Lookahead, headway, and dynamics parameters
    this.lookaheadDistance = (vehicle.type === 'motorcycle') ? 32 : 42;
    this.waypointTolerance = 26;
    this.minSafeDistance = Math.max(30, (vehicle.length || 44) * 0.75 + 12);
    this.timeHeadway = 1.3; // seconds
    this.maxTurnRate = (vehicle.type === 'motorcycle') ? 4.5 : 3.2; // rad/s
    this.maxAccel = (vehicle.type === 'motorcycle') ? 180 : 120; // px/s^2
    this.maxDecel = 280; // px/s^2

    // Interaction & Coordination State
    this.leadVehicle = null;
    this.leadDistance = Infinity;
    this.yieldWaitTimer = 0;
    this.stuckTimer = 0;
    this.overtakeState = 'NONE'; // 'NONE', 'PASSING', 'RETURNING'
    this.overtakeTargetLaneY = 0;
    this.nominalLaneY = vehicle.y || 600;
    this.overtakeTimer = 0;

    // Initialize initial destination & route on the road graph
    this.initDestinationAndRoute();
  }

  /**
   * Determine initial destination based on vehicle position and road orientation
   */
  initDestinationAndRoute() {
    if (!this.network) return;
    const v = this.vehicle;

    if (v.route && v.route.road === 'side') {
      // Side road vehicle heading South: destination is Eastbound main road
      this.destination = { x: 1750, y: 600, road: 'main', lane: 'eastbound' };
      this.nominalLaneY = 600;
    } else {
      const isEastbound = (Math.abs(v.heading) < 0.8 || (v.heading > -0.8 && v.heading < 0.8));
      if (isEastbound) {
        this.destination = { x: 1750, y: 600, road: 'main', lane: 'eastbound' };
        this.nominalLaneY = 600;
      } else {
        this.destination = { x: 50, y: 520, road: 'main', lane: 'westbound' };
        this.nominalLaneY = 520;
      }
    }

    this.planRouteTo(this.destination);
  }

  /**
   * Plan route from current position to destination using RoadNetworkSystem
   */
  planRouteTo(destPos) {
    if (!this.network || !destPos) return null;
    const planner = new GlobalRoutePlanner(this.network);
    const startPos = { x: this.vehicle.x, y: this.vehicle.y, heading: this.vehicle.heading };
    const routeObj = planner.planRoute(startPos, destPos);

    if (routeObj && Array.isArray(routeObj.waypoints) && routeObj.waypoints.length > 0) {
      this.route = routeObj;
      this.routeWaypoints = routeObj.waypoints;
      this.currentWaypointIndex = 0;
      this.destination = destPos;
      return this.route;
    }
    return null;
  }

  /**
   * Select next destination when current destination is reached
   */
  onReachedDestination() {
    const v = this.vehicle;
    this.overtakeState = 'NONE';
    this.overtakeTimer = 0;

    if (v.x >= 1680) {
      // At East end of main road -> smoothly transition to Westbound lane (node-mw-0 -> node-mw-12)
      this.destination = { x: 50, y: 520, road: 'main', lane: 'westbound' };
      this.nominalLaneY = 520;
    } else if (v.x <= 120) {
      // At West end of main road -> smoothly transition to Eastbound lane (node-me-0 -> node-me-12)
      this.destination = { x: 1750, y: 600, road: 'main', lane: 'eastbound' };
      this.nominalLaneY = 600;
    } else if (v.y <= 90 && (v.x >= 840 && v.x <= 980)) {
      // At top of side road -> transition to Southbound lane towards main road
      this.destination = { x: 1750, y: 600, road: 'main', lane: 'eastbound' };
      this.nominalLaneY = 600;
    } else {
      // Default to Eastbound
      this.destination = { x: 1750, y: 600, road: 'main', lane: 'eastbound' };
      this.nominalLaneY = 600;
    }

    this.planRouteTo(this.destination);
  }

  /**
   * Continuous navigation update loop:
   * Waypoint following + Intersection Yielding + Queue Headway + Safe Overtaking + Boundary Control
   */
  update(dt, egoVehicle, allEntities, environmentData) {
    if (!this.vehicle.active) return;
    if (dt <= 0 || dt > 0.1) dt = 0.016;

    const v = this.vehicle;

    // 1. Ensure a valid active route exists
    if (!this.routeWaypoints || this.routeWaypoints.length === 0 || this.currentWaypointIndex >= this.routeWaypoints.length) {
      this.onReachedDestination();
    }
    if (!this.routeWaypoints || this.routeWaypoints.length === 0) return;

    // 2. Advance Waypoint Progress
    let currWp = this.routeWaypoints[this.currentWaypointIndex];
    let distToWp = Math.hypot(currWp.x - v.x, currWp.y - v.y);

    while (distToWp < this.waypointTolerance && this.currentWaypointIndex < this.routeWaypoints.length - 1) {
      this.currentWaypointIndex++;
      currWp = this.routeWaypoints[this.currentWaypointIndex];
      distToWp = Math.hypot(currWp.x - v.x, currWp.y - v.y);
    }

    // Check destination arrival
    const lastWp = this.routeWaypoints[this.routeWaypoints.length - 1];
    const distToEnd = Math.hypot(lastWp.x - v.x, lastWp.y - v.y);
    if (this.currentWaypointIndex >= this.routeWaypoints.length - 1 && distToEnd < 35) {
      this.onReachedDestination();
      return;
    }

    // 3. Pure Pursuit Lookahead Target
    let lookaheadTarget = currWp;
    for (let i = this.currentWaypointIndex; i < this.routeWaypoints.length; i++) {
      const wp = this.routeWaypoints[i];
      const d = Math.hypot(wp.x - v.x, wp.y - v.y);
      if (d >= this.lookaheadDistance) {
        lookaheadTarget = wp;
        break;
      }
      lookaheadTarget = wp;
    }

    // Apply active overtaking lateral target if passing
    let targetX = lookaheadTarget.x;
    let targetY = lookaheadTarget.y;
    if (this.overtakeState === 'PASSING') {
      targetY = this.overtakeTargetLaneY;
    }

    const targetHeading = Math.atan2(targetY - v.y, targetX - v.x);
    let headingDiff = targetHeading - v.heading;
    while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
    while (headingDiff < -Math.PI) headingDiff += Math.PI * 2;

    // 4. Surroundings Perception (Ego & Other Dynamic Entities)
    const cosH = Math.cos(v.heading);
    const sinH = Math.sin(v.heading);

    const candidates = [];
    if (egoVehicle) candidates.push(egoVehicle);
    if (Array.isArray(allEntities)) {
      for (const ent of allEntities) {
        if (ent.id !== v.id && ent.active) candidates.push(ent);
      }
    }

    this.leadVehicle = null;
    this.leadDistance = Infinity;

    for (const cand of candidates) {
      // Skip irrelevant sidewalk pedestrians
      if ((cand.type === 'pedestrian' || cand.type === 'animal') && window.StaticCollisionSystem) {
        if (window.StaticCollisionSystem.isSidewalk(cand.x, cand.y) && !window.StaticCollisionSystem.isMovingTowardRoad(cand.x, cand.y, cand.heading || 0, cand.speed || 0)) {
          continue;
        }
      }

      const dx = cand.x - v.x;
      const dy = cand.y - v.y;
      const fwd = dx * cosH + dy * sinH;
      const lat = Math.abs(dy * cosH - dx * sinH);
      const candRadius = cand.radius || 14;

      // In vehicle forward travel corridor and travelling in same direction
      let headingDiffCand = Math.abs((cand.heading || 0) - v.heading);
      while (headingDiffCand > Math.PI) headingDiffCand -= Math.PI * 2;
      headingDiffCand = Math.abs(headingDiffCand);
      const isSameDirection = headingDiffCand < 1.1;

      if (fwd > 0 && fwd < 140 && lat < (v.radius + candRadius + 8) && isSameDirection) {
        if (fwd < this.leadDistance) {
          this.leadDistance = fwd;
          this.leadVehicle = cand;
        }
      }
    }

    let targetSpeed = currWp.speedLimit ? Math.min(v.baseSpeed, currWp.speedLimit) : v.baseSpeed;
    let interactionDecision = 'CRUISING';

    // 5. Intersection Yielding & Priority Coordination (T-Junction Conflict Zone)
    // Side road vehicle approaching junction (Y in [360, 465], heading South)
    const isApproachingJunction = (v.y >= 360 && v.y <= 465 && v.x >= 840 && v.x <= 980 && Math.sin(v.heading) > 0.5);
    if (isApproachingJunction) {
      // Check for main road through traffic across conflict zone (X in [720, 1120], Y in [490, 630])
      let mainRoadTrafficActive = false;
      for (const cand of candidates) {
        if (cand.x >= 720 && cand.x <= 1120 && cand.y >= 490 && cand.y <= 630) {
          const candSpeed = Math.abs(cand.speed || 0);
          if (candSpeed > 6.0 || Math.hypot(cand.x - 920, cand.y - 560) < 95) {
            mainRoadTrafficActive = true;
            break;
          }
        }
      }

      if (mainRoadTrafficActive) {
        this.yieldWaitTimer += dt;
        // Priority escalation / deadlock prevention: if waited > 2.5s, grant right of way
        if (this.yieldWaitTimer < 2.5) {
          if (v.y >= 430) {
            targetSpeed = 0;
            interactionDecision = 'YIELDING';
          } else {
            targetSpeed = Math.min(targetSpeed, 25);
            interactionDecision = 'YIELDING';
          }
        } else {
          // Escalated priority: proceed smoothly through the turn
          targetSpeed = Math.min(targetSpeed, 35);
          interactionDecision = 'CRUISING';
        }
      } else {
        this.yieldWaitTimer = 0;
      }
    } else {
      this.yieldWaitTimer = 0;
    }

    // 6. Progressive Multi-Vehicle Queue Headway & Anti-Clustering
    if (this.leadVehicle && interactionDecision !== 'YIELDING') {
      const lead = this.leadVehicle;
      const d = this.leadDistance;
      const leadSpeed = Math.max(0, lead.speed || 0);
      const isLeadStopped = (leadSpeed < 5.0);
      const safeDistance = isLeadStopped ? 55 : Math.max(this.minSafeDistance, this.timeHeadway * Math.max(v.speed, leadSpeed));
      const bumperBuffer = 26;

      if (d <= bumperBuffer || (isLeadStopped && d < 48)) {
        // Standstill queue buffer -> full stop
        targetSpeed = 0;
        interactionDecision = 'STOPPED';
      } else if (d < safeDistance + 35) {
        if (d <= safeDistance) {
          // Scaling speed progressively down to leadSpeed in queue
          const gapRatio = Math.max(0, (d - bumperBuffer) / (safeDistance - bumperBuffer));
          targetSpeed = Math.min(targetSpeed, leadSpeed * Math.min(1.0, gapRatio));
        } else {
          // Approach buffer zone (smooth deceleration into queue)
          const blend = (d - safeDistance) / 35;
          const approachSpeed = leadSpeed + (v.baseSpeed - leadSpeed) * blend;
          targetSpeed = Math.min(targetSpeed, approachSpeed);
        }
        interactionDecision = (targetSpeed < 5) ? 'STOPPED' : 'FOLLOWING';
      }
    }

    // 7. Safe Controlled Overtaking of Stopped Blockage / Obstacle
    if (this.leadVehicle && (this.leadVehicle.speed || 0) < 5 && this.leadDistance < 65) {
      this.stuckTimer += dt;
    } else {
      this.stuckTimer = 0;
    }

    // If stuck behind stationary obstacle for > 1.2s on Main Road and overtaking enabled
    const allowsOvertake = (v.canOvertake !== false);
    if (allowsOvertake && this.stuckTimer > 1.2 && this.overtakeState === 'NONE' && (v.y >= 480 && v.y <= 640)) {
      const medianY = 560; // Center median corridor
      // Check if median corridor is clear of oncoming traffic within 130px
      let medianClear = true;
      for (const cand of candidates) {
        if (Math.abs(cand.y - medianY) < 28 && Math.abs(cand.x - v.x) < 130) {
          medianClear = false;
          break;
        }
      }

      if (medianClear && window.StaticCollisionSystem && !window.StaticCollisionSystem.checkSolidCollision(v.x + 40, medianY, v.radius).collided) {
        this.overtakeState = 'PASSING';
        this.overtakeTargetLaneY = medianY;
        this.overtakeTimer = 0;
      }
    }

    if (this.overtakeState === 'PASSING') {
      this.overtakeTimer += dt;
      targetSpeed = Math.min(v.baseSpeed * 0.75, 55);
      interactionDecision = 'OVERTAKING';

      // Check if cleared the obstacle
      const leadObstacleX = this.leadVehicle ? this.leadVehicle.x : (v.x + 50);
      if (v.x > leadObstacleX + 45 || this.overtakeTimer > 4.0) {
        this.overtakeState = 'RETURNING';
      }
    } else if (this.overtakeState === 'RETURNING') {
      targetSpeed = v.baseSpeed;
      interactionDecision = 'CRUISING';
      if (Math.abs(v.y - this.nominalLaneY) < 10) {
        this.overtakeState = 'NONE';
        this.overtakeTimer = 0;
      }
    }

    // 8. Basic Obstacle & Pothole Awareness
    for (let s = 15; s <= 65; s += 10) {
      const checkX = v.x + cosH * s;
      const checkY = v.y + sinH * s;
      let isBlocked = false;

      if (window.StaticCollisionSystem && window.StaticCollisionSystem.checkSolidCollision(checkX, checkY, v.radius).collided) {
        isBlocked = true;
      }

      if (!isBlocked && environmentData && Array.isArray(environmentData.debris)) {
        for (const d of environmentData.debris) {
          const hw = (d.width || 30) / 2;
          const hh = (d.height || 30) / 2;
          if (Math.abs(checkX - d.x) < hw + v.radius && Math.abs(checkY - d.y) < hh + v.radius) {
            isBlocked = true;
            break;
          }
        }
      }

      if (isBlocked) {
        targetSpeed = 0;
        interactionDecision = 'STOPPED';
        break;
      }
    }

    // Pothole ahead in corridor: moderate speed reduction
    if (window.StaticCollisionSystem) {
      const potCheck = window.StaticCollisionSystem.checkPothole(v.x + cosH * 35, v.y + sinH * 35, v.radius * 0.7);
      if (potCheck.inPothole) {
        targetSpeed = Math.min(targetSpeed, v.baseSpeed * 0.55);
      }
    }

    this.navState = interactionDecision;

    // 9. Smooth Acceleration & Deceleration Physics
    if (v.speed < targetSpeed) {
      v.speed = Math.min(targetSpeed, v.speed + this.maxAccel * dt);
    } else if (v.speed > targetSpeed) {
      v.speed = Math.max(targetSpeed, v.speed - this.maxDecel * dt);
    }

    if (v.speed < 1 && targetSpeed === 0) {
      v.speed = 0;
      v.isStopped = true;
      v.isBraking = true;
      v.state = 'STOPPED';
    } else {
      v.isStopped = false;
      v.isBraking = (v.speed > targetSpeed + 5);
      v.state = this.navState;
    }

    // 10. Steering & Heading Integration
    if (v.speed > 0) {
      const turnStep = Math.sign(headingDiff) * Math.min(Math.abs(headingDiff), this.maxTurnRate * dt);
      v.heading += turnStep;
      v.heading = Math.atan2(Math.sin(v.heading), Math.cos(v.heading));
    }

    // 11. Position Integration with Strict Boundary Prevention
    const proposedX = v.x + v.speed * Math.cos(v.heading) * dt;
    const proposedY = v.y + v.speed * Math.sin(v.heading) * dt;

    if (window.StaticCollisionSystem) {
      const solidCheck = window.StaticCollisionSystem.checkSolidCollision(proposedX, proposedY, v.radius);
      const isSidewalk = window.StaticCollisionSystem.isSidewalk(proposedX, proposedY);
      let solidBlocked = solidCheck.collided;

      if (!solidBlocked && environmentData && Array.isArray(environmentData.debris)) {
        for (const d of environmentData.debris) {
          const hw = (d.width || 30) / 2;
          const hh = (d.height || 30) / 2;
          if (Math.abs(proposedX - d.x) < hw + v.radius && Math.abs(proposedY - d.y) < hh + v.radius) {
            solidBlocked = true;
            break;
          }
        }
      }

      if (solidBlocked || isSidewalk) {
        // Stop before solid object or sidewalk
        v.speed = 0;
        v.isStopped = true;
        v.isBraking = true;
        v.state = 'STOPPED';
      } else {
        v.x = proposedX;
        v.y = proposedY;
      }
    } else {
      v.x = proposedX;
      v.y = proposedY;
    }
  }
}

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
    this.baseSpeed = config.baseSpeed !== undefined ? config.baseSpeed : (this.speed > 0 ? this.speed : 60);

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
    this.behavior = config.behavior || 'CRUISING';

    // Local Navigation & Planning State
    this.isBraking = false;
    this.isStopped = false;
    this.state = 'CRUISING'; // 'CRUISING', 'FOLLOWING', 'AVOIDING', 'BRAKING', 'STOPPED', 'CROSSING', 'WANDERING'
    this.selectedPath = null;
    this.speedJitter = 0;
    this.stateTimer = Math.random() * 3 + 2;
    this.pauseTimer = 0;
    this.swayPhase = Math.random() * Math.PI * 2;
    this.lateralOffset = 0;
    this.targetLateralOffset = 0;
    this.crossingDirection = config.crossingDirection || 1; // +1 down, -1 up
    this.active = true;

    // Shared PPO Policy Actuator States
    this.maxSteeringAngle = 0.58;
    this.maxSteeringRate = 2.8; // First-order rate limiter (rad/s)
    this.steeringAngle = 0;
    this.angularVelocity = 0;
    this.lateralVelocity = 0;
    this.lastAction = [0, 0, 0];
    this.prevSteerAction = 0;
    this.prevCrossTrack = 0;

    // Initialize dedicated TrafficAgentNavigator for road vehicles (cars, motorcycles, auto-rickshaws)
    if (this.type === 'car' || this.type === 'motorcycle' || this.type === 'auto_rickshaw') {
      this.navigator = new TrafficAgentNavigator(this, window.RoadNetworkSystem || RoadNetworkSystem);
    } else {
      this.navigator = null;
    }
  }

  /**
   * Universal continuous RL action execution for any road vehicle via the shared PPO policy
   * Directly controls vehicle physics through continuous steering, throttle, and brake inputs.
   */
  applyRLAction(action, dt) {
    if (!this.active || !Array.isArray(action)) return;
    if (dt <= 0 || dt > 0.1) dt = 0.016;

    const steerCmd = Math.max(-1.0, Math.min(1.0, action[0] || 0));
    const throttleCmd = Math.max(0.0, Math.min(1.0, action[1] || 0));
    const brakeCmd = Math.max(0.0, Math.min(1.0, action[2] || 0));

    // 1. Steering Actuator Dynamics (First-Order Rate Limiter)
    const targetSteerAngle = steerCmd * this.maxSteeringAngle;
    const maxDelta = this.maxSteeringRate * dt;
    const steerDiff = targetSteerAngle - this.steeringAngle;
    const clampedDelta = Math.max(-maxDelta, Math.min(maxDelta, steerDiff));
    this.steeringAngle += clampedDelta;
    this.steeringAngle = Math.max(-this.maxSteeringAngle, Math.min(this.maxSteeringAngle, this.steeringAngle));

    // 2. Throttle, Brake & Friction Dynamics
    const maxSpeed = this.type === 'motorcycle' ? 110 : (this.type === 'auto_rickshaw' ? 70 : 90);
    const accelRate = 140;
    const brakeRate = 260;

    if (brakeCmd > 0.05) {
      if (Math.abs(this.speed) > 1.5) {
        this.speed -= brakeRate * brakeCmd * dt;
        if (this.speed < 0) this.speed = 0;
      } else {
        this.speed = 0;
      }
    } else if (throttleCmd > 0.05) {
      this.speed += accelRate * throttleCmd * dt;
      this.speed = Math.min(maxSpeed, this.speed);
    } else {
      // Rolling drag
      if (Math.abs(this.speed) > 1) {
        this.speed -= 40 * dt;
        if (this.speed < 0) this.speed = 0;
      } else {
        this.speed = 0;
      }
    }

    // 3. Kinematic Bicycle Yaw Rate & Heading Integration
    const wheelbase = this.length * 0.65;
    if (Math.abs(this.speed) > 0.05) {
      this.angularVelocity = (this.speed / wheelbase) * Math.tan(this.steeringAngle);
      this.heading += this.angularVelocity * dt;
      this.heading = Math.atan2(Math.sin(this.heading), Math.cos(this.heading));
    } else {
      this.angularVelocity = 0;
    }

    // 4. Position Integration
    this.x += this.speed * Math.cos(this.heading) * dt;
    this.y += this.speed * Math.sin(this.heading) * dt;

    this.updatePersistentDestination();
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
    if (!this.destination) {
      this.initPersistentDestination();
    }
    if (!this.destination) return;

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
        this.updatePedestrian(dt, egoVehicle);
        break;
      case 'animal':
        this.updateAnimal(dt, egoVehicle);
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
        // If pedestrian/animal is on sidewalk and not moving toward road, skip collision on road path
        if ((threat.type === 'pedestrian' || threat.type === 'animal') && window.StaticCollisionSystem) {
          if (window.StaticCollisionSystem.isSidewalk(threat.x, threat.y) && !window.StaticCollisionSystem.isMovingTowardRoad(threat.x, threat.y, threat.heading || 0, threat.speed || 0)) {
            continue;
          }
        }

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
   * Powered by dedicated TrafficAgentNavigator system
   */
  navigateTrafficVehicle(dt, egoVehicle, allEntities, environmentData) {
    if (!this.navigator) {
      this.navigator = new TrafficAgentNavigator(this, window.RoadNetworkSystem || RoadNetworkSystem);
    }
    this.navigator.update(dt, egoVehicle, allEntities, environmentData);
  }

  updatePedestrian(dt, egoVehicle) {
    if (this.pauseTimer > 0) {
      this.pauseTimer -= dt;
      return;
    }

    if (this.behavior === 'CROSSING') {
      // Crossing from one sidewalk to the other steadily
      this.y += this.speed * this.crossingDirection * dt;
      this.heading = this.crossingDirection > 0 ? Math.PI / 2 : -Math.PI / 2;

      // When crossed fully
      if (this.crossingDirection > 0 && this.y >= 680) {
        this.y = 680;
        this.pauseTimer = Math.random() * 2.5 + 1.5;
        this.crossingDirection = -1;
        this.behavior = 'SIDEWALK';
        this.heading = Math.PI; // Walk along sidewalk
      } else if (this.crossingDirection < 0 && this.y <= 435) {
        this.y = 435;
        this.pauseTimer = Math.random() * 2.5 + 1.5;
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

  updateAnimal(dt, egoVehicle) {
    if (this.subType === 'cow') {
      // Solvability Guarantee: If ego vehicle is stopped or waiting nearby on the same road, the cow gives way
      if (egoVehicle) {
        const dx = this.x - egoVehicle.x;
        const dy = this.y - egoVehicle.y;
        const distToEgo = Math.hypot(dx, dy);
        // If cow is ahead in drivable corridor and ego is approaching/close
        if (distToEgo < 90 && dx > -10 && dx < 80 && Math.abs(dy) < 35) {
          this.pauseTimer = 0;
          this.behavior = 'WANDERING';
          const targetY = (this.y < 560) ? 490 : 630;
          this.heading = Math.atan2(targetY - this.y, 10);
        }
      }

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
        // Cows stop occasionally on shoulder/roadside
        if (Math.random() < 0.35) {
          this.pauseTimer = Math.random() * 3 + 1.5;
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
        if (Math.random() < 0.25) {
          this.pauseTimer = Math.random() * 1.5 + 0.5; // Sniff / pause
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
 * In the current RL training environment, cars, bikes, and autos are disabled while
 * pedestrians and animals (cows, dogs) are kept active. All vehicle code is preserved
 * in createRoadVehicles() so they can be reintroduced seamlessly.
 */
class TrafficManager {
  constructor() {
    this.entities = [];
    this.isEnabled = true;
    this.includeVehicles = true; // Road vehicles (cars, bikes, autos) enabled via Shared-Policy RL
    this.initDefaultEntities();
  }

  /**
   * Road vehicle definitions (3 Cars, 2 Motorcycles, 2 Auto-Rickshaws)
   * Controlled via the single universal Shared PPO Policy
   */
  createRoadVehicles() {
    return [
      // --- 1. Dynamic Cars (3) ---
      new DynamicObject({
        id: 'car-1',
        type: 'car',
        subType: 'sedan',
        x: 380,
        y: 600,
        length: 50,
        width: 25,
        heading: 0, // Eastbound
        speed: 70,
        color: '#dc2626',
        route: { road: 'main', laneY: 600 }
      }),
      new DynamicObject({
        id: 'car-2',
        type: 'car',
        subType: 'hatchback',
        x: 1350,
        y: 520,
        length: 46,
        width: 23,
        heading: Math.PI, // Westbound
        speed: 75,
        color: '#2563eb',
        route: { road: 'main', laneY: 520 }
      }),
      new DynamicObject({
        id: 'car-3',
        type: 'car',
        subType: 'taxi',
        x: 880,
        y: 120,
        length: 46,
        width: 23,
        heading: Math.PI / 2, // Southbound on side road
        speed: 60,
        color: '#f59e0b',
        route: { road: 'side', laneX: 880 }
      }),

      // --- 2. Motorcycles & Bikes (2) ---
      new DynamicObject({
        id: 'bike-1',
        type: 'motorcycle',
        subType: 'sport_bike',
        x: 220,
        y: 600,
        length: 28,
        width: 12,
        heading: 0, // Eastbound
        speed: 85,
        color: '#ea580c',
        route: { road: 'main', laneY: 600 }
      }),
      new DynamicObject({
        id: 'bike-2',
        type: 'motorcycle',
        subType: 'commuter',
        x: 1480,
        y: 520,
        length: 27,
        width: 11,
        heading: Math.PI, // Westbound
        speed: 85,
        color: '#1e293b',
        route: { road: 'main', laneY: 520 }
      }),

      // --- 3. Auto-Rickshaws (2) ---
      new DynamicObject({
        id: 'auto-1',
        type: 'auto_rickshaw',
        subType: 'cng_auto',
        x: 700,
        y: 600,
        length: 44,
        width: 26,
        heading: 0, // Eastbound
        speed: 60,
        color: '#15803d',
        route: { road: 'main', laneY: 600 }
      }),
      new DynamicObject({
        id: 'auto-2',
        type: 'auto_rickshaw',
        subType: 'cng_auto',
        x: 1050,
        y: 520,
        length: 44,
        width: 26,
        heading: Math.PI, // Westbound
        speed: 60,
        color: '#15803d',
        route: { road: 'main', laneY: 520 }
      })
    ];
  }

  /**
   * Active Dynamic Pedestrians and Animals for RL training environment
   */
  createPedestriansAndAnimals() {
    return [
      // --- Pedestrians (4) ---
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

      // --- Animals (3: 1 Cow, 2 Dogs) ---
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

  initDefaultEntities() {
    this.entities = this.createPedestriansAndAnimals();
    if (this.includeVehicles) {
      this.entities.push(...this.createRoadVehicles());
    }
  }

  setIncludeVehicles(enabled) {
    this.includeVehicles = !!enabled;
    this.reset();
  }

  update(dt, egoVehicle, environmentData, sharedPolicy, isTraining = false) {
    if (!this.isEnabled) return;
    const ego = egoVehicle || (window.SimulationEngine ? window.SimulationEngine.getEgoVehicle() : null);
    const env = environmentData || window.EnvironmentData;

    for (const entity of this.entities) {
      if (!entity.active) continue;

      const isRoadVehicle = (entity.type === 'car' || entity.type === 'motorcycle' || entity.type === 'auto_rickshaw');
      if (isRoadVehicle && sharedPolicy) {
        // Run universal shared PPO policy for this traffic road vehicle
        const targetY = (entity.route && entity.route.laneY) ? entity.route.laneY : (entity.destination ? entity.destination.y : 560);
        const crossTrack = (entity.y - targetY) / 95;
        const crossTrackRate = (crossTrack - (entity.prevCrossTrack || 0)) / Math.max(0.001, dt);
        const lastSteerChange = (entity.lastAction ? entity.lastAction[0] : 0) - (entity.prevSteerAction || 0);

        const obs = sharedPolicy.getObservation(entity, env, null, this.entities, entity.lastAction || [0, 0, 0], lastSteerChange, crossTrackRate);
        const actData = sharedPolicy.selectAction(obs, isTraining);
        entity.prevSteerAction = entity.lastAction ? entity.lastAction[0] : 0;
        entity.lastAction = actData.action;
        entity.prevCrossTrack = crossTrack;

        entity.applyRLAction(actData.action, dt);
      } else {
        entity.update(dt, ego, this.entities, env);
      }
    }
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
  update(egoVehicle, arg2, arg3, dt) {
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

    // Flexible parameter resolution: support both (ego, static, dynamic) and (ego, dynamic, static)
    let staticObstacles = null;
    let dynamicObjects = [];

    if (Array.isArray(arg2)) {
      dynamicObjects = arg2;
      staticObstacles = arg3;
    } else if (Array.isArray(arg3)) {
      staticObstacles = arg2;
      dynamicObjects = arg3;
    } else if (arg2 && typeof arg2.getEntities === 'function') {
      dynamicObjects = arg2.getEntities();
      staticObstacles = arg3;
    } else if (arg3 && typeof arg3.getEntities === 'function') {
      staticObstacles = arg2;
      dynamicObjects = arg3.getEntities();
    } else {
      staticObstacles = arg2;
      dynamicObjects = [];
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
          length: dyn.length || 18,
          width: dyn.width || 18,
          radius: (dyn.radius !== undefined) ? dyn.radius : (Math.max(dyn.length || 18, dyn.width || 18) / 2),
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

      // Spatial road/sidewalk classification
      const onRoad = window.StaticCollisionSystem ? window.StaticCollisionSystem.isDrivableRoad(cand.x, cand.y) : true;
      const onSidewalk = window.StaticCollisionSystem ? window.StaticCollisionSystem.isSidewalk(cand.x, cand.y) : false;
      const headingToRoad = window.StaticCollisionSystem ? window.StaticCollisionSystem.isMovingTowardRoad(cand.x, cand.y, cand.heading, cand.speed) : false;
      const location = onSidewalk ? 'SIDEWALK' : (onRoad ? 'ROAD' : 'OFF_ROAD');

      // Candidate Velocity Vector
      const candVx = (cand.speed || 0) * Math.cos(cand.heading || 0);
      const candVy = (cand.speed || 0) * Math.sin(cand.heading || 0);

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

      // Forward corridor alignment: is candidate directly in ego vehicle's travel path?
      const forwardCorridorDist = dx * Math.cos(egoHeading) + dy * Math.sin(egoHeading);
      const lateralCorridorDist = dy * Math.cos(egoHeading) - dx * Math.sin(egoHeading);
      const absLatDist = Math.abs(lateralCorridorDist);
      // Use tighter corridor for stationary/parked objects to avoid false positives from roadside items
      const isParkedOrStaticObj = (!cand.isDynamic || Math.abs(cand.speed || 0) < 2) && (cand.type === 'parked_object' || cand.type === 'debris');
      const corridorHalfWidth = isParkedOrStaticObj ? Math.min(cand.radius, 14) : (cand.radius + 18);
      const inEgoTravelCorridor = (forwardCorridorDist > 0 && forwardCorridorDist < 160 && absLatDist < corridorHalfWidth);
      const isApproaching = closingSpeed > 1.5;
      const isBehindAndReceding = (forwardCorridorDist < -8 && closingSpeed <= 1.0);

      // Relative Position (AHEAD / BEHIND / LEFT / RIGHT)
      let relativePosition = 'AHEAD';
      if (forwardCorridorDist > 14) {
        relativePosition = 'AHEAD';
      } else if (forwardCorridorDist < -14) {
        relativePosition = 'BEHIND';
      } else {
        relativePosition = (lateralCorridorDist < 0) ? 'LEFT' : 'RIGHT';
      }

      // Relative Movement Classification
      let relativeMovement = 'STATIONARY';
      const isDynamic = !!cand.isDynamic && Math.abs(cand.speed || 0) > 2.0;

      if (!isDynamic) {
        relativeMovement = 'STATIONARY';
      } else if (cand.type === 'pedestrian' || cand.type === 'animal') {
        if (candVy < -3) relativeMovement = 'CROSSING_RIGHT_TO_LEFT';
        else if (candVy > 3) relativeMovement = 'CROSSING_LEFT_TO_RIGHT';
        else if (closingSpeed > 1.5) relativeMovement = 'APPROACHING';
        else if (closingSpeed < -1.5) relativeMovement = 'RECEDING';
        else relativeMovement = 'PARALLEL';
      } else {
        let headingDiff = Math.abs((cand.heading || 0) - egoHeading);
        while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
        headingDiff = Math.abs(headingDiff);

        if (headingDiff > 2.4) {
          relativeMovement = 'ONCOMING';
        } else if (headingDiff < 0.6) {
          relativeMovement = (closingSpeed > 2.0) ? 'APPROACHING' : (closingSpeed < -2.0 ? 'RECEDING' : 'PARALLEL');
        } else {
          relativeMovement = (candVy < 0) ? 'CROSSING_RIGHT_TO_LEFT' : 'CROSSING_LEFT_TO_RIGHT';
        }
      }

      // Trajectory Intersection Check
      const intersectsTrajectory = inEgoTravelCorridor && onRoad;

      // Predicted Future Intersection (temporal projection 0 - 3.5s)
      let predictedIntersection = false;
      let tIntersection = null;
      if (cand.isDynamic && Math.abs(cand.speed || 0) > 1.0) {
        for (let t = 0.2; t <= 3.5; t += 0.2) {
          const futureEgoX = egoX + egoVx * t;
          const futureEgoY = egoY + egoVy * t;
          const futureCandX = cand.x + candVx * t;
          const futureCandY = cand.y + candVy * t;
          const futureDist = Math.hypot(futureCandX - futureEgoX, futureCandY - futureEgoY);
          if (futureDist < (egoRadius + cand.radius + 10)) {
            predictedIntersection = true;
            tIntersection = t;
            break;
          }
        }
      } else if (inEgoTravelCorridor) {
        predictedIntersection = true;
        tIntersection = (egoSpeed > 5) ? (forwardCorridorDist / egoSpeed) : Infinity;
      }

      // Available Safe Gap around object
      const leftGap = Math.max(0, (cand.y - (cand.radius || 12)) - 465);
      const rightGap = Math.max(0, 655 - (cand.y + (cand.radius || 12)));
      const safeGap = {
        left: leftGap,
        right: rightGap,
        canPassLeft: leftGap > 36,
        canPassRight: rightGap > 36,
        hasSafeAlternative: (leftGap > 36 || rightGap > 36)
      };

      // Relevance Classification
      let relevance = 'IRRELEVANT';

      if (isBehindAndReceding || (onSidewalk && !headingToRoad)) {
        relevance = 'IRRELEVANT';
      } else if (onSidewalk && headingToRoad) {
        relevance = 'MONITOR';
      } else if (cand.type === 'pothole') {
        relevance = inEgoTravelCorridor ? 'MONITOR' : 'IRRELEVANT';
      } else if (cand.type === 'parked_object') {
        // Parked objects are roadside hazards - only relevant if directly blocking the tight corridor
        relevance = inEgoTravelCorridor ? 'MONITOR' : 'IRRELEVANT';
      } else if (isDynamic && relativePosition === 'AHEAD' && (relativeMovement === 'PARALLEL' || relativeMovement === 'APPROACHING') && absLatDist < 26) {
        relevance = 'FOLLOW_LEAD';
      } else if (relativeMovement === 'ONCOMING' && absLatDist > 24) {
        relevance = 'MONITOR';
      } else if (inEgoTravelCorridor || predictedIntersection) {
        relevance = 'THREAT';
      } else if (!cand.isDynamic && forwardCorridorDist > 0 && forwardCorridorDist < 90 && absLatDist < 35) {
        relevance = 'BLOCKAGE';
      } else {
        relevance = 'MONITOR';
      }

      // Trajectory-Aware Collision Risk Assessment
      let riskLevel = 'SAFE';

      if (relevance === 'IRRELEVANT') {
        riskLevel = 'SAFE';
        sCount++;
      } else if (relevance === 'FOLLOW_LEAD') {
        if (closingSpeed > 4.0 || forwardCorridorDist < 35) {
          riskLevel = 'CAUTION';
          cCount++;
          if (ttc < minTTC) minTTC = ttc;
        } else {
          riskLevel = 'SAFE';
          sCount++;
        }
      } else if (cand.type === 'pothole') {
        if (inEgoTravelCorridor && forwardCorridorDist < 120 && egoSpeed > 10) {
          riskLevel = 'CAUTION';
          cCount++;
        } else {
          riskLevel = 'SAFE';
          sCount++;
        }
      } else if (cand.type === 'pedestrian' || cand.type === 'animal') {
        if (onSidewalk && headingToRoad) {
          if (forwardCorridorDist > 0 && forwardCorridorDist < 110 && absLatDist < 55) {
            riskLevel = (ttc <= 1.8 || forwardCorridorDist < 40) ? 'DANGER' : 'CAUTION';
            if (riskLevel === 'DANGER') dCount++; else cCount++;
            if (ttc < minTTC) minTTC = ttc;
          } else {
            riskLevel = 'SAFE';
            sCount++;
          }
        } else if (onRoad) {
          const lateralSpeedTowardCorridor = (dy > 0 && candVy < -3) || (dy < 0 && candVy > 3);
          const tCross = lateralSpeedTowardCorridor ? Math.abs(dy) / (Math.abs(candVy) || 1) : Infinity;

          if (inEgoTravelCorridor) {
            riskLevel = (ttc <= 2.2 || forwardCorridorDist < 45 || isApproaching) ? 'DANGER' : 'CAUTION';
            if (riskLevel === 'DANGER') dCount++; else cCount++;
            if (ttc < minTTC) minTTC = ttc;
          } else if (lateralSpeedTowardCorridor && tCross < 5.0 && forwardCorridorDist > 0 && forwardCorridorDist < 130) {
            riskLevel = (tCross < 2.0 || forwardCorridorDist < 40) ? 'DANGER' : 'CAUTION';
            if (riskLevel === 'DANGER') dCount++; else cCount++;
            if (ttc < minTTC) minTTC = (ttc === Infinity ? tCross : Math.min(ttc, tCross));
          } else {
            riskLevel = 'SAFE';
            sCount++;
          }
        } else {
          riskLevel = 'SAFE';
          sCount++;
        }
      } else {
        // Solid Obstacles & Dynamic Road Vehicles
        const isStationary = (!cand.isDynamic || Math.abs(cand.speed || 0) < 5);
        const directCorridorWidth = isStationary ? 16 : (cand.radius + 6);
        const isDirectFrontal = (absLatDist < directCorridorWidth);

        if (cand.type === 'building' || (onSidewalk && !cand.isDynamic && !inEgoTravelCorridor)) {
          riskLevel = 'SAFE';
          sCount++;
        } else if (inEgoTravelCorridor && isDirectFrontal) {
          if (!isStationary && ttc <= this.dangerTTC && isApproaching) {
            riskLevel = 'DANGER';
            dCount++;
            if (ttc < minTTC) minTTC = ttc;
          } else if (forwardCorridorDist <= 35 && egoSpeed > 15) {
            riskLevel = 'DANGER';
            dCount++;
            if (ttc < minTTC) minTTC = ttc;
          } else if (forwardCorridorDist <= 75 && isStationary) {
            riskLevel = 'CAUTION';
            cCount++;
            if (ttc < minTTC) minTTC = ttc;
          } else if (!isStationary && ttc <= this.cautionTTC && isApproaching) {
            riskLevel = 'CAUTION';
            cCount++;
            if (ttc < minTTC) minTTC = ttc;
          } else {
            riskLevel = 'SAFE';
            sCount++;
          }
        } else {
          riskLevel = 'SAFE';
          sCount++;
        }
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
        ttc: (ttc === Infinity || riskLevel === 'SAFE') ? null : ttc,
        riskLevel: riskLevel,
        isDynamic: cand.isDynamic,
        length: cand.length,
        width: cand.width,
        radius: cand.radius,
        bounds: cand.bounds,
        onRoad: onRoad,
        onSidewalk: onSidewalk,
        headingToRoad: headingToRoad,
        location: location,
        relativePosition: relativePosition,
        relativeMovement: relativeMovement,
        intersectsTrajectory: intersectsTrajectory,
        predictedIntersection: predictedIntersection,
        tIntersection: tIntersection,
        safeGap: safeGap,
        relevance: relevance
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
    this.horizonDistance = options.horizonDistance || 120; // px (approx 30 m)
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

      // Continuity / Hysteresis bonus: slight preference for sustaining active trajectory to eliminate jitter
      if (this.selectedPath && path.id === this.selectedPath.id) {
        path.totalCost -= 50;
      }

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

        // If object is classified as IRRELEVANT (sidewalk bystander, behind receding), skip collision scoring on path
        if (obs.relevance === 'IRRELEVANT' || ((obs.type === 'pedestrian' || obs.type === 'animal') && obs.onSidewalk && !obs.headingToRoad)) {
          continue;
        }

        // Oriented Bounding Box Collision & Clearance Check
        const obsHeading = obs.heading || 0;
        const cosO = Math.cos(obsHeading);
        const sinO = Math.sin(obsHeading);
        const dx = wp.x - obs.x;
        const dy = wp.y - obs.y;
        const longDist = Math.abs(dx * cosO + dy * sinO);
        const latDist = Math.abs(dy * cosO - dx * sinO);

        const egoHalfWidth = 9;
        const egoHalfLength = 16;
        const obsHalfWidth = (obs.width || (obs.radius ? obs.radius * 1.5 : 18)) / 2;
        const obsHalfLength = (obs.length || (obs.radius ? obs.radius * 2 : 32)) / 2;

        const collisionLat = obsHalfWidth + egoHalfWidth;
        const collisionLong = obsHalfLength + egoHalfLength;

        const clearance = Math.hypot(Math.max(0, longDist - collisionLong), Math.max(0, latDist - collisionLat));
        if (clearance < minClearance) {
          minClearance = clearance;
        }

        // Direct Collision Check (inside physical extent)
        if (longDist <= collisionLong && latDist <= collisionLat) {
          hasCollision = true;
          if (collisionIdx === -1) {
            collisionIdx = i;
            collisionDist = wp.s;
            collisionObs = obs;
          }
          break;
        }

        // Proximity Soft Penalty (clearance < 40px)
        if (clearance < 40) {
          const threatMult = obs.riskLevel === 'DANGER' ? 2.5 : (obs.riskLevel === 'CAUTION' ? 1.6 : 1.0);
          const penalty = threatMult * (this.weights.clearance / (clearance + 4));
          totalObstacleCost += penalty;
        }
      }

      if (hasCollision) break;

      // C. Pothole Road Surface Hazard Check (passable surface hazard, prefer clean tarmac when practical)
      const potholeCheck = StaticCollisionSystem.checkPothole(wp.x, wp.y, egoRadius * 0.8);
      if (potholeCheck.inPothole) {
        potholePenalty += 400; // Moderate penalty: prefers avoiding when practical, passable if avoiding would cause collision
      }

      // D. Drivable Road Envelope Check (using vehicle body footprint width)
      const vehicleHalfW = 10; // Vehicle body half-width
      const onMainRoad = ((wp.y - vehicleHalfW) >= 465 && (wp.y + vehicleHalfW) <= 655) && (wp.x >= 0 && wp.x <= 1800);
      const onSideRoad = ((wp.x - vehicleHalfW) >= 860 && (wp.x + vehicleHalfW) <= 980) && (wp.y >= 0 && wp.y <= 560);

      if (!onMainRoad && !onSideRoad) {
        isOffRoad = true;
        roadCost += 5000;
      } else {
        // Mild preference for lane center without blocking valid drivable road edge bypasses
        if (onMainRoad) {
          if (wp.y > 645) {
            roadCost += (wp.y - 645) * 15;
          } else if (wp.y < 475) {
            roadCost += (475 - wp.y) * 15;
          }
        }
      }
    }

    // --- 2. Destination / Route Progress & Alignment (Global Road Route Aware Scoring) ---
    let targetX = destination.x;
    let targetY = destination.y;
    let targetHeading = Math.atan2(destination.y - endWp.y, destination.x - endWp.x);

    // If global route planner is active, score progress toward the route lookahead point
    if (typeof GlobalRouteSystem !== 'undefined') {
      const navTarget = GlobalRouteSystem.getNavigationTarget({ x: egoX, y: egoY, heading: endWp.heading }, 65);
      if (navTarget && navTarget.hasRoute && navTarget.targetLookahead) {
        targetX = navTarget.targetLookahead.x;
        targetY = navTarget.targetLookahead.y;
        targetHeading = navTarget.targetHeading;
      }
    }

    const startDistToTarget = Math.hypot(targetX - egoX, targetY - egoY);
    const endDistToTarget = Math.hypot(targetX - endWp.x, targetY - endWp.y);
    const deltaTarget = endDistToTarget - startDistToTarget;

    let goalProgressCost = 0;
    if (deltaTarget < 0) {
      // Moving along global route -> Significant reward
      goalProgressCost = deltaTarget * 4.2;
    } else {
      // Moving away or sideways from route -> Heavy penalty
      goalProgressCost = 1400 + deltaTarget * 6.5;
    }

    // Alignment angle toward route tangent vector
    let headingDiff = targetHeading - endWp.heading;
    while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
    while (headingDiff < -Math.PI) headingDiff += Math.PI * 2;
    const goalHeadingCost = Math.abs(headingDiff) * 220;

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
          heading: det.heading || 0,
          speed: det.speed || 0,
          radius: det.radius || 15,
          riskLevel: det.riskLevel || 'SAFE',
          relevance: det.relevance || 'MONITOR',
          onRoad: det.onRoad,
          onSidewalk: det.onSidewalk,
          headingToRoad: det.headingToRoad
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
  update(egoVehicle, environmentData, perceptionData, pathPlannerData, trafficManager, dt, isStuck = false) {
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

    const dynamicEntities = trafficManager ? (typeof trafficManager.getEntities === 'function' ? trafficManager.getEntities() : (Array.isArray(trafficManager) ? trafficManager : [])) : [];

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
      environmentData,
      isStuck
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
      candidatePaths, dynamicEntities, environmentData, isStuck
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
        // Sidewalk pedestrians not entering road do not cause collision stop
        if ((obj.type === 'pedestrian' || obj.type === 'animal') && obj.onSidewalk && !obj.headingToRoad) {
          continue;
        }

        const dx = obj.x - egoX;
        const dy = obj.y - egoY;
        const forwardDist = dx * cosH + dy * sinH;
        const lateralDist = Math.abs(dy * cosH - dx * sinH);

        const criticalLat = (plannerState === 'AVOIDANCE') ? 12 : 18;
        const criticalLong = (plannerState === 'AVOIDANCE') ? 18 : this.config.criticalDistance;

        if (forwardDist > 0 && forwardDist < criticalLong && lateralDist < criticalLat) {
          imminentDanger = true;
          dangerReason = `Obstacle ahead (${obj.type || 'hazard'} ${Math.round(forwardDist)}px)`;
          break;
        }
      }
    }

    // B. Check Selected Path Feasibility
    let pathFeasible = true;
    let replanReason = '';
    const feasibleCount = candidatePaths.filter(p => p.status === 'FEASIBLE').length;

    if (!selectedPath || selectedPath.status === 'COLLISION' || selectedPath.status === 'OFF_ROAD' || plannerState === 'EMERGENCY_STOP') {
      pathFeasible = false;
      replanReason = selectedPath ? `Path blocked (${selectedPath.status})` : 'No safe path available';
    }

    // C. Check Front Traffic Blockage / Bottleneck
    let frontBlocked = false;
    let frontBlockReason = '';
    let slowFrontVehicles = 0;

    for (const obj of detectedObjects) {
      if ((obj.type === 'pedestrian' || obj.type === 'animal') && obj.onSidewalk && !obj.headingToRoad) {
        continue;
      }

      const dx = obj.x - egoX;
      const dy = obj.y - egoY;
      const forwardDist = dx * cosH + dy * sinH;
      const lateralDist = Math.abs(dy * cosH - dx * sinH);

      if (forwardDist > 5 && forwardDist < 75 && lateralDist < 26) {
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

    // E. Check Cross-Traffic / Crossing Pedestrian Conflict
    let crossTrafficDetected = false;
    let crossTrafficReason = '';

    const inIntersectionZone = (
      egoX >= this.config.intersectionZone.minX - 50 &&
      egoX <= this.config.intersectionZone.maxX &&
      egoY >= this.config.intersectionZone.minY &&
      egoY <= this.config.intersectionZone.maxY
    );

    for (const obj of detectedObjects) {
      if (obj.isDynamic && (obj.speed || 0) >= 8 && (obj.type === 'car' || obj.type === 'motorcycle' || obj.type === 'auto_rickshaw' || obj.type === 'truck' || obj.type === 'bus')) {
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
      if ((obj.type === 'pedestrian' || obj.type === 'animal')) {
        // Only trigger crossing caution/yield if entity is on the road OR actively entering road
        const isThreatening = obj.onRoad || obj.headingToRoad;
        if (!isThreatening) continue;

        const dx = obj.x - egoX;
        const dy = obj.y - egoY;
        const forwardDist = dx * cosH + dy * sinH;
        const lateralDist = Math.abs(dy * cosH - dx * sinH);
        if (forwardDist > 0 && forwardDist < 85 && lateralDist < 35) {
          crossTrafficDetected = true;
          crossTrafficReason = `Crossing ${obj.type} ahead`;
          break;
        }
      }
    }

    // F. Lead Vehicle Detection (for smooth FOLLOWING and SLOW regulation)
    let leadVehicle = null;
    let leadFollow = false;
    let leadSlowing = false;

    for (const obj of detectedObjects) {
      if (obj.isDynamic && (obj.type === 'car' || obj.type === 'motorcycle' || obj.type === 'auto_rickshaw' || obj.type === 'truck' || obj.type === 'bus')) {
        const dx = obj.x - egoX;
        const dy = obj.y - egoY;
        const forwardDist = dx * cosH + dy * sinH;
        const lateralDist = Math.abs(dy * cosH - dx * sinH);

        let headingDiff = Math.abs((obj.heading || 0) - egoHeading);
        while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
        headingDiff = Math.abs(headingDiff);

        // Vehicle ahead in same corridor moving generally in same direction
        if (obj.isDynamic && (obj.speed || 0) >= 8 && forwardDist > 15 && forwardDist < 160 && lateralDist < 26 && headingDiff < 0.6) {
          leadVehicle = obj;
          const closingSpeed = obj.closingSpeed || 0;
          if (closingSpeed > 3.0 || (obj.speed !== undefined && obj.speed < egoSpeed - 5)) {
            leadSlowing = true;
          } else if (obj.speed !== undefined && obj.speed >= 10 && forwardDist >= 30) {
            leadFollow = true;
          }
          break;
        }
      }
    }

    // G. Pedestrian Approaching Curb / Monitoring
    let pedestrianApproachingCurb = false;
    for (const obj of detectedObjects) {
      if ((obj.type === 'pedestrian' || obj.type === 'animal') && obj.onSidewalk && obj.headingToRoad) {
        if (obj.distance < 110) {
          pedestrianApproachingCurb = true;
          break;
        }
      }
    }

    // H. Check Developing Risk / Hazard Caution
    let developingRisk = false;
    let cautionReason = '';

    if (overallRisk === 'CAUTION' || (minTTC > this.config.imminentTTC && minTTC <= this.config.cautionTTC)) {
      developingRisk = true;
      cautionReason = `Developing collision risk (TTC ${minTTC < 10 ? minTTC.toFixed(1) + 's' : 'caution'})`;
    } else if (plannerState === 'AVOIDANCE' && overallRisk !== 'SAFE') {
      // Only slow for avoidance when there are actual detected threats, not just static roadside objects
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
      feasibleCount,
      frontBlocked,
      frontBlockReason,
      rearClear,
      rearBlocker,
      crossTrafficDetected,
      crossTrafficReason,
      leadVehicle,
      leadFollow,
      leadSlowing,
      pedestrianApproachingCurb,
      developingRisk,
      cautionReason,
      egoSpeed,
      inIntersectionZone,
      isStuck: !!isStuck
    };
  }

  /**
   * Deterministic Priority Hierarchy Decision Resolution
   * 1. IMMINENT COLLISION -> STOP
   * 2. CROSSING PEDESTRIAN / CROSS TRAFFIC -> YIELD / WAIT
   * 3. STUCK RECOVERY -> REVERSE (if clear rear) or WAIT
   * 4. TRAFFIC BLOCKAGE -> REVERSE (if clear rear) or WAIT (waiting for gap)
   * 5. INFEASIBLE TRAJECTORY -> REPLAN (if alternative exists) or WAIT
   * 6. LEAD VEHICLE SLOWING -> SLOW
   * 7. LEAD VEHICLE CRUISE -> FOLLOW
   * 8. PEDESTRIAN AT CURB -> SLOW (MONITOR)
   * 9. DEVELOPING RISK / AVOIDANCE -> SLOW
   * 10. DEFAULT / SIDEWALK DETECTION -> GO
   */
  evaluatePriority(ctx) {
    // 1. HIGHEST PRIORITY: Imminent Collision Threat -> STOP
    if (ctx.imminentDanger) {
      return {
        decision: 'STOP',
        reason: ctx.dangerReason || 'Imminent collision threat'
      };
    }

    // 2. Cross Traffic / Intersection Incursion / Crossing Pedestrian -> YIELD or WAIT
    if (ctx.crossTrafficDetected) {
      if (ctx.inIntersectionZone) {
        return {
          decision: 'WAIT',
          reason: ctx.crossTrafficReason || 'Waiting at busy intersection'
        };
      } else {
        return {
          decision: 'YIELD',
          reason: ctx.crossTrafficReason || 'Yielding to crossing traffic / pedestrian'
        };
      }
    }

    // 3. Stuck Recovery Handling (Ego commanded to move but trapped for > 2.5s)
    if (ctx.isStuck) {
      if (ctx.rearClear) {
        return {
          decision: 'REVERSE',
          reason: 'Deadlock recovery (vehicle stuck - reversing)'
        };
      } else {
        return {
          decision: 'WAIT',
          reason: 'Waiting for traffic blockage to clear (vehicle stuck)'
        };
      }
    }

    // 4. Traffic Blockage Resolution -> WAIT for gap (or REVERSE only if persistently stuck)
    if (ctx.frontBlocked) {
      if (!ctx.pathFeasible) {
        if (ctx.isStuck && ctx.rearClear) {
          return {
            decision: 'REVERSE',
            reason: 'Reversing to clear persistent traffic blockage'
          };
        } else {
          return {
            decision: 'WAIT',
            reason: 'Traffic blocked - waiting for gap to open'
          };
        }
      }
    }

    // 5. Infeasible Trajectory -> REPLAN (or WAIT if no feasible paths exist)
    if (!ctx.pathFeasible) {
      if (ctx.feasibleCount > 0) {
        return {
          decision: 'REPLAN',
          reason: ctx.replanReason || 'Searching for safe alternative trajectory'
        };
      } else {
        return {
          decision: 'WAIT',
          reason: 'No safe trajectory available - waiting for gap'
        };
      }
    }

    // 6. Lead Vehicle Slowing Down -> SLOW
    if (ctx.leadSlowing) {
      return {
        decision: 'SLOW',
        reason: 'Lead vehicle slowing ahead - maintaining buffer'
      };
    }

    // 7. Lead Vehicle Cruising Normally Ahead -> FOLLOW
    if (ctx.leadFollow) {
      return {
        decision: 'FOLLOW',
        reason: 'Following lead vehicle at safe distance'
      };
    }

    // 8. Pedestrian Approaching Curb -> SLOW / MONITOR
    if (ctx.pedestrianApproachingCurb) {
      return {
        decision: 'SLOW',
        reason: 'Monitoring pedestrian approaching road curb'
      };
    }

    // 9. Developing Risk / Hazard Caution / Avoidance Maneuver -> SLOW
    if (ctx.developingRisk) {
      return {
        decision: 'SLOW',
        reason: ctx.cautionReason || 'Developing risk - reducing speed'
      };
    }

    // 10. DEFAULT: Normal Clear Driving -> GO (Sidewalk pedestrians and non-threatening objects do not stop car)
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
      leadVehicle: this.state ? this.state.leadVehicle : null,
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
    this.recoveryState = 'FOLLOW_ROUTE'; // 'FOLLOW_ROUTE', 'BLOCKED', 'RECOVERY', 'REPLAN'
    this.targetSpeed = 0;
    this.targetHeading = 0;
    this.hasArrived = false;
    this.arrivalDistance = 45; // px

    // Persistent Stuck Detection & Recovery State Machine
    this.stuckTimer = 0;
    this.lastStuckCheckPos = { x: 0, y: 0 };
    this.isStuck = false;
    this.lastProgressTime = Date.now();
    this.lastMinDistRemaining = null;
    this.recoveryAttempts = 0;
    this.maxRecoveryAttempts = 3;
    this.lastFailedLocation = null;
    this.failedLocations = [];
    this.recoveryTimer = 0;

    // Actuation parameters
    this.config = {
      cruiseSpeed: 75,          // px/s (~18 km/h smooth city cruising)
      cautionSpeed: 40,         // px/s (~10 km/h)
      replanSpeed: 25,          // px/s (~6 km/h)
      reverseSpeed: -26,        // px/s (controlled backward speed)
      lookaheadWaypointIdx: 3,  // Lookahead point index along 18-point trajectory spline (~30-40px)
      steerGain: 2.2,           // Proportional lateral tracking gain
      steerRateLimit: 6.5,      // rad/s (smooth steering rate limit)
      speedDecelGain: 20,       // Longitudinal braking sensitivity
      speedAccelGain: 25,       // Longitudinal throttle sensitivity
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
    this.stuckTimer = 0;
    this.isStuck = false;
    this.recoveryState = 'FOLLOW_ROUTE';
    this.recoveryAttempts = 0;
    this.lastProgressTime = Date.now();
    this.lastMinDistRemaining = null;
    
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
      if (egoVehicle) {
        this.lastStuckCheckPos = { x: egoVehicle.x, y: egoVehicle.y };
      }
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

    // 2. Check Global Route Navigation & Dynamic Speed-Scheduled Lookahead
    const destX = destination ? destination.x : 1650;
    const destY = destination ? destination.y : 560;
    const distToGoal = Math.hypot(destX - egoX, destY - egoY);

    // Speed-Scheduled Lookahead Distance: 45px at low speed up to 90px at cruising speed
    const dynamicLookahead = Math.max(45, Math.min(90, 0.65 * Math.abs(currentSpeed) + 45));
    let nav = null;
    if (typeof GlobalRouteSystem !== 'undefined') {
      nav = GlobalRouteSystem.getNavigationTarget(egoVehicle, dynamicLookahead);
    }

    // Destination Arrival Check
    if (distToGoal <= this.arrivalDistance || (nav && nav.navState === 'ARRIVED') || this.hasArrived) {
      this.hasArrived = true;
      this.controlState = 'ARRIVED';
      this.recoveryState = 'FOLLOW_ROUTE';
      this.targetSpeed = 0;
      egoVehicle.inputs.throttle = 0;
      egoVehicle.inputs.brake = 1.0;
      egoVehicle.inputs.steer = 0;
      egoVehicle.inputs.handbrake = (Math.abs(currentSpeed) < 1.0);
      return;
    }

    // 3. Persistent Route Progress & Stuck Detection
    if (nav && nav.distanceRemainingAlongRoute !== undefined) {
      if (this.lastMinDistRemaining === null || nav.distanceRemainingAlongRoute < this.lastMinDistRemaining - 2.5) {
        this.lastMinDistRemaining = nav.distanceRemainingAlongRoute;
        this.lastProgressTime = Date.now();
        this.stuckTimer = 0;
        if (this.recoveryState === 'FOLLOW_ROUTE') {
          this.recoveryAttempts = 0;
        }
      }
    }

    const isTrapped = (this.controlState === 'WAITING' || this.controlState === 'STOPPED') && !this.hasArrived;
    const isCommandingMove = (this.targetSpeed > 15 && this.controlState !== 'ARRIVED');
    const isStationary = Math.abs(currentSpeed) < 8.0 && Math.hypot(egoX - this.lastStuckCheckPos.x, egoY - this.lastStuckCheckPos.y) < 3.5;

    if ((isCommandingMove || isTrapped) && isStationary) {
      this.stuckTimer += dt;
    } else if (this.controlState !== 'REVERSING') {
      this.stuckTimer = 0;
      this.lastStuckCheckPos = { x: egoX, y: egoY };
    }

    const isTimedOut = (Date.now() - this.lastProgressTime > 2200);
    this.isStuck = (this.stuckTimer >= 2.2 || (isTimedOut && isCommandingMove));

    // 4. Recovery State Machine Transitions
    if (this.isStuck && this.recoveryState === 'FOLLOW_ROUTE') {
      this.recoveryState = 'BLOCKED';
    }

    if (this.recoveryState === 'BLOCKED') {
      // Record failed location
      const failedSpot = { x: egoX, y: egoY, time: Date.now() };
      this.lastFailedLocation = failedSpot;

      // Count repeated failures at same location
      let recentFails = 1;
      for (const f of this.failedLocations) {
        if (Math.hypot(egoX - f.x, egoY - f.y) < 40 && (Date.now() - f.time < 30000)) {
          recentFails++;
        }
      }
      this.failedLocations.push(failedSpot);

      if (recentFails >= 2 && typeof GlobalRouteSystem !== 'undefined') {
        // Mark location as blocked in GlobalRouteSystem to force alternate route
        GlobalRouteSystem.markBlockedLocation(egoX, egoY, 45, 20000);
      }

      const rearSafe = this.checkRearSafety(egoVehicle, perceptionData, trafficManager);
      if (rearSafe && this.recoveryAttempts < this.maxRecoveryAttempts) {
        this.recoveryState = 'RECOVERY';
        this.recoveryTimer = 0;
        this.recoveryAttempts++;
      } else {
        // Rear blocked or exceeded max attempts -> safe wait
        this.controlState = 'WAITING';
        this.targetSpeed = 0;
        egoVehicle.inputs.throttle = 0;
        egoVehicle.inputs.brake = 1.0;
        egoVehicle.inputs.steer = 0;
        return;
      }
    }

    if (this.recoveryState === 'RECOVERY') {
      this.controlState = 'REVERSING';
      this.targetSpeed = this.config.reverseSpeed; // -26 px/s
      egoVehicle.inputs.throttle = -0.65;
      egoVehicle.inputs.brake = 0;

      // Active reverse steering with reverse kinematics
      let revTargetHeading = (nav && nav.targetHeading !== undefined) ? nav.targetHeading : 0;
      let revHeadingDiff = egoHeading - revTargetHeading;
      while (revHeadingDiff > Math.PI) revHeadingDiff -= Math.PI * 2;
      while (revHeadingDiff < -Math.PI) revHeadingDiff += Math.PI * 2;

      const cte = (nav && nav.crossTrackError !== undefined) ? nav.crossTrackError : 0;
      // In reverse: positive heading error (pointed right/down) -> steer right (+delta) to rotate CCW back to road
      const revSteerCmd = Math.max(-0.85, Math.min(0.85, 1.35 * revHeadingDiff + 0.015 * cte));
      egoVehicle.inputs.steer += (revSteerCmd - egoVehicle.inputs.steer) * 8.0 * dt;

      this.recoveryTimer += dt;
      const dRev = this.lastFailedLocation ? Math.hypot(egoX - this.lastFailedLocation.x, egoY - this.lastFailedLocation.y) : 50;

      if (dRev >= 45 || this.recoveryTimer >= 2.0) {
        this.recoveryState = 'REPLAN';
      }
      return;
    }

    if (this.recoveryState === 'REPLAN') {
      this.controlState = 'REPLANNING';
      if (typeof GlobalRouteSystem !== 'undefined') {
        GlobalRouteSystem.planRoute(egoVehicle, destination);
      }
      this.recoveryState = 'FOLLOW_ROUTE';
      this.lastProgressTime = Date.now();
      this.lastMinDistRemaining = null;
      this.stuckTimer = 0;
      this.isStuck = false;
    }

    // 5. Normal Safety Decision & Path Planning Integration
    const decision = decisionData ? decisionData.decision : 'GO';
    const selectedPath = pathPlannerData ? pathPlannerData.selectedPath : null;
    const plannerState = pathPlannerData ? pathPlannerData.plannerState : 'OPTIMAL';

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
        const rearSafe = this.checkRearSafety(egoVehicle, perceptionData, trafficManager);
        if (rearSafe) {
          this.controlState = 'REVERSING';
          this.targetSpeed = this.config.reverseSpeed;
          egoVehicle.inputs.throttle = -0.65;
          egoVehicle.inputs.brake = 0;

          let revTargetHeading = (nav && nav.targetHeading !== undefined) ? nav.targetHeading : 0;
          let revHeadingDiff = egoHeading - revTargetHeading;
          while (revHeadingDiff > Math.PI) revHeadingDiff -= Math.PI * 2;
          while (revHeadingDiff < -Math.PI) revHeadingDiff += Math.PI * 2;

          const cte = (nav && nav.crossTrackError !== undefined) ? nav.crossTrackError : 0;
          const revSteerCmd = Math.max(-0.85, Math.min(0.85, 1.35 * revHeadingDiff + 0.015 * cte));
          egoVehicle.inputs.steer += (revSteerCmd - egoVehicle.inputs.steer) * 8.0 * dt;
          return;
        } else {
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

      case 'FOLLOW':
        this.controlState = 'FOLLOWING';
        if (decisionData && decisionData.leadVehicle) {
          const lead = decisionData.leadVehicle;
          const leadSpeed = (lead.speed !== undefined) ? lead.speed : this.config.cautionSpeed;
          const leadDist = lead.distance || Math.hypot(lead.x - egoX, lead.y - egoY);
          const desiredHeadway = Math.max(45, 1.8 * Math.abs(currentSpeed));
          const gapError = leadDist - desiredHeadway;
          this.targetSpeed = Math.max(0, Math.min(this.config.cruiseSpeed, leadSpeed + 0.65 * gapError));
        } else {
          this.targetSpeed = this.config.cautionSpeed;
        }
        break;

      case 'SLOW':
        this.controlState = (plannerState === 'AVOIDANCE') ? 'AVOIDING' : 'BRAKING';
        if (decisionData && decisionData.leadVehicle) {
          const lead = decisionData.leadVehicle;
          const leadSpeed = (lead.speed !== undefined) ? lead.speed : 20;
          this.targetSpeed = Math.max(15, Math.min(this.config.cautionSpeed, leadSpeed - 5));
        } else {
          this.targetSpeed = this.config.cautionSpeed;
        }
        break;

      case 'GO':
      default:
        this.controlState = 'CRUISING';
        this.targetSpeed = this.config.cruiseSpeed;
        break;
    }

    // Adaptive Speed Regulation for Curves, Turns, Intersections & Approaching Destination
    if (nav && nav.hasRoute) {
      if (nav.navState === 'ARRIVING') {
        const arrivalRatio = Math.max(0.15, Math.min(1.0, nav.distanceToGoal / 140));
        this.targetSpeed = Math.min(this.targetSpeed, Math.max(22, this.config.cruiseSpeed * arrivalRatio));
      } else if (nav.isTurn) {
        this.targetSpeed = Math.min(this.targetSpeed, 40);
      }
    }

    // 6. Lateral Route Following & Path Tracking (Pure Pursuit + Alignment Damping + Straightening)
    let desiredSteerAngle = 0;

    if (this.controlState === 'WAITING' || this.controlState === 'STOPPED' || this.isStuck) {
      desiredSteerAngle = 0;
      egoVehicle.inputs.steer = 0;
    } else if (selectedPath && selectedPath.waypoints && selectedPath.waypoints.length > 0 && plannerState === 'AVOIDANCE') {
      // Local obstacle avoidance trajectory override
      const lookaheadIdx = Math.min(this.config.lookaheadWaypointIdx, selectedPath.waypoints.length - 1);
      const targetWp = selectedPath.waypoints[lookaheadIdx] || selectedPath.endPoint;

      const targetHeading = Math.atan2(targetWp.y - egoY, targetWp.x - egoX);
      let headingError = targetHeading - egoHeading;
      while (headingError > Math.PI) headingError -= Math.PI * 2;
      while (headingError < -Math.PI) headingError += Math.PI * 2;

      desiredSteerAngle = Math.max(-0.65, Math.min(0.65, 0.85 * (selectedPath.steerAngle || 0) + 0.65 * headingError));
    } else if (nav && nav.hasRoute && nav.targetLookahead) {
      // Global Route Waypoint Pure Pursuit with Segment Alignment Damping
      const targetWp = nav.targetLookahead;
      const targetDist = Math.hypot(targetWp.x - egoX, targetWp.y - egoY) || dynamicLookahead;
      const targetHeading = Math.atan2(targetWp.y - egoY, targetWp.x - egoX);
      let lookaheadHeadingError = targetHeading - egoHeading;
      while (lookaheadHeadingError > Math.PI) lookaheadHeadingError -= Math.PI * 2;
      while (lookaheadHeadingError < -Math.PI) lookaheadHeadingError += Math.PI * 2;

      // Route Segment Heading Alignment (provides yaw rate derivative damping)
      let routeAlignError = nav.targetHeading - egoHeading;
      while (routeAlignError > Math.PI) routeAlignError -= Math.PI * 2;
      while (routeAlignError < -Math.PI) routeAlignError += Math.PI * 2;

      // Geometric Curvature from Kinematic Bicycle Model: delta = atan2(2 * L * sin(alpha), targetDist)
      const L = egoVehicle.wheelbase || 26;
      const delta = Math.atan2(2 * L * Math.sin(lookaheadHeadingError), targetDist);
      let targetSteer = 0.85 * (delta / (egoVehicle.maxSteeringAngle || 0.55)) + 0.15 * (routeAlignError / (egoVehicle.maxSteeringAngle || 0.55));

      // Straightening Deadband: when closely aligned to route, center steering to eliminate snake-like oscillations
      if (Math.abs(lookaheadHeadingError) < 0.035 && Math.abs(routeAlignError) < 0.035 && Math.abs(nav.crossTrackError) < 3.5) {
        targetSteer = 0.20 * (routeAlignError / (egoVehicle.maxSteeringAngle || 0.55));
      }

      desiredSteerAngle = Math.max(-0.85, Math.min(0.85, targetSteer));

      // Curvature speed reduction
      if (Math.abs(desiredSteerAngle) > 0.42) {
        this.targetSpeed = Math.min(this.targetSpeed, this.config.cautionSpeed);
      }
    } else {
      const directHeading = Math.atan2(destY - egoY, destX - egoX);
      let headingError = directHeading - egoHeading;
      while (headingError > Math.PI) headingError -= Math.PI * 2;
      while (headingError < -Math.PI) headingError += Math.PI * 2;
      desiredSteerAngle = Math.max(-1.0, Math.min(1.0, headingError * 1.5));
    }

    // Set direct continuous steering command to vehicle steering actuator
    if (this.controlState !== 'WAITING' && this.controlState !== 'STOPPED' && !this.isStuck) {
      egoVehicle.inputs.steer = Math.max(-1.0, Math.min(1.0, desiredSteerAngle));
    }

    // 7. Longitudinal Speed Control (Throttle & Progressive Braking)
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
      egoVehicle.inputs.throttle = 0.1;
      egoVehicle.inputs.brake = 0;
      egoVehicle.inputs.handbrake = false;
    }

    // Strict Reverse Guard: Prevent unintended backward movement unless explicitly commanded
    if (this.controlState !== 'REVERSING') {
      egoVehicle.inputs.throttle = Math.max(0, egoVehicle.inputs.throttle);
      if (egoVehicle.speed < 0) {
        egoVehicle.speed = 0;
      }
    }
  }

  /**
   * Verify real-time rear safety clearance buffer before allowing REVERSE
   */
  checkRearSafety(egoVehicle, perceptionData, trafficManager) {
    const cosH = Math.cos(egoVehicle.heading);
    const sinH = Math.sin(egoVehicle.heading);

    const candidates = [];
    if (trafficManager && Array.isArray(trafficManager.getEntities())) {
      candidates.push(...trafficManager.getEntities());
    }
    if (perceptionData && Array.isArray(perceptionData.detectedObjects)) {
      candidates.push(...perceptionData.detectedObjects);
    }

    for (const dyn of candidates) {
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
   11. MODULE 7: REINFORCEMENT LEARNING EGO AGENT & TRAINING MANAGER
   ============================================================================ */

/**
 * Multi-Layer Perceptron (MLP) Neural Network with Adam Optimizer
 * Supports continuous forward and backward propagation in pure vanilla JavaScript.
 */
class MLPNet {
  constructor(inputDim, hiddenDims, outputDim, outputActivations = ['tanh', 'sigmoid', 'sigmoid']) {
    this.inputDim = inputDim;
    this.hiddenDims = hiddenDims;
    this.outputDim = outputDim;
    this.outputActivations = outputActivations;

    this.weights = [];
    this.biases = [];
    this.m_w = [];
    this.v_w = [];
    this.m_b = [];
    this.v_b = [];

    const dims = [inputDim, ...hiddenDims, outputDim];
    for (let i = 0; i < dims.length - 1; i++) {
      const fanIn = dims[i];
      const fanOut = dims[i + 1];
      const scale = Math.sqrt(2.0 / fanIn);

      const w = new Float32Array(fanIn * fanOut);
      for (let j = 0; j < w.length; j++) {
        w[j] = (Math.random() * 2 - 1) * scale;
      }
      const b = new Float32Array(fanOut).fill(0.01);

      this.weights.push(w);
      this.biases.push(b);
      this.m_w.push(new Float32Array(w.length));
      this.v_w.push(new Float32Array(w.length));
      this.m_b.push(new Float32Array(b.length));
      this.v_b.push(new Float32Array(b.length));
    }
  }

  forward(input) {
    let curr = new Float32Array(input);
    const activations = [curr];
    const numLayers = this.weights.length;

    for (let l = 0; l < numLayers; l++) {
      const fanIn = l === 0 ? this.inputDim : this.hiddenDims[l - 1];
      const fanOut = l === numLayers - 1 ? this.outputDim : this.hiddenDims[l];
      const w = this.weights[l];
      const b = this.biases[l];

      const next = new Float32Array(fanOut);
      for (let j = 0; j < fanOut; j++) {
        let sum = b[j];
        for (let i = 0; i < fanIn; i++) {
          sum += curr[i] * w[i * fanOut + j];
        }

        if (l < numLayers - 1) {
          next[j] = Math.tanh(sum);
        } else {
          const act = this.outputActivations[j] || this.outputActivations[0] || 'linear';
          if (act === 'tanh') {
            next[j] = Math.tanh(sum);
          } else if (act === 'sigmoid') {
            next[j] = 1 / (1 + Math.exp(-Math.max(-10, Math.min(10, sum))));
          } else {
            next[j] = sum;
          }
        }
      }
      curr = next;
      activations.push(curr);
    }
    return { output: Array.from(curr), activations };
  }

  backward(activations, gradOutput) {
    const numLayers = this.weights.length;
    let gradCurrent = new Float32Array(gradOutput);

    const gradWeights = [];
    const gradBiases = [];

    for (let l = numLayers - 1; l >= 0; l--) {
      const input = activations[l];
      const output = activations[l + 1];
      const fanIn = l === 0 ? this.inputDim : this.hiddenDims[l - 1];
      const fanOut = l === numLayers - 1 ? this.outputDim : this.hiddenDims[l];
      const w = this.weights[l];

      const gradZ = new Float32Array(fanOut);
      for (let j = 0; j < fanOut; j++) {
        if (l < numLayers - 1) {
          gradZ[j] = gradCurrent[j] * (1 - output[j] * output[j]);
        } else {
          const act = this.outputActivations[j] || this.outputActivations[0] || 'linear';
          if (act === 'tanh') {
            gradZ[j] = gradCurrent[j] * (1 - output[j] * output[j]);
          } else if (act === 'sigmoid') {
            gradZ[j] = gradCurrent[j] * (output[j] * (1 - output[j]));
          } else {
            gradZ[j] = gradCurrent[j];
          }
        }
      }

      const gw = new Float32Array(fanIn * fanOut);
      const gb = new Float32Array(fanOut);

      for (let j = 0; j < fanOut; j++) {
        gb[j] = gradZ[j];
        for (let i = 0; i < fanIn; i++) {
          gw[i * fanOut + j] = input[i] * gradZ[j];
        }
      }

      gradWeights.unshift(gw);
      gradBiases.unshift(gb);

      if (l > 0) {
        const gradPrev = new Float32Array(fanIn);
        for (let i = 0; i < fanIn; i++) {
          let sum = 0;
          for (let j = 0; j < fanOut; j++) {
            sum += w[i * fanOut + j] * gradZ[j];
          }
          gradPrev[i] = sum;
        }
        gradCurrent = gradPrev;
      }
    }

    return { gradWeights, gradBiases };
  }

  applyGradients(gradWeights, gradBiases, lr = 0.001, beta1 = 0.9, beta2 = 0.999, eps = 1e-8) {
    for (let l = 0; l < this.weights.length; l++) {
      const w = this.weights[l];
      const b = this.biases[l];
      const gw = gradWeights[l];
      const gb = gradBiases[l];
      const mw = this.m_w[l];
      const vw = this.v_w[l];
      const mb = this.m_b[l];
      const vb = this.v_b[l];

      for (let i = 0; i < w.length; i++) {
        mw[i] = beta1 * mw[i] + (1 - beta1) * gw[i];
        vw[i] = beta2 * vw[i] + (1 - beta2) * (gw[i] * gw[i]);
        const mHat = mw[i] / (1 - beta1);
        const vHat = vw[i] / (1 - beta2);
        w[i] -= (lr * mHat) / (Math.sqrt(vHat) + eps);
      }

      for (let j = 0; j < b.length; j++) {
        mb[j] = beta1 * mb[j] + (1 - beta1) * gb[j];
        vb[j] = beta2 * vb[j] + (1 - beta2) * (gb[j] * gb[j]);
        const mHat = mb[j] / (1 - beta1);
        const vHat = vb[j] / (1 - beta2);
        b[j] -= (lr * mHat) / (Math.sqrt(vHat) + eps);
      }
    }
  }

  serialize() {
    return {
      inputDim: this.inputDim,
      hiddenDims: this.hiddenDims,
      outputDim: this.outputDim,
      outputActivations: this.outputActivations,
      weights: this.weights.map(w => Array.from(w)),
      biases: this.biases.map(b => Array.from(b))
    };
  }

  deserialize(data) {
    this.inputDim = data.inputDim;
    this.hiddenDims = data.hiddenDims;
    this.outputDim = data.outputDim;
    this.outputActivations = data.outputActivations;
    this.weights = data.weights.map(w => new Float32Array(w));
    this.biases = data.biases.map(b => new Float32Array(b));
  }
}

/**
 * Continuous Proximal Policy Optimization (PPO-Clip) Agent for Ego Vehicle
 * Directly outputs continuous steering [-1, 1], throttle [0, 1], and brake [0, 1]
 * from continuous 26-dimensional control-state simulation environment state observations.
 */
class EgoPPOAgent {
  constructor(options = {}) {
    this.obsDim = options.obsDim || 26;
    this.actionDim = options.actionDim || 3;
    this.hiddenDims = options.hiddenDims || [64, 64];

    // Actor: outputs mean continuous actions mu = [steer, throttle, brake]
    this.actor = new MLPNet(this.obsDim, this.hiddenDims, this.actionDim, ['tanh', 'sigmoid', 'sigmoid']);
    // Critic: outputs scalar state-value estimate V(s)
    this.critic = new MLPNet(this.obsDim, this.hiddenDims, 1, ['linear']);

    // Continuous Gaussian policy standard deviations [steer, throttle, brake]
    this.logStd = new Float32Array([-0.693, -1.2, -1.2]); // std approx [0.50, 0.30, 0.30]
    this.minStd = new Float32Array([0.06, 0.04, 0.04]);
    this.stdDecay = 0.997;

    // Hyperparameters
    this.gamma = 0.99;
    this.gaeLambda = 0.95;
    this.clipEps = 0.20;
    this.actorLr = 0.0006;
    this.criticLr = 0.0012;
    this.ppoEpochs = 4;

    this.episodesTrained = 0;
    this.bestReward = -Infinity;
    this.rollout = [];
    this.checkpoints = {};
  }

  get std() {
    return [
      Math.max(this.minStd[0], Math.exp(this.logStd[0])),
      Math.max(this.minStd[1], Math.exp(this.logStd[1])),
      Math.max(this.minStd[2], Math.exp(this.logStd[2]))
    ];
  }

  /**
   * Extract rich, 26-D control-state normalized continuous observation vector
   * Supports Ego Vehicle and all traffic road vehicles (cars, motorcycles, auto-rickshaws) via the single shared policy.
   */
  getObservation(vehicle, env, routePlanner, dynamicEntities, prevAction = [0, 0, 0], lastSteerChange = 0, crossTrackRate = 0) {
    if (!vehicle || !env) return new Array(this.obsDim).fill(0);

    const dest = vehicle.destination || (env && env.destination) || { x: 1650, y: 560 };
    const dx = dest.x - vehicle.x;
    const dy = dest.y - vehicle.y;
    const distToGoal = Math.hypot(dx, dy);
    const destAngle = Math.atan2(dy, dx);
    let relAngle = destAngle - vehicle.heading;
    while (relAngle > Math.PI) relAngle -= Math.PI * 2;
    while (relAngle < -Math.PI) relAngle += Math.PI * 2;

    // Desired road travel heading based on vehicle route phase / heading
    let desiredHeading = 0.0;
    if (vehicle.route && vehicle.route.road === 'side') {
      desiredHeading = Math.PI / 2; // Northbound side road
    } else if (Math.abs(vehicle.heading - Math.PI) < Math.PI / 2 || (vehicle.routePhase === 'WESTBOUND')) {
      desiredHeading = Math.PI; // Westbound main road
    }
    let headingErr = vehicle.heading - desiredHeading;
    while (headingErr > Math.PI) headingErr -= Math.PI * 2;
    while (headingErr < -Math.PI) headingErr += Math.PI * 2;

    // Lateral velocity perpendicular to road heading
    const lateralVel = (vehicle.speed || 0) * Math.sin(headingErr);

    const topEdgeDist = Math.max(0, Math.min(1.0, (vehicle.y - 465) / 190));
    const bottomEdgeDist = Math.max(0, Math.min(1.0, (655 - vehicle.y) / 190));
    const targetY = (vehicle.route && vehicle.route.laneY) ? vehicle.route.laneY : (vehicle.destination ? vehicle.destination.y : 560);
    const crossTrack = Math.max(-1.0, Math.min(1.0, (vehicle.y - targetY) / 95));

    // 5 Raycast range sensors [-60°, -30°, 0°, +30°, +60°]
    const rayAngles = [-Math.PI / 3, -Math.PI / 6, 0, Math.PI / 6, Math.PI / 3];
    const rayDistances = [];
    const maxRayLen = 140;

    for (const offset of rayAngles) {
      const rayAngle = vehicle.heading + offset;
      const cosA = Math.cos(rayAngle);
      const sinA = Math.sin(rayAngle);
      let hitDist = maxRayLen;

      if (typeof window !== 'undefined' && window.StaticCollisionSystem) {
        for (let d = 10; d <= maxRayLen; d += 15) {
          const px = vehicle.x + d * cosA;
          const py = vehicle.y + d * sinA;
          if (!window.StaticCollisionSystem.isDrivableRoad(px, py)) {
            hitDist = d;
            break;
          }
        }
      }
      rayDistances.push(hitDist / maxRayLen);
    }

    // Nearest Static Hazard (Pothole / Debris)
    let nearestHazardDist = 1.0;
    let nearestHazardAngle = 0.0;
    if (env && Array.isArray(env.potholes)) {
      let minDist = 150;
      for (const p of env.potholes) {
        const pdx = p.x - vehicle.x;
        const pdy = p.y - vehicle.y;
        const fwd = pdx * Math.cos(vehicle.heading) + pdy * Math.sin(vehicle.heading);
        if (fwd > -5 && fwd < 150) {
          const d = Math.hypot(pdx, pdy);
          if (d < minDist) {
            minDist = d;
            nearestHazardDist = Math.max(0, Math.min(1.0, d / 150));
            let ang = Math.atan2(pdy, pdx) - vehicle.heading;
            while (ang > Math.PI) ang -= Math.PI * 2;
            while (ang < -Math.PI) ang += Math.PI * 2;
            nearestHazardAngle = ang / Math.PI;
          }
        }
      }
    }

    // Nearest Dynamic Agent (Pedestrians, Animals, Other Vehicles, Ego)
    let nearestDynDist = 1.0;
    let nearestDynAngle = 0.0;
    const dyns = dynamicEntities || (typeof window !== 'undefined' && window.SimulationEngine && window.SimulationEngine.getDynamicObjects ? window.SimulationEngine.getDynamicObjects() : []);
    const ego = typeof window !== 'undefined' && window.SimulationEngine ? window.SimulationEngine.getEgoVehicle() : null;

    const candidateEntities = Array.isArray(dyns) ? [...dyns] : [];
    if (ego && ego !== vehicle && !candidateEntities.includes(ego)) {
      candidateEntities.push(ego);
    }

    let minDist = 180;
    for (const d of candidateEntities) {
      if (d === vehicle || (d.active !== undefined && !d.active)) continue;
      const dX = d.x - vehicle.x;
      const dY = d.y - vehicle.y;
      const forwardD = dX * Math.cos(vehicle.heading) + dY * Math.sin(vehicle.heading);
      if (forwardD > -10 && forwardD < 180) {
        const euclidean = Math.hypot(dX, dY);
        if (euclidean < minDist) {
          minDist = euclidean;
          nearestDynDist = Math.max(0, Math.min(1.0, euclidean / 180));
          let ang = Math.atan2(dY, dX) - vehicle.heading;
          while (ang > Math.PI) ang -= Math.PI * 2;
          while (ang < -Math.PI) ang += Math.PI * 2;
          nearestDynAngle = ang / Math.PI;
        }
      }
    }

    return [
      Math.max(0, Math.min(1.0, vehicle.x / 1800)),
      Math.max(0, Math.min(1.0, vehicle.y / 700)),
      Math.max(-0.5, Math.min(1.0, (vehicle.speed || 0) / 100)),
      Math.cos(vehicle.heading),
      Math.sin(vehicle.heading),
      Math.max(-1.0, Math.min(1.0, (vehicle.angularVelocity || 0) / 2.5)), // 1. Angular velocity / heading change rate
      Math.max(-1.0, Math.min(1.0, lateralVel / 80)),                      // 2. Lateral velocity
      Math.max(-1.0, Math.min(1.0, (vehicle.steeringAngle || 0) / (vehicle.maxSteeringAngle || 0.58))), // 3. Current steering angle
      Math.max(-1.0, Math.min(1.0, lastSteerChange / 2.0)),                // 4. Steering change from previous step
      crossTrack,                                                          // 5. Signed cross-track error
      Math.max(-1.0, Math.min(1.0, crossTrackRate / 3.0)),                 // 6. Rate of change of cross-track error
      headingErr / Math.PI,                                                // 7. Heading error relative to desired travel direction
      Math.max(-1.0, Math.min(1.0, dx / 1600)),
      Math.max(-1.0, Math.min(1.0, dy / 1600)),
      Math.min(1.0, distToGoal / 1600),
      relAngle / Math.PI,
      topEdgeDist,
      bottomEdgeDist,
      rayDistances[0],
      rayDistances[1],
      rayDistances[2],
      rayDistances[3],
      rayDistances[4],
      nearestHazardDist,
      nearestHazardAngle,
      nearestDynDist
    ];
  }

  /**
   * Sample action from continuous Gaussian policy
   * Uses exploration noise during TRAINING; uses deterministic mean action during EVALUATION.
   */
  selectAction(obs, isTraining = true) {
    const actRes = this.actor.forward(obs);
    const mu = actRes.output; // [steer, throttle, brake]
    const criticRes = this.critic.forward(obs);
    const value = criticRes.output[0];

    const currentStd = this.std;
    const action = [...mu];
    let logProb = 0;

    if (isTraining) {
      // Gaussian sampling with Box-Muller transform
      for (let i = 0; i < 3; i++) {
        const u1 = Math.max(1e-7, Math.random());
        const u2 = Math.random();
        const z = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        action[i] = mu[i] + currentStd[i] * z;
      }
    }

    // Clamp actions to valid vehicle physics ranges
    action[0] = Math.max(-1.0, Math.min(1.0, action[0]));
    action[1] = Math.max(0.0, Math.min(1.0, action[1]));
    action[2] = Math.max(0.0, Math.min(1.0, action[2]));

    // Compute log probability density of sampled action under Gaussian distribution
    for (let i = 0; i < 3; i++) {
      const diff = (action[i] - mu[i]) / currentStd[i];
      logProb += -0.5 * (diff * diff + 2.0 * Math.log(currentStd[i]) + 1.837877);
    }

    return {
      action,
      meanAction: mu,
      value,
      logProb,
      actorActivations: actRes.activations,
      criticActivations: criticRes.activations
    };
  }

  /**
   * Record step into PPO rollout buffer
   */
  recordStep(obs, actionData, reward, done) {
    this.rollout.push({
      obs,
      action: actionData.action,
      meanAction: actionData.meanAction,
      value: actionData.value,
      logProb: actionData.logProb,
      actorActivations: actionData.actorActivations,
      criticActivations: actionData.criticActivations,
      reward,
      done
    });
  }

  /**
   * Optimize policy using PPO-Clip with GAE advantage estimation
   */
  trainPPO() {
    const N = this.rollout.length;
    if (N === 0) return 0;

    // 1. Compute Generalized Advantage Estimator (GAE)
    const returns = new Float32Array(N);
    const advantages = new Float32Array(N);
    let gae = 0;

    for (let t = N - 1; t >= 0; t--) {
      const step = this.rollout[t];
      const nextValue = (t === N - 1 || step.done) ? 0 : this.rollout[t + 1].value;
      const delta = step.reward + this.gamma * nextValue * (step.done ? 0 : 1) - step.value;
      gae = delta + this.gamma * this.gaeLambda * (step.done ? 0 : 1) * gae;
      advantages[t] = gae;
      returns[t] = advantages[t] + step.value;
    }

    // Normalize advantages for gradient stability
    let meanAdv = 0;
    for (let t = 0; t < N; t++) meanAdv += advantages[t];
    meanAdv /= N;
    let varAdv = 0;
    for (let t = 0; t < N; t++) varAdv += (advantages[t] - meanAdv) ** 2;
    const stdAdv = Math.sqrt(varAdv / N) + 1e-8;

    for (let t = 0; t < N; t++) {
      advantages[t] = (advantages[t] - meanAdv) / stdAdv;
    }

    // 2. Multi-epoch PPO Clipped Updates
    const currentStd = this.std;

    for (let epoch = 0; epoch < this.ppoEpochs; epoch++) {
      for (let t = 0; t < N; t++) {
        const step = this.rollout[t];
        const adv = advantages[t];

        // Forward pass to get current policy action mean
        const actRes = this.actor.forward(step.obs);
        const muCurr = actRes.output;

        // Current log-prob under updated policy
        let logProbCurr = 0;
        for (let i = 0; i < 3; i++) {
          const diff = (step.action[i] - muCurr[i]) / currentStd[i];
          logProbCurr += -0.5 * (diff * diff + 2.0 * Math.log(currentStd[i]) + 1.837877);
        }

        // Probability ratio r_t(theta) = exp(logpi_curr - logpi_old)
        const ratio = Math.exp(Math.max(-10, Math.min(10, logProbCurr - step.logProb)));
        const clippedRatio = Math.max(1.0 - this.clipEps, Math.min(1.0 + this.clipEps, ratio));

        // PPO Clipped Gradient Direction
        let gradMultiplier = ratio * adv;
        if (adv > 0 && ratio > 1.0 + this.clipEps) {
          gradMultiplier = 0; // Clipped upper bound
        } else if (adv < 0 && ratio < 1.0 - this.clipEps) {
          gradMultiplier = 0; // Clipped lower bound
        }

        // Actor Gradient wrt mu: - gradMultiplier * (a - mu) / std^2
        const gradActorOutput = [
          -(step.action[0] - muCurr[0]) / (currentStd[0] * currentStd[0] + 1e-6) * gradMultiplier,
          -(step.action[1] - muCurr[1]) / (currentStd[1] * currentStd[1] + 1e-6) * gradMultiplier,
          -(step.action[2] - muCurr[2]) / (currentStd[2] * currentStd[2] + 1e-6) * gradMultiplier
        ];

        const actorGrads = this.actor.backward(actRes.activations, gradActorOutput);
        this.actor.applyGradients(actorGrads.gradWeights, actorGrads.gradBiases, this.actorLr);

        // Critic Gradient: 2 * (V(s) - Return)
        const criticRes = this.critic.forward(step.obs);
        const gradCriticOutput = [2.0 * (criticRes.output[0] - returns[t])];
        const criticGrads = this.critic.backward(criticRes.activations, gradCriticOutput);
        this.critic.applyGradients(criticGrads.gradWeights, criticGrads.gradBiases, this.criticLr);
      }
    }

    this.episodesTrained++;
    for (let i = 0; i < 3; i++) {
      this.logStd[i] = Math.log(Math.max(this.minStd[i], Math.exp(this.logStd[i]) * this.stdDecay));
    }
    this.rollout = [];
    return N;
  }

  saveCheckpoint(episode, name) {
    const ckptKey = name || `ppo_checkpoint_ep${episode}`;
    const data = this.save(ckptKey);
    this.checkpoints[ckptKey] = { episode, bestReward: this.bestReward, date: Date.now() };
    return data;
  }

  save(key = 'ego_ppo_model') {
    const data = {
      episodesTrained: this.episodesTrained,
      bestReward: this.bestReward,
      logStd: Array.from(this.logStd),
      actor: this.actor.serialize(),
      critic: this.critic.serialize()
    };
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem(key, JSON.stringify(data));
      console.log(`[EgoPPOAgent] PPO model saved to localStorage key "${key}".`);
    }
    return data;
  }

  load(key = 'ego_ppo_model') {
    if (typeof localStorage === 'undefined') return false;
    const raw = localStorage.getItem(key);
    if (!raw) return false;
    try {
      const data = JSON.parse(raw);
      this.episodesTrained = data.episodesTrained || 0;
      this.bestReward = data.bestReward || -Infinity;
      if (data.logStd) this.logStd = new Float32Array(data.logStd);
      if (data.actor) this.actor.deserialize(data.actor);
      if (data.critic) this.critic.deserialize(data.critic);
      console.log(`[EgoPPOAgent] PPO model loaded successfully (episodes: ${this.episodesTrained}).`);
      return true;
    } catch (e) {
      console.error('[EgoPPOAgent] Failed to load model:', e);
      return false;
    }
  }
}

// Universal Shared-Policy Alias: Ego and all traffic vehicles reuse this single policy model
const EgoRLAgent = EgoPPOAgent;
const SharedPPOPolicy = EgoPPOAgent;

/**
 * Reinforcement Learning Training & Evaluation Manager
 * Controls the episode training loop, reward computation, 100-episode statistics tracking, and HUD.
 */
class RLTrainingManager {
  constructor(simulationEngine, rlAgent) {
    this.engine = simulationEngine;
    this.agent = rlAgent || new EgoPPOAgent();
    this.mode = 'OFF'; // 'OFF', 'TRAINING', 'EVALUATION'
    
    this.maxEpisodeSteps = 1200;
    this.currentStep = 0;
    this.currentEpisode = 0;
    this.currentEpisodeReward = 0;
    this.prevDistanceToGoal = 0;
    this.prevEgoX = 140;
    this.prevEgoY = 580;

    // Cumulative and rolling statistics
    this.episodeRewards = [];
    this.episodeSuccesses = [];
    this.episodeLengths = [];
    this.episodeAbsSteer = [];
    this.episodeSteerChanges = [];
    this.episodeLateralErrors = [];

    this.currentEpAbsSteerSum = 0;
    this.currentEpSteerChangeSum = 0;
    this.currentEpLateralErrorSum = 0;

    this.totalCollisions = 0;
    this.totalRoadDepartures = 0;
    this.totalSuccesses = 0;

    this.lastTerminationReason = 'NONE';
    this.lastAction = [0, 0, 0];
    this.prevSteerAction = 0;
    this.prevCrossTrack = 0;
  }

  isActive() {
    return this.mode === 'TRAINING' || this.mode === 'EVALUATION';
  }

  isTraining() {
    return this.mode === 'TRAINING';
  }

  isEvaluation() {
    return this.mode === 'EVALUATION';
  }

  setMode(mode) {
    this.mode = mode; // 'OFF', 'TRAINING', 'EVALUATION'
    if (this.isActive()) {
      this.resetEpisode();
      if (this.isTraining() && this.agent.episodesTrained === 0) {
        this.agent.load();
      }
    }
    console.log(`[RLTrainingManager] Mode set to ${this.mode}`);
  }

  resetEpisode() {
    const ego = this.engine.getEgoVehicle();
    const env = window.EnvironmentData;
    if (!ego) return;

    // 1. Reset Ego Vehicle to start position
    ego.reset(140, 580, 0);

    // 2. Reset dynamic pedestrians and animals to valid initial positions
    if (this.engine.trafficManager) {
      this.engine.trafficManager.reset();
    }

    // 3. Reset destination route planner
    if (this.engine.routePlanner && env && env.destination) {
      this.engine.routePlanner.planRoute(ego, env.destination);
    }

    this.currentStep = 0;
    this.currentEpisode++;
    this.currentEpisodeReward = 0;
    this.lastTerminationReason = 'RUNNING';
    this.prevSteerAction = 0;
    this.prevCrossTrack = 0;
    this.prevEgoX = 140;
    this.prevEgoY = 580;

    this.currentEpAbsSteerSum = 0;
    this.currentEpSteerChangeSum = 0;
    this.currentEpLateralErrorSum = 0;

    const dest = env.destination || { x: 1650, y: 560 };
    this.prevDistanceToGoal = Math.hypot(dest.x - ego.x, dest.y - ego.y);
  }

  update(dt) {
    if (!this.isActive()) return;

    const ego = this.engine.getEgoVehicle();
    const env = window.EnvironmentData;
    const routePlanner = this.engine.getRoutePlanner();
    if (!ego || !env) return;

    // 1. Advance dynamic pedestrians, animals, and road vehicles (via shared PPO policy if active)
    const isTraining = this.isTraining();
    if (this.engine.trafficManager && this.engine.trafficManager.isEnabled) {
      this.engine.trafficManager.update(dt, ego, env, this.agent, isTraining);
    }

    // 2. Update perception system
    if (this.engine.detectionManager) {
      this.engine.detectionManager.update(
        ego,
        env,
        this.engine.trafficManager && this.engine.trafficManager.isEnabled ? this.engine.trafficManager.getEntities() : [],
        dt
      );
    }

    // 3. Control-State Observations
    const currentCrossTrack = (ego.y - 560) / 95;
    const crossTrackRate = (currentCrossTrack - this.prevCrossTrack) / Math.max(0.001, dt);
    const lastSteerChange = this.lastAction[0] - this.prevSteerAction;

    const dynEntities = this.engine.trafficManager && this.engine.trafficManager.isEnabled ? this.engine.trafficManager.getEntities() : [];
    const obs = this.agent.getObservation(ego, env, routePlanner, dynEntities, this.lastAction, lastSteerChange, crossTrackRate);
    
    // Select action: Gaussian sampling in training, deterministic policy inference in evaluation
    const actData = this.agent.selectAction(obs, isTraining);
    this.lastAction = actData.action;

    // Apply continuous PPO actions directly to EgoVehicle physics inputs
    ego.inputs.steer = actData.action[0];
    ego.inputs.throttle = actData.action[1];
    ego.inputs.brake = actData.action[2];
    ego.inputs.handbrake = false;

    // Advance vehicle physics
    ego.update(dt);
    this.currentStep++;

    // Track step smoothness metrics
    const steerDelta = Math.abs(actData.action[0] - this.prevSteerAction);
    const latError = Math.abs(ego.y - 560);
    this.currentEpAbsSteerSum += Math.abs(actData.action[0]);
    this.currentEpSteerChangeSum += steerDelta;
    this.currentEpLateralErrorSum += latError;

    // 4. Compute Step Reward
    const dest = env.destination || { x: 1650, y: 560 };
    const currDist = Math.hypot(dest.x - ego.x, dest.y - ego.y);

    // Valid road route longitudinal progress reward (along road direction 0 rad)
    const desiredHeading = 0.0;
    const deltaLongitudinal = (ego.x - this.prevEgoX) * Math.cos(desiredHeading) + (ego.y - this.prevEgoY) * Math.sin(desiredHeading);
    this.prevEgoX = ego.x;
    this.prevEgoY = ego.y;
    this.prevDistanceToGoal = currDist;

    let reward = deltaLongitudinal * 2.2; // Longitudinal road progress reward

    // Centerline / Road corridor centering bonus
    reward += 0.30 * (1 - Math.min(1, latError / 45));

    // Heading alignment bonus with desired road travel direction
    reward += 0.20 * Math.max(0, Math.cos(ego.heading - desiredHeading));

    // Forward velocity bonus
    reward += 0.25 * Math.max(0, (ego.speed || 0) / 80);

    // Smoothness penalty: Penalize rapid steering changes and excessive steering magnitude
    reward -= 0.08 * (steerDelta * steerDelta);
    if (actData.action[0] * this.prevSteerAction < 0) {
      // Rapid sign reversal penalty (left-right snake oscillation)
      reward -= 0.12 * steerDelta;
    }
    reward -= 0.03 * (actData.action[0] * actData.action[0]);

    // Lateral divergence penalty: If lateral error is increasing away from centerline
    const newCrossTrack = (ego.y - 560) / 95;
    const newCrossTrackRate = (newCrossTrack - currentCrossTrack) / Math.max(0.001, dt);
    if (newCrossTrack * newCrossTrackRate > 0) {
      reward -= 0.15 * Math.abs(newCrossTrack * newCrossTrackRate);
    }

    this.prevSteerAction = actData.action[0];
    this.prevCrossTrack = newCrossTrack;

    // Step cost
    reward -= 0.04;

    // Stationary penalty (prevents car from standing still)
    if (Math.abs(ego.speed) < 2.0 && currDist > 60) {
      reward -= 0.30;
    }

    // Check Termination Conditions
    let done = false;
    let reason = 'RUNNING';
    let isSuccess = false;

    const isOffRoad = typeof window !== 'undefined' && window.StaticCollisionSystem
      ? !window.StaticCollisionSystem.isDrivableRoad(ego.x, ego.y)
      : false;
    const isArrived = currDist <= 45;
    const isCollided = ego.isCollided;

    if (isArrived) {
      reward += 500.0;
      done = true;
      reason = 'ARRIVED';
      isSuccess = true;
      this.totalSuccesses++;
    } else if (isCollided) {
      reward -= 200.0;
      done = true;
      reason = 'COLLISION';
      this.totalCollisions++;
    } else if (isOffRoad) {
      reward -= 200.0;
      done = true;
      reason = 'OFF_ROAD';
      this.totalRoadDepartures++;
    } else if (this.currentStep >= this.maxEpisodeSteps) {
      reward -= 50.0;
      done = true;
      reason = 'TIMEOUT';
    }

    this.currentEpisodeReward += reward;

    if (isTraining) {
      this.agent.recordStep(obs, actData, reward, done);
    }

    if (done) {
      this.lastTerminationReason = reason;

      const avgEpSteer = this.currentStep > 0 ? (this.currentEpAbsSteerSum / this.currentStep) : 0;
      const avgEpSteerChange = this.currentStep > 0 ? (this.currentEpSteerChangeSum / this.currentStep) : 0;
      const avgEpLatErr = this.currentStep > 0 ? (this.currentEpLateralErrorSum / this.currentStep) : 0;

      // Track rolling history (last 100 episodes)
      this.episodeRewards.push(this.currentEpisodeReward);
      this.episodeSuccesses.push(isSuccess ? 1 : 0);
      this.episodeLengths.push(this.currentStep);
      this.episodeAbsSteer.push(avgEpSteer);
      this.episodeSteerChanges.push(avgEpSteerChange);
      this.episodeLateralErrors.push(avgEpLatErr);

      if (this.episodeRewards.length > 100) this.episodeRewards.shift();
      if (this.episodeSuccesses.length > 100) this.episodeSuccesses.shift();
      if (this.episodeLengths.length > 100) this.episodeLengths.shift();
      if (this.episodeAbsSteer.length > 100) this.episodeAbsSteer.shift();
      if (this.episodeSteerChanges.length > 100) this.episodeSteerChanges.shift();
      if (this.episodeLateralErrors.length > 100) this.episodeLateralErrors.shift();

      // Display after every episode
      console.log(`[PPO Episode ${this.currentEpisode}] Reward: ${this.currentEpisodeReward.toFixed(2)} | Success: ${isSuccess} | Steps: ${this.currentStep} | DistToGoal: ${currDist.toFixed(1)}px | Reason: ${reason} | Total Collisions: ${this.totalCollisions} | Total OffRoad: ${this.totalRoadDepartures} | AbsSteer: ${avgEpSteer.toFixed(3)} | SteerChg: ${avgEpSteerChange.toFixed(3)} | LatErr: ${avgEpLatErr.toFixed(1)}px`);

      // Periodically display 100-episode statistics (every 10 or 25 episodes)
      if (this.currentEpisode % 10 === 0) {
        const stats = this.get100EpisodeStats();
        console.log(`=== [PPO Progress Report: Ep ${this.currentEpisode}] AvgReward(100): ${stats.avgReward100} | SuccessRate(100): ${stats.successRate100} | AvgLength(100): ${stats.avgLength100} | AvgAbsSteer: ${stats.avgAbsSteer100} | AvgSteerChg: ${stats.avgSteerChange100} | AvgLatErr: ${stats.avgLateralError100}px ===`);
        if (isTraining) {
          this.agent.saveCheckpoint(this.currentEpisode);
        }
      }

      // Checkpoint best reward
      if (this.currentEpisodeReward > this.agent.bestReward) {
        this.agent.bestReward = this.currentEpisodeReward;
        if (isTraining) this.agent.save('ego_ppo_best_model');
      }

      // Update PPO policy network on episode completion
      if (isTraining) {
        this.agent.trainPPO();
      }

      // Reset environment for next episode
      this.resetEpisode();
    }
  }

  get100EpisodeStats() {
    const len = this.episodeRewards.length || 1;
    const avgReward = (this.episodeRewards.reduce((a, b) => a + b, 0) / len).toFixed(2);
    const successRate = ((this.episodeSuccesses.reduce((a, b) => a + b, 0) / len) * 100).toFixed(1) + '%';
    const avgLength = (this.episodeLengths.reduce((a, b) => a + b, 0) / len).toFixed(1);
    const avgAbsSteer = (this.episodeAbsSteer.reduce((a, b) => a + b, 0) / len).toFixed(3);
    const avgSteerChg = (this.episodeSteerChanges.reduce((a, b) => a + b, 0) / len).toFixed(3);
    const avgLatErr = (this.episodeLateralErrors.reduce((a, b) => a + b, 0) / len).toFixed(1);

    return {
      windowSize: len,
      avgReward100: avgReward,
      successRate100: successRate,
      avgLength100: avgLength,
      avgAbsSteer100: avgAbsSteer,
      avgSteerChange100: avgSteerChg,
      avgLateralError100: avgLatErr
    };
  }

  getMetrics() {
    const stats100 = this.get100EpisodeStats();
    const currentDist = (this.prevDistanceToGoal || 0).toFixed(1);

    return {
      mode: this.mode,
      episode: this.currentEpisode,
      step: this.currentStep,
      currentReward: this.currentEpisodeReward.toFixed(1),
      avgReward100: stats100.avgReward100,
      successRate100: stats100.successRate100,
      avgLength100: stats100.avgLength100,
      avgAbsSteer100: stats100.avgAbsSteer100,
      avgSteerChange100: stats100.avgSteerChange100,
      avgLateralError100: stats100.avgLateralError100,
      bestReward: this.agent.bestReward === -Infinity ? '0.0' : this.agent.bestReward.toFixed(1),
      totalCollisions: this.totalCollisions,
      totalOffRoad: this.totalRoadDepartures,
      distToGoal: `${currentDist}px`,
      lastReason: this.lastTerminationReason,
      steer: this.lastAction[0].toFixed(2),
      throttle: this.lastAction[1].toFixed(2),
      brake: this.lastAction[2].toFixed(2)
    };
  }

  /**
   * Render real-time PPO HUD Overlay on simulation canvas
   */
  renderHUD(ctx, camera) {
    if (!this.isActive() || !ctx) return;

    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0); // Screen coordinates

    const m = this.getMetrics();
    const roadVehicles = this.engine && this.engine.trafficManager ? this.engine.trafficManager.getEntities().filter(e => e.type === 'car' || e.type === 'motorcycle' || e.type === 'auto_rickshaw') : [];
    const rlVehicleCount = 1 + roadVehicles.length;

    const cardX = 20;
    const cardY = 120;
    const cardW = 290;
    const cardH = 260;

    // Glassmorphic Card Background
    ctx.fillStyle = 'rgba(15, 23, 42, 0.94)';
    ctx.strokeStyle = this.isTraining() ? 'rgba(6, 182, 212, 0.7)' : 'rgba(16, 185, 129, 0.7)';
    ctx.lineWidth = 1.5;

    ctx.beginPath();
    ctx.roundRect(cardX, cardY, cardW, cardH, 10);
    ctx.fill();
    ctx.stroke();

    // Card Header Badge
    ctx.fillStyle = this.isTraining() ? '#06b6d4' : '#10b981';
    ctx.font = 'bold 12px "Segoe UI", sans-serif';
    ctx.fillText(`🧠 SHARED-POLICY MARL (${this.mode})`, cardX + 14, cardY + 22);

    ctx.fillStyle = '#10b981';
    ctx.font = 'bold 11px "Segoe UI", sans-serif';
    ctx.fillText(`TRAFFIC RL: ON  |  RL VEHICLES: ${rlVehicleCount} (${roadVehicles.length} Traffic + Ego)`, cardX + 14, cardY + 40);

    ctx.fillStyle = '#94a3b8';
    ctx.font = '11px "Segoe UI", sans-serif';
    ctx.fillText(`Episode: ${m.episode}  (Step: ${m.step})`, cardX + 14, cardY + 58);
    ctx.fillText(`Reward: ${m.currentReward}  |  Avg(100): ${m.avgReward100}`, cardX + 14, cardY + 76);
    ctx.fillText(`Success (100): ${m.successRate100}  |  AvgLen: ${m.avgLength100}`, cardX + 14, cardY + 94);
    ctx.fillText(`Smoothness: SteerΔ ${m.avgSteerChange100} | LatErr ${m.avgLateralError100}px`, cardX + 14, cardY + 112);
    ctx.fillText(`Collisions: ${m.totalCollisions}  |  Off-Road: ${m.totalOffRoad}`, cardX + 14, cardY + 130);
    ctx.fillText(`Goal Dist: ${m.distToGoal}  |  Best: ${m.bestReward}`, cardX + 14, cardY + 148);
    ctx.fillText(`Last End: ${m.lastReason}`, cardX + 14, cardY + 166);

    // Continuous Control Gauges
    ctx.fillText(`Steering: [ ${m.steer} ]`, cardX + 14, cardY + 190);
    ctx.fillText(`Throttle: [ ${m.throttle} ]  Brake: [ ${m.brake} ]`, cardX + 14, cardY + 210);

    // Mini Steering Meter Bar
    const barX = cardX + 150;
    const barY = cardY + 182;
    const barW = 115;
    const barH = 10;
    ctx.fillStyle = 'rgba(30, 41, 59, 0.9)';
    ctx.fillRect(barX, barY, barW, barH);
    ctx.strokeStyle = '#475569';
    ctx.strokeRect(barX, barY, barW, barH);

    // Center tick
    ctx.fillStyle = '#94a3b8';
    ctx.fillRect(barX + barW / 2 - 1, barY - 2, 2, barH + 4);

    // Steer fill
    const steerVal = parseFloat(m.steer);
    ctx.fillStyle = '#06b6d4';
    if (steerVal >= 0) {
      ctx.fillRect(barX + barW / 2, barY + 1, (steerVal * barW) / 2, barH - 2);
    } else {
      ctx.fillRect(barX + barW / 2 + (steerVal * barW) / 2, barY + 1, (-steerVal * barW) / 2, barH - 2);
    }

    ctx.restore();
  }
}

/* ============================================================================
   12. SIMULATION CORE APPLICATION CONTROLLER & GAME LOOP
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

    // Module 7: Reinforcement Learning Ego Agent & Training Manager
    this.rlAgent = new EgoRLAgent();
    this.rlTrainingManager = new RLTrainingManager(this, this.rlAgent);

    // Road Network & Global Route Planner Systems
    this.roadNetwork = RoadNetworkSystem;
    this.routePlanner = GlobalRouteSystem;

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

    // Auto-load existing RL policy model from localStorage if available
    if (this.rlAgent) {
      this.rlAgent.load();
    }

    // Compute initial Global Route to Destination
    if (this.routePlanner && this.egoVehicle) {
      this.routePlanner.planRoute(this.egoVehicle, EnvironmentData.destination);
    }

    // Start 60 FPS Continuous Simulation Loop
    this.lastTime = performance.now();
    requestAnimationFrame((time) => this.loop(time));

    console.log('[SimulationEngine] Modules 1-7 Active: Environment, Ego Vehicle, Traffic, Perception, Path Planner, Decision Maker, Autonomous Control & Reinforcement Learning loaded.');
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

      // Module 7 RL Mode Elements
      btnRlTrain: document.getElementById('btn-rl-train'),
      btnRlEval: document.getElementById('btn-rl-eval'),
      btnRlSave: document.getElementById('btn-rl-save'),
      btnRlLoad: document.getElementById('btn-rl-load'),

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

      // Global Route Navigation Telemetry Elements
      routePill: document.getElementById('route-pill'),
      routeNodeDisplay: document.getElementById('route-node-display'),
      routeProgressDisplay: document.getElementById('route-progress-display'),
      routeNavState: document.getElementById('route-nav-state'),

      // Controls
      btnRoute: document.getElementById('btn-route'),
      btnNetwork: document.getElementById('btn-network'),
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
        if (this.rlTrainingManager && this.rlTrainingManager.isActive()) {
          this.rlTrainingManager.setMode('OFF');
        }
        this.toggleAutonomousMode();
      });
    }

    // RL Training Button
    if (this.dom.btnRlTrain) {
      this.dom.btnRlTrain.addEventListener('click', () => {
        this.toggleRLTraining();
      });
    }

    // RL Evaluation Button
    if (this.dom.btnRlEval) {
      this.dom.btnRlEval.addEventListener('click', () => {
        this.toggleRLEvaluation();
      });
    }

    // RL Save Model Button
    if (this.dom.btnRlSave) {
      this.dom.btnRlSave.addEventListener('click', () => {
        this.saveRLModel();
      });
    }

    // RL Load Model Button
    if (this.dom.btnRlLoad) {
      this.dom.btnRlLoad.addEventListener('click', () => {
        this.loadRLModel();
      });
    }

    // Global Route Plan Debug Toggle Button
    if (this.dom.btnRoute) {
      this.dom.btnRoute.addEventListener('click', () => {
        this.toggleRoute();
      });
    }

    // Road Network Graph Debug Toggle Button
    if (this.dom.btnNetwork) {
      this.dom.btnNetwork.addEventListener('click', () => {
        this.toggleRoadNetwork();
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
        if (this.rlTrainingManager && this.rlTrainingManager.isActive()) {
          this.rlTrainingManager.resetEpisode();
        }
        if (this.routePlanner) {
          this.routePlanner.planRoute(this.egoVehicle, EnvironmentData.destination);
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

    // Keyboard Shortcuts (M: Mode, K: RL Train, E: RL Eval, O: Route, N: Network, Space: Stop/Manual, G: Grid, L: Labels, R: Reset View, T: Toggle Traffic, V: Radar, C: Paths)
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      const key = e.key.toLowerCase();
      if (key === 'm') {
        if (this.rlTrainingManager && this.rlTrainingManager.isActive()) {
          this.rlTrainingManager.setMode('OFF');
        }
        this.toggleAutonomousMode();
      } else if (key === 'k') {
        this.toggleRLTraining();
      } else if (key === 'e') {
        this.toggleRLEvaluation();
      } else if (key === 'o') {
        this.toggleRoute();
      } else if (key === 'n') {
        this.toggleRoadNetwork();
      } else if (e.code === 'Space') {
        // Emergency Stop & Return to Manual Teleoperation
        if (this.rlTrainingManager && this.rlTrainingManager.isActive()) {
          this.rlTrainingManager.setMode('OFF');
          this.updateModeUI(false, 'MANUAL');
        }
        if (this.autonomousController && this.autonomousController.isAutonomous) {
          this.autonomousController.toggleMode(false, this.egoVehicle);
          this.updateModeUI(false, 'MANUAL');
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
   * Toggle Global Route debug visualization
   */
  toggleRoute(forceState) {
    if (!this.routePlanner) return false;
    const state = this.routePlanner.toggleDebug(forceState);
    if (this.dom.btnRoute) {
      this.dom.btnRoute.classList.toggle('active', state);
    }
    return state;
  }

  /**
   * Toggle Road Network Graph debug visualization
   */
  toggleRoadNetwork(forceState) {
    if (!this.roadNetwork) return false;
    const state = this.roadNetwork.toggleDebug(forceState);
    if (this.dom.btnNetwork) {
      this.dom.btnNetwork.classList.toggle('active', state);
    }
    return state;
  }

  /**
   * Toggle between Manual and Autonomous driving mode
   */
  toggleAutonomousMode(forceState) {
    if (!this.autonomousController) return false;
    const isAuto = this.autonomousController.toggleMode(forceState, this.egoVehicle);
    this.updateModeUI(isAuto, isAuto ? 'AUTONOMOUS' : 'MANUAL');
    return isAuto;
  }

  /**
   * Toggle RL Training Mode
   */
  toggleRLTraining(forceState) {
    if (!this.rlTrainingManager) return false;
    const isTrain = (forceState !== undefined) ? forceState : (this.rlTrainingManager.mode !== 'TRAINING');
    if (isTrain) {
      if (this.autonomousController) this.autonomousController.toggleMode(false, this.egoVehicle);
      this.rlTrainingManager.setMode('TRAINING');
      this.updateModeUI(false, 'RL_TRAINING');
    } else {
      this.rlTrainingManager.setMode('OFF');
      this.updateModeUI(false, 'MANUAL');
    }
    return isTrain;
  }

  /**
   * Toggle RL Evaluation Mode
   */
  toggleRLEvaluation(forceState) {
    if (!this.rlTrainingManager) return false;
    const isEval = (forceState !== undefined) ? forceState : (this.rlTrainingManager.mode !== 'EVALUATION');
    if (isEval) {
      if (this.autonomousController) this.autonomousController.toggleMode(false, this.egoVehicle);
      this.rlTrainingManager.setMode('EVALUATION');
      this.updateModeUI(false, 'RL_EVALUATION');
    } else {
      this.rlTrainingManager.setMode('OFF');
      this.updateModeUI(false, 'MANUAL');
    }
    return isEval;
  }

  saveRLModel() {
    if (this.rlAgent) {
      this.rlAgent.save();
      console.log('[SimulationEngine] RL Policy Model saved to localStorage.');
    }
  }

  loadRLModel() {
    if (this.rlAgent) {
      const ok = this.rlAgent.load();
      if (ok) {
        console.log(`[SimulationEngine] RL Policy Model loaded. (Episodes: ${this.rlAgent.episodesTrained})`);
      }
    }
  }

  updateModeUI(isAuto, customLabel) {
    const label = customLabel || (isAuto ? 'AUTONOMOUS' : 'MANUAL');

    if (this.dom.btnMode) {
      this.dom.btnMode.classList.toggle('autonomous', isAuto);
      if (this.dom.btnModeText) {
        this.dom.btnModeText.textContent = isAuto ? 'Manual Mode' : 'Autonomous';
      }
    }

    if (this.dom.btnRlTrain) {
      this.dom.btnRlTrain.classList.toggle('active', label === 'RL_TRAINING');
    }

    if (this.dom.btnRlEval) {
      this.dom.btnRlEval.classList.toggle('active', label === 'RL_EVALUATION');
    }

    if (this.dom.badgeMode) {
      this.dom.badgeMode.textContent = label === 'RL_TRAINING' ? 'RL Training' : (label === 'RL_EVALUATION' ? 'RL Evaluation' : (isAuto ? 'Autonomous AI' : 'Manual Teleop'));
      this.dom.badgeMode.className = 'badge-mode ' + (label === 'RL_TRAINING' ? 'rl-train' : (label === 'RL_EVALUATION' ? 'rl-eval' : (isAuto ? 'autonomous' : 'manual')));
    }

    if (this.dom.modePill) {
      this.dom.modePill.className = 'telemetry-pill mode-pill ' + (label === 'RL_TRAINING' ? 'rl-train' : (label === 'RL_EVALUATION' ? 'rl-eval' : (isAuto ? 'autonomous' : 'manual')));
    }

    if (this.dom.modeDot) {
      this.dom.modeDot.className = 'pill-dot mode-dot ' + (label === 'RL_TRAINING' ? 'rl-train' : (label === 'RL_EVALUATION' ? 'rl-eval' : (isAuto ? 'autonomous' : 'manual')));
    }

    if (this.dom.modeValue) {
      this.dom.modeValue.textContent = label;
      this.dom.modeValue.className = 'pill-value ' + (label === 'RL_TRAINING' ? 'accent-cyan' : (label === 'RL_EVALUATION' ? 'accent-emerald' : (isAuto ? 'accent-emerald' : 'accent-cyan')));
    }
  }

  /**
   * Main Simulation Loop
   */
  loop(currentTime) {
    const dt = Math.min((currentTime - this.lastTime) / 1000, 0.1);
    this.lastTime = currentTime;

    // 0. Update Global Route Plan (A* Route over Road Network)
    if (this.routePlanner) {
      this.routePlanner.update(this.egoVehicle, EnvironmentData.destination);
    }

    // 1. RL Agent Control OR Rule-based Autonomous / Manual Driving
    if (this.rlTrainingManager && this.rlTrainingManager.isActive()) {
      // RL Agent directly chooses continuous steering/throttle/braking actions
      this.rlTrainingManager.update(dt);
    } else {
      // Standard Autonomous / Manual Driving
      if (this.detectionManager) {
        this.detectionManager.update(
          this.egoVehicle,
          EnvironmentData,
          this.trafficManager ? this.trafficManager.getEntities() : [],
          dt
        );
      }

      if (this.pathPlanner) {
        this.pathPlanner.update(
          this.egoVehicle,
          EnvironmentData,
          this.getPerceptionData(),
          dt
        );
      }

      if (this.decisionManager) {
        this.decisionManager.update(
          this.egoVehicle,
          EnvironmentData,
          this.getPerceptionData(),
          this.getPathPlannerData(),
          this.trafficManager,
          dt,
          this.autonomousController ? this.autonomousController.isStuck : false
        );
      }

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

      if (this.egoVehicle) {
        this.egoVehicle.update(dt);
      }
    }

    // 2. Dynamic Traffic update (disabled during RL training for pure ego learning)
    const isTrafficDisabled = (this.rlTrainingManager && this.rlTrainingManager.isActive() && this.rlTrainingManager.disableDynamicTraffic);
    if (this.trafficManager && !isTrafficDisabled) {
      this.trafficManager.update(dt, this.egoVehicle, EnvironmentData);
    }

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

    // Update Global Route Navigation Telemetry
    if (this.routePlanner) {
      const nav = this.routePlanner.getNavTelemetry();
      if (this.dom.routeNodeDisplay) {
        this.dom.routeNodeDisplay.textContent = nav.currentNode ? `${nav.currentNode}` : '--';
      }
      if (this.dom.routeProgressDisplay) {
        this.dom.routeProgressDisplay.textContent = `${nav.progressPercent}% (${nav.distanceToGoal}px)`;
      }
      if (this.dom.routeNavState) {
        this.dom.routeNavState.textContent = nav.navState || 'TRACKING';
        this.dom.routeNavState.className = 'pill-value ' + (nav.navState === 'ARRIVED' ? 'accent-emerald' : (nav.isTurn ? 'accent-amber' : 'accent-cyan'));
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

    // Render Road Network Graph Overlay (Module Navigation Foundation - debug toggled with N)
    if (this.roadNetwork) {
      this.roadNetwork.renderDebug(this.ctx, this.camera);
    }

    // Render Global Planned Route Overlay (A* Route Path - debug toggled with O)
    if (this.routePlanner) {
      this.routePlanner.renderDebug(this.ctx, this.camera);
    }

    // Render Dynamic Traffic Objects (Module 3)
    if (this.trafficManager) {
      this.trafficManager.render(this.ctx);
    }

    // Render Perception Radar & Collision Risk Highlights (Module 4)
    if (this.detectionManager) {
      this.detectionManager.render(this.ctx, this.egoVehicle);
    }

    // Render Adaptive Candidate Paths & Selected Path (Module 5) - Only when not in RL mode
    if (this.pathPlanner && !(this.rlTrainingManager && this.rlTrainingManager.isActive())) {
      this.pathPlanner.render(this.ctx);
    }

    // Render Ego Vehicle (Module 2)
    if (this.egoVehicle) {
      this.egoVehicle.render(this.ctx);
    }

    // Render Real-time Reinforcement Learning HUD Overlay (Module 7)
    if (this.rlTrainingManager && this.rlTrainingManager.isActive()) {
      this.rlTrainingManager.renderHUD(this.ctx, this.camera);
    } else if (this.trafficManager) {
      // Render Shared-Policy Traffic RL HUD Badge in Manual / Autonomous mode
      const roadVehicles = this.trafficManager.getEntities().filter(e => e.type === 'car' || e.type === 'motorcycle' || e.type === 'auto_rickshaw');
      if (roadVehicles.length > 0 && this.ctx) {
        this.ctx.save();
        this.ctx.setTransform(1, 0, 0, 1, 0, 0);
        this.ctx.fillStyle = 'rgba(15, 23, 42, 0.90)';
        this.ctx.strokeStyle = 'rgba(16, 185, 129, 0.6)';
        this.ctx.lineWidth = 1.2;
        this.ctx.beginPath();
        this.ctx.roundRect(20, 120, 240, 52, 8);
        this.ctx.fill();
        this.ctx.stroke();

        this.ctx.fillStyle = '#10b981';
        this.ctx.font = 'bold 12px "Segoe UI", sans-serif';
        this.ctx.fillText('🤖 SHARED-POLICY TRAFFIC RL', 32, 140);
        this.ctx.fillStyle = '#94a3b8';
        this.ctx.font = '11px "Segoe UI", sans-serif';
        this.ctx.fillText(`TRAFFIC RL: ON  |  RL VEHICLES: ${1 + roadVehicles.length}`, 32, 158);
        this.ctx.restore();
      }
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

  getRLAgent() {
    return this.rlAgent;
  }

  getRLTrainingManager() {
    return this.rlTrainingManager;
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
   * Accessor for Adaptive Path Planner Data (Module 5)
   */
  getPathPlannerData() {
    return {
      plannerState: this.pathPlanner ? this.pathPlanner.plannerState : 'OPTIMAL',
      selectedPath: this.pathPlanner ? this.pathPlanner.selectedPath : null,
      candidatePaths: this.pathPlanner ? this.pathPlanner.candidatePaths : [],
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

  /**
   * Road Network System Accessors (Deterministic Road Network Foundation)
   */
  getRoadGraph() {
    return this.roadNetwork ? this.roadNetwork.getGraph() : null;
  }

  getNearestRoadNode(pos, filter) {
    return this.roadNetwork ? this.roadNetwork.getNearestNode(pos, filter) : null;
  }

  getNearestRoadSegment(pos, filter) {
    return this.roadNetwork ? this.roadNetwork.getNearestSegment(pos, filter) : null;
  }

  getConnectedRoadNodes(node) {
    return this.roadNetwork ? this.roadNetwork.getConnectedNodes(node) : [];
  }

  isOnRoad(pos) {
    return this.roadNetwork ? this.roadNetwork.isOnRoad(pos) : false;
  }

  /**
   * Global Route Planner Accessors (A* Route over Road Network)
   */
  getGlobalRoute() {
    return this.routePlanner ? this.routePlanner.getRoute() : null;
  }

  getRoutePlanner() {
    return this.routePlanner;
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
window.RoadNetworkSystem = RoadNetworkSystem;
window.RoadNetworkGraph = RoadNetworkGraph;
window.RoadNode = RoadNode;
window.RoadSegment = RoadSegment;
window.GlobalRoutePlanner = GlobalRoutePlanner;
window.GlobalRouteSystem = GlobalRouteSystem;
window.MLPNet = MLPNet;
window.EgoPPOAgent = EgoPPOAgent;
window.EgoRLAgent = EgoRLAgent;
window.SharedPPOPolicy = SharedPPOPolicy;
window.RLTrainingManager = RLTrainingManager;

// Global Direct Accessors
window.getRoadGraph = () => RoadNetworkSystem.getGraph();
window.getNearestRoadNode = (pos, filter) => RoadNetworkSystem.getNearestNode(pos, filter);
window.getNearestRoadSegment = (pos, filter) => RoadNetworkSystem.getNearestSegment(pos, filter);
window.getConnectedRoadNodes = (node) => RoadNetworkSystem.getConnectedNodes(node);
window.isOnRoad = (pos) => RoadNetworkSystem.isOnRoad(pos);
window.getGlobalRoute = () => SimulationEngine.getGlobalRoute();

// Boot on DOM Ready
document.addEventListener('DOMContentLoaded', () => {
  SimulationEngine.init();
});
