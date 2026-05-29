// server.js
// Authoritative Game Server – 2D Top-Down Survival Maze
// Node.js + WebSockets, 30 Hz physics, 10 Hz broadcast, strict FOV culling

const WebSocket = require('ws');

// ===================== CONSTANTS =====================
const WORLD_W = 2100;
const WORLD_H = 2100;
const CELL_SIZE = 100;                 // each maze cell is 100x100
const GRID_COLS = 21;                  // 21 columns
const GRID_ROWS = 21;                  // 21 rows

const PLAYER_RADIUS = 15;
const ENEMY_RADIUS = 20;
const BULLET_RADIUS = 3;

const PLAYER_SPEED = 200;              // pixels per second
const ENEMY_SPEED = 120;
const BULLET_SPEED = 600;

const SHOOT_COOLDOWN_MS = 400;
const STUN_DURATION_MS = 2000;

const PHYSICS_TICK = 1000 / 30;        // ~33.33 ms
const BROADCAST_TICK = 1000 / 10;      // 100 ms
const FOV_HALF_ANGLE_DEG = 75;         // half of 150° cone
const FOV_HALF_ANGLE_RAD = (FOV_HALF_ANGLE_DEG * Math.PI) / 180;

const PATH_RECALC_INTERVAL_MS = 500;   // how often enemy recalculates A* path

// ===================== UTILITY FUNCTIONS =====================

/** 2D Manhattan distance */
function manhattan(col1, row1, col2, row2) {
  return Math.abs(col1 - col2) + Math.abs(row1 - row2);
}

/** Euclidean distance squared */
function distSq(x1, y1, x2, y2) {
  return (x1 - x2) ** 2 + (y1 - y2) ** 2;
}


function normalizeAngle(angle) {
  while (angle > Math.PI) angle -= 2 * Math.PI;
  while (angle < -Math.PI) angle += 2 * Math.PI;
  return angle;
}

/** Check if two line segments intersect (excludes collinear touches) */
function segmentsIntersect(p1, p2, p3, p4) {
  function ccw(A, B, C) {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
  }
  return (
    ccw(p1, p3, p4) !== ccw(p2, p3, p4) &&
    ccw(p1, p2, p3) !== ccw(p1, p2, p4)
  );
}

// ===================== MAZE GENERATION =====================

/**
 * Generates a perfect maze on a 20x20 grid using recursive backtracker (DFS).
 * @returns {boolean[][]} grid[col][row] – true = wall, false = passage.
 */
function generateMaze() {
  const grid = Array.from({ length: GRID_COLS }, () => Array(GRID_ROWS).fill(true));
  const stack = [];
  // Start carving from (1,1) to leave a solid border
  grid[1][1] = false;
  stack.push([1, 1]);

  const dirs = [[1, 0], [-1, 0], [0, 1], [0, -1]];

  while (stack.length > 0) {
    const [cx, cy] = stack[stack.length - 1];
    const neighbours = [];
    for (const [dx, dy] of dirs) {
      // Step by 2 to leave walls between passages
      const nx = cx + dx * 2, ny = cy + dy * 2;
      if (nx > 0 && nx < GRID_COLS - 1 && ny > 0 && ny < GRID_ROWS - 1 && grid[nx][ny]) {
        neighbours.push({nx, ny, mx: cx + dx, my: cy + dy});
      }
    }
    if (neighbours.length > 0) {
      const {nx, ny, mx, my} = neighbours[Math.floor(Math.random() * neighbours.length)];
      grid[mx][my] = false; // carve wall between
      grid[nx][ny] = false; // carve destination
      stack.push([nx, ny]);
    } else {
      stack.pop();
    }
  }
  
  // Remove some random walls to create loops and a more organic, less strict maze
  for (let i = 0; i < 40; i++) {
    const rx = 1 + Math.floor(Math.random() * (GRID_COLS - 2));
    const ry = 1 + Math.floor(Math.random() * (GRID_ROWS - 2));
    grid[rx][ry] = false;
  }
  
  return grid;
}

/**
 * Converts the boolean grid into an array of Wall AABBs.
 * Each wall cell becomes { x, y, w, h }.
 */
function wallsFromGrid(grid) {
  const walls = [];
  for (let col = 0; col < GRID_COLS; col++) {
    for (let row = 0; row < GRID_ROWS; row++) {
      if (grid[col][row]) {
        walls.push({
          x: col * CELL_SIZE,
          y: row * CELL_SIZE,
          w: CELL_SIZE,
          h: CELL_SIZE
        });
      }
    }
  }
  return walls;
}

// ===================== PATHFINDING (A*) =====================

/**
 * A* pathfinding on the maze grid.
 * @param {boolean[][]} grid - passable = false.
 * @param {number} sCol start col
 * @param {number} sRow start row
 * @param {number} eCol target col
 * @param {number} eRow target row
 * @returns {Array<{col:number, row:number}>|null} sequence of cells from start to target (excluding start)
 */
function findPath(grid, sCol, sRow, eCol, eRow) {
  if (
    sCol < 0 || sCol >= GRID_COLS || sRow < 0 || sRow >= GRID_ROWS ||
    eCol < 0 || eCol >= GRID_COLS || eRow < 0 || eRow >= GRID_ROWS
  ) return null;
  if (grid[sCol][sRow] || grid[eCol][eRow]) return null; // start or target blocked

  const open = [];
  const closed = Array.from({ length: GRID_COLS }, () => Array(GRID_ROWS).fill(false));
  const startNode = {
    col: sCol, row: sRow,
    g: 0,
    h: manhattan(sCol, sRow, eCol, eRow),
    f: 0,
    parent: null
  };
  startNode.f = startNode.g + startNode.h;
  open.push(startNode);

  const dirs = [[1, 0], [-1, 0], [0, 1], [0, -1]];

  while (open.length > 0) {
    // sort by f value (simple priority queue for small grid)
    open.sort((a, b) => a.f - b.f);
    const current = open.shift();

    if (current.col === eCol && current.row === eRow) {
      // reconstruct path (skip start)
      const path = [];
      let node = current;
      while (node.parent) {
        path.push({ col: node.col, row: node.row });
        node = node.parent;
      }
      return path.reverse();
    }

    closed[current.col][current.row] = true;

    for (const [dx, dy] of dirs) {
      const nx = current.col + dx, ny = current.row + dy;
      if (nx < 0 || nx >= GRID_COLS || ny < 0 || ny >= GRID_ROWS) continue;
      if (grid[nx][ny] || closed[nx][ny]) continue;

      const tentG = current.g + 1; // cost = 1 per cell
      const existing = open.find(n => n.col === nx && n.row === ny);
      if (!existing) {
        const h = manhattan(nx, ny, eCol, eRow);
        const node = {
          col: nx, row: ny,
          g: tentG, h, f: tentG + h,
          parent: current
        };
        open.push(node);
      } else if (tentG < existing.g) {
        existing.g = tentG;
        existing.f = tentG + existing.h;
        existing.parent = current;
      }
    }
  }
  return null; // no path
}

// ===================== COLLISION & PHYSICS =====================

/**
 * Circle vs AABB test. Returns true if the circle overlaps the rectangle.
 */
function circleVsAABB(cx, cy, radius, rect) {
  const closestX = Math.max(rect.x, Math.min(cx, rect.x + rect.w));
  const closestY = Math.max(rect.y, Math.min(cy, rect.y + rect.h));
  const dx = cx - closestX;
  const dy = cy - closestY;
  return (dx * dx + dy * dy) < (radius * radius);
}

/**
 * Resolves circle vs AABB collisions and applies sliding.
 * Modifies the entity's x, y, vx, vy in place.
 * @param {object} entity - must have {x, y, vx, vy}
 * @param {number} radius
 * @param {Array} walls
 */
function moveEntityWithCollision(entity, radius, walls) {
  for (let iter = 0; iter < 3; iter++) {
    let collided = false;
    for (const wall of walls) {
      // closest point on AABB to entity center
      const closestX = Math.max(wall.x, Math.min(entity.x, wall.x + wall.w));
      const closestY = Math.max(wall.y, Math.min(entity.y, wall.y + wall.h));
      const dx = entity.x - closestX;
      const dy = entity.y - closestY;
      const distSq = dx * dx + dy * dy;

      if (distSq < radius * radius && distSq > 0) {
        const dist = Math.sqrt(distSq);
        const overlap = radius - dist;
        // push out along the collision normal (from wall to entity)
        entity.x += (dx / dist) * overlap;
        entity.y += (dy / dist) * overlap;

        // zero out velocity component along normal to slide
        const dot = entity.vx * (dx / dist) + entity.vy * (dy / dist);
        entity.vx -= dot * (dx / dist);
        entity.vy -= dot * (dy / dist);
        collided = true;
      } else if (distSq === 0) {
        // entity center exactly on closest point (edge case)
        entity.x += radius;
        collided = true;
      }
    }
    if (!collided) break;
  }
}

function isEnemyVisible(player, enemy, walls) {
  const dx = enemy.x - player.x;
  const dy = enemy.y - player.y;
  const dist = Math.hypot(dx, dy);

  // The client renders a 360-degree base light up to 0.3 * FOV_RADIUS
  const inBaseCircle = dist <= (player.fovRadius * 0.3);

  const angleToEnemy = Math.atan2(dy, dx);
  let angleDiff = normalizeAngle(angleToEnemy - player.aim_angle);
  const inCone = Math.abs(angleDiff) <= (player.fov_half_rad * 1.4);

  // It must be in the base circle OR in the flashlight cone
  if (!inBaseCircle && !inCone) return false;

  // raycast: segment from player to enemy
  const p1 = { x: player.x, y: player.y };
  const p2 = { x: enemy.x, y: enemy.y };

  for (const wall of walls) {
    // test against each of the four edges of the AABB
    const topLeft = { x: wall.x, y: wall.y };
    const topRight = { x: wall.x + wall.w, y: wall.y };
    const botLeft = { x: wall.x, y: wall.y + wall.h };
    const botRight = { x: wall.x + wall.w, y: wall.y + wall.h };

    if (
      segmentsIntersect(p1, p2, topLeft, topRight) ||
      segmentsIntersect(p1, p2, botLeft, botRight) ||
      segmentsIntersect(p1, p2, topLeft, botLeft) ||
      segmentsIntersect(p1, p2, topRight, botRight)
    ) {
      return false; 
    }
  }
  return true;
}

const mazeGrid = generateMaze();
const walls = wallsFromGrid(mazeGrid);

const exitRect = {
  x: (GRID_COLS - 2) * CELL_SIZE,
  y: (GRID_ROWS - 2) * CELL_SIZE,
  w: CELL_SIZE,
  h: CELL_SIZE
};

const player = {
  x: CELL_SIZE * 1.5,
  y: CELL_SIZE * 1.5,
  aim_angle: 0,
  vx: 0,
  vy: 0,
  fov_half_rad: FOV_HALF_ANGLE_RAD,
  fovRadius: 400
};

let gameState = 'waiting'; // 'waiting', 'playing', 'win', 'lose'

function randomFarCell() {
  const openCells = [];
  for (let col = 0; col < GRID_COLS; col++) {
    for (let row = 0; row < GRID_ROWS; row++) {
      if (!mazeGrid[col][row]) openCells.push({ col, row });
    }
  }
  for (let i = openCells.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    [openCells[i], openCells[j]] = [openCells[j], openCells[i]];
  }
  for (const cell of openCells) {
    const cx = cell.col * CELL_SIZE + CELL_SIZE / 2;
    const cy = cell.row * CELL_SIZE + CELL_SIZE / 2;
    if (distSq(cx, cy, player.x, player.y) >= 800 * 800) {
      return { x: cx, y: cy, cell };
    }
  }
  const fallback = openCells[0];
  return {
    x: fallback.col * CELL_SIZE + CELL_SIZE / 2,
    y: fallback.row * CELL_SIZE + CELL_SIZE / 2,
    cell: fallback
  };
}

const enemySpawn = randomFarCell();
const enemy = {
  x: enemySpawn.x,
  y: enemySpawn.y,
  vx: 0,
  vy: 0,
  is_stunned: false,
  stun_end_time: 0,
  path: [],                    
  targetPlayerCell: null,      
  lastPathTime: 0
};

const bullets = [];            

let storedInput = {
  vx: 0,
  vy: 0,
  aim_angle: 0,
  is_shooting: false
};

let lastShotTime = 0;

function physicsUpdate() {
  if (gameState !== 'playing') return;

  const now = Date.now();

  player.vx = storedInput.vx * PLAYER_SPEED;
  player.vy = storedInput.vy * PLAYER_SPEED;
  player.aim_angle = storedInput.aim_angle;

  player.x += player.vx / 30;   
  player.y += player.vy / 30;
  moveEntityWithCollision(player, PLAYER_RADIUS, walls);

  if (storedInput.is_shooting && (now - lastShotTime) >= SHOOT_COOLDOWN_MS) {
    lastShotTime = now;
    const dirX = Math.cos(player.aim_angle);
    const dirY = Math.sin(player.aim_angle);
    bullets.push({
      x: player.x,
      y: player.y,
      vx: dirX * BULLET_SPEED,
      vy: dirY * BULLET_SPEED
    });
    broadcastSound('shoot');
  }

  if (enemy.is_stunned && now >= enemy.stun_end_time) {
    enemy.is_stunned = false;
  }

  if (!enemy.is_stunned) {
    const enemyCol = Math.floor(enemy.x / CELL_SIZE);
    const enemyRow = Math.floor(enemy.y / CELL_SIZE);
    const playerCol = Math.floor(player.x / CELL_SIZE);
    const playerRow = Math.floor(player.y / CELL_SIZE);

    const distToPlayer = Math.hypot(player.x - enemy.x, player.y - enemy.y);
    if (distToPlayer < CELL_SIZE * 1.5) {
      // Direct chase if very close (fixes pathfinding failures when in same cell)
      const dx = player.x - enemy.x;
      const dy = player.y - enemy.y;
      enemy.vx = (dx / distToPlayer) * ENEMY_SPEED;
      enemy.vy = (dy / distToPlayer) * ENEMY_SPEED;
    } else {
      if (
        now - enemy.lastPathTime > PATH_RECALC_INTERVAL_MS ||
        !enemy.targetPlayerCell ||
        enemy.targetPlayerCell.col !== playerCol ||
        enemy.targetPlayerCell.row !== playerRow
      ) {
        enemy.lastPathTime = now;
        enemy.targetPlayerCell = { col: playerCol, row: playerRow };
        const path = findPath(mazeGrid, enemyCol, enemyRow, playerCol, playerRow);
        enemy.path = path ? path.map(cell => ({
          col: cell.col,
          row: cell.row,
          x: cell.col * CELL_SIZE + CELL_SIZE / 2,
          y: cell.row * CELL_SIZE + CELL_SIZE / 2
        })) : [];
      }

      if (enemy.path.length > 0) {
        const wp = enemy.path[0];
        const dx = wp.x - enemy.x;
        const dy = wp.y - enemy.y;
        const distToWp = Math.sqrt(dx * dx + dy * dy);

        if (distToWp < 3) {
          enemy.path.shift();
        } else {
          enemy.vx = (dx / distToWp) * ENEMY_SPEED;
          enemy.vy = (dy / distToWp) * ENEMY_SPEED;
        }
      } else {
        enemy.vx = 0;
        enemy.vy = 0;
      }
    }

    enemy.x += enemy.vx / 30;
    enemy.y += enemy.vy / 30;
    moveEntityWithCollision(enemy, ENEMY_RADIUS, walls);
  } else {
    enemy.vx = 0;
    enemy.vy = 0;
  }

  for (let i = bullets.length - 1; i >= 0; i--) {
    const b = bullets[i];
    b.x += b.vx / 30;
    b.y += b.vy / 30;

    if (b.x < 0 || b.x > WORLD_W || b.y < 0 || b.y > WORLD_H) {
      bullets.splice(i, 1);
      continue;
    }

    let hitWall = false;
    for (const wall of walls) {
      if (circleVsAABB(b.x, b.y, BULLET_RADIUS, wall)) {
        hitWall = true;
        break;
      }
    }
    if (hitWall) {
      bullets.splice(i, 1);
      continue;
    }

    const d2 = distSq(b.x, b.y, enemy.x, enemy.y);
    if (d2 < (BULLET_RADIUS + ENEMY_RADIUS) ** 2) {
      bullets.splice(i, 1);
      enemy.is_stunned = true;
      enemy.stun_end_time = now + STUN_DURATION_MS;
      // reset enemy velocity
      enemy.vx = 0;
      enemy.vy = 0;
      broadcastSound('stun');
      continue;
    }
  }

  // Win / Lose Conditions
  if (gameState === 'playing') {
    if (!enemy.is_stunned && distSq(player.x, player.y, enemy.x, enemy.y) < (PLAYER_RADIUS + ENEMY_RADIUS + 10) ** 2) {
      gameState = 'lose';
      broadcastSound('kill');
    } else if (circleVsAABB(player.x, player.y, PLAYER_RADIUS, exitRect)) {
      gameState = 'win';
      broadcastSound('win');
    }
  }
}

function broadcastSound(soundId) {
  const msg = JSON.stringify({ type: 'sound', sound: soundId });
  for (const ws of viewers) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(msg);
    }
  }
}

function broadcastUpdate() {
  const visible = isEnemyVisible(player, enemy, walls);
  const dist = Math.hypot(enemy.x - player.x, enemy.y - player.y);

  const basePayload = {
    type: "update",
    gameState: gameState,
    player: {
      x: Math.round(player.x),
      y: Math.round(player.y),
      aim_angle: player.aim_angle
    },
    enemy_dist: Math.round(dist),
    enemy_stunned: enemy.is_stunned,
    bullets: bullets.map(b => ({
      x: Math.round(b.x),
      y: Math.round(b.y)
    }))
  };

  const payloadVisible = { ...basePayload };
  if (visible) {
    payloadVisible.enemy = {
      x: Math.round(enemy.x),
      y: Math.round(enemy.y),
      is_stunned: enemy.is_stunned
    };
  }
  const msg = JSON.stringify(payloadVisible);

  for (const ws of viewers) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(msg);
    }
  }
}

const wss = new WebSocket.Server({ port: 8080 });
const viewers = new Set();     
let controller = null;          


const initMsg = {
  type: "init",
  map_width: WORLD_W,
  map_height: WORLD_H,
  walls: walls,
  exit: exitRect
};
wss.on('connection', (ws) => {

  viewers.add(ws);
  ws.send(JSON.stringify(initMsg));

  ws.on('message', (data) => {

    console.log('Received input:', data.toString());

    try {
      const msg = JSON.parse(data);

      if (msg.type === 'config') {
        if (msg.fov) {
          player.fov_half_rad = (msg.fov / 2) * Math.PI / 180;
        }
        if (msg.fovRadius) {
          player.fovRadius = msg.fovRadius;
        }
      } else if (
        msg.vx !== undefined && msg.vy !== undefined &&
        msg.aim_angle !== undefined && msg.is_shooting !== undefined
      ) {
        if (gameState !== 'playing' && msg.is_shooting && !storedInput.is_shooting) {
          gameState = 'playing';
          player.x = CELL_SIZE * 1.5;
          player.y = CELL_SIZE * 1.5;
          const newSpawn = randomFarCell();
          enemy.x = newSpawn.x;
          enemy.y = newSpawn.y;
          enemy.is_stunned = false;
          enemy.path = [];
          enemy.lastPathTime = 0;
          enemy.targetPlayerCell = null;
          bullets.length = 0;
          broadcastSound('start');
        }

        if (controller && controller !== ws) {
          controller.close(); 
        }
        controller = ws;
        viewers.delete(ws);  

        storedInput.vx = msg.vx;
        storedInput.vy = msg.vy;
        storedInput.aim_angle = msg.aim_angle;
        storedInput.is_shooting = msg.is_shooting;
      }
    } catch (e) {

    }
  });

  ws.on('close', () => {
    if (ws === controller) {
      controller = null;
      storedInput.vx = 0;
      storedInput.vy = 0;
      storedInput.is_shooting = false;
    }
    viewers.delete(ws);
  });

  ws.on('error', (err) => {
    console.error('WebSocket error:', err.message);
    if (ws === controller) {
      controller = null;
      storedInput.vx = 0;
      storedInput.vy = 0;
      storedInput.is_shooting = false;
    }
    viewers.delete(ws);
  });
});

setInterval(physicsUpdate, PHYSICS_TICK);
setInterval(broadcastUpdate, BROADCAST_TICK);

console.log('Authoritative server running on port 8080');
console.log(`Maze generated (${walls.length} wall cells). Player at (${player.x}, ${player.y})`);
console.log(`Enemy spawned at (${Math.round(enemy.x)}, ${Math.round(enemy.y)})`);