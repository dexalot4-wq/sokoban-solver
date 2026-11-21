// Room-Optimal Push Solver for sokoban-solver-01-run-20261121.
//
// Idea: the generator builds each level as a hallway plus 1-4 rooms, each
// room holding 3-4 boxes and as many goals, joined to the hallway by a single
// door; a box pushed out of a room can never come back, so the rooms are
// independent and the level's push optimum is the sum of the rooms' optima.
// This program therefore splits a level into rooms and solves every room
// push-optimally on its own, then walks the player from room to room.
//
// Per room it runs A* over pushes: a state is the sorted box squares plus the
// player's reachable region (named by its lowest square). The heuristic is the
// cheapest assignment of boxes to goals under walking distances that ignore
// boxes (admissible: one push moves one box one square, so it can lower the
// assignment cost by at most one). Two deadlocks are pruned, both safe (they
// never cut a solvable state): a box on a dead square (one from which no push
// sequence, ignoring the other boxes, reaches any goal) and a frozen 2x2
// block of walls and boxes holding a box off its goal.
//
// Everything is budgeted by work, never by the clock: fixed expansion caps,
// fixed direction order, no randomness, no wall-clock reads, so two runs
// write byte-identical answers.
//
// Written from scratch for this competition. No code is copied from the kit.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions in a fixed order: up, down, left, right. The first one wins ties.
const DIRS = [
  { dx: 0, dy: -1, walk: 'u', push: 'U' },
  { dx: 0, dy: 1, walk: 'd', push: 'D' },
  { dx: -1, dy: 0, walk: 'l', push: 'L' },
  { dx: 1, dy: 0, walk: 'r', push: 'R' },
];

// Work budgets (expansions), never the clock.
const ASTAR_CAP = 250_000;
const BFS_CAP = 250_000;
const MOVE_ASTAR_CAP = 200_000;

/** Parse XSB text into board geometry. Short rows pad with walls; the outer frame is wall. */
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const size = w * h;
  const wall = new Uint8Array(size).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  goals.sort((a, b) => a - b);
  boxes.sort((a, b) => a - b);
  return { w, h, size, wall, goals, boxes, player };
}

/** Squares from which a lone box can be pushed onto some goal (pull the goals back over open floor). */
function liveSquares(level) {
  const { w, wall, goals, size } = level;
  const live = new Uint8Array(size);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (const dir of DIRS) {
      const d = dir.dy * w + dir.dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/** Walking distances from `from` over floor cells of `wall` (ignoring boxes). -1 where unreachable. */
function distancesFrom(wall, w, size, from) {
  const dist = new Int32Array(size).fill(-1);
  const queue = [from];
  dist[from] = 0;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    for (const dir of DIRS) {
      const q = p + dir.dy * w + dir.dx;
      if (!wall[q] && dist[q] < 0) {
        dist[q] = dist[p] + 1;
        queue.push(q);
      }
    }
  }
  return dist;
}

/**
 * The player's reachable region from `from` with `occupied` in the way.
 * Returns the lowest square of the region and marks it in `seen` with `stamp`.
 */
function regionLow(wall, w, occupied, from, seen, stamp, stack) {
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    for (const dir of DIRS) {
      const q = p + dir.dy * w + dir.dx;
      if (!wall[q] && !occupied[q] && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

/** True when the box just pushed to `t` sits in a frozen 2x2 block of walls and boxes with a box off its goal. */
function frozen2x2(wall, w, h, occupied, goal, t) {
  const tx = t % w;
  const ty = Math.floor(t / w);
  for (let oy = -1; oy <= 0; oy++) {
    for (let ox = -1; ox <= 0; ox++) {
      const ax = tx + ox;
      const ay = ty + oy;
      if (ax < 0 || ay < 0 || ax + 1 >= w || ay + 1 >= h) continue;
      const a = ay * w + ax;
      const b = a + 1;
      const c = a + w;
      const d = c + 1;
      if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
      if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
    }
  }
  return false;
}

/**
 * Split a level into rooms: cut the hallway row's dead squares out of the
 * floor (a box in the hallway reaches no goal), and each remaining component
 * holding boxes is a room. Null when the level is no set of rooms.
 */
function splitRooms(level, live) {
  const { w, h, wall, goals, boxes, player } = level;
  const row = Math.floor(player / w);
  const cut = new Uint8Array(level.size);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(level.size).fill(-1);
  const rooms = [];
  for (let s = 0; s < level.size; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const dir of DIRS) {
        const q = squares[i] + dir.dy * w + dir.dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g]].goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Shortest walk from `from` to `to` around walls and boxes, in lower-case letters. */
function walkTo(wall, w, occupied, from, to) {
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const dir of DIRS) {
      const q = p + dir.dy * w + dir.dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = dir.walk;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return null;
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

/** A binary heap on [f, -g, seq]: lowest f first, deeper g first, earliest first. */
class Heap {
  constructor() {
    this.a = [];
  }
  get size() {
    return this.a.length;
  }
  push(e) {
    const a = this.a;
    let i = a.length;
    a.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (Heap.less(a[i], a[p])) {
        const t = a[i]; a[i] = a[p]; a[p] = t;
        i = p;
      } else break;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= a.length) break;
        if (c + 1 < a.length && Heap.less(a[c + 1], a[c])) c++;
        if (!Heap.less(a[c], a[i])) break;
        const t = a[i]; a[i] = a[c]; a[c] = t;
        i = c;
      }
    }
    return top;
  }
  static less(x, y) {
    return x[0] < y[0] || (x[0] === y[0] && (x[1] < y[1] || (x[1] === y[1] && x[2] < y[2])));
  }
}

/**
 * Push-optimal A* for one small problem (a room on the whole board, or the
 * whole level as a fallback): {pushes, moves} or null when the cap is hit.
 * `sub` holds w, h, size, wall, goals (sorted), boxes (sorted), player.
 */
function solvePushAStar(sub, cap) {
  const { w, h, size, wall, goals, boxes, player } = sub;
  const n = boxes.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(sub);
  if (boxes.some((b) => !live[b])) return null;
  // Walking distance from every goal, for the assignment heuristic.
  const distTo = goals.map((g) => distancesFrom(wall, w, size, g));
  const perms = permutations(n);
  const heuristic = (bs) => {
    let best = Infinity;
    for (const perm of perms) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = distTo[perm[i]][bs[i]];
        if (d < 0) { s = Infinity; break; }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  };
  const solved = (bs) => bs.every((b) => goal[b]);

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const keyOf = (bs, low) => {
    let k = low;
    for (const b of bs) k = k * size + b;
    return k;
  };

  const stateBoxes = [boxes.slice()];
  const statePlayer = [player];
  const stateLow = [0];
  const parent = [-1];
  const pushFrom = [-1];
  const pushTo = [-1];
  const pushDir = [-1];
  const gScore = [0];
  const best = new Map();
  for (const b of boxes) occupied[b] = 1;
  const startLow = regionLow(wall, w, occupied, player, seen, ++stamp, stack);
  for (const b of boxes) occupied[b] = 0;
  stateLow[0] = startLow;
  best.set(keyOf(boxes, startLow), 0);
  const heap = new Heap();
  heap.push([heuristic(boxes), 0, 0, 0]);
  let seq = 1;
  let expanded = 0;
  let found = solved(boxes) ? 0 : -1;

  while (heap.size > 0 && found < 0) {
    const [, negG, , s] = heap.pop();
    const g = -negG;
    if (best.get(keyOf(stateBoxes[s], stateLow[s])) !== g) continue;
    if (expanded >= cap) return null;
    expanded++;
    const bs = stateBoxes[s];
    for (const b of bs) occupied[b] = 1;
    regionLow(wall, w, occupied, statePlayer[s], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < n && found < 0; bi++) {
      const b = bs[bi];
      for (let di = 0; di < 4; di++) {
        const dir = DIRS[di];
        const d = dir.dy * w + dir.dx;
        const behind = b - d;
        const t = b + d;
        if (seen[behind] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        if (frozen2x2(wall, w, h, occupied, goal, t)) {
          occupied[t] = 0;
          occupied[b] = 1;
          continue;
        }
        const next = bs.slice();
        next[bi] = t;
        next.sort((x, y) => x - y);
        const low = regionLow(wall, w, occupied, b, mark, ++stamp, stack);
        occupied[t] = 0;
        occupied[b] = 1;
        const key = keyOf(next, low);
        const ng = g + 1;
        const known = best.get(key);
        if (known !== undefined && known <= ng) continue;
        best.set(key, ng);
        const idx = stateBoxes.length;
        stateBoxes.push(next);
        statePlayer.push(b);
        stateLow.push(low);
        parent.push(s);
        pushFrom.push(b);
        pushTo.push(t);
        pushDir.push(di);
        gScore.push(ng);
        if (solved(next)) {
          found = idx;
          break;
        }
        heap.push([ng + heuristic(next), -ng, seq++, idx]);
      }
    }
    for (const b of bs) occupied[b] = 0;
  }
  if (found < 0) return null;
  return { pushes: gScore[found], moves: buildMoves(sub, stateBoxes, statePlayer, parent, pushFrom, pushTo, pushDir, found) };
}

/** Rebuild the move string of a found push path: walk to each push square, then push. */
function permutations(n) {
  if (n === 0) return [[]];
  const out = [];
  const used = new Array(n).fill(false);
  const cur = [];
  (function rec() {
    if (cur.length === n) {
      out.push(cur.slice());
      return;
    }
    for (let i = 0; i < n; i++) {
      if (used[i]) continue;
      used[i] = true;
      cur.push(i);
      rec();
      cur.pop();
      used[i] = false;
    }
  })();
  return out;
}

/** Rebuild the move string of a found push path: walk to each push square, then push. */
function buildMoves(sub, stateBoxes, statePlayer, parent, pushFrom, pushTo, pushDir, found) {
  const { w, wall } = sub;
  const path = [];
  for (let s = found; s > 0; s = parent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = sub.player;
  let boxes = sub.boxes;
  const occupied = new Uint8Array(sub.size);
  for (const s of path) {
    const from = pushFrom[s];
    const to = pushTo[s];
    const dir = DIRS[pushDir[s]];
    for (const b of boxes) occupied[b] = 1;
    const walk = walkTo(wall, w, occupied, player, from - (to - from));
    for (const b of boxes) occupied[b] = 0;
    if (walk === null) throw new Error('walk to push square failed');
    moves += walk + dir.push;
    player = from;
    boxes = stateBoxes[s];
  }
  return moves;
}

/** Plain push BFS (no heuristic) as a fallback when A* hits its cap. */
function solvePushBfs(sub, cap) {
  const { w, h, size, wall, goals, boxes, player } = sub;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(sub);
  if (boxes.some((b) => !live[b])) return null;
  const solved = (bs) => bs.every((b) => goal[b]);
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const keyOf = (bs, low) => {
    let k = low;
    for (const b of bs) k = k * size + b;
    return k;
  };
  const stateBoxes = [boxes.slice()];
  const statePlayer = [player];
  const parent = [-1];
  const pushFrom = [-1];
  const pushTo = [-1];
  const pushDir = [-1];
  const index = new Map();
  for (const b of boxes) occupied[b] = 1;
  index.set(keyOf(boxes, regionLow(wall, w, occupied, player, seen, ++stamp, stack)), 0);
  for (const b of boxes) occupied[b] = 0;
  let found = solved(boxes) ? 0 : -1;
  for (let head = 0; found < 0 && head < stateBoxes.length; head++) {
    if (head >= cap) return null;
    const bs = stateBoxes[head];
    for (const b of bs) occupied[b] = 1;
    regionLow(wall, w, occupied, statePlayer[head], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < bs.length && found < 0; bi++) {
      const b = bs[bi];
      for (let di = 0; di < 4; di++) {
        const dir = DIRS[di];
        const d = dir.dy * w + dir.dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        if (frozen2x2(wall, w, h, occupied, goal, t)) {
          occupied[t] = 0;
          occupied[b] = 1;
          continue;
        }
        const next = bs.slice();
        next[bi] = t;
        next.sort((x, y) => x - y);
        const low = regionLow(wall, w, occupied, b, mark, ++stamp, stack);
        occupied[t] = 0;
        occupied[b] = 1;
        const key = keyOf(next, low);
        if (index.has(key)) continue;
        if (stateBoxes.length >= cap) {
          for (const x of bs) occupied[x] = 0;
          return null;
        }
        index.set(key, stateBoxes.length);
        stateBoxes.push(next);
        statePlayer.push(b);
        parent.push(head);
        pushFrom.push(b);
        pushTo.push(t);
        pushDir.push(di);
        if (solved(next)) {
          found = stateBoxes.length - 1;
          break;
        }
      }
    }
    for (const b of bs) occupied[b] = 0;
  }
  if (found < 0) return null;
  return { pushes: pathLen(parent, found), moves: buildMoves(sub, stateBoxes, statePlayer, parent, pushFrom, pushTo, pushDir, found) };
}

function pathLen(parent, s) {
  let n = 0;
  for (let t = s; t > 0; t = parent[t]) n++;
  return n;
}

/** Move-based A* (fewest moves, not pushes) as a last resort: any solution beats none. */
function solveMoveAStar(sub, cap) {
  const { w, size, wall, goals, boxes, player } = sub;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const dist = (b) => {
    let m = Infinity;
    for (const g of goals) m = Math.min(m, Math.abs((g % w) - (b % w)) + Math.abs(Math.floor(g / w) - Math.floor(b / w)));
    return m;
  };
  const est = (bs) => bs.reduce((a, b) => a + dist(b), 0);
  const keyOf = (p, bs) => `${p}|${bs.join(',')}`;
  const solved = (bs) => bs.every((b) => goal[b]);
  const stateP = [player];
  const stateB = [boxes.slice()];
  const gScore = [0];
  const parent = [-1];
  const step = [''];
  const best = new Map([[keyOf(player, boxes), 0]]);
  const heap = new Heap();
  heap.push([est(boxes), 0, 0, 0]);
  let seq = 1;
  let expanded = 0;
  let found = solved(boxes) ? 0 : -1;
  while (heap.size > 0 && found < 0) {
    const [, negG, , s] = heap.pop();
    const g = -negG;
    if (best.get(keyOf(stateP[s], stateB[s])) !== g) continue;
    if (expanded >= cap) return null;
    expanded++;
    const p = stateP[s];
    const bs = stateB[s];
    for (const dir of DIRS) {
      const d = dir.dy * w + dir.dx;
      const q = p + d;
      if (wall[q]) continue;
      let nb = bs;
      const at = bs.indexOf(q);
      if (at >= 0) {
        if (wall[q + d] || bs.includes(q + d)) continue;
        nb = bs.slice();
        nb[at] = q + d;
        nb.sort((a, b) => a - b);
      }
      const key = keyOf(q, nb);
      const known = best.get(key);
      if (known !== undefined && known <= g + 1) continue;
      best.set(key, g + 1);
      const idx = stateP.length;
      stateP.push(q);
      stateB.push(nb);
      gScore.push(g + 1);
      parent.push(s);
      step.push(dir.walk);
      if (solved(nb)) {
        found = idx;
        break;
      }
      heap.push([g + 1 + est(nb), -(g + 1), seq++, idx]);
    }
  }
  if (found < 0) return null;
  let moves = '';
  let pushes = 0;
  for (let t = found; t > 0; t = parent[t]) {
    moves = step[t] + moves;
    if (stateB[parent[t]].includes(stateP[t])) pushes++;
  }
  return { pushes, moves };
}

/** Solve one room's sub-problem, trying stronger methods first. */
function solveRoom(sub) {
  return solvePushAStar(sub, ASTAR_CAP) ?? solvePushBfs(sub, BFS_CAP) ?? solveMoveAStar(sub, MOVE_ASTAR_CAP);
}

/** Advance the player square by replaying moves (walk letters move one square each). */
function endOf(w, from, moves) {
  let p = from;
  for (const ch of moves) {
    const c = ch.toLowerCase();
    p += c === 'u' ? -w : c === 'd' ? w : c === 'l' ? -1 : 1;
  }
  return p;
}

/** Solve a whole level: room by room when it splits, else one whole-level search. */
function solveLevel(text) {
  const level = parseLevel(text);
  const live = liveSquares(level);
  const rooms = splitRooms(level, live);
  if (rooms !== null) {
    let player = level.player;
    let moves = '';
    let pushes = 0;
    const currentBoxes = level.boxes.slice();
    for (const room of rooms) {
      const wall = level.wall.slice();
      // Every other room's boxes stand as walls (on their goals once solved).
      for (const other of rooms) {
        if (other === room) continue;
        for (const b of other.boxes) wall[b] = 1;
      }
      const sub = {
        w: level.w, h: level.h, size: level.size, wall,
        goals: room.goals.slice().sort((a, b) => a - b),
        boxes: room.boxes.slice().sort((a, b) => a - b),
        player,
      };
      const solved = solveRoom(sub);
      if (solved === null) return '';
      pushes += solved.pushes;
      moves += solved.moves;
      player = endOf(level.w, player, solved.moves);
      // The room's boxes now sit on its goals.
      for (let i = 0; i < room.boxes.length; i++) room.boxes[i] = room.goals[i];
    }
    return moves;
  }
  // No room split: one search over the whole level, then a move-based one.
  const whole = { ...level, goals: level.goals.slice(), boxes: level.boxes.slice(), player: level.player };
  const solved = solvePushAStar(whole, ASTAR_CAP) ?? solveMoveAStar(whole, MOVE_ASTAR_CAP);
  return solved === null ? '' : solved.moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
