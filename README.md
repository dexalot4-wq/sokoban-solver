[![sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/dexalot4-wq/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/dexalot4-wq/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

# Room-Optimal Push Solver

A Sokoban solver for the [sokoban-solver-01-run-20261121 bounty on make.fun](https://make.fun/bounty/sokoban-solver-01-run-20261121): one program answers 5 levels in a single run, in as few pushes as possible.

## How it works

The generator builds every level as a hallway plus 1–4 rooms, each room holding 3–4 boxes and as many goals behind a single door; a box pushed out of a room can never come back, so the rooms are independent and the level's push optimum is the sum of the rooms' optima. This solver therefore:

1. **Splits the level into rooms** — it cuts the hallway row's dead squares out of the floor and takes each remaining component holding boxes as a room.
2. **Solves every room push-optimally** with A* over pushes. A state is the sorted box squares plus the player's reachable region (named by its lowest square). The heuristic is the cheapest assignment of boxes to goals under walking distances that ignore boxes (admissible: one push moves one box one square). Two deadlocks are pruned, both safe: dead squares (no push sequence from them reaches a goal) and frozen 2×2 blocks of walls and boxes.
3. **Walks the player from room to room**, stitching the rooms' move strings into one answer per level.

Fallbacks, in order, if a room's A* ever hits its work cap: plain push BFS, then a move-based A* (any solution beats none). Everything is budgeted by work, never by the clock — fixed expansion caps, fixed direction order, no randomness — so two runs write byte-identical answers.

## Results

Practice set (kit's `grade-local`): **score 1,000,000** — every instance solved in exactly its push-optimal count:

| instance | pushes | P\* |
|----------|--------|-----|
| i001 | 13 | 13 |
| i002 | 16 | 16 |
| i003 | 29 | 29 |
| i004 | 47 | 47 |
| i005 | 60 | 60 |

The house baseline (plain-astar 1.0.0) scores 200,000 on the same set (only i001). The solver was also run on 16 freshly drawn sets from random seeds: 1,000,000 on every one, each set in ~0.7 s locally, far inside the 60 s wall budget.

## Run it

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` is plain JavaScript in a `.ts` file (Node 22 type stripping), built-ins only, no network, no dependencies.
