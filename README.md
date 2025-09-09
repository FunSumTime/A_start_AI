# 8-Puzzle A* Solver

## Overview
This is a simple command-line program that solves the **8-puzzle** using the **A\*** search algorithm.  
It supports multiple heuristics and reports search metrics like nodes expanded, nodes generated, max frontier size, and solution cost.

- **State representation:** a flat 9-tuple/list in row-major order, with `0` as the blank.
- **Goal (default):** `(1,2,3,4,5,6,7,8,0)`

---

## Requirements
- Python **3.8+**
- No extra dependencies (only uses Python standard library).

---

## How to Run
From the folder containing `A_star_search.py`:

```bash
python3 A_star_search.py
```

## Heuristics

h0 — Zero: always returns 0 (Uniform-Cost Search).

h1 — Misplaced Tiles: counts tiles not in their goal positions (ignores the blank).

h2 — Manhattan Distance: sums the |row − goal_row| + |col − goal_col| for each tile (ignores the blank).
→ Recommended, strongest admissible heuristic here.
