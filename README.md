# A* Pathfinding Engine

This repository contains a custom, high-performance A* search implementation with:

- **Indexed Min-Heap (custom priority queue)**
- **Dynamic edge-weight support**
- **Admissible and non-admissible heuristics**
- **Benchmarking engine with CSV output**

---

## Features

### ✔ Custom Priority Queue  
An optimized indexed binary min-heap supporting:
- O(log n) push/pop  
- Efficient decrease-key  
- Node re-opening for dynamic weight updates  

### ✔ Grid Graph with Dynamic Edge Weights  
- Supports obstacles  
- Optional diagonal movement  
- Per-edge weight overrides  
- Prevents diagonal corner cutting  

### ✔ Heuristics Included  
| Heuristic | Admissible | Notes |
|----------|------------|-------|
| Octile | ✔ Yes | Excellent for 8-direction grids |
| Weighted Manhattan | ❌ No | Faster but may overestimate |

---

## Benchmarking

Run automated benchmarks using:

```bash
python src/benchmark_runner.py
```

Default benchmark sizes:
- **30×30**
- **100×100**
- **200×200**

Outputs:
- Clean summary printed in the console  
- CSV written to: `results/benchmarks_runner_output.csv`

---

## How to Run

### 1. Clone the Repository
```bash
git clone https://github.com/alagupandi3181999/a-star-pathfinding.git
cd a-star-pathfinding
```

### 2. Run Benchmark
```bash
PYTHONPATH=src python src/benchmark_runner.py
```

On Windows PowerShell:
```powershell
$env:PYTHONPATH="src"; python src/benchmark_runner.py
```

---

## Project Structure

```
alagupandi3181999-a-star-pathfinding/
├── README.md
├── LICENSE
├── results/
│   └── benchmarks_runner_output.csv
└── src/
    ├── astar_dynamic.py
    └── benchmark_runner.py
```

---

## License

This project is released under the **MIT License**.

---

## Author

**alagupandi3181999**  
2025 - Custom A* Pathfinding Engine  
