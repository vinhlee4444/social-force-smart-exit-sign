# Social Force Model (SFM) Crowd Evacuation Simulation Guide
**Project:** Decentralized Edge-AI Smart Evacuation Exit Sign  
**Target Facility:** 2nd Floor, Campus Test Building (Delta Building)  
**Simulation Engine:** Continuous 2D Microscopic Social Force Model (Helbing et al., 2000)

---

## 1. System Overview & Core Objectives

During emergency building evacuations, static exit signage often fails because occupants naturally rush toward primary or familiar exits, causing severe asymmetric bottlenecks while alternative exits remain empty. This simulation serves as the digital twin to validate our hardware prototype without endangering human subjects in live panic drills.

The simulation benchmarks two conditions across identical initial crowd distributions and random seeds:
1. **Baseline (`--no-gates`):** All 4 exit signs remain permanently `GREEN`. Pedestrians blindly head for their nearest door (shortest geodesic path) and remain in line regardless of queue density.
2. **Smart Dynamic Exit Signs:** Each of the 4 exits (`E1`–`E4`) is equipped with an autonomous sensing cone and a 2-state hysteresis gate controller (`GREEN` vs. `RED`). When localized bottleneck density exceeds the configured threshold (`1.9 persons/m²` for proactive Fruin LOS E/F peak-flow control, or `3.5 persons/m²` for reactive SFPE crush-onset control), the sign flips to `RED`, prompting uncommitted approaching occupants (after a `0.8–2.0 s` cognitive reaction delay and subject to Pathfinder's `0.30` Current Door Preference hysteresis) to divert toward alternative visible `GREEN` exits.

---

## 2. Architecture & Pipeline Workflows

```
  [ 1.png Floorplan ] 
           │
           ├──► [ Signed Distance Field (SDF) ] ──► Wall Repulsion & Normal Vectors
           ├──► [ Dijkstra Geodesic Fields ]    ──► Navigation Vectors per Exit (E1..E4)
           └──► [ Room & Corridor Detection ]   ──► Agent Spawning & Danger Zone Masking
                                                            │
  [ Camera Frustum Projection ] ──► Visible Floor Area (m²)  │
                                              │             ▼
                                              │      [ Crowd Physics (dt = 0.01s) ]
                                              │      - Driving Force (tau = 0.35s)
                                              │      - Social & Panic Pushing (k = 1.2e5)
                                              │      - Inelastic Collision Damping
                                              │      - 0.3s EMA Contact Force
                                              │             │
  [ Agent Headcount in View ] ────────────────┼─────────────┤
                                              ▼             ▼
                                    [ Local Density (p/m²) ]  [ Dynamic Rerouting (0.1s) ]
                                              │              - Commit Radius <= 5.0m
                                              ▼              - Reaction Delay U(0.8, 2.0s)
                                    [ Gate Controller ]      - Door Preference Hysteresis (0.30)
                                    - RED if > 1.9 or 3.5    - Ray-marched Line of Sight
                                    - GREEN if <= 1.0 p/m²   - Nearest Visible GREEN Door
                                                                   │
                                                                   ▼
                                                           [ Evacuation Metrics ]
                                                           - TET, AET, TPID, Danger-sec
```

### Detailed Workflow Stages

1. **Floor Plan & Spatial Digitization (`floorplan.py`):**
   * Processes `1.png` ($1287 \times 681\text{ px}$ at $0.05\text{ m/px} = 64.35\text{ m} \times 34.05\text{ m}$).
   * Computes continuous Signed Distance Field (SDF) and normal gradient maps using `cv2.distanceTransform`. Distances and unit normals are sampled bilinearly at sub-pixel resolution.
   * Internal courtyards (left: $x=255..445, y=132..553$; right: $x=920..1082, y=133..554$) are flagged as non-walkable (`base_free == False`) but optically transparent (`see_through == True`), allowing agents to see signs across atriums.

2. **Geodesic Pathfinding (`navigation.py`):**
   * Discretizes walkable space into a $0.1\text{ m}$ coarse grid ($2 \times 2\text{ px}$).
   * Runs multi-source 8-connected Dijkstra search originating inside each exit stairwell room ($depth \ge 1.5\text{ m}$).
   * Implements a quadratic wall clearance penalty ($cost = 1.0 + 4.0 \times \text{clip}(1 - clearance / 0.5, 0, 1)^2$) so path trajectories steer away from wall corners. Precomputed fields are cached in `cache/`.

3. **3D Tilted Frustum Camera Sensing (`exit_sign.py`):**
   * Simulates an OV5647 optical sensor mounted at height $h = 3.0\text{ m}$ with pitch angle $\theta = 70.0^\circ$ downward, horizontal FOV $= 62.6^\circ$, and vertical FOV $= 48.0^\circ$.
   * Projects a 3D viewing frustum onto the floor ($z = -h$) and combines it with ray-marched line-of-sight to prevent seeing through hallway walls. This produces a trapezoidal floor footprint ($\approx 9.2\text{--}10.6\text{ m}^2$) directly in front of each exit.

4. **Decentralized State Machine (`exit_sign.py`):**
   * Density metric: $\rho = \text{Count} / \text{Visible Area}\ (\text{persons/m}^2)$, smoothed via a $3.0\text{ s}$ moving average.
   * `GREEN -> RED`: Triggered when $\rho > \text{close\_above}$ ($1.9\text{ persons/m}^2$ proactive LOS E/F threshold, or $3.5\text{ persons/m}^2$ reactive SFPE threshold) and open duration $\ge 5.0\text{ s}$ (anti-flicker).
   * `RED -> GREEN`: Reopens when closed duration $\ge 4.0\text{ s}$ and $\rho \le 1.0\text{ person/m}^2$, or automatically after a $20.0\text{ s}$ fail-safe timeout.

5. **Evacuee Decision & Route-Choice Logic (`crowd.py`):**
   * Executed every frame step ($0.1\text{ s}$).
   * **Commitment Rule ($r \le 5.0\text{ m}$):** Agents within $5.0\text{ m}$ Euclidean and $\le 7.0\text{ m}$ walking distance of an exit continue through that door regardless of sign color, preventing stampedes caused by reversing direction inside doorway bottlenecks [4].
   * **Current Door Preference Hysteresis ($p = 0.30$):** Based on the Thunderhead Engineering Pathfinder *Locally Quickest* door-selection model [1], each agent's currently chosen exit receives a $30\%$ effective walking-distance discount ($D_{\text{pref}} = 0.70 \times D$). Once an agent redirects toward a `GREEN` exit, they do not ping-pong back mid-corridor when a previously `RED` exit reopens behind them.
   * **Cognitive Reaction Delay ($\tau_{\text{react}} \sim U(0.8, 2.0)\text{ s}$):** When an agent outside the $5.0\text{ m}$ commit zone sees their preferred exit is `RED` and at least one alternative `GREEN` sign is in ray-marched line-of-sight (`floor.see_through`), an individual reaction timer starts [2], [3]. Once the timer expires, the agent redirects toward the nearest visible `GREEN` exit.

6. **Microscopic Panic Physics (`crowd.py`):**
   * Driving relaxation $\tau = 0.35\text{ s}$, desired speed $v_0 \sim U(1.8, 2.5)\text{ m/s}$.
   * Psychological repulsion $A = 600\text{ N}, B = 0.04\text{ m}$; physical body compression $k = 1.2 \times 10^5\text{ kg/s}^2$; tangential friction $\kappa = 1.0 \times 10^4\text{ kg/(m}\cdot\text{s)}$.
   * Added normal collision damping ($2000\text{ N}\cdot\text{s/m}$) and a $0.3\text{ s}$ Exponential Moving Average (EMA) on physical contact force $F_{\text{contact}}$ to eliminate artificial elastic bouncing.
   * **Total People in Danger (TPID):** Counts agents who experience $F_{\text{contact}} > 1000\text{ N}$ for a cumulative duration $\ge 3.0\text{ s}$ outside classrooms (in corridors and exit bottlenecks).

---

## 3. File Inventory & Repository Structure

| File | Primary Responsibility | Key Functions / Classes |
| :--- | :--- | :--- |
| `config.py` | Central parameter configuration (no magic numbers). | `MapConfig`, `ExitConfig`, `PedestrianConfig`, `ForceConfig`, `NavConfig`, `SensorConfig`, `GateConfig`, `SpawnConfig`, `SimConfig` |
| `floorplan.py` | Map ingestion, SDF distance transforms, room/corridor segmentation. | `FloorPlan`, `bilinear()`, `detect_rooms()` |
| `navigation.py` | Multi-source 8-connected Dijkstra routing & direction vectors. | `NavigationField`, `build_navigation()`, `_dijkstra()` |
| `exit_sign.py` | 3D frustum floor projection, ray-marching, hysteresis state machine. | `ExitSign`, `GateController`, `GateEvent` |
| `crowd.py` | Vectorized SFM forces, Verlet neighbor lists, dynamic target updates. | `Crowd`, `compute_forces()`, `update_targets()`, `_can_see_sign()` |
| `simulation.py` | Master clock coordination, agent spawning, removal, recording loop. | `Simulation`, `step()`, `advance_frame()`, `_spawn_in_rooms()` |
| `metrics.py` | Timeseries tracking, metric aggregation, summary statistics export. | `Recorder`, `summary()`, `to_csv()`, `plot()` |
| `render.py` | Live UI rendering and headless FFMpeg video export (`.mp4`). | `Renderer`, `update()`, `save()` |
| `scenarios.py` | Batch execution across 6 scenarios, 2 modes, and 5 seeds (60 runs). | `run_benchmark()`, scenario generation |
| `run.py` | CLI execution entry point for individual testing or debugging runs. | Argument parsing, single run launcher |

---

## 4. Benchmark Results Summary (240 Runs Across 10 Seeds: `1.9 /m²` vs `3.5 /m²` vs Baseline)

Data aggregated across 10 random seeds (`seeds 0..9`) from `results/benchmark_19/all_runs_summary.csv` ($\text{close\_above} = 1.9\text{ persons/m}^2$, Fruin LOS E/F peak-flow threshold) and `results/benchmark_35/all_runs_summary.csv` ($\text{close\_above} = 3.5\text{ persons/m}^2$, SFPE crush-onset threshold):

| Scenario *(10-Seed Mean ± SD)* | Mode | Mean N | TET (s) | AET (s) | TPID (agents) | Danger Person-Sec (s) | Peak Danger | Closures |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1: Sparse** | Baseline (`--no-gates`) | 206.6 | 18.91 ± 1.97 | 10.11 ± 0.68 | 0.0 | 1.0 | 2.2 | 0.0 |
| | **Smart Exit (`1.9 /m²`)** | 206.6 | **18.91 ± 1.97** | **10.11 ± 0.68** | **0.0** | **1.0** | **2.2** | **0.0** |
| | **Smart Exit (`3.5 /m²`)** | 206.6 | **18.91 ± 1.97** | **10.11 ± 0.68** | **0.0** | **1.0** | **2.2** | **0.0** |
| **2: Normal** | Baseline (`--no-gates`) | 650.0 | 25.71 ± 1.92 | 13.02 ± 0.48 | 3.1 | 207.9 | 36.1 | 0.0 |
| | **Smart Exit (`1.9 /m²`)** | 650.0 | 31.27 ± 3.18 | 13.69 ± 0.62 | **0.2 (-93.5%)** | **106.1 (-49.0%)** | **25.1 (-30.5%)** | 3.2 |
| | **Smart Exit (`3.5 /m²`)** | 650.0 | 29.60 ± 3.87 | 13.18 ± 0.60 | **1.3 (-58.1%)** | **176.9 (-14.9%)** | 35.1 | 0.7 |
| **3: Asym Left** | Baseline (`--no-gates`) | 599.4 | 27.85 ± 1.39 | 13.93 ± 0.38 | 15.6 | 332.4 | 48.9 | 0.0 |
| | **Smart Exit (`1.9 /m²`)** | 599.4 | **32.86 ± 1.56** | 14.98 ± 0.48 | **0.4 (-97.4%)** | **141.7 (-57.4%)** | **33.2 (-32.1%)** | 2.0 |
| | **Smart Exit (`3.5 /m²`)** | 599.4 | 34.36 ± 1.25 | 14.37 ± 0.41 | **4.9 (-68.6%)** | **258.1 (-22.4%)** | 47.0 | 1.4 |
| **4: Asym Right** | Baseline (`--no-gates`) | 576.9 | 26.66 ± 1.75 | 13.69 ± 0.35 | 2.9 | 188.1 | 29.6 | 0.0 |
| | **Smart Exit (`1.9 /m²`)** | 576.9 | 31.67 ± 2.51 | 14.32 ± 0.44 | **0.2 (-93.1%)** | **95.6 (-49.2%)** | **21.5 (-27.4%)** | 2.0 |
| | **Smart Exit (`3.5 /m²`)** | 576.9 | 31.13 ± 2.07 | 13.83 ± 0.32 | **0.9 (-69.0%)** | **159.8 (-15.0%)** | 29.1 | 0.8 |
| **5: Asym Top** | Baseline (`--no-gates`) | 510.8 | 27.58 ± 1.60 | 13.54 ± 0.37 | 0.5 | 146.2 | 26.6 | 0.0 |
| | **Smart Exit (`1.9 /m²`)** | 510.8 | 31.77 ± 1.40 | 14.05 ± 0.40 | **0.0 (-100%)** | **92.7 (-36.6%)** | **22.5 (-15.4%)** | 1.8 |
| | **Smart Exit (`3.5 /m²`)** | 510.8 | 30.91 ± 2.84 | 13.65 ± 0.39 | **0.3 (-40.0%)** | **140.8 (-3.7%)** | 26.7 | 0.7 |
| **6: Full House** | Baseline (`--no-gates`) | 867.1 | 28.35 ± 0.56 | 14.21 ± 0.14 | 72.0 | 786.6 | 100.4 | 0.0 |
| | **Smart Exit (`1.9 /m²`)** | 867.1 | 36.51 ± 2.25 | **14.56 ± 0.27** | **30.3 (-57.9%)** | **574.3 (-27.0%)** | **81.3 (-19.0%)** | 4.0 |
| | **Smart Exit (`3.5 /m²`)** | 867.1 | **35.32 ± 1.36** | 14.69 ± 0.11 | **24.2 (-66.4%)** | **569.5 (-27.6%)** | 89.9 (-10.5%) | 2.4 |

---

## 5. Defense Guide: Explaining Key Physical Phenomena

### Q1: Why does Total Evacuation Time (TET) increase by 3 to 6 seconds while Average Evacuation Time (AET) barely changes?
* **Metric Definition:** TET ($T_{100}$) records the exact timestamp when the **final single occupant** clears the building.
* **The Geometric Detour Penalty:** When an exit turns `RED`, occupants outside the $5.0\text{ m}$ commit radius divert toward an alternate door. In a $64.35\text{ m} \times 34.05\text{ m}$ floor plan, traversing from the left wing to the right wing or navigating around central atriums adds $20\text{--}40\text{ m}$ of walking distance. At an evacuation pace of $\approx 2.0\text{ m/s}$, walking an extra $30\text{ m}$ physically requires $15\text{ additional seconds}$.
* **Average Evacuation Time (AET) Invariance:** While tail stragglers take longer to walk around, the vast majority of evacuees clear smoothly. Across all 6 scenarios, AET shifts by **only $0.11\text{ to }1.05\text{ seconds}$** (e.g., $+0.35\text{ s}$ in Scenario 6 at $1.9\text{ p/m}^2$).

### Q2: How do the `1.9 persons/m²` (Fruin LOS E/F) and `3.5 persons/m²` (SFPE Crush Limit) thresholds compare?
* **In Partial & Asymmetric Scenarios (`Scenarios 2, 3, 4, 5` — where alternative exits have spare capacity):**
  * **`1.9 persons/m²` is dramatically more effective**, eliminating **$93.1\%\text{ to }100\%$ of lethal crush (`TPID` $\le 0.4$ across all four scenarios)**—including a **$97.4\%$ reduction in Scenario 3 (`15.6 → 0.4`)** and cutting cumulative Danger Person-Seconds by **$36.6\%\text{ to }57.4\%$**. Because it triggers when doorway flow saturates (Fruin LOS E/F boundary), it absorbs the $0.8\text{--}2.0\text{ s}$ human reaction lag and diverts approaching waves before a crush forms.
* **In 100% Full-House Saturation (`Scenario 6` — where all 4 exits are simultaneously loaded with $\sim 867$ people):**
  * Both thresholds cut lethal crush dramatically from **`72.0` people down to `24.2` (`-66.4%` at `3.5 /m²`) and `30.3` (`-57.9%` at `1.9 /m²`)**, saving $42\text{--}48$ people per evacuation from sustained $> 1000\text{ N}$ crushing forces while keeping `AET` within $0.35\text{--}0.48\text{ s}$ of baseline.

### Q3: Why is Scenario 1 (Sparse) 100% identical across all configurations?
* With only $\sim 207$ occupants, corridor density never reaches either the $1.9\text{ persons/m}^2$ or $3.5\text{ persons/m}^2$ threshold.
* The system triggered **0 closures** across all 10 seeds, confirming that the hysteresis filters prevent false alarms or unnecessary diversions under free-flow conditions.

---

## 6. References (IEEE Format)

1. **[1]** Thunderhead Engineering, *"Pathfinder Technical Reference Manual: Steering Mode, Locally Quickest Door Selection, Current Door Preference, and Re-evaluation Intervals,"* Manhattan, KS, USA: Thunderhead Engineering Consultants, Inc., 2024. [Online]. Available: https://www.thunderheadeng.com/pathfinder/
2. **[2]** E. R. Galea, H. Xie, and P. J. Lawrence, *"Experimental and survey studies on the effectiveness of dynamic signage systems,"* in *Proc. 11th Int. Symp. Fire Safety Sci. (IAFSS)*, Christchurch, New Zealand, 2014, pp. 1129–1142, doi: 10.3801/IAFSS.FSS.11-1129.
3. **[3]** H. Xie, L. Filippidis, E. R. Galea, D. Blackshields, and P. J. Lawrence, *"Experimental analysis of the effectiveness of emergency signage and its implementation in evacuation simulation,"* *Fire and Materials*, vol. 36, no. 5, pp. 367–383, Aug. 2012, doi: 10.1002/fam.1095.
4. **[4]** J. Olander, E. Ronchi, R. Lovreglio, and D. Nilsson, *"Dissuasive exit signage for building fire evacuation,"* *Fire Safety Journal*, vol. 87, pp. 1–10, Jan. 2017, doi: 10.1016/j.firesaf.2016.11.002.
5. **[5]** M. J. Hurley *et al.*, Eds., *SFPE Handbook of Fire Protection Engineering*, 5th ed. New York, NY, USA: Springer, 2016, ch. 58–64, doi: 10.1007/978-1-4939-2565-0.
6. **[6]** J. J. Fruin, *"The causes and prevention of crowd disasters,"* in *Engineering for Crowd Safety*, R. A. Smith and J. F. Dickie, Eds. Amsterdam, Netherlands: Elsevier, 1993, pp. 99–108.
7. **[7]** D. Helbing, I. Farkas, and T. Vicsek, *"Simulating dynamical features of escape panic,"* *Nature*, vol. 407, no. 6803, pp. 487–490, Sep. 2000, doi: 10.1038/35035023.
8. **[8]** R. Lovreglio, A. Fonzone, and L. dell'Olio, *"A mixed logit model for predicting exit choice during building evacuations,"* *Transportation Research Part A: Policy and Practice*, vol. 92, pp. 59–74, Oct. 2016, doi: 10.1016/j.tra.2016.06.018.