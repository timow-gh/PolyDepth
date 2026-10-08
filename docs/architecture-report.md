# PolyDepth / C2A / PQP: Architecture Report and Refactoring Plan

This report covers three repositories:

| Repo | Role | Size (src + headers) |
|---|---|---|
| **PQP** (`timow-gh/PQP`) | Base layer: bounding-volume hierarchies (OBB/RSS), collision, distance and tolerance queries | ~5.6k LOC |
| **C2A** (`timow-gh/C2A`) | Middle layer: continuous collision detection (CCD) and contact-feature extraction, built on top of PQP's internals | ~8.7k LOC (≈2.3k of it duplicated `.hbb` headers) |
| **PolyDepth** (`timow-gh/PolyDepth`) | Top layer: translational penetration depth (PD) between two triangle meshes | ~2.5k LOC + demo |

Part 1 describes what exists and how it fits together. Part 2 covers how the algorithm works, which bugs I found, and how I would refactor it.

Your impression is right: the code is convoluted. Most of that is research-prototype debt rather than difficult algorithms: copy-pasted blocks, dead code, global state, three different vector-math libraries, and an inheritance hack between C2A and PQP. The algorithm underneath is fairly compact.

---

# Part 1: Structure and architecture

## 1.1 Dependency stack

```
┌──────────────────────────────────────────────────────────────────┐
│ PolyDepthDemo (GLUT app)        CCDDemo (C2A)    PQP demos        │
└───────────────┬──────────────────────────────────────────────────┘
                │ uses MeshObject + PolyDepth()
┌───────────────▼──────────────────────────────────────────────────┐
│ PolyDepth  (static lib `polydepth`)                               │
│   PolyDepth(), SkinDepth(), FindMaximallyClearConfigure()         │
│   MeshObject (file loader + OpenGL drawing!)                       │
│   ClearConfiguration(s) (warm-start cache, ring buffer of 3)      │
└───────────────┬──────────────────────────────────────────────────┘
                │ C2A_Collide, C2A_Distance, C2A_Solve,
                │ C2A_QueryContactOnly, C2A_Model, Coord3D/Transform
┌───────────────▼──────────────────────────────────────────────────┐
│ C2A  (static lib `C2A_lib`, alias C2A::C2A)                        │
│   C2A_Model : PQP_Model  (extended BVH: C2A_BV, C2A_Tri)           │
│   C2A_Solve → C2A_QueryTimeOfContact → CA steps → BVH recursion    │
│   C2A_Collide / C2A_Distance / C2A_Tolerance (re-implemented PQP)  │
│   CInterpMotion(_Linear): motion interpolation + motion bounds     │
│   LinearMath.h: Coord3D, Matrix3x3, Quaternion, Transform          │
└───────────────┬──────────────────────────────────────────────────┘
                │ includes PQP *internal* headers: MatVec.h, BV.h,
                │ Build.h, BVTQ.h, TriDist.h, RectDist.h, OBB_Disjoint.h
┌───────────────▼──────────────────────────────────────────────────┐
│ PQP  (static lib `PQP`) — UNC Proximity Query Package v1.3        │
│   PQP_Model (tris[], BV b[]), PQP_Collide/Distance/Tolerance       │
│   BV (OBB + RSS), Build (top-down BVH), TriDist, RectDist          │
└──────────────────────────────────────────────────────────────────┘
```

**Build wiring.** Each repo has a `dependencies/` folder that pulls the layer below via CMake `FetchContent` from `github.com/timow-gh/<repo>.git` at `GIT_TAG master`. PolyDepth fetches both C2A and PQP. C2A fetches PQP again, and FetchContent de-duplicates the two by name. freeglut is fetched only for demos.

Notes on the build:
- `GIT_TAG master` makes builds non-reproducible. A push to C2A/PQP master changes PolyDepth's build silently.
- The PQP target exposes its whole `src/` directory as include path. C2A depends on headers that PQP does not install (`MatVec.h`, `Build.h`, `BVTQ.h`, `TriDist.h`, `RectDist.h`, `OBB_Disjoint.h`, `GetTime.h`). So C2A only builds against PQP's **build tree** and cannot use an installed PQP.
- **PolyDepth does not build on Linux.** `src/MeshObject.cpp` has `#include <gl/gl.h>` (lowercase path, Windows only), and the library never links OpenGL. I configured it locally with `-DFETCHCONTENT_SOURCE_DIR_*` pointing at the three checkouts. PQP and C2A compile. `PolyDepth.cpp` compiles. `MeshObject.cpp` fails with `fatal error: gl/gl.h: No such file or directory`.

## 1.2 PQP (bottom layer)

PQP is the classic UNC library (Larsen, Gottschalk et al., 1999). It is essentially unchanged; the fork only adds CMake and exports `TriContact` and `TriDistance`.

| File | Contents |
|---|---|
| `PQP.h`, `PQP_Internal.h` | `PQP_Model` (`Tri* tris`, `BV* b`, `last_tri`, `BeginModel/AddTri/EndModel`), result structs `PQP_CollideResult`, `PQP_DistanceResult`, `PQP_ToleranceResult`, and the API functions `PQP_Collide`, `PQP_Distance`, `PQP_Tolerance`, plus `TriContact`, `TriDistance` |
| `Build.cpp` | Top-down BVH construction: covariance → principal axes → split triangles by centroid along the main axis → recurse |
| `BV.cpp/.h` | `BV` node holding both an **OBB** (for collision) and an **RSS** (rectangle swept sphere, for distance) |
| `OBB_Disjoint.h` | Separating-axis OBB overlap test |
| `RectDist.h` | Rectangle–rectangle distance (RSS distance) |
| `TriDist.cpp` | Exact triangle–triangle distance with closest points |
| `MatVec.h` | C-style 3×3/3-vector macros and inlines (`MxM`, `MTxV`, `VmV`, …) |
| `BVTQ.h` | Priority queue for distance traversal |
| `PQP_Compile.h` | `PQP_REAL` (double) and BV-type switches |

Every query uses the same pattern: walk two BVHs together, recurse into the larger node, and prune with the BV test (overlap for collision, a distance lower bound for distance queries). At leaves, it runs the exact triangle test.

## 1.3 C2A (middle layer)

C2A ("Controlled Conservative Advancement", Tang, Kim & Manocha, ICRA 2009 / TVCG 2014) answers this question: two rigid bodies move from pose 0 to pose 1 along an interpolated path. At what time `t ∈ [0,1]` do they first touch, and which features touch?

### Files

| File | Contents |
|---|---|
| `include/C2A/C2A.h` | Public API: `C2A_Solve`, `C2A_Collide` (2 overloads), `C2A_Distance`, `C2A_Distance_At_ContactSpace`, `C2A_QueryTimeOfContact`, `C2A_QueryContact`, `C2A_QueryContactOnly`, `C2A_TriDistance`. Result types `C2A_TimeOfContactResult`, `C2A_ContactResult`, contact-feature types `ContactF`, `C2A_ContactFeature`, `C2A_ContactPair` |
| `include/C2A/C2A_Internal.h` | `class C2A_Model : public PQP_Model` (adds `trisConst`, center of mass, radius, **`max_clear_conf_`**, a PolyDepth concept stored in the C2A model), `C2A_DistanceResult : PQP_DistanceResult` (+ closest triangle ids) |
| `C2A_BV.h/.cpp` | `class C2A_BV : public BV` adds absolute orientation, RSS corners, sub-tree center of mass and *angular radius* (needed for motion bounds under rotation) |
| `C2A_Tri.h` | `struct C2A_Tri : Tri` adds `index_[3]`, the **original vertex indices** of the triangle. PolyDepth needs these to map contacts back to mesh vertices |
| `C2A_Build.cpp` | Copy of PQP's `Build.cpp` adapted to `C2A_BV`/`C2A_Tri` |
| `C2A_PQP.cpp` | `C2A_Model` methods (`BeginModel/AddTri/EndModel`), and **re-implementations of PQP's queries** (`C2A_Collide`, `C2A_Distance`, `C2A_Tolerance`, `C2A_Distance_At_ContactSpace`) |
| `C2A.cpp` | CCD core. Triangle distance with feature classification (`TriDist`/`C2A_TriDistance` returns *vertex/edge/face* features), the conservative-advancement loop (`C2A_QueryTimeOfContact`, `C2A_TimeOfContactStep`), BVH recursions `TOCStepRecurse_Dis` (general motion), `TOCStepRecurse_Dis_Translation` (pure translation), `TOCStepRecurse_Dis_contact` (collect all feature pairs within a threshold), and the top-level `C2A_Solve` |
| `InterpMotion.h/.cpp` | `CInterpMotion` (abstract) and `CInterpMotion_Linear`: linear translation + constant-axis rotation between two poses, `integrate(t)`, motion bounds (`computeTOC`, `CAonRSS`, `CAonNonAdjacentTriangles`) |
| `LinearMath.h` | Second math library (Bullet/SWIFT style): `Coord3D`, `Matrix3x3`, `Quaternion`, `Transform`. Operators: `%` = cross, `*` = dot, `^=` = apply transform |
| `ListTri.h` | **Function definitions in a header** for `ContactF` (included only by `C2A.cpp`; a second include would break the link) |
| `*.hbb` | Stale backup copies of `LinearMath.h` and `InterpMotion.h`. Dead files |

### The inheritance hack (important for understanding C2A)

`C2A_Model` inherits from `PQP_Model`, but it stores **`C2A_Tri[]` in the `Tri* tris` pointer and `C2A_BV[]` in the `BV* b` pointer**. The derived element types are larger than the base types, so any code that indexes `tris[i]` or `b[i]` through the base type uses the wrong stride. That has three consequences:

- C2A cannot call PQP's own `PQP_Collide`/`PQP_Distance` on a `C2A_Model`. PQP would walk the arrays with the wrong stride. So C2A **copied and re-implemented** all PQP queries in `C2A_PQP.cpp` and accesses elements through `child(n)`/`GetTriangle(i)`, which cast correctly. That copy is why C2A is so large.
- Some code still gets it wrong. `C2A_Model::ComputeCenterOfMass()` and `ComputeRadius()` (`C2A.cpp` ~2109–2167) iterate `tris[i]` with the base stride. So the model center of mass and radius are computed from garbage memory past the first triangle. They feed the rotational motion bounds.
- `PQP_Model::~PQP_Model` does `delete[] tris`/`delete[] b` through base pointers to derived arrays, which is undefined behaviour.

### C2A call sequence (`C2A_Solve`)

```
C2A_Solve(begin1, end1, model1, begin2, end2, model2, out_pose1, out_pose2, toc, ..., dres)
 ├─ build CInterpMotion_Linear motion1(begin1→end1), motion2(begin2→end2)
 ├─ global b_TanslationCCD = (both angular velocities < 1e-8)     ← global flag
 ├─ C2A_QueryTimeOfContact(motion1, motion2, dres, ...)
 │    loop (≤150): C2A_TimeOfContactStep
 │       ├─ translation-only:   TOCStepRecurse_Dis_Translation  (BVH CA, exact tri CA at leaves)
 │       └─ general motion:     TOCStepRecurse_Dis              (BVH CA with error bounds "controlled")
 │       advance t by the safe step `mint`; stop when distance ≤ d_delta or t ≥ 1
 ├─ if collision found: integrate motion to toc → out_pose1/out_pose2
 └─ C2A_QueryContact(threshold = 2*dist + 0.001)
        → TOCStepRecurse_Dis_contact: collect every triangle pair closer than threshold
          as ContactF{FeatureType_A/B (1=V,2=E,3=F), FeatureID_A/B (mesh vertex indices),
                      P_A, P_B (closest points), Distance}  into dres.cont_l
```

Conservative advancement in one sentence: if the current distance is `d`, and no point of the body can move more than `μ` along the whole path, then advancing by `Δt = d/μ` is guaranteed collision-free. Repeat until `d` is about 0. "Controlled" means the BVH traversal uses relaxed error bounds in early iterations, which is cheaper, and exact bounds near contact.

## 1.4 PolyDepth (top layer)

| File | Contents |
|---|---|
| `include/PolyDepth/PolyDepth.h` | `PolyDepth()` (main entry point, **20 parameters**), `SkinDepth()`, `FindMaximallyClearConfigure()`, `PolyDepthResult {kNoPenetration,kPenetration,kFailed}`, `PolyDepthReturnValue` |
| `include/PolyDepth/ClearConfiguration.h` | `ClearConfiguration` (a position relative to object 2, plus an optional distance), `ClearConfigurations` (fixed ring buffer of 3, used as a warm start between frames) |
| `include/PolyDepth/MeshObject.h` | `MeshObject`: loads `.tri`/`.off`, recenters to the center of mass, builds the `C2A_Model`, computes mass/inertia, **builds an OpenGL display list** and draws |
| `src/PolyDepth.cpp` | Everything above: `GuessContactFreeSpace`, `PolyDepth`, `SkinDepth`, `FindMaximallyClearConfigure` (1721 lines) |
| `src/MeshObject.cpp` | File parsing (the file is parsed **three times**: once in the constructor, then again by `ReadTriFile`/`ReadOffFile` for rendering), volume integrals for mass properties, GL |
| `PolyDepthDemo/` | GLUT app: plays two `.ani` pose sequences of two bunnies, calls `PolyDepth` every frame, draws the result |

### End-to-end call sequence (as used by the demo)

```
Setup
  MeshObject(file)
    parse vertices + triangle indices
    ComputeMassProperties → center_of_mass; vertices -= center_of_mass
    ReadTriFile/ReadOffFile (re-parse, build GL display list)
    C2A_Model::BeginModel / AddTri(p1,p2,p3, triId, i1,i2,i3) / EndModel
       └─ C2A_BuildModel (BVH), ComputeAbsTra
  [optional] FindMaximallyClearConfigure(vertices, model)    // writes model->max_clear_conf_

Per frame
  PolyDepth(&pose1, model1, &pose2, model2, last_tri1, last_tri2, verts1, verts2,
            out_pose1, out_pose2, localPD, feat1, feat2, globalPD, nIter,
            clear_conf, use_motion_coherence, use_max_clear_conf, contact_eps, cluster_res)
    1. C2A_Collide(FIRST_CONTACT) → not colliding? return kNoPenetration
    2. GuessContactFreeSpace(...)  → q_free  (a collision-free pose of object 1)
    3. loop (iterations ≤ 10 on the "separated" branch):
         a. OUT-PROJECTION:  C2A_Solve(q_free → q_in)     // CCD from free toward penetrating pose
                              → contact pose q_c + contact features
         b. build contact half-spaces in configuration space from features (VF / FV / EE)
         c. drop near-duplicate normals (dot > 0.99)
         d. IN-PROJECTION:   solve small LCP with one Gauss–Seidel sweep → d = Σ λ_i n_i
                              q_proj = q_in + d + margin
         e. if |d| > |q_c − q_in| → stop, answer = q_c
            C2A_Distance(q_proj):
              separated (≥ eps)   → q_free = q_proj, goto a
              in contact (0..eps) → stop
              penetrating         → one more C2A_Solve from just outside q_c, stop
    4. globalPD = q_c.translation − q_in.translation
    5. (local PDs: code exists but is compiled out by #ifdef LOCAL_PENETRATIION)
  demo: push the resulting pose into object1.clear_conf() for the next frame's warm start
```

### Which C2A/PQP functions PolyDepth actually uses

PolyDepth uses a small slice of the ~10k lines below it:

| Function | Where | Purpose |
|---|---|---|
| `C2A_Model` (Begin/AddTri with vertex ids/End) | MeshObject | BVH with vertex-index back-references |
| `C2A_Collide(PQP_CollideResult*, …, C2A_FIRST_CONTACT)` | PolyDepth, GuessContactFreeSpace | Boolean "in collision?" |
| `C2A_Distance` | PolyDepth, SkinDepth, FindMaximallyClearConfigure | Separation distance |
| `C2A_Solve` | PolyDepth | CCD + contact features |
| `C2A_QueryContactOnly` | SkinDepth, local-PD block | Contact features without motion |
| `Coord3D`, `Transform`, `Quaternion` | everywhere | Math |

Most of C2A (general-motion CCD, `C2A_Tolerance`, `C2A_Distance_At_ContactSpace`, `C2A_QueryTimeOfContact_TranslateCA`, screw motion, …) is unused by PolyDepth. In PolyDepth, begin and end poses always have **the same rotation**, so `C2A_Solve` always takes the translation-only path (`b_TanslationCCD = true`). The rotational CA code never runs.

---

# Part 2: How it works, problems, and how to refactor

## 2.1 The algorithm in plain terms (paper: Je et al., *PolyDepth*, SIGGRAPH/TOG 2012)

Translational penetration depth is the shortest translation `d` of object A that makes A and B interior-disjoint. Equivalently, in configuration space it is the distance from the current (penetrating) configuration `q_in` to the boundary of the configuration-space obstacle, the "contact space". That obstacle is the Minkowski difference `B ⊖ A`. It is non-convex for general meshes, so the code never builds it. PolyDepth instead walks along its surface locally:

1. **Find any free configuration `q_free`** (`GuessContactFreeSpace`). Candidates, in the code's order:
   - *Motion coherence*: the cached poses in `ClearConfigurations` from previous frames, tried nearest first. If one is still colliding, nudge it along the same direction (±k·bbox/10).
   - *Directional*: step along the centroid-difference direction, alternating ± in bbox/10 increments.
   - *Maximally clear configurations*: precomputed sample points far from the surface (`FindMaximallyClearConfigure`).
   - *Fallback*: push A far away along the centroid direction (`|bbox1|+|bbox2|`). Always free, but far.
   It takes the closest candidate that `C2A_Collide` reports as free.

2. **Out-projection (CCD).** Move A on a straight line from `q_free` to `q_in` with C2A. The first time of contact gives a configuration `q_c` exactly *on* the contact space. C2A also returns the pairs of features (vertex/edge/face) that are within a small threshold there.

3. **Local linearisation.** Each contact feature pair defines a plane in configuration space, a facet of the Minkowski difference:
   - vertex(A)–face(B) (`cl==6`) and face(A)–vertex(B) (`cl==10`): normal of the face, offset by the vertex.
   - edge(A)–edge(B) (`cl==8`): normal = cross product of the two edge directions.
   - `cl = 3*typeA + typeB` with 1=V, 2=E, 3=F. VV/VE/EV pairs are ignored.
   Each plane is stored as a point `p_i = a − b` and an outward normal `n_i`, so the local free space is approximately `{ x : n_i·x ≥ n_i·p_i }`, a convex polyhedral cone near `q_c`.

4. **In-projection (LCP).** Find the smallest translation `d` from `q_in` into that cone. The KKT conditions of `min ½|d|²  s.t.  n_iᵀ d ≥ b_i` give an LCP in the multipliers λ: `A λ − b ≥ 0, λ ≥ 0, complementary`, with `A_ij = n_i·n_j`, `b_i = n_i·p_i`, `d = Σ λ_i n_i`. The code runs one projected Gauss–Seidel sweep, then adds a safety margin `eps/2` along `d`.

5. **Check and iterate.** Measure the true distance at `q_in + d`:
   - clearly separated: the linearisation was conservative, so use it as the new, closer `q_free` and go back to step 2. The next CCD ends closer to the true PD.
   - within `eps`: done; this is a contact configuration.
   - still penetrating: the cone was not accurate (non-convexity). Do one more CCD and finish.
   If `|d|` is longer than the CCD result `|q_c − q_in|`, the CCD answer is already better, so it stops.

The result is a **local** optimum. Each iteration shortens the candidate PD, and the CCD result is always a valid upper bound because `q_c` is a real contact configuration.

**Local PD** (per contact region): contact points are clustered into a grid (`feature_clustering_resolution`). For each cluster, the global PD is projected onto that cluster's normal: `local = (PD·n) n`.

**Skin depth** (`SkinDepth`): for objects that are *not* penetrating but closer than `eps`, it reports a pseudo-PD `(eps − dist)` along the centroid direction, together with the contact features.

## 2.2 Bugs and suspicious code (found by reading; not all verified at runtime)

### PolyDepth
| # | Location | Problem |
|---|---|---|
| 1 | `MeshObject.cpp:2` | `#include <gl/gl.h>`: library doesn't compile on Linux; the core library also depends on OpenGL |
| 2 | `PolyDepth.cpp:837` | All local-PD output is compiled out (`#ifdef LOCAL_PENETRATIION`, misspelled and never defined). `local_penetration_*` are never filled by `PolyDepth()`, although the header documents them as outputs |
| 3 | `PolyDepth.cpp:1057, 1378` | Local-PD maps store `mnkn[jj]` as the normal. In `SkinDepth`, `mnkn` is freshly resized (zeros); in the `#ifdef` block it holds the *previous loop's* normals at unrelated indices. The correctly computed `normal` is ignored. So local PDs are always 0 or wrong |
| 4 | `PolyDepth.cpp:1161` | `SkinDepth`: the early "no penetration" branch builds `ret` but **doesn't return** |
| 5 | `PolyDepth.cpp:1177` | `SkinDepth` passes default (identity) transforms to `C2A_QueryContactOnly` instead of the real poses; `save_moving_transform` is identity too |
| 6 | `PolyDepth.cpp:672-673` | `number_of_contact -= degen_cnt;` executed **twice**; the shifting loop above it also skips the element that was shifted in |
| 7 | `PolyDepth.cpp:631-652` | The degeneracy test (`Length() <= 1e-15`) sits inside the inner loop, so a degenerate first normal (`kk==0`) is never rejected |
| 8 | `PolyDepth.cpp:521-627` | Feature pairs that are not VF/FV/EE (VV, VE, EV, …) leave `mnkn[jj]`/`mnkp[jj]` with stale values from the previous iteration (vector is only resized) |
| 9 | `PolyDepth.cpp:679-705` | "Projected Gauss–Seidel" is a single sweep. It clamps *other* λ_j to 0 but not λ_i itself, so negative multipliers can survive. It uses `float` arrays in a `double` code base and has a hard cap of 20 contacts (`float am[20][20]`). With 0 contacts left, `Normalize()` of a zero vector is called |
| 10 | `PolyDepth.cpp:752-827` | When the projected config lands in contact (`0 < dist < eps`), the loop breaks but `globalPD` is computed from `contact_configuration` (the *CCD* pose), not from `prj`. The better in-projection result is discarded. Worth checking against the paper |
| 11 | `PolyDepth.cpp:391` | `kNoPenetration` returns a hard-coded `distance_ = 0.01` (real distance commented out) |
| 12 | `GuessContactFreeSpace` | `directional_method_found` is initialised `true`, so it is always true, and `found_pose` can be used uninitialised. `init_config.Normalize()` runs before the zero-length check |
| 13 | global | `std::vector<Coord3D> kMaxClearConf` global; `printf` timing output in the hot path; `output_object_2_pose` documented as "no use" |
| 14 | `MeshObject` | Copy constructor shallow-copies `tri` → double `delete[]`; `c2a_model_` never deleted; `exit()` on I/O error; `name_[100]` + `strcpy` overflow |
| 15 | Demo | Draws `object2_` at `trans0` (object 1's result pose; invisible because both are bunnies); CMake copies only `bunny40k_1.ani` but the demo needs `bunny40k_2.ani` too; globals defined in `main.h` |

### C2A
| # | Location | Problem |
|---|---|---|
| 16 | `C2A_Internal.h` / `C2A_PQP.cpp` | Derived-array-through-base-pointer design (see 1.3). `ComputeCenterOfMass`/`ComputeRadius` index with the wrong stride; base destructor deletes derived arrays (UB) |
| 17 | `C2A_PQP.cpp:70` | `trisConst` is uninitialised in the constructor but `delete[]`d in the destructor (crash if `EndModel` was never called); `trisConst` is otherwise unused |
| 18 | `C2A.cpp:29` | `bool b_TanslationCCD` is a **global** set by `C2A_Solve`; the library is not thread-safe or re-entrant |
| 19 | `C2A_Solve` | Builds `dR1/dR2` and never uses them; `th_ca` ignored; always returns `TOCFound`; prints on every call |
| 20 | `C2A_TimeOfContactResult` | Only `cont_l` is initialised; all other fields (counters, `distance`, `last_tri*`) are garbage until set |
| 21 | `C2A.cpp` | `C2A_QueryTimeOfContact_TranslateCA` is a near-verbatim copy of `C2A_QueryTimeOfContact` and is unused |
| 22 | `ListTri.h` | Non-inline function definitions in a header |
| 23 | `*.hbb` | Dead backup headers |
| 24 | `last_tri` | Queries mutate the models (`o1->last_tri = …`); "const" queries have side effects |

### PQP
PQP itself is mostly fine. The problems are packaging: the CMake target installs only 5 headers while C2A needs ~7 more; `configure_package_config_file` is called before `include(CMakePackageConfigHelpers)`.

## 2.3 Why it feels convoluted (root causes)

1. **No separation of concerns.** One 400-line `while(true)` contains the CCD call, feature→plane conversion, deduplication, LCP solver, margin logic and termination logic, with control flow via `break`/`continue` from three places.
2. **Copy-paste instead of functions.** The "feature → plane" block exists three times (PolyDepth, SkinDepth, `#ifdef` block). The local-PD clustering exists twice. C2A duplicates PQP's queries and its own TOC loop.
3. **Three math libraries.** PQP raw `double[3][3]` + `MatVec.h` macros, C2A `LinearMath.h` (`Coord3D`/`Transform` with unusual operators `%` = cross, `^=` = transform), and the demo's own `MatVec.h`. Code converts between them constantly (`Transform2PQP` is defined three times).
4. **A 20-parameter C function as the API**, with output parameters, raw pointers, and caller-managed warm-start state (`last_triangle1/2`, `ClearConfigurations`).
5. **Hidden coupling.** PolyDepth looks up `object_1_vertices[FeatureID]`, which only works because MeshObject passed the *same* recentered vertex indices into `C2A_Model::AddTri`. Nothing enforces this. `max_clear_conf_` (a PolyDepth concept) lives in C2A's model.
6. **Research leftovers.** Commented-out code, `#if 1/#else`, Korean comments in a non-UTF-8 encoding, `int a = 0;` breakpoint hooks, debug `printf`s, names like `mnkn`, `tmfl1`, `tmtrp3`, `ifcfac`.
7. **Dead features.** Rotational CCD, tolerance queries, screw motion and other paths are never used by PolyDepth but make up most of C2A.

## 2.4 Refactoring plan

Order matters: **get a safety net first**, then fix and restructure. Each phase leaves a working build.

### Phase 0: Safety net (small, do first)
- Make the library build on Linux: move `MeshObject` (or at least its GL parts) out of the library into the demo; replace with a GL-free mesh loader.
- **Pin dependencies** to commit SHAs in `FetchContent_Declare` (or use `FETCHCONTENT_SOURCE_DIR_*` in CI with sibling checkouts).
- Add a **headless regression test** (no GLUT): load `bunny_noholes.tri`, play `bunny40k_1/2.ani`, record `(result, globalPD, iterations)` per frame to a golden file. Every later refactor must reproduce it (within tolerance), or the change must be an intentional, explained bug fix.
- Add simple analytic tests: two unit cubes overlapping by `δ` along x → PD = δ·x̂; sphere-ish meshes → PD ≈ r1+r2−|c1−c2|.
- Turn on warnings (`-Wall -Wextra`), ASan/UBSan in a CI job. Finding #16 should light up immediately.

### Phase 1: Fix the clear bugs
Items 2–9, 11–12, 14, 17, 20 from §2.2. They are local fixes. Item 10 (CCD pose vs. projected pose) and item 16 (C2A stride) change numerical results; land them as separate commits with updated golden files.

### Phase 2: Restructure PolyDepth into small units

Suggested layout:

```
include/polydepth/
  polydepth.h          // public API (below)
  mesh.h               // Mesh: vertices + triangles + BVH (owns the C2A model)
src/
  initial_guess.cpp    // GuessContactFreeSpace → strategies (warm start, directional, MCC, fallback)
  contact_space.cpp    // ContactF → HalfSpace{n, p}  (ONE implementation, unit-tested per VF/FV/EE case)
  lcp.cpp              // projected Gauss–Seidel with proper clamping & iteration/tolerance params
  polydepth.cpp        // the out-/in-projection loop, ~100 lines
  local_pd.cpp         // contact clustering + local PDs
  max_clear_config.cpp // FindMaximallyClearConfigure, returns a vector instead of writing globals
```

Suggested API:

```cpp
namespace polydepth {

struct Pose { Mat3 R; Vec3 t; };

class Mesh {                                    // immutable after construction
public:
  Mesh(std::vector<Vec3> vertices, std::vector<std::array<int,3>> triangles);
  // owns the C2A model; guarantees vertex indices used by contacts match `vertices()`
};

struct Options {
  double contact_tolerance = -1;                // <0 → auto (bbox / 150)
  int    max_iterations    = 10;
  bool   compute_local_pd  = false;
  double clustering_resolution = 5;
};

struct WarmStart {                              // replaces last_triangle1/2 + ClearConfigurations
  // opaque, owned by the caller, updated by the call
};

enum class Status { Separated, Penetrating, Failed };

struct LocalPD { Vec3 depth, point_on_a, point_on_b; };

struct Result {
  Status status;
  Vec3   global_pd;          // translation of A that separates A and B
  Pose   separated_pose_a;
  double distance;           // when Separated
  int    iterations;
  std::vector<LocalPD> local;
};

Result penetration_depth(const Mesh& a, const Pose& pa,
                         const Mesh& b, const Pose& pb,
                         const Options& = {}, WarmStart* = nullptr);
}
```

The main loop then reads like the paper:

```cpp
Pose q_free = initial_guess(...);
for (int it = 0; it < opt.max_iterations; ++it) {
  auto ccd   = continuous_contact(a, q_free, q_in, b, pb);      // out-projection
  auto cone  = build_contact_cone(ccd.features, a, b, ...);     // half-spaces
  Vec3 d     = project_onto_cone(cone, q_in);                   // in-projection (LCP)
  ...
}
```

### Phase 3: Slim down and fix C2A
- Replace inheritance with composition: `C2A_Model` *contains* `std::vector<C2A_Tri>` and `std::vector<C2A_BV>` and does not derive from `PQP_Model`. This removes the stride bug class.
- Make `b_TanslationCCD` a parameter or a property of the motion object.
- Delete unused code paths (or move them to an `experimental/` folder): `C2A_QueryTimeOfContact_TranslateCA`, `C2A_Tolerance`, `C2A_Distance_At_ContactSpace`, screw motion (if unused), `.hbb` files, `ListTri.h` → `.cpp`.
- Initialise result structs; make queries not mutate models (move `last_tri` coherence into an explicit cache object).
- Replace `printf` with an optional logging callback, off by default.

### Phase 4: One math library
Pick one vector/matrix type (Eigen, or a small header of your own) and use it in PolyDepth and C2A's public interfaces. Keep PQP's arrays only at the PQP boundary, behind one adapter. This removes `Transform2PQP` ×3, `MatVec.h` ×2, and the `%`/`^=` operator surprises.

### Phase 5: Repository structure (decision for you)
C2A is in practice a fork of PQP's internals, and PolyDepth uses ~10% of C2A. Options:
- **(a) Keep three repos**, pin versions, give PQP a proper installable target including the headers C2A needs. Least churn.
- **(b) Monorepo**: `third_party/pqp` (vendored, untouched), `c2a/` (trimmed to what PolyDepth needs), `polydepth/`. Easiest to refactor across the boundary and keep tests consistent. **I'd recommend this** if PolyDepth is the only consumer of C2A.

### Phase 6 (optional): Algorithmic improvements
- A proper PGS/Lemke solver with convergence check, or a direct solve for ≤3 active constraints.
- Handle VV/VE contact pairs (use the closest-point direction as the normal).
- Use the projected configuration when it is in contact (finding #10).
- Return the true separation distance for non-penetrating inputs (use `C2A_Distance`).
- Consider replacing the CCD + PQP layer with a maintained library (e.g. FCL or Coal/hpp-fcl provide BVH distance, CCD and contact queries). PolyDepth's own algorithm would then be ~500 lines on top.

---

## Appendix: Glossary of cryptic names

| Name | Meaning |
|---|---|
| `mnkp[i]` / `mnkn[i]` | Minkowski-space contact plane i: point / normal |
| `am`, `bv`, `lamv` | LCP matrix `A_ij = n_i·n_j`, vector `b_i = n_i·p_i`, multipliers λ |
| `tmfl1..3` | components of `d = Σ λ_i n_i` |
| `prj` | projected configuration `q_in + d` |
| `cl` | feature class `3*typeA + typeB` (6 = VF, 8 = EE, 10 = FV) |
| `save_moving_transform`, `moving_transform` | the input (penetrating) pose `q_in` of object 1 |
| `begin_transform_object_1` | current free pose `q_free` |
| `contact_configuration` | CCD result `q_c` |
| `absolute_contact_configuration` | contact tolerance ε (default bbox/150, comment says 0.0012·bbox) |
| `ifconf/ifcdis/ifcfac` | MCC candidates: position / distance / still-valid flag |
| `toc`, `mint` | time of contact; safe CA step |
| `b_TanslationCCD` | (misspelled) "translation-only CCD" global flag |
| `Coord3D a % b`, `a * b`, `p ^= T` | cross product, dot product, apply transform |
