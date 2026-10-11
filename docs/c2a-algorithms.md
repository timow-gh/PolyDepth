# C2A: algorithm overview and theory

C2A answers one question: **two rigid triangle meshes move from pose 0 to pose 1; when (t ∈ [0,1]) do they first touch, and which features touch?** It also re-exports PQP's static queries for its own model type.

Contents: 1. Entry points · 2. Theory: rigid motions and interpolation (incl. screw motion) · 3. Theory: motion bounds · 4. Theory: conservative advancement · 5. The CCD algorithms in C2A · 6. When to use what · 7. Pitfalls · 8. Reading list

---

## 1. What exists (entry points)

### Static (single-pose) queries: PQP re-implemented for `C2A_Model`
| Function | Input | Output | Notes |
|---|---|---|---|
| `C2A_Collide(PQP_CollideResult*, R1,T1,o1, R2,T2,o2, flag)` | two poses | overlapping triangle pairs (or first one) | OBB tree + triangle overlap |
| `C2A_Collide(C2A_DistanceResult*, …, rel_err, abs_err)` | two poses | distance-style result, OBB pruning | rarely useful |
| `C2A_Distance(C2A_DistanceResult*, …, rel_err, abs_err, qsize)` | two poses, error bounds | distance, closest points, closest triangle ids | RSS tree, branch and bound; `qsize>2` = priority-queue traversal; warm-started from `model->last_tri` |
| `C2A_Tolerance(…, tolerance)` | two poses, ε | "closer than ε?" | RSS tree |
| `C2A_Distance_At_ContactSpace(…, upper_bound)` | two poses | distance, pruned at `upper_bound` | not used by PolyDepth |
| `C2A_QueryContactOnly(res, R1,T1,o1, R2,T2,o2, threshold)` | two poses, ε | **all feature pairs** (V/E/F + vertex ids + points) within ε | the contact-feature query |

### Continuous (motion) queries
| Function | Input | Output |
|---|---|---|
| `C2A_Solve(begin1,end1,o1, begin2,end2,o2, out1,out2, toc, nIter, nContact, th_ca, res)` | start and end pose of both bodies | TOC, both poses at TOC, contact features at TOC. **High-level entry point; use this.** |
| `C2A_QueryTimeOfContact(motion1, motion2, res, o1, o2, δ_d, δ_t)` | two `CInterpMotion` objects, tolerances | TOC (and `collisionfree`) |
| `C2A_QueryContact(motion1, motion2, res, o1, o2, threshold)` | motions placed at TOC | contact features |

### Motion models (`InterpMotion.h`)
| Class / mode | Status |
|---|---|
| `CInterpMotion_Linear`: linear translation + constant-axis rotation | **the only one implemented and used** |
| same class with `#define SCREW_MOTION` | screw motion; compiled out (the macro is never defined) |
| `GMP_IM_EULER`, `GMP_IM_SLERP`, `GMP_IM_LINEAR_*_CONTACT(S)` | enum values only, **no implementation** |

### Dead code (no public caller)
`ContactRecurse`, `ContactQueueRecurse`, `DeleteDuplicatePairs`, `C2A_ContactResult`, `C2A_QueryTimeOfContact_TranslateCA`.

---

## 2. Theory: rigid motions and how to interpolate them

**A pose** is an element of SE(3): rotation R plus translation T, so `x_world = R·x_body + T`.

**CCD needs a path, not just two poses.** Infinitely many motions go from pose 0 to pose 1. The collision answer depends on which one you pick, because a different path sweeps different space. The real motion between two simulation steps is unknown, so the path is a **modelling choice**. You want one that is plausible and that gives cheap, tight motion bounds (§3).

### 2.1 "Linear" motion (C2A default, `CInterpMotion_Linear`)
```
T(t) = T0 + t·(T1 − T0)                      # body origin moves on a straight line
R(t) = R0 · Rot(axis k, angle t·θ)           # constant angular velocity θ about a fixed axis k
       where (k, θ) = axis–angle of R0⁻¹R1  (equivalently: SLERP of the quaternions)
```
- Translation and rotation are decoupled. A body point p moves as `x(t) = T(t) + R(t)p`: a straight drift plus a circular motion around the moving origin.
- **It depends on the choice of body origin.** If you move the origin, the same two poses give a different path. C2A rotates about the model's coordinate origin. That's why PolyDepth's `MeshObject` recenters vertices at the centre of mass first.
- The velocities are constant: `v = T1 − T0`, `ω = θk`. That makes the bounds in §3 simple.

### 2.2 Screw motion (in C2A behind `SCREW_MOTION`)
**Chasles' theorem:** every rigid displacement can be written as a rotation by angle θ about some axis *line* L (direction s, through point p), plus a translation d *along that same line*. The screw motion interpolates exactly that:
```
x(t) = p + Rot(s, t·θ)·(x0 − p) + t·d·s
```
- Every point moves on a **helix** around L. Points on L slide straight. If d = 0 it's a pure rotation (points move on circles); if θ = 0 it's a pure translation.
- More than "a helix": it is the canonical one-parameter motion in SE(3). In Lie-group terms, `X(t) = X0 · exp(t · log(X0⁻¹X1))`, where the "twist" `log(...)` = (ω, v) is held constant. It is **coordinate-invariant**: it doesn't depend on where you put the body origin or the world frame. "Linear" motion (§2.1) equals the screw motion only when the translation is parallel to the rotation axis through the chosen origin.
- C2A's `CScrewMotion` computes exactly these parameters from two poses: axis direction `S`, angle `b`, point on axis `p = ½(t0 + t1 + cot(b/2)·S×Δt)`, axial distance `d = Δt·S`.
- Why it is attractive for CCD: the trajectories are simple (helices), and earlier work (Redon et al. 2002) solved screw-motion CCD with interval arithmetic. Why it is not the default: the bound is a bit more complex, and C2A's tests apparently used linear motion.

### 2.3 Others (not implemented)
- Euler-angle interpolation: depends on parameterisation, gives odd paths.
- SLERP + linear translation: the same as §2.1.
- Constrained ("sliding contact") motions: listed in the enum only.

**Pure translation** (R0 = R1) is the special case shared by all models: every point moves by the same `t·v`.

---

## 3. Theory: motion bounds

Conservative advancement needs μ, an upper bound on how fast any point of a body can approach the other body along a direction n.

For linear motion, point p (body coordinates, relative to the rotation centre) has velocity
```
ẋ = v + ω × (R(t)·p)            with |R(t)p| = |p| ≤ r      (r = max distance from the rotation centre)
```
Its speed along a unit direction n is at most
```
μ(n) = v·n + |ω × n| · r          (C2A: CInterpMotion_Linear::computeTOC / computeTOC_MotionBound)
```
- **Directional bound:** projecting onto n (the closest-point direction) is much tighter than the plain `|v| + |ω|·r`.
- **Per node:** r is computed per BV node (`C2A_BV::angularRadius`, measured from the model origin). Small nodes deep in the tree have small r, so their bounds are tight.
- **Both bodies move:** use `μ_A(n) + μ_B(−n)` (C2A adds both).
- Pure translation: `μ = v·n`.

The bound is valid for the whole interval only because v and ω are **constant** along the chosen interpolation. Other interpolations need their own bound; for screw motion, r becomes the distance from the screw axis.

---

## 4. Theory: conservative advancement (CA)

Origin: Mirtich (1996), for convex polytopes.
```
t = 0
loop:
    d, n = distance and closest direction at time t
    if d < δ: return t                       # TOC (from below, never overshoots)
    Δt = (d − margin) / μ(n)
    t += Δt ; if t > 1: return "no collision"
```
- **Safe:** no contact can happen inside a step, because a gap of d cannot close at speed ≤ μ in less than d/μ. CA never tunnels.
- **Iterative:** d/μ assumes the worst case, so each step undershoots and you repeat. The steps shrink as the bodies approach. Cost is about (number of iterations) × (one distance query).
- **Termination:** tolerance δ (`d_delta = 1e-4` in `C2A_Solve`) plus an iteration cap (150). The margin is `ConservD(d) = d − 0.1·δ`.
- **Pure translation and convex pieces:** d(t) is convex in t, and μ = v·n = −d'(t), so CA *is Newton's method on d(t)*. That converges fast and never overshoots.
- **With rotation:** d(t) is not convex, so the tangent step is unsafe and the global bound μ from §3 is needed. Convergence is slower, especially for large r·|ω|.

**Making it work for non-convex meshes:** run CA against a bounding-volume hierarchy. Every BV node pair gives its own lower bound on the TOC, `d_node/μ_node`. The global step is the minimum over the "front" of node pairs you have expanded. You don't have to expand to the leaves: an unexpanded node's Δt is already safe for everything inside it.

---

## 5. The CCD algorithms in C2A

`C2A_Solve` builds two `CInterpMotion_Linear` objects. It then picks one of two algorithms automatically with the **global** flag `b_TanslationCCD = (|ω1| < 1e-8 && |ω2| < 1e-8)`.

### 5.1 General motion (rotation + translation): Controlled hierarchical CA
`C2A_QueryTimeOfContact` → `C2A_TimeOfContactStep` → `TOCStepRecurse_Dis`
```
t = 0
repeat (≤150):                                          # OUTER loop = CA iterations in global time
    pose bodies at t
    Δt = 1 ; d = distance(last closest triangle pair)    # warm start
    choose error bounds (the "controlled" part, below)
    traverse(rootA, rootB):
        for each child pair (a, b):
            d_ab = RSS distance(a, b), n_ab = its direction
            Δt_ab = d_ab / (μ_A(n_ab, r_a) + μ_B(−n_ab, r_b))
            if d_ab is "close enough to matter" (within rel/abs error of d) and Δt_ab < remaining:
                 recurse into (a, b)                     # refine
            else Δt = min(Δt, Δt_ab)                     # stop here: node bound is safe
        at leaves: exact triangle distance, Δt = min(Δt, d_tri / μ)
    if d ≤ δ_d or Δt ≤ δ_t: TOC = t ; break
    t += Δt ; if t ≥ 1: collision-free
```
**"Controlled"** (the C2A contribution) is about choosing the error bounds:
- Early iterations, far from contact: `rel_err = 3` (iterations 1–2), then `0.5`, with `abs_err` effectively off. Node pairs are refined only if they could be much closer than the current best. The traversal stays shallow and cheap, Δt is smaller (looser), but it's still safe.
- Near contact (`Δt ≤ 0.005`, or `d ≤ 0.5`, or after 5 iterations): exact (`rel = abs = 0`).
- Trade-off: a few more but much cheaper CA iterations. Note that `d ≤ 0.5` is an **absolute, unit-dependent** threshold in the code.

### 5.2 Pure translation: per-pair CA, single outer step
`TOCStepRecurse_Dis_Translation`, with `CAonRSS` and `CAonNonAdjacentTriangles`
```
best_t = 1
traverse(a, b):
    t_ab = iterate CA on the two RSS volumes (d/(v·n), ≤50 iters) → lower bound for the pair
    if t_ab ≥ best_t: prune
    leaves: t_tri = iterate CA on the two triangles (Newton on convex d(t)) ; best_t = min(best_t, t_tri)
    else recurse into both children (closer first)
TOC = best_t                               # outer loop runs once
```
Here the iteration moves **inside** each pair, where the convexity argument makes `v·n` safe. The outer loop is no longer needed: the traversal directly returns the minimum TOC over all triangle pairs. PolyDepth always uses this path, because begin and end rotations are equal.

### 5.3 Contacts at the TOC
After the TOC, `C2A_Solve` integrates both motions to `toc` (output poses) and calls `C2A_QueryContact(threshold = 2·d + 0.001)`. That query (`TOCStepRecurse_Dis_contact`) collects **every** triangle pair within the threshold, as `ContactF { FeatureType_A/B ∈ {1 V, 2 E, 3 F}, FeatureID_A/B = mesh vertex ids, P_A, P_B, Distance }`.

---

## 6. When to use what

| Situation | Use | Why |
|---|---|---|
| "Do they overlap now?" | `C2A_Collide(PQP_CollideResult*, …, C2A_FIRST_CONTACT)` | OBB overlap is the cheapest test |
| "How far apart?" | `C2A_Distance` | RSS gives a good distance lower bound |
| "Which features are within ε?" | `C2A_QueryContactOnly` | returns typed features with vertex ids |
| Motion between two poses, **with rotation** | `C2A_Solve` (auto: §5.1) | needs motion bounds with angular radius |
| Motion between two poses, **translation only** | `C2A_Solve` (auto: §5.2) | convexity makes `v·n` safe; one traversal |
| Large rotations per step | consider splitting the interval | `|ω|·r` dominates μ, so CA takes tiny steps |
| You need coordinate-invariant paths | screw motion (`SCREW_MOTION`) or your own twist-based interpolation | linear motion depends on the body origin |

Inputs required in every case: built `C2A_Model`s (BeginModel/AddTri/EndModel), with vertices **relative to the intended rotation centre**. Poses are `Transform` (C2A_Solve) or R[3][3], T[3] (static queries).

---

## 7. Pitfalls and things to know

- **The answer depends on the interpolation** (§2). "No collision" means none along *this* path.
- **The TOC is a lower bound**, accurate to δ. It is the last safe time, not the exact touching time.
- **Global state.** `b_TanslationCCD` is a global; `model->last_tri` is mutated by queries. Not thread-safe.
- **Units.** `δ = 1e-4`, the threshold `0.001`, and `d ≤ 0.5` in the controller are absolute numbers. Scale your models or the tolerances.
- **Inheritance hack.** `C2A_Model` stores `C2A_Tri[]`/`C2A_BV[]` in PQP base pointers. Never call plain `PQP_*` functions on it. `ComputeCenterOfMass`/`ComputeRadius` read with the wrong stride. That doesn't affect motion bounds, which use per-node `angularRadius` from the origin.
- **Alternatives to CA:**
  - Root-finding the exact contact-time equations: cubic polynomials when vertices move linearly (Provot 1997, Bridson et al. 2002, common in cloth simulation).
  - Interval arithmetic or Taylor models on screw motions (Redon et al. 2002; Zhang et al. 2007).
  - Exact closed form for pure translation (ray cast against the Minkowski difference).
  CA wins on simplicity and robustness for general rigid motion.

---

## 8. Reading list (in suggested order)

1. **Rigid motions / screw theory:** Lynch & Park, *Modern Robotics*, ch. 3 (twists, exponential coordinates, Chasles), free PDF. Or Murray, Li & Sastry, *A Mathematical Introduction to Robotic Manipulation*, ch. 2.
2. **Interpolating rotations:** Shoemake 1985, *Animating rotation with quaternion curves* (SLERP).
3. **Conservative advancement:** B. Mirtich, PhD thesis 1996, *Impulse-based Dynamic Simulation of Rigid Body Systems* (UC Berkeley).
4. **CA for non-convex models:** Zhang, Lee, Kim, *Interactive continuous collision detection for non-convex polyhedra* (Visual Computer 2006, "FAST").
5. **C2A:** Tang, Kim, Manocha, *C2A: Controlled conservative advancement…* (ICRA 2009); extended in *Hierarchical and controlled advancement for CCD of rigid and articulated models* (IEEE TVCG 2014).
6. **Screw-motion CCD:** Redon, Kheddar, Coquillart, *Fast continuous collision detection between rigid bodies* (Eurographics 2002).
7. **Bounding volumes:** Larsen et al., *Fast proximity queries with swept sphere volumes* (UNC TR99-018); Gottschalk et al., *OBBTree* (SIGGRAPH 1996); Ericson, *Real-Time Collision Detection* (book).
