# The algorithms behind PolyDepth: concise overview with pseudocode

Sources: the code in PolyDepth / C2A / PQP, and the papers:
- PolyDepth: Je, Tang, Lee, Lee, Kim, *PolyDepth: Real-Time Penetration Depth Computation Using Iterative Contact-Space Projection*, ACM TOG 31(1), 2012. [arXiv 1508.06181](https://arxiv.org/pdf/1508.06181)
- C2A: Tang, Kim, Manocha, *C2A: Controlled Conservative Advancement for Continuous Collision Detection of Polygonal Models*, ICRA 2009 (extended: TVCG 2014).
- PQP: Larsen, Gottschalk, Lin, Manocha, *Fast Proximity Queries with Swept Sphere Volumes*, TR99-018, and Gottschalk et al., *OBBTree*, SIGGRAPH 96.

> The paper PDFs could not be downloaded from this environment, so the paper-side descriptions come from memory. Everything marked **(code)** was checked against the source.

---

## 0. The big picture

There are four layers. Each layer only needs the one below it.

```
4. PolyDepth            penetration depth = iterate { CCD out-projection ; LCP in-projection }
3. Conservative adv.    continuous collision detection (first time of contact along a motion)
2. BVH proximity        collide? / distance? / all feature pairs closer than ε?   (tree traversal)
1. Exact primitives     triangle–triangle overlap, triangle–triangle distance with features
```

**What is computed.** For two meshes A and B, the *translational penetration depth* is

  PD(A,B) = min { ‖d‖ : interior(A + d) ∩ B = ∅ }

that is, the shortest translation of A that makes the meshes touch but not overlap. In *configuration space* (the space of A's translations) the set of overlapping translations is the Minkowski sum O = B ⊕ (−A). Its boundary ∂O is the **contact space**. So the PD is the distance from the current translation q_in to ∂O.

O is non-convex and has O(n²)…O(n⁶) complexity for general meshes. PolyDepth never builds it. It only explores ∂O locally, using collision queries.

---

## 1. Exact primitives (PQP `TriDist.cpp`, `TriContact`; C2A `C2A_TriDistance`)

**Triangle–triangle overlap.** PQP tests the 6 face/edge plane tests plus 9 edge-cross-edge separating axes. (The Möller and Guigue–Devillers tests are common alternatives.)

**Triangle–triangle distance.** The closest pair between two triangles is always one of:
- one of the 9 **edge–edge** pairs (segment–segment closest points), or
- one of the 6 **vertex–face** pairs (a vertex projected onto the other triangle's plane, if it falls inside).

```
TriDist(S, T):
  best = ∞
  for each edge e of S, each edge f of T:          # 9 pairs
      (p, q) = closest points of segments e, f
      keep if |p-q| < best                         # also remember features (E,E) or (V,…)
  for each vertex v of S: project onto plane(T); if inside T keep (V,F)
  for each vertex v of T: project onto plane(S); if inside S keep (F,V)
  if no separation found and triangles overlap: distance = 0
  return best, p, q, featureS, featureT            # feature ∈ {vertex, edge, face}
```

C2A's version (`C2A_TriDistance`, **code**) also returns the *feature type* (0 = vertex, 1 = edge, 2 = face) and the local index. That type and index later become `FeatureType_A/B` ∈ {1, 2, 3} and the mesh vertex ids. PolyDepth needs them.

---

## 2. Bounding-volume hierarchy and proximity queries (PQP)

**Build (top-down, `Build.cpp`):**
```
build(tris):
  axes = eigenvectors of the covariance of tris        # principal axes
  node.bv = fit OBB and RSS to tris in that frame
  if |tris| == 1: leaf
  split tris by centroid at the mean along the longest axis
  build(left); build(right)
```
- **OBB** (oriented box): tight; used for *overlap* tests (separating-axis test, 15 axes).
- **RSS** (rectangle swept sphere = a rectangle inflated by radius r): cheap and robust *distance lower bound* between two nodes.

All queries traverse both trees together with the same branch-and-bound pattern:

```
Distance(A, B):                        # PQP_Distance / C2A_Distance
  best = ∞
  recurse(rootA, rootB)
  recurse(a, b):
    if bvDist(a, b) ≥ best: return     # prune: lower bound can't beat best
    if leaf(a) and leaf(b): best = min(best, TriDist(a.tri, b.tri)); return
    split the bigger node; visit the closer child pair first

Collide(A, B):  same, prune when OBBs don't overlap; at leaves TriContact; stop at first hit if FIRST_CONTACT
Tolerance/contacts(A, B, ε):  same, prune when bvDist > ε; at leaves record every feature pair with dist ≤ ε
```
The last query is `C2A_QueryContact` / `TOCStepRecurse_Dis_contact` (**code**). PolyDepth uses it to obtain the contact features at a contact configuration.

---

## 3. Conservative advancement CCD (C2A)

**Problem.** A moves from pose q0 to q1 (t ∈ [0,1]) and B is static, or both move. Find the first t where they touch.

**Core idea.** Suppose the current distance is d, and no point of A can approach B faster than μ (the *motion bound*, distance per unit t). Then nothing can touch before Δt = d/μ. Advance by Δt and repeat. That is conservative advancement (CA).

```
CA(A, B, motion):
  t = 0
  loop:
    d, n = distance(A(t), B)                  # n = unit direction of closest points
    if d < δ: return t                        # time of contact (TOC)
    μ = motionBound(n)                        # translation: v·n ; with rotation: v·n + |ω×n|·r_max
    if μ ≤ 0: return no-collision
    t += d / μ
    if t > 1: return no-collision
```

**Why translation-only is easy (this is the only case PolyDepth uses, code: `b_TanslationCCD`).** For two *convex* pieces (e.g. two triangles) under translation `t·v`, the distance d(t) is a convex function of t, with slope −v·n at t = 0. So d(t) ≥ d − (v·n)t, and Δt = d/(v·n) is safe. A mesh is a union of triangles. So run CA per triangle pair and take the minimum TOC over all pairs. The BVH is used to avoid testing all pairs:

```
TOC_translation(A, B, v):                      # C2A TOCStepRecurse_Dis_Translation (code)
  best_t = 1
  recurse(a, b):
    t_ab = CA on the two RSS volumes (iterated d/(v·n) on bvDist)  # lower bound for anything inside
    if t_ab ≥ best_t: return                   # prune
    if leaf(a) and leaf(b):
        best_t = min(best_t, CA on the two triangles (iterated d/(v·n)))
        return
    split bigger node; recurse into both children (nearer first)
  return best_t
```

**"Controlled" (the C2A contribution).** Early CA iterations run the distance query with loose relative/absolute error bounds (`rel_err = 3 → 0.5`), which is cheaper. Near contact they switch to exact distance. The general (rotating) path also uses per-node motion bounds `v·n + |ω×n|·angularRadius(node)`. PolyDepth never exercises that path.

After the TOC is found, C2A places A at the TOC pose. It runs the contact query from §2 with threshold `2·d + 0.001` (**code**) and returns the list of feature pairs (V/E/F + vertex ids + closest points).

---

## 4. PolyDepth

### 4.1 The loop

```
PolyDepth(A, B, q_in):
  if not Collide(A at q_in, B): return "no penetration"

  q_f = InitialFreeConfiguration(q_in)                    # §4.4
  for it = 1 .. maxIter:
      # (1) OUT-PROJECTION: CCD along the segment q_f → q_in, lands exactly on ∂O
      t   = TOC(A moving q_f → q_in, B)
      q_c = q_f + t (q_in − q_f)                          # contact configuration
      F   = ContactFeatures(A at q_c, B, ε)               # V-F, F-V, E-E pairs

      # (2) LOCAL LINEARISATION of ∂O around q_c
      H   = { half-space h_i = (n_i, p_i) for each pair in F }   # §4.2
      remove near-parallel duplicates (n_i·n_j > 0.99)

      # (3) IN-PROJECTION: closest point to q_in in the cone ∩ h_i   # §4.3
      d   = argmin ‖d‖  s.t.  n_i·d ≥ n_i·p_i   for all i
      q_p = q_in + d + (ε/2)·d/‖d‖                        # small safety margin

      # (4) DECIDE
      if ‖q_p − q_in‖ ≥ ‖q_c − q_in‖: return q_c − q_in    # projection is no better
      dist = Distance(A at q_p, B)
      if dist > ε:            q_f = q_p ; continue          # free and closer: new start
      if 0 < dist ≤ ε:        return q_p − q_in             # in contact: done
      else (still overlapping): one more CCD from just outside q_c, return
  return q_c − q_in
```

The key invariant: **q_c is always a valid contact configuration**, so ‖q_c − q_in‖ is always an upper bound on the PD. Each loop iteration only replaces it with something shorter. The result is a *local* minimum. That is a property of the method, not a bug.

(Code note, see the report: in the "in contact" case the code returns the CCD pose q_c, not q_p, and the loop runs at most 10 iterations.)

### 4.2 From contact features to half-spaces in configuration space

A contact feature pair corresponds to a facet of the Minkowski sum. With `a*` = points of A (moved by the current pose) and `b*` = points of B (code: `mnkp`, `mnkn`; cases from `cl = 3·typeA + typeB`):

| Pair (A, B) | Point p | Normal n |
|---|---|---|
| vertex a – face (b1,b2,b3) (`cl=6`) | b1 − a | (b2 − b1) × (b3 − b1) |
| face (a1,a2,a3) – vertex b (`cl=10`) | b − a1 | (a2 − a1) × (a3 − a1) (via the Minkowski points) |
| edge (a1,a2) – edge (b1,b2) (`cl=8`) | b1 − a1 | (a2 − a1) × (b2 − b1) |

Then flip n so that n·p > 0, and normalise. The half-space of *non-overlap* for that local feature pair is { d : n·d ≥ n·p }: A must be translated past the plane. Near q_c, the free space is approximately the intersection of these half-spaces, which is a convex polyhedral cone.

VV, VE and EV pairs are skipped in the code. They are degenerate cases whose normal is ambiguous. A common fix is to use the closest-point direction as n.

### 4.3 The projection as an LCP, solved with projected Gauss–Seidel

Minimise ½‖d‖² subject to N d ≥ b (rows n_i, b_i = n_i·p_i). The KKT conditions give

  d = Nᵀλ,  w = N Nᵀ λ − b,  λ ≥ 0, w ≥ 0, λᵢwᵢ = 0

That is an LCP with matrix A = N Nᵀ (A_ij = n_i·n_j). Projected Gauss–Seidel:

```
PGS(A, b, iterations K):
  λ = 0
  repeat K times:
    for i in 1..m:
      λ_i = max(0, (b_i − Σ_{j≠i} A_ij λ_j) / A_ii)
  return d = Σ λ_i n_i
```
The code does a single sweep and clamps a little differently (see the report). With m usually small (≤ 20, often ≤ 3 *active* ones in 3-D), you can also solve this exactly with a tiny active-set QP.

### 4.4 Finding the first collision-free configuration (`GuessContactFreeSpace`, code)

Candidates are tried in this order; the closest one that is not colliding wins:
1. **Motion coherence**: the free/contact poses from previous frames (`ClearConfigurations`, 3 slots). If one now collides, nudge it along the same direction by ±k·bbox/10.
2. **Directional**: step along the centroid difference (A − B), ±k·bbox/10, k = 1..9.
3. **Maximally clear configurations (MCC)**: precomputed points far from B's surface (below).
4. **Fallback**: move A far out along the centroid direction, by ‖bbox_A‖ + ‖bbox_B‖. Always free.

**MCC precomputation** (`FindMaximallyClearConfigure`, code):
```
for each point x of a regular grid in B's bounding box (≈ N points):
    d = Distance(point x, B)        # done with a tiny triangle as "point"
    inside = (closest triangle's normal) · (tri − x) > 0
    keep x if d > 2·cell and inside
non-maximum suppression: drop x if another kept y has dist(y) > dist(x) and |x−y| < dist(y)
drop points near the 8 bbox corners
```
These are deep points of B's *medial axis*. Starting from one, the CCD hits the surface from a good direction.

### 4.5 Local penetration depth (compiled out in the code)

Cluster the contact points at the final pose into a grid with resolution `feature_clustering_resolution`. For each cluster with normal n: `localPD = (PD·n)·n`, reported together with the contact points on A and B.

### 4.6 Skin depth

If the objects do not overlap but are within ε: `pseudoPD = (ε − dist)` along the centroid direction, plus contact features. This is a heuristic for "almost touching".

---

## 5. How the pieces call each other

```
PolyDepth
 ├─ Collide (BVH, OBB + TriContact)                         §2, §1
 ├─ InitialFreeConfiguration → Collide (many times)         §4.4
 └─ loop
     ├─ TOC_translation (BVH + RSS CA + triangle CA)        §3, §2, §1
     ├─ ContactFeatures (BVH tolerance query + TriDist with features)
     ├─ half-spaces + PGS                                    §4.2–4.3
     └─ Distance (BVH + RSS + TriDist)                      §2, §1
```
