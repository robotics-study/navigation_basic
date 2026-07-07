---
title: Algorithms
layout: default
parent: English
nav_order: 1
has_children: true
permalink: /en/algorithms/
---

[🇰🇷 한국어](../../ko/algorithms/index.md) | [🇬🇧 English](index.md)

# Algorithms

Detailed pages for each implemented algorithm. Every page covers the theoretical background (with original-paper footnotes), pseudocode, properties (completeness, optimality, complexity), parameter declarations, emitted trace events, demo visualizations (GIF + intermediate-progress PNGs), and measured metrics.

## Global planning — Implemented

| Algorithm | Year | Approach | Completeness | Optimality | Original paper |
|---|---|---|---|---|---|
| [BFS](bfs.md) | 1959 | uninformed search | complete | fewest edges (optimal under unit cost) | Moore [^moore] |
| [Dijkstra](dijkstra.md) | 1959 | uninformed search | complete | cost-optimal | Dijkstra [^dijkstra] |
| [A*](astar.md) | 1968 | informed search | complete | cost-optimal (admissible h) | Hart, Nilsson & Raphael [^hart] |
| [JPS](jps.md) | 2011 | grid symmetry breaking | complete | cost-optimal (= 8-connected A*) | Harabor & Grastien [^harabor_jps] |
| [D\* Lite](dstar_lite.md) | 2002 | dynamic replanning | complete | optimal for the belief (incremental repair) | Koenig & Likhachev [^koenig] |
| [Theta\*](theta_star.md) | 2007 | any-angle search | complete | any-angle (not grid-optimal) | Nash, Daniel, Koenig & Felner [^nash] |
| [Lazy Theta\*](lazy_theta_star.md) | 2010 | any-angle search (lazy LOS) | complete | any-angle (not grid-optimal) | Nash, Koenig & Tovey [^nash_lazy] |
| [Visibility A\*](visibility_astar.md) | — | any-angle (visibility graph) | complete | cell-centre visibility shortest (not true Euclidean optimum) | — |
| [Hybrid A\*](hybrid_astar.md) | 2008 | kinodynamic search | resolution-complete | resolution-suboptimal (feasible) | Dolgov, Thrun, Montemerlo & Diebel [^dolgov] |
| [RRT](rrt.md) | 1998 | sampling | probabilistically complete | non-optimal (feasible) | LaValle [^lavalle98] |
| [RRT-Connect](rrt_connect.md) | 2000 | sampling (bidirectional) | probabilistically complete | non-optimal (feasible, single-query) | Kuffner & LaValle [^kuffner] |
| [RRT\*](rrt_star.md) | 2011 | sampling | probabilistically complete | asymptotically optimal | Karaman & Frazzoli [^karaman] |
| [Kinodynamic RRT\*](kinodynamic_rrt_star.md) | 2013 | sampling (kinodynamic) | probabilistically complete | asymptotically optimal (dynamics cost) | Webb & van den Berg [^webb] |
| [Informed RRT\*](informed_rrt_star.md) | 2014 | sampling (direct informed) | probabilistically complete | asymptotically optimal | Gammell, Srinivasa & Barfoot [^gammell] |
| [PRM](prm.md) | 1996 | sampling (roadmap) | probabilistically complete | non-optimal (fixed radius) | Kavraki et al. [^kavraki] |
| [PRM\*](prm_star.md) | 2011 | sampling (roadmap) | probabilistically complete | asymptotically optimal | Karaman & Frazzoli [^karaman] |
| [FMT\*](fmt_star.md) | 2015 | sampling (batch marching) | probabilistically complete | asymptotically optimal | Janson et al. [^janson] |
| [BIT\*](bit_star.md) | 2015 | sampling (batch + informed) | probabilistically complete | almost-surely asymptotically optimal | Gammell et al. [^gammell_bit] |
| [SST](sst.md) | 2016 | sampling (kinodynamic, forward-propagation) | probabilistically complete | near-optimal (SST) / asymptotically optimal (SST\*) | Li, Littlefield & Bekris [^li] |
| [AIT\*](ait_star.md) | 2020 | sampling (asymmetric bidirectional) | probabilistically complete | almost-surely asymptotically optimal | Strub & Gammell [^strub_ait] |
| [EIT\*](eit_star.md) | 2022 | sampling (effort-informed bidirectional) | probabilistically complete | almost-surely asymptotically optimal | Strub & Gammell [^strub_eit] |
| [FCIT\*](fcit_star.md) | 2025 | sampling (fully-connected informed) | probabilistically complete | almost-surely asymptotically optimal | Wilson, Strub & Gammell [^wilson_fcit] |
| [Fast-RRT](fast_rrt.md) | 2021 | sampling (RRT\* variant) | probabilistically complete | near-optimal (faster convergence than RRT\*) | Wu et al. [^wu] |

> **Lineage note.** The sampling-based optimal family has two threads that both converge on informed,
> batch, bidirectional search. **Single-tree optimal:** RRT\* (2011) → Informed RRT\* (2014, ellipsoidal
> direct sampling). **Roadmap/batch-optimal:** PRM → PRM\* → FMT\* → BIT\* (2015). These merge into the
> **informed-trees line** BIT\* → AIT\* (2020, asymmetric bidirectional with an adaptively computed
> reverse heuristic) → EIT\* (2022, effort-informed) → FCIT\* (2025, fully-connected, exploiting cheap
> collision checks) — the current asymptotically-optimal frontier. Separately, **RRT-Connect** (2000) is
> the bidirectional single-query descendant of RRT, and [Fast-RRT](fast_rrt.md) (2021) is an RRT\* variant
> that adds shortcutting for faster convergence. On the kinodynamic sampling branch, **Kinodynamic RRT\***
> (2013) extends RRT\*'s optimality to differential-constraint systems via an optimal steering controller,
> and **SST** (2016) drops the steering/BVP requirement entirely, staying asymptotically optimal with
> forward propagation alone by keeping the tree sparse through a witness set.
>
> The **grid-search** lineage branches too: **JPS** (2011) is a lossless successor-pruning of A\* that
> returns identical optimal paths on uniform-cost 8-connected grids with an order of magnitude fewer
> expansions, while the **any-angle** line runs Theta\* (2007) → **Lazy Theta\*** (2010, one line-of-sight
> check per expanded vertex instead of per edge) → **Visibility A\***, which relaxes every LOS-visible
> cell from each expanded root to settle the cell-centre visibility-graph optimum — dominating Theta\*,
> though still an approximation (turns pinned to cell centres, not the true Euclidean any-angle optimum).
> Sources stay in a flat directory layout; the tree above is the lineage, not the folder structure.

## Planned (Not Yet Implemented)

| Category | Algorithm | Original paper |
|---|---|---|
| local_planning | DWA | Fox, Burgard & Thrun (1997) [^fox] |
| local_planning | Pure Pursuit | Coulter (1992) [^coulter] |
| local_planning | VFH | Borenstein & Koren (1991) [^borenstein] |
| local_planning | MPC | — (survey: Rawlings, Mayne & Diehl 2017) |
| multi_agent | Prioritized planning | Erdmann & Lozano-Pérez (1987) [^erdmann] |
| multi_agent | CBS | Sharon et al. (2015) [^sharon] |

## Running the Demos

All demos take the same CLI arguments in both languages:

```bash
# Python
python python/demos/demo_<algo>.py \
  --map maps/grid/<map>.yaml --scenario maps/scenarios/<map>_s1.yaml \
  --params configs/global_planning/<algo>.yaml --trace out/<algo>.jsonl \
  [--seed N] [--connectivity 4|8]

# C++ (identical arguments)
./cpp/build/demos/demo_<algo> --map ... --scenario ... --params ... --trace ...

# visualization: animated GIF + mid-search PNG snapshots
python tools/viz/replay.py out/<algo>.jsonl --gif out/<algo>.gif --snapshots out/snaps/
```

## References

[^moore]: Moore, E. F. (1959). "The shortest path through a maze." *Proceedings of the International Symposium on the Theory of Switching*, Harvard University Press, 285–292.
[^dijkstra]: Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs." *Numerische Mathematik*, 1, 269–271. [doi:10.1007/BF01386390](https://doi.org/10.1007/BF01386390)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
[^nash]: Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). "Theta\*: Any-Angle Path Planning on Grids." *Proc. AAAI Conference on Artificial Intelligence*, 1177–1183. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/11009)
[^koenig]: Koenig, S., & Likhachev, M. (2002). "D\* Lite." *Proc. AAAI Conference on Artificial Intelligence*, 476–483. [PDF](https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf)
[^dolgov]: Dolgov, D., Thrun, S., Montemerlo, M., & Diebel, J. (2008). "Practical Search Techniques in Path Planning for Autonomous Driving." *Proc. STAIR (AAAI Workshop)*. [PDF](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
[^lavalle98]: LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning." Technical Report TR 98-11, Computer Science Dept., Iowa State University. [PDF](https://lavalle.pl/papers/Lav98c.pdf)
[^karaman]: Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *The International Journal of Robotics Research*, 30(7), 846–894. [doi:10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
[^kavraki]: Kavraki, L. E., Švestka, P., Latombe, J.-C., & Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces." *IEEE Transactions on Robotics and Automation*, 12(4), 566–580. [doi:10.1109/70.508439](https://doi.org/10.1109/70.508439)
[^janson]: Janson, L., Schmerling, E., Clark, A., & Pavone, M. (2015). "Fast marching tree: A fast marching sampling-based method for optimal motion planning in many dimensions." *The International Journal of Robotics Research*, 34(7), 883–921. [doi:10.1177/0278364915577958](https://doi.org/10.1177/0278364915577958) · [PDF (arXiv)](https://arxiv.org/abs/1306.3532)
[^gammell_bit]: Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2015). "Batch Informed Trees (BIT\*): Sampling-based optimal planning via the heuristically guided search of implicit random geometric graphs." *Proc. IEEE ICRA*, 3067–3074. [doi:10.1109/ICRA.2015.7139620](https://doi.org/10.1109/ICRA.2015.7139620) · [PDF (arXiv)](https://arxiv.org/abs/1405.5848)
[^wu]: Wu, Z., Meng, Z., Zhao, W., & Wu, Z. (2021). "Fast-RRT: A RRT-Based Optimal Path Finding Method." *Applied Sciences*, 11(24), 11777. [doi:10.3390/app112411777](https://doi.org/10.3390/app112411777) · [PDF (open access)](https://www.mdpi.com/2076-3417/11/24/11777)
[^strub_ait]: Strub, M. P., & Gammell, J. D. (2020). "Adaptively Informed Trees (AIT\*): Fast Asymptotically Optimal Path Planning through Adaptive Heuristics." *Proc. IEEE ICRA*, 3191–3198. [doi:10.1109/ICRA40945.2020.9197338](https://doi.org/10.1109/ICRA40945.2020.9197338) · Extended in Strub & Gammell (2022), *IJRR* 41(4), 390–417. [PDF (arXiv)](https://arxiv.org/abs/2111.01877)
[^strub_eit]: Strub, M. P., & Gammell, J. D. (2022). "Adaptively Informed Trees (AIT\*) and Effort Informed Trees (EIT\*): Asymmetric bidirectional sampling-based path planning." *The International Journal of Robotics Research*, 41(4), 390–417. [doi:10.1177/02783649211069572](https://doi.org/10.1177/02783649211069572) · [PDF (arXiv)](https://arxiv.org/abs/2111.01877)
[^wilson_fcit]: Wilson, T., Strub, M. P., & Gammell, J. D. (2025). "Nearest-Neighbourless Asymptotically Optimal Motion Planning with Fully Connected Informed Trees (FCIT\*)." *Proc. IEEE ICRA*. [PDF (arXiv:2411.17902)](https://arxiv.org/abs/2411.17902)
[^harabor_jps]: Harabor, D., & Grastien, A. (2011). "Online Graph Pruning for Pathfinding on Grid Maps." *Proc. AAAI Conference on Artificial Intelligence*, 1114–1119. [PDF](https://ojs.aaai.org/index.php/AAAI/article/view/7994)
[^nash_lazy]: Nash, A., Koenig, S., & Tovey, C. (2010). "Lazy Theta\*: Any-Angle Path Planning and Path Length Analysis in 3D." *Proc. AAAI Conference on Artificial Intelligence*, 147–154. [PDF](http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf)
[^webb]: Webb, D. J., & van den Berg, J. (2013). "Kinodynamic RRT\*: Asymptotically Optimal Motion Planning for Robots with Linear Dynamics." *Proc. IEEE ICRA*, 5054–5061. [doi:10.1109/ICRA.2013.6631299](https://doi.org/10.1109/ICRA.2013.6631299) · [PDF (arXiv)](https://arxiv.org/abs/1205.5088)
[^li]: Li, Y., Littlefield, Z., & Bekris, K. E. (2016). "Asymptotically optimal sampling-based kinodynamic planning." *The International Journal of Robotics Research*, 35(5), 528–564. [doi:10.1177/0278364915614386](https://doi.org/10.1177/0278364915614386) · [PDF (arXiv)](https://arxiv.org/abs/1407.2896)
[^kuffner]: Kuffner, J. J., & LaValle, S. M. (2000). "RRT-Connect: An efficient approach to single-query path planning." *Proc. IEEE ICRA*, 995–1001. [doi:10.1109/ROBOT.2000.844730](https://doi.org/10.1109/ROBOT.2000.844730)
[^gammell]: Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). "Informed RRT\*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic." *Proc. IEEE/RSJ IROS*, 2997–3004. [doi:10.1109/IROS.2014.6942976](https://doi.org/10.1109/IROS.2014.6942976) · [PDF (arXiv)](https://arxiv.org/abs/1404.2334)
[^fox]: Fox, D., Burgard, W., & Thrun, S. (1997). "The dynamic window approach to collision avoidance." *IEEE Robotics & Automation Magazine*, 4(1), 23–33. [doi:10.1109/100.580977](https://doi.org/10.1109/100.580977)
[^coulter]: Coulter, R. C. (1992). "Implementation of the Pure Pursuit path tracking algorithm." Technical Report CMU-RI-TR-92-01, Robotics Institute, Carnegie Mellon University. [PDF](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
[^borenstein]: Borenstein, J., & Koren, Y. (1991). "The vector field histogram — fast obstacle avoidance for mobile robots." *IEEE Transactions on Robotics and Automation*, 7(3), 278–288. [doi:10.1109/70.88137](https://doi.org/10.1109/70.88137)
[^erdmann]: Erdmann, M., & Lozano-Pérez, T. (1987). "On multiple moving objects." *Algorithmica*, 2, 477–521. [doi:10.1007/BF01840371](https://doi.org/10.1007/BF01840371)
[^sharon]: Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015). "Conflict-based search for optimal multi-agent pathfinding." *Artificial Intelligence*, 219, 40–66. [doi:10.1016/j.artint.2014.11.006](https://doi.org/10.1016/j.artint.2014.11.006)
