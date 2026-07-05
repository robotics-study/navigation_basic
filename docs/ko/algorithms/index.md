---
title: 알고리즘
layout: default
parent: 한국어
nav_order: 1
has_children: true
permalink: /ko/algorithms/
---

[🇰🇷 한국어](index.md) | [🇬🇧 English](../../en/algorithms/index.md)

# 알고리즘

구현된 알고리즘별 상세 페이지. 각 페이지는 이론적 배경(원 논문 각주 포함), pseudocode,
성질(완전성·최적성·복잡도), 파라미터 선언, 방출 trace 이벤트, demo 시각화(GIF + 중간 과정 PNG)와
실측 metric 을 담는다.

## Global planning — 구현 완료

| 알고리즘 | 연도 | 접근 | 완전성 | 최적성 | 원 논문 |
|---|---|---|---|---|---|
| [BFS](bfs.md) | 1959 | uninformed search | complete | edge 수 최소 (unit cost 에서 최적) | Moore [^moore] |
| [Dijkstra](dijkstra.md) | 1959 | uninformed search | complete | cost 최적 | Dijkstra [^dijkstra] |
| [A*](astar.md) | 1968 | informed search | complete | cost 최적 (admissible h) | Hart, Nilsson & Raphael [^hart] |
| [RRT](rrt.md) | 1998 | sampling | probabilistically complete | 비최적 (feasible) | LaValle [^lavalle98] |
| [RRT*](rrt_star.md) | 2011 | sampling | probabilistically complete | asymptotically optimal | Karaman & Frazzoli [^karaman] |
| [Fast-RRT](fast_rrt.md) | 2021 | sampling | probabilistically complete | near-optimal (RRT* 대비 고속 수렴) | Wu et al. [^wu] |

## 계획 (미구현)

| 카테고리 | 알고리즘 | 원 논문 |
|---|---|---|
| global_planning | RRT-Connect | Kuffner & LaValle (2000) [^kuffner] |
| global_planning | Informed RRT* | Gammell, Srinivasa & Barfoot (2014) [^gammell] |
| local_planning | DWA | Fox, Burgard & Thrun (1997) [^fox] |
| local_planning | Pure Pursuit | Coulter (1992) [^coulter] |
| local_planning | VFH | Borenstein & Koren (1991) [^borenstein] |
| local_planning | MPC | — (survey: Rawlings, Mayne & Diehl 2017) |
| multi_agent | Prioritized planning | Erdmann & Lozano-Pérez (1987) [^erdmann] |
| multi_agent | CBS | Sharon et al. (2015) [^sharon] |

## 공통 실행 방법

모든 데모는 두 언어에서 동일한 CLI 인자를 갖는다:

```bash
# Python
python python/demos/demo_<algo>.py \
  --map maps/grid/<map>.yaml --scenario maps/scenarios/<map>_s1.yaml \
  --params configs/global_planning/<algo>.yaml --trace out/<algo>.jsonl \
  [--seed N] [--connectivity 4|8]

# C++ (동일 인자)
./cpp/build/demos/demo_<algo> --map ... --scenario ... --params ... --trace ...

# 시각화: GIF 애니메이션 + 탐색 중간 과정 PNG 스냅샷
python tools/viz/replay.py out/<algo>.jsonl --gif out/<algo>.gif --snapshots out/snaps/
```

## References

[^moore]: Moore, E. F. (1959). "The shortest path through a maze." *Proceedings of the International Symposium on the Theory of Switching*, Harvard University Press, 285–292.
[^dijkstra]: Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs." *Numerische Mathematik*, 1, 269–271. [doi:10.1007/BF01386390](https://doi.org/10.1007/BF01386390)
[^hart]: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
[^lavalle98]: LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning." Technical Report TR 98-11, Computer Science Dept., Iowa State University. [PDF](https://lavalle.pl/papers/Lav98c.pdf)
[^karaman]: Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *The International Journal of Robotics Research*, 30(7), 846–894. [doi:10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761) · [PDF (arXiv)](https://arxiv.org/abs/1105.1186)
[^wu]: Wu, Z., Meng, Z., Zhao, W., & Wu, Z. (2021). "Fast-RRT: A RRT-Based Optimal Path Finding Method." *Applied Sciences*, 11(24), 11777. [doi:10.3390/app112411777](https://doi.org/10.3390/app112411777) · [PDF (open access)](https://www.mdpi.com/2076-3417/11/24/11777)
[^kuffner]: Kuffner, J. J., & LaValle, S. M. (2000). "RRT-Connect: An efficient approach to single-query path planning." *Proc. IEEE ICRA*, 995–1001. [doi:10.1109/ROBOT.2000.844730](https://doi.org/10.1109/ROBOT.2000.844730)
[^gammell]: Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). "Informed RRT*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic." *Proc. IEEE/RSJ IROS*, 2997–3004. [doi:10.1109/IROS.2014.6942976](https://doi.org/10.1109/IROS.2014.6942976) · [PDF (arXiv)](https://arxiv.org/abs/1404.2334)
[^fox]: Fox, D., Burgard, W., & Thrun, S. (1997). "The dynamic window approach to collision avoidance." *IEEE Robotics & Automation Magazine*, 4(1), 23–33. [doi:10.1109/100.580977](https://doi.org/10.1109/100.580977)
[^coulter]: Coulter, R. C. (1992). "Implementation of the Pure Pursuit path tracking algorithm." Technical Report CMU-RI-TR-92-01, Robotics Institute, Carnegie Mellon University. [PDF](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
[^borenstein]: Borenstein, J., & Koren, Y. (1991). "The vector field histogram — fast obstacle avoidance for mobile robots." *IEEE Transactions on Robotics and Automation*, 7(3), 278–288. [doi:10.1109/70.88137](https://doi.org/10.1109/70.88137)
[^erdmann]: Erdmann, M., & Lozano-Pérez, T. (1987). "On multiple moving objects." *Algorithmica*, 2, 477–521. [doi:10.1007/BF01840371](https://doi.org/10.1007/BF01840371)
[^sharon]: Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015). "Conflict-based search for optimal multi-agent pathfinding." *Artificial Intelligence*, 219, 40–66. [doi:10.1016/j.artint.2014.11.006](https://doi.org/10.1016/j.artint.2014.11.006)
